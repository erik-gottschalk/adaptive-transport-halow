/*
 * Copyright 2026 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "mmhal_core.h"

#include "adaptive_transport_controller.h"

/** Logging sink, as in adaptive_transport.c. */
#define DEMO_LOG(...) printf(__VA_ARGS__)

/** Minimum pooled sample count in the change-detection ring before it acts. */
#define ATC_CD_MIN_SAMPLES 10

/** Standard deviations the windowed estimate must diverge from p_udp_hat by before change
 * detection snaps p_udp_hat to it. Lower towards degradation, which costs the loss budget, than
 * towards recovery, which only costs energy. */
#define ATC_CD_SIGMA_DOWN 2.0
#define ATC_CD_SIGMA_UP 3.0

/** Weak prior of the time estimator, in UDP samples: q_hat = (lost + 0.05) / (lost + received +
 * 0.5), i.e. half a sample at loss 0.1. Only keeps q_hat defined before the first report and
 * after long stretches without UDP. */
#define ATC_EST_PRIOR_LOST 0.05
#define ATC_EST_PRIOR_WEIGHT 0.5

/** Default random source: hardware TRNG (via mmhal_random_u32()) scaled to [0, 1). */
static double atc_default_random(void)
{
    return (double)mmhal_random_u32(0, UINT32_MAX) / ((double)UINT32_MAX + 1.0);
}

void atc_init(struct adaptive_transport_controller *ctl,
             int x, double L, int N0, double eps, int calibration_packets,
             bool use_change_detection)
{
    int i;

    ctl->x = x;
    ctl->L = L;
    ctl->N0 = N0;
    ctl->eps = eps;
    ctl->use_change_detection = use_change_detection;
    ctl->a_L = 1.0;

    ctl->p_udp_hat = 0.5;
    ctl->last_window_sent = 0;
    ctl->last_window_received = 0;
    ctl->p_tcp_hat = 0.99;   /* already >= p_udp_hat; every later update goes through
                              * atc_set_p_tcp_hat() */

    ctl->cycle_counter = 1;
    ctl->udp_since_last_tcp = 0;
    ctl->pending_feedback_udp = 0;
    ctl->carry_sent_udp = 0;
    ctl->carry_received_udp = 0;
    ctl->drop_next_window = false;

    ctl->calibrating = true;
    ctl->calibration_packets = calibration_packets;
    ctl->calibration_udp_remaining = calibration_packets;

    for (i = 0; i < ATC_CD_WINDOW_SIZE; i++)
    {
        ctl->cd_window_sent[i] = 0;
        ctl->cd_window_received[i] = 0;
    }
    ctl->cd_window_index = 0;
    ctl->cd_window_count = 0;

    ctl->random_probe = ATC_DEFAULT_RANDOM_PROBE;

    ctl->use_time_estimator = ATC_DEFAULT_USE_TIME_ESTIMATOR;
    ctl->est_half_life = ATC_DEFAULT_EST_HALF_LIFE;
    ctl->age_half_life = ATC_DEFAULT_AGE_HALF_LIFE;
    ctl->age_target_factor = ATC_DEFAULT_AGE_TARGET_FACTOR;
    ctl->est_lost = 0.0;
    ctl->est_received = 0.0;
    ctl->age_weight = 1.0;

    ctl->debt_horizon = ATC_DEFAULT_DEBT_HORIZON;
    ctl->debt = 0.0;
    ctl->debt_per_class = ATC_DEFAULT_DEBT_PER_CLASS;
    ctl->debt_tcp_failures = ATC_DEFAULT_DEBT_TCP_FAILURES;
    ctl->debt_class = 0;
    for (int k = 0; k < ATC_MAX_DEBT_CLASSES; k++)
    {
        ctl->debt_by_class[k] = 0.0;
    }
    ctl->udp_class_count = 0;

    ctl->total_packets = 0;
    ctl->total_udp = 0;
    ctl->total_tcp = 0;

    ctl->random_fn = atc_default_random;

    DEMO_LOG("ATC: init x=%d L=%.4f N0=%d calibration_packets=%d use_change_detection=%d\n",
           ctl->x, ctl->L, ctl->N0, ctl->calibration_packets, (int)ctl->use_change_detection);
    DEMO_LOG("ATC: estimator use_time_estimator=%d est_half_life=%.1f age_half_life=%.1f "
           "age_target_factor=%.2f debt_horizon=%.1f debt_per_class=%d random_probe=%d\n",
           (int)ctl->use_time_estimator, ctl->est_half_life, ctl->age_half_life,
           ctl->age_target_factor, ctl->debt_horizon, (int)ctl->debt_per_class,
           (int)ctl->random_probe);
}

/** Sample-size weighted update of a running probability estimate (memory N0 samples). */
static double atc_weighted_update(double p_hat, int received, int sent, int N0)
{
    double p_obs;
    double w;

    if (sent == 0)
    {
        return p_hat;
    }

    p_obs = (double)received / (double)sent;
    w = (double)sent / (double)(sent + N0);

    return (1.0 - w) * p_hat + w * p_obs;
}

/** The only setter of ctl->p_tcp_hat: floors it at p_udp_hat (see the struct). */
static void atc_set_p_tcp_hat(struct adaptive_transport_controller *ctl, double value)
{
    if (value < ctl->p_udp_hat)
    {
        DEMO_LOG("ATC: p_tcp_hat floored at p_udp_hat=%.4f (was heading to %.4f)\n",
               ctl->p_udp_hat, value);
        ctl->p_tcp_hat = ctl->p_udp_hat;
    }
    else
    {
        ctl->p_tcp_hat = value;
    }
}

/**
 * Change detection (use_change_detection, earlier estimator): pool the last ATC_CD_WINDOW_SIZE
 * windows' UDP counts and, if the result is more than the direction's sigma threshold away from
 * p_udp_hat, snap p_udp_hat to it.
 */
static void atc_check_change_detection(struct adaptive_transport_controller *ctl,
                                       int received_udp, int sent_udp)
{
    int sent_total = 0;
    int received_total = 0;
    int i;
    double windowed_p;
    double std_dev;
    double sigma_threshold;

    ctl->cd_window_sent[ctl->cd_window_index] = sent_udp;
    ctl->cd_window_received[ctl->cd_window_index] = received_udp;
    ctl->cd_window_index = (ctl->cd_window_index + 1) % ATC_CD_WINDOW_SIZE;
    if (ctl->cd_window_count < ATC_CD_WINDOW_SIZE)
    {
        ctl->cd_window_count++;
    }

    for (i = 0; i < ctl->cd_window_count; i++)
    {
        sent_total += ctl->cd_window_sent[i];
        received_total += ctl->cd_window_received[i];
    }

    if (sent_total < ATC_CD_MIN_SAMPLES)
    {
        return;
    }

    windowed_p = (double)received_total / (double)sent_total;
    std_dev = sqrt(ctl->p_udp_hat * (1.0 - ctl->p_udp_hat) / (double)sent_total);

    if (std_dev <= 0.0)
    {
        return;
    }

    sigma_threshold = (windowed_p < ctl->p_udp_hat) ? ATC_CD_SIGMA_DOWN : ATC_CD_SIGMA_UP;

    if (fabs(windowed_p - ctl->p_udp_hat) > sigma_threshold * std_dev)
    {
        double old_p_udp_hat = ctl->p_udp_hat;

        ctl->p_udp_hat = windowed_p;
        DEMO_LOG("ATC: change-detected: windowed %d/%d UDP received (p=%.4f) vs p_udp_hat=%.4f "
               "(%.1f sigma, threshold %.1f) -> reset to %.4f\n",
               received_total, sent_total, windowed_p, old_p_udp_hat,
               fabs(windowed_p - old_p_udp_hat) / std_dev, sigma_threshold, ctl->p_udp_hat);
    }
}

/** Time estimator: q_hat from the decayed counts. */
static double atc_time_estimate(const struct adaptive_transport_controller *ctl)
{
    return (ctl->est_lost + ATC_EST_PRIOR_LOST) /
           (ctl->est_lost + ctl->est_received + ATC_EST_PRIOR_WEIGHT);
}

/** Time estimator, once per message: age the counts and the last report by one message. */
static void atc_time_estimator_tick(struct adaptive_transport_controller *ctl)
{
    double decay = pow(2.0, -1.0 / ctl->est_half_life);

    ctl->est_lost *= decay;
    ctl->est_received *= decay;
    ctl->age_weight *= pow(2.0, -1.0 / ctl->age_half_life);
    ctl->p_udp_hat = 1.0 - atc_time_estimate(ctl);
}

/** Whether the loss debt is on (see debt_horizon). */
static bool atc_debt_enabled(const struct adaptive_transport_controller *ctl)
{
    return ctl->debt_horizon > 0;
}

/** Per-class debt: the class of the message being planned/sent, clamped to the valid range. */
static int atc_debt_class(const struct adaptive_transport_controller *ctl)
{
    return (ctl->debt_class >= 0 && ctl->debt_class < ATC_MAX_DEBT_CLASSES) ? ctl->debt_class : 0;
}

/** debt_tcp_failures: charge a TCP send that failed locally, as one loss, to its class's debt (the
 *  shared account without debt_per_class). */
static void atc_debt_charge_tcp_failure(struct adaptive_transport_controller *ctl)
{
    if (!ctl->debt_tcp_failures || !atc_debt_enabled(ctl))
    {
        return;
    }
    if (ctl->debt_per_class)
    {
        ctl->debt_by_class[atc_debt_class(ctl)] += 1.0;
    }
    else
    {
        ctl->debt += 1.0;
    }
}

/** Per-class debt: count, per class, the @p n UDP sends that end @p skip sends before the most
 *  recent one. Returns how many of them the ring still held. */
static int atc_count_udp_classes(const struct adaptive_transport_controller *ctl, int skip, int n,
                                 int counts[ATC_MAX_DEBT_CLASSES])
{
    int found = 0;

    for (int k = 0; k < ATC_MAX_DEBT_CLASSES; k++)
    {
        counts[k] = 0;
    }
    for (int i = 0; i < n; i++)
    {
        uint32_t back = (uint32_t)(skip + i + 1);
        if (back > ctl->udp_class_count || back > ATC_DEBT_CLASS_RING)
        {
            break;
        }
        uint32_t slot = (ctl->udp_class_count - back) % ATC_DEBT_CLASS_RING;
        counts[(ctl->udp_class_ring[slot / 4] >> (2 * (slot % 4))) & 3]++;
        found++;
    }
    return found;
}

/**
 * Fold one window's report into the loss debt and the delivery estimate. @p received_udp is
 * clamped to @p sent_udp here, on the window actually applied: a single window's count can exceed
 * its sent count when a late request let the next window's arrivals be counted into it, and that
 * excess corrects the next window once the two are merged (see atc_resolve_held_reply()).
 */
static void atc_apply_feedback(struct adaptive_transport_controller *ctl,
                               int sent_udp, int received_udp)
{
    int server_received = received_udp;
    double old_p_udp_hat;

    if (received_udp > sent_udp)
    {
        received_udp = sent_udp;
    }
    if (received_udp < 0)
    {
        received_udp = 0;
    }
    if (sent_udp > 0)
    {
        ctl->last_window_sent = sent_udp;
        ctl->last_window_received = received_udp;
        /* Loss debt: charge the reported losses -- per class in proportion to the window's UDP
         * sends of each (see debt_per_class). */
        if (atc_debt_enabled(ctl) && !ctl->debt_per_class)
        {
            ctl->debt += sent_udp - received_udp;
        }
        else if (atc_debt_enabled(ctl))
        {
            int counts[ATC_MAX_DEBT_CLASSES];
            int found = atc_count_udp_classes(ctl, ctl->udp_since_last_tcp, sent_udp, counts);

            for (int k = 0; k < ATC_MAX_DEBT_CLASSES; k++)
            {
                if (found > 0)
                {
                    ctl->debt_by_class[k] += (double)(sent_udp - received_udp) * counts[k] / found;
                }
            }
            if (found == 0)       /* ring overrun: charge the current class */
            {
                ctl->debt_by_class[atc_debt_class(ctl)] += sent_udp - received_udp;
            }
        }
    }

    old_p_udp_hat = ctl->p_udp_hat;
    if (ctl->use_time_estimator)
    {
        /* Add the window to the decayed counts and restart the ageing. Calibration needs no special
         * case: the counts start empty, so the first window sets the estimate on its own. */
        if (sent_udp > 0)
        {
            ctl->est_lost += sent_udp - received_udp;
            ctl->est_received += received_udp;
            ctl->age_weight = 1.0;
            ctl->p_udp_hat = 1.0 - atc_time_estimate(ctl);
        }
        DEMO_LOG("ATC: %s: %d/%d UDP received (server reported %d received), "
               "p_udp_hat %.4f -> %.4f\n", ctl->calibrating ? "calibration complete" : "feedback",
               received_udp, sent_udp, server_received, old_p_udp_hat, ctl->p_udp_hat);
        ctl->calibrating = false;
    }
    else if (ctl->calibrating)
    {
        /* Calibration: set p_udp_hat from this one window instead of blending it in. */
        if (sent_udp > 0)
        {
            ctl->p_udp_hat = (double)received_udp / (double)sent_udp;
        }
        ctl->calibrating = false;
        DEMO_LOG("ATC: calibration complete: %d/%d UDP received, p_udp_hat %.4f -> %.4f (set)\n",
               received_udp, sent_udp, old_p_udp_hat, ctl->p_udp_hat);
    }
    else
    {
        ctl->p_udp_hat = atc_weighted_update(ctl->p_udp_hat, received_udp, sent_udp, ctl->N0);
        DEMO_LOG("ATC: feedback: %d/%d UDP received (server reported %d received), "
               "p_udp_hat %.4f -> %.4f\n",
               received_udp, sent_udp, server_received, old_p_udp_hat, ctl->p_udp_hat);

        /* A window without UDP carries no evidence about UDP and would only flush the ring. */
        if (ctl->use_change_detection && sent_udp > 0)
        {
            atc_check_change_detection(ctl, received_udp, sent_udp);
        }
    }
}

double atc_compute_free_udp_probability(const struct adaptive_transport_controller *ctl)
{
    double qU = 1.0 - ctl->p_udp_hat;
    double qT = 1.0 - ctl->p_tcp_hat;

    double forced_fraction;

    /* Time estimator: plan with the estimate relaxed towards the ageing target as the last report
     * ages -- pessimistic side only. */
    if (ctl->use_time_estimator)
    {
        double q_age = ctl->age_target_factor * ctl->L;

        if (qU < q_age)
        {
            qU = q_age - (q_age - qU) * ctl->age_weight;
        }
    }
    double free_fraction;
    double numerator;
    double denominator;
    double p_free_udp;

    /* The budget formula below assumes qU > qT. Otherwise there is no reliability cost to
     * preferring UDP, so free slots take it -- with random_probe all but 1/x of them, so that some
     * TCP, and so some feedback, continues. */
    if (qU <= qT)
    {
        return ctl->random_probe ? 1.0 - 1.0 / (double)ctl->x : 1.0;
    }

    /* random_probe: no forced slots, the probe is the 1/x floor applied below. */
    forced_fraction = ctl->random_probe ? 0.0 : 1.0 / (double)ctl->x;
    free_fraction = ctl->random_probe ? 1.0 : 1.0 - 2.0 / (double)ctl->x;

    double L_plan = ctl->L;

    /* Loss debt: repay reported losses above budget over debt_horizon messages, counting the UDP
     * sends not yet reported at the current estimate. */
    if (atc_debt_enabled(ctl))
    {
        int open = ctl->udp_since_last_tcp + ctl->pending_feedback_udp + ctl->carry_sent_udp;
        double debt = ctl->debt + (1.0 - ctl->p_udp_hat) * open;

        if (ctl->debt_per_class)
        {
            int counts[ATC_MAX_DEBT_CLASSES];
            int cls = atc_debt_class(ctl);

            atc_count_udp_classes(ctl, 0, open, counts);
            debt = ctl->debt_by_class[cls] + (1.0 - ctl->p_udp_hat) * counts[cls];
        }

        if (debt > 0)
        {
            L_plan -= debt / ctl->debt_horizon;
            if (L_plan < 0)
            {
                L_plan = 0;
            }
        }
    }

    numerator = ctl->a_L * L_plan - qT - forced_fraction * (qU - qT);
    denominator = free_fraction * (qU - qT) + ctl->eps;

    p_free_udp = numerator / denominator;

    if (ctl->random_probe && p_free_udp < 1.0 / (double)ctl->x)
    {
        return 1.0 / (double)ctl->x;
    }
    if (p_free_udp < 0.0)
    {
        return 0.0;
    }
    if (p_free_udp > 1.0)
    {
        return 1.0;
    }
    return p_free_udp;
}

enum atc_protocol atc_next_protocol(struct adaptive_transport_controller *ctl)
{
    double p_free_udp;
    double sample;

    ctl->total_packets++;

    /* Loss debt: every message earns L of budget (the message's own L in a mixed workload). */
    if (atc_debt_enabled(ctl))
    {
        double *debt = ctl->debt_per_class ? &ctl->debt_by_class[atc_debt_class(ctl)] : &ctl->debt;

        *debt -= ctl->L;
        if (*debt < 0.0)
        {
            *debt = 0.0;
        }
    }

    if (ctl->use_time_estimator)
    {
        atc_time_estimator_tick(ctl);
    }

    /* Always TCP if there is no loss budget to spend on UDP. */
    if (ctl->L <= 0.0)
    {
        /* Advance the cycle like any other slot, wrapping at x: in a mixed workload an L = 0
         * message can land on the forced-feedback slot, which this send then takes the place of. */
        ctl->cycle_counter = (ctl->cycle_counter >= ctl->x) ? 1 : ctl->cycle_counter + 1;
        DEMO_LOG("ATC: -> TCP (L=%.4f <= 0, no loss budget for UDP)\n", ctl->L);
        return ATC_PROTOCOL_TCP;
    }

    /* Startup calibration: one window of UDP, then TCP sends requesting its report until one
     * arrives. The cycle starts afterwards. */
    if (ctl->calibrating)
    {
        if (ctl->calibration_udp_remaining > 0)
        {
            ctl->calibration_udp_remaining--;
            DEMO_LOG("ATC: -> UDP (calibration, %d/%d remaining)\n",
                   ctl->calibration_udp_remaining, ctl->calibration_packets);
            return ATC_PROTOCOL_UDP;
        }

        DEMO_LOG("ATC: -> TCP (calibration complete, requesting feedback for %d UDP sent)\n",
               ctl->udp_since_last_tcp);
        return ATC_PROTOCOL_TCP;
    }

    /* Fixed cycle: forced UDP probe at slot x - 1, forced TCP feedback at slot x. With
     * random_probe slot x is a free slot, and the counter only numbers the log lines. */
    if (ctl->cycle_counter == ctl->x - 1 && !ctl->random_probe)
    {
        ctl->cycle_counter++;
        DEMO_LOG("ATC: -> UDP (forced probe, cycle %d/%d)\n", ctl->cycle_counter - 1, ctl->x);
        return ATC_PROTOCOL_UDP;
    }

    if (ctl->cycle_counter == ctl->x)
    {
        if (!ctl->random_probe)
        {
            ctl->cycle_counter = 1;
            DEMO_LOG("ATC: -> TCP (forced feedback, cycle %d/%d, %d UDP sent since last feedback)\n",
                   ctl->x, ctl->x, ctl->udp_since_last_tcp);
            return ATC_PROTOCOL_TCP;
        }
        ctl->cycle_counter = 0;
    }

    /* Free (probabilistic) slot. */
    p_free_udp = atc_compute_free_udp_probability(ctl);
    sample = ctl->random_fn != NULL ? ctl->random_fn() : atc_default_random();

    ctl->cycle_counter++;
    DEMO_LOG("ATC: -> %s (free slot, cycle %d/%d, p_free_udp=%.4f sample=%.4f, "
           "p_udp_hat=%.4f p_tcp_hat=%.4f a_L=%.4f age_weight=%.3f debt=%.2f)\n",
           (sample < p_free_udp) ? "UDP" : "TCP", ctl->cycle_counter - 1, ctl->x,
           p_free_udp, sample, ctl->p_udp_hat, ctl->p_tcp_hat, ctl->a_L, ctl->age_weight,
           ctl->debt_per_class ? ctl->debt_by_class[atc_debt_class(ctl)] : ctl->debt);
    return (sample < p_free_udp) ? ATC_PROTOCOL_UDP : ATC_PROTOCOL_TCP;
}

int atc_udp_since_last_tcp(const struct adaptive_transport_controller *ctl)
{
    return ctl->udp_since_last_tcp;
}

bool atc_reply_wanted(const struct adaptive_transport_controller *ctl)
{
    return ctl->udp_since_last_tcp > 0;
}

/** Fold one TCP send's outcome into p_tcp_hat. Each send has its own connection, so its outcome is
 *  settled, independent of every other send, when it is reported. */
static void atc_tcp_isolated_update(struct adaptive_transport_controller *ctl, bool success)
{
    double old_p_tcp_hat = ctl->p_tcp_hat;

    atc_set_p_tcp_hat(ctl, atc_weighted_update(ctl->p_tcp_hat, success ? 1 : 0, 1, ctl->N0));
    DEMO_LOG("ATC: tcp isolated update (non-persistent, success=%d), p_tcp_hat %.4f -> %.4f\n",
           (int)success, old_p_tcp_hat, ctl->p_tcp_hat);
}

/**
 * A window dropped without its report. Outside calibration that costs one window of evidence;
 * during calibration it would leave nothing to request a report on, so the calibration burst
 * restarts.
 */
static void atc_feedback_window_lost(struct adaptive_transport_controller *ctl)
{
    if (ctl->calibrating)
    {
        ctl->calibration_udp_remaining = ctl->calibration_packets;
        DEMO_LOG("ATC: calibration feedback lost, restarting calibration burst (%d UDP)\n",
               ctl->calibration_packets);
    }

    /* A window carried over from a late report can only be applied merged with the window that
     * follows it; if that one is lost, the pair's count is unknown. */
    if (ctl->carry_sent_udp > 0)
    {
        DEMO_LOG("ATC: dropping carried-over window of %d UDP sent as unobserved\n",
               ctl->carry_sent_udp);
        ctl->carry_sent_udp = 0;
        ctl->carry_received_udp = 0;
    }
}

/**
 * Apply an in-line report: merged with any window carried over from a late report, or dropped if
 * a given-up late report left its count unreliable. The window closed by an in-line report is
 * always clean -- nothing further is sent until it arrives -- which makes it the point where a
 * carried-over pair's combined count becomes exact.
 */
static void atc_apply_window_feedback(struct adaptive_transport_controller *ctl, int sent_udp,
                                      int received_udp)
{
    if (ctl->drop_next_window)
    {
        ctl->drop_next_window = false;
        DEMO_LOG("ATC: window of %d UDP sent follows a given-up late reply, dropped as "
               "unobserved\n", sent_udp);
        atc_feedback_window_lost(ctl);
        return;
    }

    if (ctl->carry_sent_udp > 0)
    {
        DEMO_LOG("ATC: merging window (%d sent, %d received) with carried-over window (%d sent, "
               "%d received)\n", sent_udp, received_udp, ctl->carry_sent_udp,
               ctl->carry_received_udp);
        sent_udp += ctl->carry_sent_udp;
        received_udp += ctl->carry_received_udp;
        ctl->carry_sent_udp = 0;
        ctl->carry_received_udp = 0;
    }

    atc_apply_feedback(ctl, sent_udp, received_udp);
}

void atc_report_result(struct adaptive_transport_controller *ctl, enum atc_protocol protocol,
                       bool success, bool reply_requested, bool reply_received,
                       int received_udp_count)
{
    if (protocol == ATC_PROTOCOL_UDP)
    {
        ctl->total_udp++;
        /* Counted towards the next request whatever the local outcome: a handed-off send says
         * nothing about delivery. */
        ctl->udp_since_last_tcp++;
        {
            uint32_t slot = ctl->udp_class_count % ATC_DEBT_CLASS_RING;
            unsigned shift = 2 * (slot % 4);

            ctl->udp_class_ring[slot / 4] =
                (uint8_t)((ctl->udp_class_ring[slot / 4] & ~(3u << shift)) |
                          ((unsigned)atc_debt_class(ctl) << shift));
            ctl->udp_class_count++;
        }
        return;
    }

    ctl->total_tcp++;
    if (!success)
    {
        atc_debt_charge_tcp_failure(ctl);
    }

    if (reply_requested && success)
    {
        /* The receiver resets its window when the request arrives, whether or not its report then
         * makes it back, so the window resets here, at send time. If the request itself was lost,
         * the next window's count is understated, the safe direction (see the clamp in
         * atc_apply_feedback()). */
        int sent_udp = ctl->udp_since_last_tcp;
        ctl->udp_since_last_tcp = 0;

        if (reply_received)
        {
            atc_tcp_isolated_update(ctl, true);
            atc_apply_window_feedback(ctl, sent_udp, received_udp_count);
            return;
        }

        /* The report alone was lost: evidence about this send; its window is dropped as
         * unobserved. */
        atc_tcp_isolated_update(ctl, false);
        atc_feedback_window_lost(ctl);
        return;
    }

    /* No report requested (no evidence either way, like a UDP send), or the connect or send
     * failed (the request never reached the receiver, so its window was not reset). */
    atc_tcp_isolated_update(ctl, success);
}

void atc_report_tcp_reply_held(struct adaptive_transport_controller *ctl)
{
    ctl->total_tcp++;
    ctl->pending_feedback_udp = ctl->udp_since_last_tcp;
    ctl->udp_since_last_tcp = 0;
    DEMO_LOG("ATC: reply for %d UDP sent is late, holding it outstanding\n",
           ctl->pending_feedback_udp);
}

void atc_resolve_held_reply(struct adaptive_transport_controller *ctl, bool reply_received,
                            int received_udp_count)
{
    int sent_udp = ctl->pending_feedback_udp;

    if (sent_udp <= 0)
    {
        DEMO_LOG("ATC: held reply resolved with no window outstanding, ignored\n");
        return;
    }
    ctl->pending_feedback_udp = 0;

    atc_tcp_isolated_update(ctl, reply_received);

    if (!reply_received)
    {
        /* Unknown whether the request ever reached the receiver, so also whether it reset its
         * count partway through the next window: that window's count is unreliable too. */
        DEMO_LOG("ATC: gave up on held reply for %d UDP sent, window dropped as unobserved\n",
               sent_udp);
        atc_feedback_window_lost(ctl);
        if (ctl->udp_since_last_tcp > 0)
        {
            ctl->drop_next_window = true;
        }
        return;
    }

    if (ctl->drop_next_window)
    {
        /* This window itself follows a given-up request, so its own count is unreliable -- and,
         * being late as well, it may have absorbed part of the next window's count. */
        ctl->drop_next_window = ctl->udp_since_last_tcp > 0;
        DEMO_LOG("ATC: held window of %d UDP sent follows a given-up late reply, dropped as "
               "unobserved\n", sent_udp);
        atc_feedback_window_lost(ctl);
        return;
    }

    /* The request was late, so the receiver may have counted some of the next window's UDP into
     * this window and missed them from the next; the two windows' combined count is exact, so carry
     * this one over and apply it merged with the next. If no UDP has been sent since the request,
     * nothing can have been miscounted, and it is applied right away (always so in calibration). */
    ctl->carry_sent_udp += sent_udp;
    ctl->carry_received_udp += received_udp_count;
    if (ctl->udp_since_last_tcp == 0)
    {
        int carry_sent = ctl->carry_sent_udp;
        int carry_received = ctl->carry_received_udp;

        ctl->carry_sent_udp = 0;
        ctl->carry_received_udp = 0;
        atc_apply_feedback(ctl, carry_sent, carry_received);
    }
    else
    {
        DEMO_LOG("ATC: late window (%d sent, %d received) carried over to merge with the next\n",
               sent_udp, received_udp_count);
    }
}

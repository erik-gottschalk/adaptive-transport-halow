/*
 * Copyright 2026 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Adaptive UDP/TCP transport controller.
 *
 * For every message the controller chooses UDP with probability
 * p = clamp((L - q_T) / (q_U - q_T), 1/x, 1), so that the expected loss rate stays at the budget
 * @c L while as many messages as the budget allows go as UDP. The floor 1/x is the probe: UDP
 * keeps being sent, and so q_U keeps being measured, even when the budget allows none. Feedback
 * needs no slot of its own: every TCP send asks for a delivery report while UDP is unreported, and
 * the ageing of the estimate (below) turns slots into TCP sends once the last report grows old
 * (random_probe, on by default).
 *
 * Fixed cycle (random_probe = false): a forced UDP probe and a forced TCP feedback send once every
 * @c x messages, and the probability above for the x - 2 other messages of each cycle, with the
 * forced sends taken out of the budget.
 *
 * Before this, the controller runs a one-off calibration: one window of @c calibration_packets
 * UDP sends, then a TCP send whose report sets the initial estimate. If that report is lost, the
 * calibration burst starts again.
 *
 * This module only makes the UDP/TCP decision and keeps the estimates; it does no network I/O.
 * UDP delivery is learned only in aggregate, when the receiver answers a TCP send's request with
 * the number of UDP messages it received since the previous request (see
 * @ref atc_udp_since_last_tcp and @ref atc_report_result).
 *
 * UDP delivery estimate (use_time_estimator, on by default): a time counter with ageing. The lost
 * and received UDP counts of each report are added to two running totals that decay by half every
 * est_half_life *messages* (UDP or TCP alike); q_hat = lost / (lost + received). For the decision
 * the controller plans with q_hat relaxed towards age_target_factor * L as the last report ages
 * (half-way after age_half_life messages), and only in that pessimistic direction, so the plan
 * grows more cautious without fresh feedback and the TCP sends it then chooses request a report.
 *
 * Earlier estimator (use_time_estimator = false), for comparison: p_udp_hat is blended at each
 * report with weight n/(n + N0), i.e. a memory of N0 UDP samples, optionally with
 * use_change_detection (a short ring of recent windows; p_udp_hat snaps to the pooled windowed
 * estimate when it strays too many standard deviations from the current one).
 *
 * Per message:
 *  -# @ref atc_next_protocol decides the transport. For TCP, @ref atc_reply_wanted says whether to
 *     request a report and @ref atc_udp_since_last_tcp is the request byte.
 *  -# After the send, @ref atc_report_result reports the outcome (and a report, if one arrived in
 *     line); a report that missed the in-line wait is reported with
 *     @ref atc_report_tcp_reply_held and later resolved with @ref atc_resolve_held_reply.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of past feedback windows kept for change detection (see use_change_detection). */
#define ATC_CD_WINDOW_SIZE 8

/** Default parameters (the receiver's run settings override them, see remote_config.h). */
#define ATC_DEFAULT_X 50
#define ATC_DEFAULT_L 0.025
#define ATC_DEFAULT_N0 50
#define ATC_DEFAULT_EPS 1e-9
#define ATC_DEFAULT_CALIBRATION_PACKETS 10
#define ATC_DEFAULT_USE_CHANGE_DETECTION true

/** Defaults of the time-counter estimator with ageing. Half-lives are in messages: at one message
 *  per second they equal seconds. */
#define ATC_DEFAULT_USE_TIME_ESTIMATOR true
#define ATC_DEFAULT_EST_HALF_LIFE 20.0
#define ATC_DEFAULT_AGE_HALF_LIFE 20.0
#define ATC_DEFAULT_AGE_TARGET_FACTOR 2.0
/** Loss debt (see debt_horizon): repayment horizon in messages (0: off). */
#define ATC_DEFAULT_DEBT_HORIZON 250.0
/** Loss debt: charge locally failed TCP sends to it too (see debt_tcp_failures). */
#define ATC_DEFAULT_DEBT_TCP_FAILURES false
/** Per-class loss debt (see debt_per_class): number of classes, and how many recent UDP sends'
 *  classes are kept to attribute a window's losses (more than any window plus the UDP sent since
 *  it can span: windows are capped at 255 by the 1-byte request), packed 2 bits each. */
#define ATC_MAX_DEBT_CLASSES 4
#define ATC_DEBT_CLASS_RING 512
#define ATC_DEFAULT_DEBT_PER_CLASS true
/** Probe as a floor 1/x on the UDP probability, no forced slots (see random_probe). */
#define ATC_DEFAULT_RANDOM_PROBE true

/** Transport protocol selected by the controller for a given message. */
enum atc_protocol
{
    /** Send the message as UDP. */
    ATC_PROTOCOL_UDP,
    /** Send the message as TCP. */
    ATC_PROTOCOL_TCP,
};

/**
 * Adaptive transport controller instance. Initialize with @ref atc_init before use; treat the
 * state fields as private (they are visible so that the struct can be allocated by the caller).
 */
struct adaptive_transport_controller
{
    /* Design parameters, set by atc_init(). */
    int x;
    double L;
    int N0;
    double eps;
    bool use_change_detection;

    /* Scale of the loss budget in the plan, 1.0 (the simulator varies it). */
    double a_L;

    /* Estimates of the UDP and TCP delivery probability. p_tcp_hat is kept floored at p_udp_hat
     * (atc_set_p_tcp_hat(), its only setter): the budget formula assumes TCP is never less
     * reliable than UDP, and without the floor a run of local TCP failures could push the estimate
     * below it and latch every message at UDP. */
    double p_udp_hat;
    double p_tcp_hat;

    /* Diagnostics: the most recently applied window's UDP counts (after merging and clamping). */
    int last_window_sent;
    int last_window_received;

    /* Cycle state. udp_since_last_tcp is also the request byte of the next report request. */
    int cycle_counter;
    int udp_since_last_tcp;

    /* A held report (see @ref atc_report_tcp_reply_held): the UDP count of its window, 0 if none
     * is outstanding. */
    int pending_feedback_udp;

    /* Late-report window merging (see @ref atc_resolve_held_reply). A late request can reach the
     * receiver after some of the next window's UDP, which it then counts into the earlier window;
     * only the two windows' combined count is exact. carry_sent_udp / carry_received_udp hold a
     * resolved late window until it can be applied merged with the next (carry_sent_udp == 0: none
     * carried). drop_next_window is set when a late report was given up after UDP of the next
     * window had already been sent: whether the receiver reset partway through that window is then
     * unknown, so its count is dropped as unobserved. */
    int carry_sent_udp;
    int carry_received_udp;
    bool drop_next_window;

    /* Startup calibration. */
    bool calibrating;
    int calibration_packets;
    int calibration_udp_remaining;

    /* Change detection (use_change_detection, earlier estimator only): the last
     * ATC_CD_WINDOW_SIZE windows' (sent, received) UDP counts. */
    int cd_window_sent[ATC_CD_WINDOW_SIZE];
    int cd_window_received[ATC_CD_WINDOW_SIZE];
    int cd_window_index;
    int cd_window_count;

    /* Probe as a floor on the UDP probability (ATC_DEFAULT_RANDOM_PROBE, on): every slot is free,
     * UDP with probability clamp(p_free_udp, 1/x, 1), and the budget formula has no forced terms.
     * Reports come from the TCP sends the slots choose (ageing makes them pick TCP once the last
     * report grows old). False: the fixed cycle, a forced UDP probe at slot x - 1 and a forced TCP
     * feedback send at slot x. */
    bool random_probe;

    /* Time-counter estimator with ageing (on by default). est_lost and est_received are the
     * decayed UDP loss/delivery counts behind q_hat; age_weight is
     * 2^(-messages since the last applied report / age_half_life). While on, p_udp_hat holds the
     * unaged estimate 1 - q_hat (for logs and the p_tcp_hat floor); the ageing is applied in
     * atc_compute_free_udp_probability(). */
    bool use_time_estimator;
    double est_half_life;        /* messages */
    double age_half_life;        /* messages */
    double age_target_factor;    /* planning target without fresh feedback, as a multiple of L */
    double est_lost;
    double est_received;
    double age_weight;

    /* Loss debt (debt_horizon > 0). The plan spends L / q_hat as if q_hat were exact, and since that
     * is convex in q_hat a noisy estimate overspends on a channel steadily above L. The debt is a
     * virtual queue on the delivered loss: it grows by every lost UDP message a report reveals and
     * shrinks by L per message (the message's own L in a mixed workload), floored at 0; while
     * positive -- counting the UDP sent but not yet reported at q_hat -- the plan uses the budget
     * L - debt / debt_horizon. */
    double debt_horizon;
    double debt;

    /* Per-class loss debt (debt_per_class): one account per message class in a mixed workload, so
     * that each class holds its own L rather than only the pooled total. The caller sets debt_class
     * (0 .. ATC_MAX_DEBT_CLASSES - 1) together with L before each atc_next_protocol(). A report only
     * says how many UDP of a window arrived, not which, so a window's losses are split over the
     * classes in proportion to their UDP sends in it, from a ring of the classes of the last
     * ATC_DEBT_CLASS_RING UDP sends. */
    bool debt_per_class;
    /* Charge a TCP send that failed locally (connect or send error) to its class's debt, as one
     * loss: the reports count UDP only, and under bursty loss a run of drops can outlast a
     * connection setup. */
    bool debt_tcp_failures;
    int debt_class;
    double debt_by_class[ATC_MAX_DEBT_CLASSES];
    uint8_t udp_class_ring[ATC_DEBT_CLASS_RING / 4];     /* 2 bits per UDP send */
    uint32_t udp_class_count;

    /* Statistics. */
    uint32_t total_packets;
    uint32_t total_udp;
    uint32_t total_tcp;

    /** Source of pseudo-random values in [0, 1) for the UDP/TCP choice; the hardware generator
     *  (mmhal_random_u32()) by default, replaceable after atc_init() (the simulator does). */
    double (*random_fn)(void);
};

/**
 * Initialize a controller.
 *
 * @param ctl      Controller instance to initialize.
 * @param x        The UDP probability is floored at 1/@p x; with the fixed cycle
 *                  (random_probe = false) the cycle length.
 * @param L        Loss budget.
 * @param N0       Memory of the weighted update, in samples: of p_tcp_hat, and of p_udp_hat with the
 *                  earlier estimator.
 * @param eps      Small value that keeps the budget formula's division finite.
 * @param calibration_packets  Length of the startup calibration burst of UDP sends.
 * @param use_change_detection  Earlier estimator only (see the struct).
 */
void atc_init(struct adaptive_transport_controller *ctl,
             int x, double L, int N0, double eps, int calibration_packets,
             bool use_change_detection);

/**
 * Decide the transport of the next message and advance the controller's counters. Must be
 * followed by @ref atc_report_result (or @ref atc_report_tcp_reply_held) once it is sent.
 */
enum atc_protocol atc_next_protocol(struct adaptive_transport_controller *ctl);

/** The number of UDP messages sent since the last report request: the request byte. */
int atc_udp_since_last_tcp(const struct adaptive_transport_controller *ctl);

/** Whether the next TCP send should request a report: when UDP has been sent since the last
 *  request. The caller additionally asks for none while a held report is outstanding. */
bool atc_reply_wanted(const struct adaptive_transport_controller *ctl);

/**
 * Report the outcome of a send.
 *
 * @param ctl             Controller instance.
 * @param protocol        The protocol used (as returned by @ref atc_next_protocol).
 * @param success         Whether the send itself completed locally. For UDP no ground truth for
 *                        delivery exists; for TCP false means the connect or send failed.
 * @param reply_requested Whether this TCP send asked for a report.
 * @param reply_received  Whether the requested report arrived in line.
 * @param received_udp_count  The report: UDP messages the receiver counted since the previous
 *                        request (may exceed the request after a late report; clamped
 *                        internally). 0 unless a report arrived.
 *
 * The UDP window is reset as soon as a request was sent, whether or not its report arrives: the
 * receiver resets its own count when the request arrives.
 */
void atc_report_result(struct adaptive_transport_controller *ctl, enum atc_protocol protocol,
                       bool success, bool reply_requested, bool reply_received,
                       int received_udp_count);

/**
 * Report a TCP send whose requested report did not arrive in line, but whose socket the caller
 * holds open to collect it later. Use this instead of @ref atc_report_result for such a send, and
 * resolve it later with exactly one @ref atc_resolve_held_reply. A report that misses the in-line
 * wait is almost always late, not lost. While it is outstanding, the caller requests no other
 * report.
 */
void atc_report_tcp_reply_held(struct adaptive_transport_controller *ctl);

/**
 * Resolve a send reported via @ref atc_report_tcp_reply_held: its report arrived, or the caller
 * gave up on it (a TCP failure, and the window dropped as unobserved). Because the request was
 * late, the receiver may have counted some of the next window's UDP into this one, so an arrived
 * report is carried over and applied merged with the next window, unless no UDP has been sent
 * since the request.
 *
 * @param received_udp_count  The report, when @p reply_received; 0 otherwise.
 */
void atc_resolve_held_reply(struct adaptive_transport_controller *ctl, bool reply_received,
                            int received_udp_count);

/**
 * The probability that the next free slot uses UDP, from the current estimates (also used by
 * @ref atc_next_protocol; exposed for logging).
 */
double atc_compute_free_udp_probability(const struct adaptive_transport_controller *ctl);

#ifdef __cplusplus
}
#endif

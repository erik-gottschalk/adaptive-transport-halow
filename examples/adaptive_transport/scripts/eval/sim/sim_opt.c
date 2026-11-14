/* Monte Carlo for the feedback window and the loss estimator.
 *
 * The real controller with the firmware's time model (late replies held and merged), and:
 *  - a hook replacing only the controller's p_udp_hat update (calibration set + N0 blend; build.sh
 *    compiles a patched copy, atc_hook.o) by an alternative estimator. Cycle, p_free, window
 *    merging and p_tcp_hat stay the real code. EST=base reproduces the controller exactly;
 *  - channels: SCHEDULE (a fixed loss schedule) or a random regime-switching channel with fades, either
 *    with Gilbert loss bursts. The channel is drawn per seed from its own stream before the run
 *    (common random numbers: every variant sees the same per-second loss outcomes);
 *  - energy from the measured fresh-connection constants (paper Tables energy_costs, gc_sweep),
 *    and excess loss over EXWIN-second tumbling windows.
 *
 * env: SCHEDULE L MODE (EST=base: 1 = with change detection) SEEDS WARMUP WINDOW TRACE
 *  CHAN=phys: link-margin channel, parameters at gen_phys(); CHDUMP=<file>: write "q loss" per
 *  second and seed, and skip the controller
 *  MSG_DT (phys/markov): seconds between messages; all other times are then in messages
 *  CHAN=sched|markov  DUR (markov/phys capture, s)  DWELL (mean regime dwell, s)  QMIN QMAX (levels,
 *  log-uniform)  FADE_EVERY (mean s between fades, 0 = none)  FADE_MIN FADE_MAX (s)
 *  FADE_QMIN FADE_QMAX  BURST (mean loss-run length in s; <= 1/(1-q) means i.i.d.)
 *  CRN=0: draw losses sequentially (i.i.d. only)
 *  EST=base|asym|time|twin|hmm|oracle|invs  MINV (invs: losses kept)  N0 (base, asym-free)  N0D N0U (asym)  THALF (time, twin
 *  slow; s)  TFAST (twin, s)  TF_Q0 TF_W0 (time/twin prior mean and weight, 0.1 and 0.5)  NU (hmm change rate per s)  TEMPER (hmm likelihood exponent)
 *  HMM_TICK=1 (mix the change prior in every second, not only at feedback)  HM HQMIN HQMAX (grid)
 *  HDIFF (hmm: sd of the ln q random walk per sqrt(s))
 *  Z (plan with posterior mean + Z sd)  VQ_R (loss-debt repayment time in s; 0 = off)  EXWIN
 *  ALPHA (plan with budget ALPHA * L via a_L; the common conservatism knob for frontiers)
 *  EST=fw: the controller's built-in time estimator + ageing (FW_TE, FW_TA half-lives in messages);
 *  overshoot remedies in the controller: FW_DEBT_R (loss-debt horizon, messages)
 *  FW_DEBT_PERCLASS (1: one debt account per class, 0: shared)
 *  FW_DEBT_TCP (1: charge locally failed TCP sends to the debt, the controller's debt_tcp_failures)
 *  TCP_FAIL_C TCP_FAIL_K: a TCP connection fails to open with probability C * p^K (default C = 0:
 *  TCP always delivers). C = 0.75, K = 2 fits the bursty hardware captures (runs of 3): ~3 % at a
 *  steady 20 % (measured 2.4-7.7 %), ~0.25 % over ge_regime_s1 (measured 0.15-0.21 %)
 *  FW_RANDOM_PROBE (default: the controller's, 1): no forced slots, UDP probability floored at
 *  1/x; 0 = the fixed cycle
 *  MIX_L, MIX_SHARE: mixed-criticality workload as in the firmware (comma lists, up to 4
 *  classes): each message draws its class from its own random stream and is planned with that
 *  class's L; MIXOUT=<file> gets one line per seed, "n lost udp" per class
 *  EST=blend: the N0 blend held in the simulator (= base MODE=0, but ageing can be layered on)
 *  AGE_T (s; ageing half-life, 0 = off)  AGE_QL (ageing target as a multiple of L, 2)  AGE_Q0 (absolute)
 * stdout, one line per seed: per phase "n udp lost alarms" | per phase "wn wudp wlost react" |
 *  "energy_mJ excess_msgs nwin nviol fopt_sum feedbacks deficit_msgs e_pure_tcp_mJ nunder sq
 *  e_clair_msg_mJ e_clair_mJ e_clair_tcp_mJ l_clair_msg" (markov: one phase); the clairvoyant bound
 *  -- every message at the true slow loss rate's optimal mix min(1, L/q) (UDP share; TCP lossless),
 *  with no probe and no feedback -- on this variant's own messages (e_clair_msg, against e_pure_tcp)
 *  and on one message per nominal slot (e_clair, against pure TCP on the same slots e_clair_tcp);
 *  l_clair_msg its expected losses on the variant's own messages, sum of min(q, L): the most loss
 *  a controller that never exceeds L at the current q can deliver; base L
 *  FW_PRIOR_Q FW_PRIOR_W (EST=fw): the time estimator's prior mean and weight (0.1, 0.5)
 *  excess = sum over EXWIN windows of (lost - L n)+ ; deficit = sum of (sum f_opt - UDP sent)+
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "adaptive_transport_controller.h"

int sim_cd_resets, sim_floor_hits;
/* The time estimator's prior (the controller's ATC_EST_PRIOR_*, routed here by build.sh). */
double sim_prior_lost = 0.05, sim_prior_weight = 0.5;
static uint64_t rng, rng_ch, rng_mix;
static double next_u(uint64_t *s)
{
    uint64_t z = (*s += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return (double)((z ^ (z >> 31)) >> 11) / 9007199254740992.0;
}
static double urand(void) { return next_u(&rng); }
static double crand(void) { return next_u(&rng_ch); }
uint32_t mmhal_random_u32(uint32_t min, uint32_t max)
{
    return min + (uint32_t)(urand() * ((double)max - min));
}

static double env_d(const char *k, double d) { const char *v = getenv(k); return v ? atof(v) : d; }

/* ---- hardware model per message, fresh connection, interpolated in loss p (linear, extrapolated
 * from the outer segments) ----
 * Built in: the earlier constants (paper Tables energy_costs, gc_sweep), UDP flat, TCP sends
 * taking the 1 s slot plus a 3 s wait when the reply is late, with probability 0.6 p.
 * HW_TABLE=<file> (written by hw_table.py from the baseline captures) replaces all of it, one row
 * per p: "p e_udp e_tcp e_tcpr d_tcp d_tcpr late" -- energy per attempted message [mJ], the mean
 * time a send takes beyond the 1 s interval [s] (d_tcpr including the late-reply waits), and the
 * late-reply probability. '#' starts a comment. */
#define HW_MAX 16
enum { HW_EUDP, HW_ET0, HW_ETR, HW_DT0, HW_DTR, HW_LATE, HW_NCOL };
static int hw_n = 5;
static double hw_p[HW_MAX] = {0.0, 0.05, 0.10, 0.20, 0.40};
static double hw[HW_NCOL][HW_MAX] = {
    {14.22, 14.22, 14.22, 14.22, 14.22},
    {24.56, 28.30, 32.54, 41.82, 60.38},      /* 40 %: the old linear extrapolation from 10-20 % */
    {28.40, 34.92, 44.36, 54.44, 74.60},
    {0, 0, 0, 0, 0},
    {0.0, 0.09, 0.18, 0.36, 0.72},            /* 3 s x 0.6 p */
    {0.0, 0.03, 0.06, 0.12, 0.24},
};

static double hw_at(int col, double p)
{
    int i = 0;
    while (i + 2 < hw_n && p > hw_p[i + 1]) i++;
    if (hw_n == 1) return hw[col][0];
    double v = hw[col][i] + (hw[col][i + 1] - hw[col][i]) * (p - hw_p[i]) / (hw_p[i + 1] - hw_p[i]);
    return v < 0 ? 0 : v;
}

static void hw_load(const char *path)
{
    FILE *f = fopen(path, "r");
    char line[512];
    int n = 0;
    if (!f) { fprintf(stderr, "HW_TABLE: cannot open %s\n", path); exit(2); }
    while (fgets(line, sizeof line, f) && n < HW_MAX)
    {
        char *hash = strchr(line, '#');
        double v[HW_NCOL + 1];
        if (hash) *hash = 0;
        if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                   &v[6]) != 7)
            continue;
        hw_p[n] = v[0];
        for (int c = 0; c < HW_NCOL; c++) hw[c][n] = v[c + 1];
        n++;
    }
    fclose(f);
    if (n < 2) { fprintf(stderr, "HW_TABLE: need at least two rows in %s\n", path); exit(2); }
    hw_n = n;
}

/* ---- estimators behind the hook ---- */
enum { EST_BASE, EST_ASYM, EST_TIME, EST_TWIN, EST_HMM, EST_ORACLE, EST_INVS, EST_BLEND };
#define MMAX 256
static int est;
static double plan_z, true_q, L_cfg;
static double asym_n0d, asym_n0u;
static double tf_a, tf_b, tf_g, tf2_a, tf2_b, tf2_g;      /* lost / received pseudo-counts, decay */
static double TF_A0 = 0.05, TF_B0 = 0.45;                   /* prior pseudo-counts (TF_Q0, TF_W0) */
static int hm_m, hm_tick;
static double hm_q[MMAX], hm_lq[MMAX], hm_l1q[MMAX], hm_w[MMAX], hm_nu, hm_temper, hm_dt;
static double hm_diff, hm_step;                              /* ln q random walk, sd per sqrt(s) */
static double vq_r, vq_q;                                    /* loss debt (virtual queue), msgs */
/* Inverse sampling: q_hat = M / (reported UDP samples since the M-th most recent loss). Loss
 * positions inside a window are unknown and spread evenly; iv_pos is a ring of the last M. */
#define INV_MAX 64
static int iv_m, iv_head;
static double iv_pos[INV_MAX], iv_s, iv_q0;
/* Hybrids on inverse sampling (2026-10-05, overshoot study):
 *  HYB_Z > 0: plan with max(inverse q_hat, fast time counter's mean - HYB_Z sd) -- the fast counter
 *    (THALF, prior TF_Q0/TF_W0) takes over only when it is significantly worse.
 *  CS_H > 0: one-sided CUSUM over the feedback windows against the current q_hat (q1 = CS_RATIO
 *    q0); on an alarm the estimator forgets every loss before the change point (iv_floor) and
 *    estimates from the segment since, until M losses are newer than it. CS_UP=1 adds the
 *    improvement side (q1 = q0 / CS_RATIO). CS_MIN: UDP samples in the segment before acting. */
static double hy_z, iv_floor, cs_h, cs_ratio, cs_s, cs_su, cs_start, cs_startu;
static int cs_up, cs_min;
static double alpha;                                         /* plan with budget alpha * L */
/* EST=blend: the controller's N0 blend kept in the simulator, so that ageing can sit on top of it
 * without feeding the aged plan back into the estimate. */
static double bl_p, bl_n0;
/* Ageing (AGE_T > 0, any in-sim estimator): plan with the estimate relaxed towards q_age as the
 * last feedback gets older, pessimistic side only:
 *   q_plan = q_age - (q_age - q_hat) 2^(-age / AGE_T)   if q_hat < q_age, else q_hat,
 * q_age = AGE_Q0 if set, else AGE_QL * L. g_t is the simulation time, t_fb the last feedback. */
static double age_t, age_q0, age_ql, g_t, t_fb;

/* Prior dynamics over dt seconds: a random walk in ln q (3-tap diffusion on the log grid, reflecting
 * ends), then a jump to a fresh log-uniform level with probability 1 - exp(-NU dt). */
static void hm_mix(double dt)
{
    double h = 1.0 - exp(-hm_nu * dt);
    if (hm_diff > 0 && dt > 0)
    {
        double v = hm_diff * hm_diff * dt / (hm_step * hm_step);
        int reps = (int)ceil(v / 0.5);
        double a = v / reps / 2.0, tmp[MMAX];
        for (int r = 0; r < reps; r++)
        {
            for (int i = 0; i < hm_m; i++)
            {
                double lo = hm_w[i > 0 ? i - 1 : 0], hi = hm_w[i < hm_m - 1 ? i + 1 : hm_m - 1];
                tmp[i] = (1.0 - 2.0 * a) * hm_w[i] + a * (lo + hi);
            }
            memcpy(hm_w, tmp, hm_m * sizeof(double));
        }
    }
    for (int i = 0; i < hm_m; i++) hm_w[i] = (1.0 - h) * hm_w[i] + h / hm_m;
}

static void est_reset(void)
{
    tf_a = tf_b = tf2_a = tf2_b = 0.0;
    vq_q = 0.0;
    hm_dt = 0.0;
    for (int i = 0; i < hm_m; i++) hm_w[i] = 1.0 / hm_m;
    iv_s = 0.0;
    iv_head = 0;
    for (int i = 0; i < iv_m; i++) iv_pos[i] = -(iv_m - i) / iv_q0;   /* prior: q0 */
    iv_floor = -1e300;
    cs_s = cs_su = 0.0;
    cs_start = cs_startu = 0.0;
    bl_p = 0.5;
    t_fb = g_t;
}

static void est_moments(double *m, double *sd);

/* CUSUM step over one window (k lost of n) against the current q_hat; returns the change point
 * (cumulative UDP sample index) on an alarm, or NAN. *s: statistic, *start: segment start. */
static double cs_step(double *s, double *start, double q1_factor, int k, int n)
{
    double q0, sd, q1;
    est_moments(&q0, &sd);
    if (q0 < 0.005) q0 = 0.005;
    q1 = q0 * q1_factor;
    if (q1 > 0.95) q1 = 0.95;
    if (q1 < 0.001) q1 = 0.001;
    double llr = k * log(q1 / q0) + (n - k) * log((1.0 - q1) / (1.0 - q0));
    if (*s <= 0.0 && *s + llr > 0.0) *start = iv_s;          /* segment opens before this window */
    *s = *s + llr > 0.0 ? *s + llr : 0.0;
    if (*s > cs_h && iv_s + n - *start >= cs_min)
    {
        *s = 0.0;
        return *start;
    }
    return NAN;
}

static void est_observe(int r, int n)
{
    int k = n - r;
    if (est == EST_INVS && cs_h > 0 && n > 0)
    {
        double cp = cs_step(&cs_s, &cs_start, cs_ratio, k, n);
        if (cs_up && isnan(cp)) cp = cs_step(&cs_su, &cs_startu, 1.0 / cs_ratio, k, n);
        if (!isnan(cp))
        {
            iv_floor = cp;
            cs_s = cs_su = 0.0;
            sim_cd_resets++;
        }
    }
    if (est == EST_INVS)
    {
        for (int j = 1; j <= k; j++)
        {
            iv_pos[iv_head] = iv_s + (double)n * j / (k + 1);
            iv_head = (iv_head + 1) % iv_m;
        }
        iv_s += n;
    }
    tf_a += k; tf_b += r;
    tf2_a += k; tf2_b += r;
    if (est == EST_HMM)
    {
        double mx = -1e300, s = 0.0, ll[MMAX];
        if (!hm_tick) { hm_mix(hm_dt); hm_dt = 0.0; }
        for (int i = 0; i < hm_m; i++)
        {
            ll[i] = hm_temper * (k * hm_lq[i] + r * hm_l1q[i]);
            if (ll[i] > mx) mx = ll[i];
        }
        for (int i = 0; i < hm_m; i++) s += (hm_w[i] *= exp(ll[i] - mx));
        for (int i = 0; i < hm_m; i++) hm_w[i] /= s;
    }
}

/* Posterior mean and sd of the UDP loss probability q. */
static void est_moments(double *m, double *sd)
{
    double v = 0.0;
    if (est == EST_TIME || est == EST_TWIN)
    {
        double A = tf_a + TF_A0, B = tf_b + TF_B0;
        *m = A / (A + B);
        v = *m * (1.0 - *m) / (A + B + 1.0);
        if (est == EST_TWIN)
        {
            double A2 = tf2_a + TF_A0, B2 = tf2_b + TF_B0, m2 = A2 / (A2 + B2);
            if (m2 > *m) { *m = m2; v = m2 * (1.0 - m2) / (A2 + B2 + 1.0); }
        }
    }
    else if (est == EST_INVS)
    {
        double T = iv_s - iv_pos[iv_head];          /* oldest of the last M losses */
        double M = iv_m;
        if (iv_floor > iv_pos[iv_head])             /* after a change point: the segment only */
        {
            int newer = 0;
            for (int i = 0; i < iv_m; i++) newer += iv_pos[i] >= iv_floor;
            T = iv_s - iv_floor;
            M = newer > 0 ? newer : 0.5;
        }
        *m = M / (T > M ? T : M);
        v = *m * *m * (1.0 - *m) / M;               /* CV^2 of 1/q_hat is (1-q)/M */
        if (hy_z > 0)
        {
            double A = tf_a + TF_A0, B = tf_b + TF_B0, mf = A / (A + B);
            double lcb = mf - hy_z * sqrt(mf * (1.0 - mf) / (A + B + 1.0));
            if (lcb > *m) { *m = lcb; v = mf * (1.0 - mf) / (A + B + 1.0); }
        }
    }
    else if (est == EST_HMM)
    {
        double s1 = 0.0, s2 = 0.0;
        for (int i = 0; i < hm_m; i++) { s1 += hm_w[i] * hm_q[i]; s2 += hm_w[i] * hm_q[i] * hm_q[i]; }
        *m = s1;
        v = s2 - s1 * s1;
    }
    else if (est == EST_BLEND)
    {
        *m = 1.0 - bl_p;
        v = *m * (1.0 - *m) / (bl_n0 + 1.0);
    }
    else
    {
        *m = true_q;
    }
    *sd = v > 0 ? sqrt(v) : 0.0;
}

static double est_plan_p(void)
{
    double m, sd, q;
    est_moments(&m, &sd);
    q = m + plan_z * sd;
    if (age_t > 0)
    {
        double qa = age_q0 > 0 ? age_q0 : age_ql * L_cfg;
        if (q < qa) q = qa - (qa - q) * pow(2.0, -(g_t - t_fb) / age_t);
    }
    return 1.0 - (q < 0 ? 0 : q > 1 ? 1 : q);
}

double sim_udp_calib(struct adaptive_transport_controller *ctl, int received, int sent)
{
    (void)ctl;
    t_fb = g_t;
    if (est == EST_BASE || est == EST_ASYM) return (double)received / (double)sent;
    if (est == EST_BLEND) bl_p = (double)received / (double)sent;
    else est_observe(received, sent);
    return est_plan_p();
}

double sim_udp_blend(struct adaptive_transport_controller *ctl, int received, int sent)
{
    double p = ctl->p_udp_hat, w;
    int n0 = ctl->N0;
    if (sent > 0)
    {
        vq_q += sent - received;
        if (vq_q < 0) vq_q = 0;
        t_fb = g_t;
    }
    switch (est)
    {
    case EST_ASYM:
        if (sent > 0) n0 = (double)received / sent < p ? (int)asym_n0d : (int)asym_n0u;
        /* fall through */
    case EST_BASE:      /* atc_weighted_update(), verbatim */
        if (sent == 0) return p;
        w = (double)sent / (double)(sent + n0);
        return (1.0 - w) * p + w * ((double)received / (double)sent);
    case EST_BLEND:
        if (sent > 0)
        {
            w = (double)sent / (double)(sent + bl_n0);
            bl_p = (1.0 - w) * bl_p + w * ((double)received / (double)sent);
        }
        return est_plan_p();
    default:
        if (sent > 0) est_observe(received, sent);
        return est_plan_p();
    }
}

/* Once per message, before the decision: time passage, then the plan the controller acts on. */
static void est_tick(struct adaptive_transport_controller *c, double dt)
{
    tf_a *= pow(tf_g, dt); tf_b *= pow(tf_g, dt);
    tf2_a *= pow(tf2_g, dt); tf2_b *= pow(tf2_g, dt);
    if (est == EST_HMM) { if (hm_tick) hm_mix(dt); else hm_dt += dt; }
    if (est != EST_BASE && est != EST_ASYM && !c->calibrating) c->p_udp_hat = est_plan_p();

    vq_q -= L_cfg;                 /* each message earns L of budget; losses are added at feedback */
    c->a_L = alpha;
    if (vq_r > 0)
    {
        int open = c->udp_since_last_tcp + c->pending_feedback_udp + c->carry_sent_udp;
        double debt = vq_q + (1.0 - c->p_udp_hat) * open;
        double l_eff = L_cfg - (debt > 0 ? debt : 0) / vq_r;
        c->a_L = alpha * (l_eff > 0 ? l_eff : 0) / L_cfg;
    }
}

/* ---- channel ---- */
#define MAXPH 2048
static double *ch_q;
static unsigned char *ch_loss;

static void fill(int a, int b, double q, int n)
{
    for (int i = a < 0 ? 0 : a; i < b && i < n; i++) ch_q[i] = q;
}

/* Physical channel (CHAN=phys): per-second link margin in dB,
 *   x(t) = M0 + S(t) - I(t) - O(t)        (slow: what a tracker can follow)
 *   y(t) = x(t) + F(t) - B(t)             (instantaneous)
 * S: log-normal shadowing, Gauss-Markov (SIG_S dB, correlation time TAU_S s);
 * I: interference episodes, on/off Markov (mean gap INT_EVERY, mean length INT_MEAN s), each with an
 *    SINR loss drawn from U[0.5, 1.5] INT_DB;
 * O: obstructions, Poisson (mean gap FADE_EVERY s), length U[FADE_MIN, FADE_MAX], loss U[0.5,1.5] OBS_DB;
 * F: Rician fading (K_RICE, linear) in dB, complex Gauss-Markov with correlation time TAU_C s;
 * B: short interference bursts, on/off Markov (mean gap SB_EVERY, mean length SB_MEAN s), SB_DB dB.
 * A message is lost with PER(y) = 1 / (1 + exp(y / W_PER)) -- the post-MAC-retry waterfall.
 * ch_q (the true slow loss rate, used as f_opt's q) = E[PER | x], averaging F and B. */
static double ph_m0, ph_sig_s, ph_tau_s, ph_k, ph_tau_c, ph_w;
static double sb_every, sb_mean, sb_db, int_every, int_mean, int_db, obs_every, obs_min, obs_max, obs_db;
#define GX_MIN (-40.0)
#define GX_STEP 0.05
#define GX_N 2401
static double gx_tab[GX_N];

static double per(double y) { return 1.0 / (1.0 + exp(y / ph_w)); }
static double gauss(void)
{
    double u = crand(), v = crand();
    return sqrt(-2.0 * log(1.0 - u)) * cos(2.0 * M_PI * v);
}
static double gx(double x)          /* E_F[PER(x + F)] */
{
    double f = (x - GX_MIN) / GX_STEP;
    int i = (int)floor(f);
    if (i < 0) return gx_tab[0];
    if (i >= GX_N - 1) return gx_tab[GX_N - 1];
    return gx_tab[i] + (gx_tab[i + 1] - gx_tab[i]) * (f - i);
}

static void phys_init(void)
{
    enum { NB = 1600, NR = 400, NP = 128 };
    static double hist[NB];
    const double fmin = -70.0, fstep = 0.05;
    double sig, los, tot = 0;

    ph_m0 = env_d("M0", 14); ph_sig_s = env_d("SIG_S", 5); ph_tau_s = env_d("TAU_S", 600);
    ph_k = env_d("K_RICE", 4); ph_tau_c = env_d("TAU_C", 1.5); ph_w = env_d("W_PER", 0.5);
    sb_every = env_d("SB_EVERY", 120); sb_mean = env_d("SB_MEAN", 3); sb_db = env_d("SB_DB", 15);
    int_every = env_d("INT_EVERY", 1200); int_mean = env_d("INT_MEAN", 180); int_db = env_d("INT_DB", 8);
    obs_every = env_d("FADE_EVERY", 1800); obs_min = env_d("FADE_MIN", 20);
    obs_max = env_d("FADE_MAX", 120); obs_db = env_d("OBS_DB", 15);

    /* distribution of F = 10 log10 |h|^2, h = los + n, n ~ CN(0, 1/(K+1)), on a polar grid */
    sig = sqrt(0.5 / (ph_k + 1.0));
    los = sqrt(ph_k / (ph_k + 1.0));
    for (int ir = 0; ir < NR; ir++)
    {
        double dr = 7.0 * sig / NR, r = (ir + 0.5) * dr;
        double wr = r / (sig * sig) * exp(-r * r / (2 * sig * sig)) * dr;
        for (int ip = 0; ip < NP; ip++)
        {
            double phi = 2 * M_PI * (ip + 0.5) / NP;
            double h2 = los * los + r * r + 2 * los * r * cos(phi);
            int b = (int)((10.0 * log10(h2 > 1e-12 ? h2 : 1e-12) - fmin) / fstep);
            if (b < 0) b = 0;
            if (b >= NB) b = NB - 1;
            hist[b] += wr / NP;
            tot += wr / NP;
        }
    }
    for (int i = 0; i < GX_N; i++)
    {
        double x = GX_MIN + i * GX_STEP, g = 0;
        for (int b = 0; b < NB; b++)
            if (hist[b] > 0) g += hist[b] * per(x + fmin + (b + 0.5) * fstep);
        gx_tab[i] = g / tot;
    }
}

static void gen_phys(int n)
{
    double a_s = exp(-1.0 / ph_tau_s), rho = ph_tau_c > 0 ? exp(-1.0 / ph_tau_c) : 0.0;
    double sig = sqrt(0.5 / (ph_k + 1.0)), los = sqrt(ph_k / (ph_k + 1.0));
    double S = ph_sig_s * gauss(), nr = sig * gauss(), ni = sig * gauss();
    double pi_sb = sb_every > 0 ? sb_mean / (sb_every + sb_mean) : 0.0;
    double int_att = 0, obs_att = 0;
    int sb_on = 0, int_on = 0, obs_left = 0;

    for (int i = 0; i < n; i++)
    {
        S = a_s * S + sqrt(1 - a_s * a_s) * ph_sig_s * gauss();
        nr = rho * nr + sqrt(1 - rho * rho) * sig * gauss();
        ni = rho * ni + sqrt(1 - rho * rho) * sig * gauss();
        if (sb_every > 0) sb_on = sb_on ? crand() >= 1.0 / sb_mean : crand() < 1.0 / sb_every;
        if (int_every > 0)
        {
            if (int_on) int_on = crand() >= 1.0 / int_mean;
            else if (crand() < 1.0 / int_every) { int_on = 1; int_att = int_db * (0.5 + crand()); }
        }
        if (obs_left > 0) obs_left--;
        else if (obs_every > 0 && crand() < 1.0 / obs_every)
        {
            obs_left = (int)(obs_min + crand() * (obs_max - obs_min));
            obs_att = obs_db * (0.5 + crand());
        }
        double x = ph_m0 + S - (int_on ? int_att : 0) - (obs_left > 0 ? obs_att : 0);
        double h2 = (los + nr) * (los + nr) + ni * ni;
        double y = x + 10.0 * log10(h2 > 1e-12 ? h2 : 1e-12) - (sb_on ? sb_db : 0);
        ch_q[i] = (1 - pi_sb) * gx(x) + pi_sb * gx(x - sb_db);
        ch_loss[i] = crand() < per(y);
    }
}

int main(void)
{
    double dur[MAXPH], pl[MAXPH], start[MAXPH];
    int nph = 0;
    char sched[16384];
    const char *chan = getenv("CHAN") ? getenv("CHAN") : "sched";
    const char *est_s = getenv("EST") ? getenv("EST") : "base";
    int markov = !strcmp(chan, "markov"), phys = !strcmp(chan, "phys");
    FILE *chdump = getenv("CHDUMP") ? fopen(getenv("CHDUMP"), "w") : NULL;
    double t_end = 0;

    if (phys) phys_init();
    if (getenv("HW_TABLE")) hw_load(getenv("HW_TABLE"));
    if (markov || phys)
    {
        dur[0] = env_d("DUR", 4 * 3600);
        pl[0] = NAN;
        nph = 1;
    }
    else
    {
        const char *src = getenv("SCHEDULE") ? getenv("SCHEDULE") : "300:0.02";
        /* Refuse rather than truncate: a schedule cut short silently simulates a shorter run. */
        if (strlen(src) >= sizeof sched)
        {
            fprintf(stderr, "SCHEDULE longer than %zu characters\n", sizeof sched - 1);
            return 2;
        }
        strcpy(sched, src);
        for (char *tok = strtok(sched, ","); tok; tok = strtok(NULL, ","))
        {
            if (nph == MAXPH)
            {
                fprintf(stderr, "SCHEDULE has more than %d phases\n", MAXPH);
                return 2;
            }
            sscanf(tok, "%lf:%lf", &dur[nph], &pl[nph]);
            nph++;
        }
    }
    for (int i = 0; i < nph; i++) { start[i] = t_end; t_end += dur[i]; }

    double L = env_d("L", 0.05), warmup = env_d("WARMUP", 300), window = env_d("WINDOW", 120);
    int mode = (int)env_d("MODE", 0), seeds = (int)env_d("SEEDS", 200), crn = (int)env_d("CRN", 1);
    int seed0 = (int)env_d("SEED0", 0);   /* offset of the seed numbers: fresh realizations, same scenario */
    double burst = env_d("BURST", 0), exwin = env_d("EXWIN", 300);
    double tcp_fail_c = env_d("TCP_FAIL_C", 0), tcp_fail_k = env_d("TCP_FAIL_K", 2);
    double dwell = env_d("DWELL", 900), qmin = env_d("QMIN", 0.005), qmax = env_d("QMAX", 0.40);
    double fade_every = env_d("FADE_EVERY", 0), fade_min = env_d("FADE_MIN", 20);
    double fade_max = env_d("FADE_MAX", 120), fqmin = env_d("FADE_QMIN", 0.4);
    double fqmax = env_d("FADE_QMAX", 0.7);

    L_cfg = L;
    est = !strcmp(est_s, "asym") ? EST_ASYM : !strcmp(est_s, "time") ? EST_TIME
        : !strcmp(est_s, "twin") ? EST_TWIN : !strcmp(est_s, "hmm") ? EST_HMM
        : !strcmp(est_s, "oracle") ? EST_ORACLE : !strcmp(est_s, "invs") ? EST_INVS
        : !strcmp(est_s, "blend") ? EST_BLEND : EST_BASE;
    int fw_est = !strcmp(est_s, "fw");
    bl_n0 = env_d("N0", 50);
    age_t = env_d("AGE_T", 0); age_q0 = env_d("AGE_Q0", 0); age_ql = env_d("AGE_QL", 2.0);
    iv_m = (int)env_d("MINV", 4);
    if (iv_m > INV_MAX) iv_m = INV_MAX;
    if (iv_m < 1) iv_m = 1;
    iv_q0 = env_d("INV_Q0", 0.05);
    hy_z = env_d("HYB_Z", 0);
    cs_h = env_d("CS_H", 0); cs_ratio = env_d("CS_RATIO", 2.0);
    cs_up = (int)env_d("CS_UP", 0); cs_min = (int)env_d("CS_MIN", 10);
    plan_z = env_d("Z", 0);
    asym_n0d = env_d("N0D", 10); asym_n0u = env_d("N0U", 50);
    tf_g = pow(2.0, -1.0 / env_d("THALF", 120)); tf2_g = pow(2.0, -1.0 / env_d("TFAST", 20));
    TF_A0 = env_d("TF_Q0", 0.1) * env_d("TF_W0", 0.5);
    TF_B0 = (1.0 - env_d("TF_Q0", 0.1)) * env_d("TF_W0", 0.5);
    hm_nu = env_d("NU", 1.0 / 600); hm_temper = env_d("TEMPER", 1.0); hm_diff = env_d("HDIFF", 0);
    hm_tick = (int)env_d("HMM_TICK", 0);
    hm_m = (int)env_d("HM", 64);
    if (hm_m > MMAX) hm_m = MMAX;
    {
        double a = log(env_d("HQMIN", 0.002)), b = log(env_d("HQMAX", 0.9));
        hm_step = (b - a) / (hm_m - 1);
        for (int i = 0; i < hm_m; i++)
        {
            hm_q[i] = exp(a + (b - a) * i / (hm_m - 1));
            hm_lq[i] = log(hm_q[i]);
            hm_l1q[i] = log1p(-hm_q[i]);
        }
    }
    vq_r = env_d("VQ_R", 0);
    alpha = env_d("ALPHA", 1.0);

    /* MSG_DT: seconds between messages. The phys/markov channel still evolves per second; the
     * simulation clock t (and DUR, WARMUP, EXWIN, THALF, AGE_T, VQ_R) then counts message slots. */
    double msg_dt = (markov || phys) ? env_d("MSG_DT", 1) : 1.0;
    int nsec = (int)ceil((warmup + t_end + 16) * msg_dt) + 16;
    ch_q = malloc(nsec * sizeof *ch_q);
    ch_loss = malloc(nsec);
    int nex = (int)ceil(t_end / exwin) + 1;
    int *ex_n = malloc(nex * sizeof(int)), *ex_l = malloc(nex * sizeof(int));
    int *ex_u = malloc(nex * sizeof(int));
    double *ex_f = malloc(nex * sizeof(double));

    double mix_L[4], mix_cum[4];
    int mix_n = 0;
    if (getenv("MIX_L") && getenv("MIX_SHARE"))
    {
        char a[128], b[128];
        double sh[4], tot = 0;
        int ns = 0;
        strncpy(a, getenv("MIX_L"), sizeof a - 1); a[sizeof a - 1] = 0;
        strncpy(b, getenv("MIX_SHARE"), sizeof b - 1); b[sizeof b - 1] = 0;
        for (char *tok = strtok(a, ","); tok && mix_n < 4; tok = strtok(NULL, ",")) mix_L[mix_n++] = atof(tok);
        for (char *tok = strtok(b, ","); tok && ns < 4; tok = strtok(NULL, ",")) { sh[ns] = atof(tok); tot += sh[ns++]; }
        if (ns != mix_n) { fprintf(stderr, "MIX_L and MIX_SHARE differ in length\n"); return 2; }
        for (int k = 0; k < mix_n; k++) { mix_cum[k] = (k ? mix_cum[k - 1] : 0) + sh[k] / tot; }
    }
    FILE *mixout = getenv("MIXOUT") ? fopen(getenv("MIXOUT"), "w") : NULL;

    const char *trace = getenv("TRACE");
    /* PKTDUMP=<file>: one line per message inside the capture, for the first PKTSEEDS seeds:
     * "seed t udp delivered reply" (t in message-time units, see MSG_DT). */
    FILE *pktdump = getenv("PKTDUMP") ? fopen(getenv("PKTDUMP"), "w") : NULL;
    int pktseeds = (int)env_d("PKTSEEDS", 1);
    int nbins = (int)(t_end / 5.0) + 1;
    double *b_est = calloc(nbins, sizeof(double)), *b_n = calloc(nbins, sizeof(double));
    double *b_udp = calloc(nbins, sizeof(double)), *b_lost = calloc(nbins, sizeof(double));
    double *b_raw = calloc(nbins, sizeof(double)), *b_rawn = calloc(nbins, sizeof(double));
    double *b_q = calloc(nbins, sizeof(double));

    for (int s = 0; s < seeds; s++)
    {
        struct adaptive_transport_controller c;
        int n[MAXPH] = {0}, udp[MAXPH] = {0}, lost[MAXPH] = {0}, alarms[MAXPH] = {0};
        int wn[MAXPH] = {0}, wudp[MAXPH] = {0}, wlost[MAXPH] = {0}, feedbacks = 0;
        double react[MAXPH], energy = 0, fopt_sum = 0, e_tcp = 0, e_clair = 0, e_clair_tcp = 0;
        double e_clair_msg = 0, l_clair_msg = 0;
        for (int i = 0; i < nph; i++) react[i] = NAN;
        memset(ex_n, 0, nex * sizeof(int));
        memset(ex_l, 0, nex * sizeof(int));
        memset(ex_u, 0, nex * sizeof(int));
        memset(ex_f, 0, nex * sizeof(double));

        /* channel for this seed, index i <-> t = i - warmup */
        rng_ch = 0xBADC0DEULL + (uint64_t)(s + seed0) * 7919ULL;
        if (phys)
        {
            gen_phys(nsec);
        }
        else if (markov)
        {
            for (double t0 = 0; t0 < nsec;)
            {
                double d = -dwell * log(1.0 - crand());
                fill((int)t0, (int)(t0 + d) + 1, exp(log(qmin) + crand() * log(qmax / qmin)), nsec);
                t0 = floor(t0 + d) + 1;
            }
            if (fade_every > 0)
                for (double t0 = -fade_every * log(1.0 - crand()); t0 < nsec;)
                {
                    double d = fade_min + crand() * (fade_max - fade_min);
                    fill((int)t0, (int)(t0 + d), fqmin + crand() * (fqmax - fqmin), nsec);
                    t0 += d - fade_every * log(1.0 - crand());
                }
        }
        else
        {
            for (int i = 0; i < nsec; i++)
            {
                double t = i - warmup;
                int ph = 0;
                while (ph + 1 < nph && t >= start[ph + 1]) ph++;
                ch_q[i] = t < 0 ? pl[0] : pl[ph];
            }
        }
        if (!phys)
        {
            int st = 0;
            for (int i = 0; i < nsec; i++)
            {
                double q = ch_q[i], u = crand();
                if (burst <= 1.0 / (1.0 - q) + 1e-9) st = u < q;            /* i.i.d. */
                else st = st ? !(u < 1.0 / burst) : u < q / ((1.0 - q) * burst);
                ch_loss[i] = (unsigned char)st;
            }
        }
        if (chdump)
        {
            for (int i = 0; i < nsec; i++) fprintf(chdump, "%.6f %d\n", ch_q[i], ch_loss[i]);
            continue;
        }

        rng = 0xC0FFEEULL + (uint64_t)(s + seed0) * 104729ULL;
        rng_mix = 0x5EEDULL + (uint64_t)(s + seed0) * 15485863ULL;
        int mx_n[4] = {0}, mx_lost[4] = {0}, mx_udp[4] = {0}, cls = 0;
        sim_cd_resets = 0;
        atc_init(&c, 50, L, (int)env_d("N0", 50), 1e-9, 10, mode == 1);
        /* EST=fw: the controller's own time estimator + ageing; every other EST uses the legacy
         * blend path, which the hook replaces. */
        c.use_time_estimator = fw_est;
        c.est_half_life = env_d("FW_TE", c.est_half_life);
        c.age_half_life = env_d("FW_TA", c.age_half_life);
        c.random_fn = urand;
        c.random_probe = env_d("FW_RANDOM_PROBE", c.random_probe) != 0;
        /* Overshoot remedies in the controller itself (see its header), off unless set. */
        c.debt_horizon = env_d("FW_DEBT_R", 0);
        c.debt_per_class = env_d("FW_DEBT_PERCLASS", 1) != 0;
        c.debt_tcp_failures = env_d("FW_DEBT_TCP", 0) != 0;
        sim_prior_weight = env_d("FW_PRIOR_W", 0.5);
        sim_prior_lost = env_d("FW_PRIOR_Q", 0.1) * sim_prior_weight;
        est_reset();

        /* The server: win_recv, the UDP received since its last reply. */
        int win_sent = 0, win_recv = 0;
        int held_left = -1, held_recv = 0, held_never = 0;
        double dt = 0.0;

        for (double t = -warmup; t < t_end;)
        {
            int ph = 0, sec = (int)((t + warmup) * msg_dt);
            while (ph + 1 < nph && t >= start[ph + 1]) ph++;
            double p = ch_q[sec];
            true_q = p;

            if (held_left >= 0 && --held_left < 0)
            {
                atc_resolve_held_reply(&c, !held_never, held_recv);
            }

            g_t = t;
            est_tick(&c, dt);
            int resets_before = sim_cd_resets;
            if (mix_n > 0)
            {
                double u = next_u(&rng_mix);
                cls = 0;
                while (cls < mix_n - 1 && u >= mix_cum[cls]) cls++;
                c.L = mix_L[cls];
                c.debt_class = cls;
            }
            enum atc_protocol proto = atc_next_protocol(&c);
            int is_udp = proto == ATC_PROTOCOL_UDP, delivered = 1, want = 0;
            dt = 1.0;

            if (is_udp)
            {
                delivered = crn ? !ch_loss[sec] : urand() >= p;
                win_sent++;
                win_recv += delivered;
                atc_report_result(&c, proto, true, false, false, 0);
            }
            else if (tcp_fail_c > 0 && urand() < tcp_fail_c * pow(p, tcp_fail_k))
            {
                /* TCP_FAIL_C: the connection fails to open (hardware under bursts: a run of drops
                 * outlasts the setup's retries). Lost; the request never reached the server, so no
                 * reply and the UDP window carries over. No extra draw when TCP_FAIL_C = 0. */
                delivered = 0;
                double block = hw_at(HW_DT0, p);
                if (block > 1e-9) dt += block / msg_dt;
                atc_report_result(&c, proto, false, atc_reply_wanted(&c), false, 0);
            }
            else
            {
                want = atc_reply_wanted(&c) && held_left < 0;
                int reply = win_recv;
                int late = want && urand() < hw_at(HW_LATE, p);
                /* Time beyond the slot: a send's own blocking time, the reply-carrying one's net of
                 * its late-reply waits (added below when this one is late). Zero in the built-in
                 * model, so it reproduces the earlier runs exactly. */
                double block = want ? hw_at(HW_DTR, p) - 3.0 * hw_at(HW_LATE, p) : hw_at(HW_DT0, p);
                if (block > 1e-9) dt += block / msg_dt;   /* built-in model: 0 up to rounding */
                if (late)
                {
                    dt += 3.0 / msg_dt;
                    atc_report_tcp_reply_held(&c);
                    held_left = 1 + (int)(urand() * 8);
                    if (msg_dt >= 8) held_left = 1;     /* a late reply (median 5.5 s) beats the next message */
                    held_recv = reply;
                    held_never = urand() < 0.003;
                }
                else
                {
                    atc_report_result(&c, proto, true, want, want, want ? reply : 0);
                }
                if (want) { win_sent = 0; win_recv = 0; }
            }

            if (t >= 0)
            {
                int b = (int)(t / 5.0), w = (int)(t / exwin);
                if (mix_n > 0) { mx_n[cls]++; mx_lost[cls] += !delivered; mx_udp[cls] += is_udp; }
                if (pktdump && s < pktseeds)
                    fprintf(pktdump, "%d %.2f %d %d %d\n", s, t, is_udp, delivered, want);
                b_est[b] += c.p_udp_hat; b_n[b] += 1; b_udp[b] += is_udp; b_lost[b] += !delivered;
                b_q[b] += p;
                if (c.last_window_sent > 0)
                {
                    b_raw[b] += (double)c.last_window_received / c.last_window_sent;
                    b_rawn[b] += 1;
                }
                n[ph]++; udp[ph] += is_udp; lost[ph] += !delivered;
                alarms[ph] += sim_cd_resets - resets_before;
                ex_n[w]++; ex_l[w] += !delivered; ex_u[w] += is_udp;
                ex_f[w] += p > L ? L / p : 1.0;
                energy += hw_at(is_udp ? HW_EUDP : want ? HW_ETR : HW_ET0, p);
                e_tcp += hw_at(HW_ET0, p);
                feedbacks += want;
                fopt_sum += p > L ? L / p : 1.0;
                {
                    double f = p > L ? L / p : 1.0;
                    e_clair_msg += f * hw_at(HW_EUDP, p) + (1.0 - f) * hw_at(HW_ET0, p);
                    l_clair_msg += f * p;
                }
                if (t < start[ph] + window) { wn[ph]++; wudp[ph] += is_udp; wlost[ph] += !delivered; }
                if (ph > 0 && isnan(react[ph]))
                {
                    double target = 1.0 - (pl[ph - 1] + pl[ph]) / 2.0;
                    int hit = pl[ph] > pl[ph - 1] ? c.p_udp_hat <= target : c.p_udp_hat >= target;
                    if (hit) react[ph] = t + dt - start[ph];
                }
            }
            t += dt;
        }

        double excess = 0, deficit = 0, sq = 0;
        int nwin = 0, nviol = 0, nunder = 0;
        for (int w = 0; w < nex; w++)
        {
            if (ex_n[w] < exwin / 2) continue;      /* drop a trailing partial window */
            nwin++;
            if (ex_l[w] > L * ex_n[w]) excess += ex_l[w] - L * ex_n[w];
            if (ex_l[w] > 1.5 * L * ex_n[w]) nviol++;
            if (ex_l[w] < 0.5 * L * ex_n[w]) nunder++;
            sq += pow((ex_l[w] - L * ex_n[w]) / (L * ex_n[w]), 2);
            if (ex_f[w] > ex_u[w]) deficit += ex_f[w] - ex_u[w];
        }
        if (mixout)
        {
            for (int k = 0; k < mix_n; k++) fprintf(mixout, "%d %d %d ", mx_n[k], mx_lost[k], mx_udp[k]);
            fprintf(mixout, "\n");
        }
        for (int i = 0; i < nph; i++) printf("%d %d %d %d ", n[i], udp[i], lost[i], alarms[i]);
        printf("|");
        for (int i = 0; i < nph; i++) printf(" %d %d %d %.1f", wn[i], wudp[i], wlost[i], react[i]);
        /* The clairvoyant bound twice: on this variant's own messages (e_clair_msg, against e_tcp:
         * the cheapest any controller could send the same messages), and on the channel itself, one
         * message per nominal slot (e_clair against pure TCP on the same slots, e_clair_tcp). They
         * differ because TCP's blocking stretches the time between messages, most on a bad channel,
         * so each variant's messages sample the channel differently. */
        for (double tk = 0; tk < t_end; tk += 1.0)
        {
            double pk = ch_q[(int)((tk + warmup) * msg_dt)];
            double f = pk > L ? L / pk : 1.0;
            e_clair += f * hw_at(HW_EUDP, pk) + (1.0 - f) * hw_at(HW_ET0, pk);
            e_clair_tcp += hw_at(HW_ET0, pk);
        }
        /* nunder: windows below 0.5 L; sq: sum over windows of ((lost - L n) / (L n))^2 */
        printf(" | %.1f %.2f %d %d %.1f %d %.1f %.1f %d %.3f %.1f %.1f %.1f %.2f\n", energy, excess, nwin,
               nviol, fopt_sum, feedbacks, deficit, e_tcp, nunder, sq, e_clair_msg, e_clair, e_clair_tcp,
               l_clair_msg);
    }
    if (pktdump) fclose(pktdump);
    if (mixout) fclose(mixout);
    if (trace)
    {
        FILE *f = fopen(trace, "w");
        for (int b = 0; b < nbins; b++)
            if (b_n[b] > 0)
                fprintf(f, "%.1f %.5f %.5f %.5f %.5f %.5f\n", b * 5.0 + 2.5, b_est[b] / b_n[b],
                        b_udp[b] / b_n[b], b_lost[b] / b_n[b],
                        b_rawn[b] > 0 ? b_raw[b] / b_rawn[b] : NAN, b_q[b] / b_n[b]);
        fclose(f);
    }
    return 0;
}

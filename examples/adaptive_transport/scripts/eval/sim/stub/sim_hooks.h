/* Force-included into the sim_opt copy of adaptive_transport_controller.c (see build.sh), whose two
 * p_udp_hat assignments -- the calibration set and the N0 blend -- are rewritten to call these.
 * The simulator's default implementations reproduce the original expressions exactly. */
#pragma once
struct adaptive_transport_controller;
double sim_udp_calib(struct adaptive_transport_controller *ctl, int received, int sent);
double sim_udp_blend(struct adaptive_transport_controller *ctl, int received, int sent);
/* The time estimator's prior, ATC_EST_PRIOR_LOST / ATC_EST_PRIOR_WEIGHT in the controller. */
extern double sim_prior_lost, sim_prior_weight;

#!/usr/bin/env bash
# Build the controller simulators against the real ../../../src/adaptive_transport_controller.c.
#   sim_opt     the controller with an estimator hook on random and bursty channels, with energy
set -euo pipefail
cd "$(dirname "$0")"
SRC=../../../src
# sim_opt: the same controller with its two p_udp_hat assignments routed through sim hooks, and the
# time estimator's prior through two variables (FW_PRIOR_Q, FW_PRIOR_W; defaults as the controller)
sed -e 's/ctl->p_udp_hat = atc_weighted_update(ctl->p_udp_hat, received_udp, sent_udp, ctl->N0);/ctl->p_udp_hat = sim_udp_blend(ctl, received_udp, sent_udp);/' \
    -e 's|ctl->p_udp_hat = (double)received_udp / (double)sent_udp;|ctl->p_udp_hat = sim_udp_calib(ctl, received_udp, sent_udp);|' \
    -e 's|^#define ATC_EST_PRIOR_LOST .*|#define ATC_EST_PRIOR_LOST sim_prior_lost|' \
    -e 's|^#define ATC_EST_PRIOR_WEIGHT .*|#define ATC_EST_PRIOR_WEIGHT sim_prior_weight|' \
    $SRC/adaptive_transport_controller.c > atc_hook.c
test "$(grep -c 'p_udp_hat = sim_udp_\(blend\|calib\)(ctl' atc_hook.c)" = 2 || { echo "hook patch did not apply"; exit 1; }
test "$(grep -c '^#define ATC_EST_PRIOR_[A-Z]* sim_prior_' atc_hook.c)" = 2 || { echo "prior hook did not apply"; exit 1; }
gcc -O2 -std=gnu11 -w -Istub -I$SRC -include stub/quiet_log.h -include stub/sim_hooks.h -c atc_hook.c -o atc_hook.o
gcc -O2 -std=gnu11 -Wall -Istub -I$SRC sim_opt.c atc_hook.o -lm -o sim_opt
echo built: sim_opt

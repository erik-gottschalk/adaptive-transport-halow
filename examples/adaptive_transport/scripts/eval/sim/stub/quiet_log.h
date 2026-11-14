/* Force-included into adaptive_transport_controller.c: swallow its DEMO_LOG printf output but
 * count the events the simulation cares about (window-CD resets and CUSUM alarms both count as
 * sim_cd_resets). */
#pragma once
#include <stdio.h>
#include <string.h>
extern int sim_cd_resets, sim_floor_hits;
#define printf(fmt, ...) \
    (((strstr((fmt), "change-detected") || strstr((fmt), "cusum degradation")) ? \
      sim_cd_resets++ : 0), \
     (strstr((fmt), "floored") ? sim_floor_hits++ : 0), 0)

#
# Copyright 2026 Morse Micro
#
# SPDX-License-Identifier: Apache-2.0
#

# newlib-nano's floating-point printf/snprintf: without it every %f/%g in the logs and in the
# remote configuration's confirmation prints an empty string. Appended here rather than on the make
# command line, where some make versions would discard the Makefile's own LINKFLAGS.
LINKFLAGS += -Wl,-u,_printf_float

# lwIP's pools (defaults: 4 netconns, 5 TCP PCBs, 16 TCP segments). The application opens a fresh
# TCP connection for every TCP message, and under loss a closed connection's PCB, netconn and
# segments stay alive for seconds while its FIN is retransmitted; with the defaults, socket() and
# connect() then fail on the device itself (ENOBUFS, ENOMEM). These sizes leave headroom up to 20 %
# injected loss.
BUILD_DEFINES += MEMP_NUM_NETCONN=12 MEMP_NUM_TCP_PCB=32 MEMP_NUM_TCP_SEG=48

# `make DEMO_SLEEP_MODE=1`: WNM sleep with the chip powered down between sends (see
# adaptive_transport.c). Unset or 0: the default firmware.
ifneq ($(DEMO_SLEEP_MODE),)
BUILD_DEFINES += DEMO_SLEEP_MODE=$(DEMO_SLEEP_MODE)
endif

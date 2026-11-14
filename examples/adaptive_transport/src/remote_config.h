/*
 * Copyright 2026 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief One-shot remote experiment configuration query.
 *
 * Lets the reference server (server.py) hand the device its experiment configuration at boot,
 * instead of every parameter being fixed at compile time. adaptive_transport.c calls
 * @ref remote_config_fetch once, before bringing up the adaptive transport controller or sending
 * any demo packet (including the controller's own startup calibration burst), so the whole run
 * -- calibration included -- uses whichever configuration the server hands back. The query uses
 * its own short-lived TCP connection, separate from (and closed well before) the
 * TCP connection used for the actual packet stream, so nothing about calibration or steady-state
 * operation is affected by it.
 *
 * Wire protocol, all on one TCP connection that the client closes at the end:
 *  -# Client -> server: a single request line, "GET_CONFIG\n".
 *  -# Server -> client: the configuration body -- one "KEY=VALUE" pair per line (LF or CRLF
 *     terminated), followed by a blank line marking the end of the message. Booleans are "0"/"1".
 *  -# Client -> server: **only if step 2 was received and at least one key recognized**, the
 *     confirmation body -- the same "KEY=VALUE\n...blank line" format, but reflecting @c cfg's
 *     final, merged state (every recognized key, whether it came from the server's response or
 *     was left at the caller's default), so the server's log records exactly what the device is
 *     about to run with. If the query failed outright (no connection, nothing returned), there is
 *     nothing to confirm and this step is skipped entirely.
 *  -# Either side closes; the client always closes its end once step 3 (if any) has been sent.
 *
 * Recognized keys:
 *
 *   DEMO_USE_ADAPTIVE_TRANSPORT   bool
 *   DEMO_PACKET_USE_TCP           bool
 *   DEMO_TCP_WAIT_REPLY           bool
 *   ATC_X                         int
 *   ATC_L                         float
 *   ATC_N0                        int
 *   ATC_EPS                       float
 *   ATC_CALIBRATION_PACKETS       int
  *   ATC_USE_CHANGE_DETECTION      bool
 *   ATC_DEBT_R                    float (loss-debt horizon in messages, 0 = off)
  *   ATC_DEBT_PER_CLASS            bool
 *   ATC_DEBT_TCP                  bool (charge locally failed TCP sends to the loss debt)
 *   ATC_RANDOM_PROBE              bool (probe as a 1/ATC_X floor on the UDP probability; 0 =
 *                                 the fixed cycle)
 *   SEND_INTERVAL_MS              int
 *   UDP_ACK                       bool (single-protocol UDP: acknowledged UDP; the server switches
 *                                 with it, from the same key of its own config)
 *   MIX_L                         comma-separated floats (up to REMOTE_CONFIG_MAX_MIX_CLASSES)
 *   MIX_SHARE                     comma-separated floats, one per MIX_L entry
 *
 * Unrecognized keys in the server's response are ignored (forward compatibility). Keys the
 * response omits leave whatever value the caller had already put in @c cfg untouched -- so the
 * caller should pre-populate @c cfg with the firmware's hard-coded defaults before calling, and a
 * query that fails outright (no listener, connection drop, nothing received before the timeout)
 * leaves every field at that default, exactly as if remote configuration wasn't in use at all.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of message classes in a mixed-criticality workload (MIX_L / MIX_SHARE). */
#define REMOTE_CONFIG_MAX_MIX_CLASSES 4

/** Remote experiment configuration, as fetched from the server (see the file-level comment). */
struct remote_config
{
    bool use_adaptive_transport;
    bool packet_use_tcp;
    bool tcp_wait_reply;

    int atc_x;
    double atc_L;
    int atc_N0;
    double atc_eps;
    int atc_calibration_packets;
    bool atc_use_change_detection;
    /** Loss debt (see debt_horizon in adaptive_transport_controller.h). */
    double atc_debt_horizon;
    bool atc_debt_per_class;
    bool atc_debt_tcp;
    /** Probe as a floor on the UDP probability instead of the forced cycle (see random_probe). */
    bool atc_random_probe;

    /** Pause after each send before the next one, in milliseconds (the send itself comes on
     *  top); defaults to DEMO_SEND_INTERVAL_S. */
    uint32_t send_interval_ms;

    /** Fixed-UDP mode only (packet_use_tcp and use_adaptive_transport off): every message waits
     *  for an application-level acknowledgment from the server and is retransmitted on timeout,
     *  with CoAP's confirmable-message rules (see send_udp_ack_demo_packet()). A reference point
     *  for UDP made reliable above the transport, not used by the controller. */
    bool udp_ack;

    /** Mixed-criticality workload (adaptive transport only): each message is assigned one of
     *  mix_L_count classes at random, class i with probability mix_share[i] / sum(mix_share), and
     *  the controller plans that message with loss tolerance mix_L[i] instead of atc_L. 0 classes
     *  (the default, "MIX_L=" on the wire) disables this and every message uses atc_L. The
     *  application validates that both lists have the same length before using them. */
    int mix_L_count;
    double mix_L[REMOTE_CONFIG_MAX_MIX_CLASSES];
    int mix_share_count;
    double mix_share[REMOTE_CONFIG_MAX_MIX_CLASSES];
};

/**
 * Query the server for remote experiment configuration, overwriting whichever fields of @p cfg
 * its response specifies.
 *
 * @param cfg         In/out configuration. Must be pre-populated by the caller with the desired
 *                    fallback (normally the firmware's hard-coded defaults); only fields present
 *                    in the server's response are overwritten, and the whole call is a no-op on
 *                    @p cfg if the query fails or nothing is received.
 * @param server_ip   Server IPv4 address, dotted-decimal string.
 * @param port        Server's config query TCP port.
 * @param timeout_ms  Timeout for the whole query (connect is not separately bounded, matching the
 *                    demo packet sends elsewhere in this application; only the receive side is
 *                    bounded by this timeout).
 *
 * @return true if at least one field of @p cfg was overwritten from the server's response, false
 *         if @p cfg was left entirely at its caller-supplied defaults.
 */
bool remote_config_fetch(struct remote_config *cfg, const char *server_ip, uint16_t port,
                         uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

/*
 * Copyright 2022-2026 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Periodic messages over UDP or TCP, the transport chosen per message by the adaptive
 * transport controller (adaptive_transport_controller.h) or fixed for a run.
 *
 * At boot the device fetches the run's settings from the receiver (remote_config.h), then sends
 * one DEMO_PAYLOAD_LEN-byte message per send interval: as UDP, as TCP over a fresh connection
 * (with or without a delivery report), as acknowledged UDP, or as the controller decides. Two
 * debug pins mark every send and its transport for the current measurement.
 *
 * @note It is assumed that you have followed the steps in the @ref GETTING_STARTED guide and are
 * therefore familiar with how to build, flash, and monitor an application using the MM-IoT-SDK
 * framework.
 */

#include <string.h>
#include <sys/time.h>
#include "mmosal.h"
#include "mmwlan.h"
#include "mmipal.h"

#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "lwip/priv/tcp_priv.h"

#include "mmhal_app.h"
#include "mmhal_core.h"

#include "mm_app_common.h"
#include "adaptive_transport_controller.h"
#include "remote_config.h"

/** Logging sink used throughout this file (and adaptive_transport_controller.c /
 *  remote_config.c) in place of a bare printf(). */
#define DEMO_LOG(...) printf(__VA_ARGS__)

/* The receiver (scripts/server.py), IPv4. Set it in server_config.local.h next to this file
 * (gitignored; see server_config.local.h.example); the default is a placeholder from the
 * documentation range (RFC 5737) and reaches no server. */
#if __has_include("server_config.local.h")
#include "server_config.local.h"
#endif
#ifndef TCP_TARGET_IP_STR
#define TCP_TARGET_IP_STR "192.0.2.1"
#endif
#define TCP_TARGET_PORT 64015
#define UDP_TARGET_PORT 64015

/** How long a TCP send waits in line for the receiver's 1-byte delivery report. */
#define TCP_REPLY_TIMEOUT_MS 3000

/** A report that misses the in-line wait is almost always late rather than lost (TCP's own
 *  retransmissions take longer than the wait): the socket is then held open, polled once per send
 *  interval, and closed when the report arrives or this long after the send (late reports were
 *  seen up to ~45 s after the send). See atc_report_tcp_reply_held(). */
#define TCP_REPLY_HOLD_MS 60000

/** Defaults of the run settings, used if the receiver sends none (see remote_config.h). */
#define DEMO_USE_ADAPTIVE_TRANSPORT 1
#define DEMO_PACKET_USE_TCP 1
#define DEMO_TCP_WAIT_REPLY 0
#define DEMO_SEND_INTERVAL_S 1

/** The remote-configuration query: a short connection of its own on this port of the receiver. */
#define REMOTE_CONFIG_TARGET_PORT 64016
#define REMOTE_CONFIG_TIMEOUT_MS 3000

/** What the HaLow module does between two sends (`make DEMO_SLEEP_MODE=1`):
 *    0: stay associated in 802.11 power save, waking for beacons.
 *    1: WNM sleep with the chip powered down. The association and its keys survive, so a wake
 *       costs the chip's boot and the WNM exit exchange, but no association or DHCP.
 *  Only the HaLow module sleeps; the MCU stays up. Meant for the single-protocol baselines at a
 *  send interval of several seconds. */
#ifndef DEMO_SLEEP_MODE
#define DEMO_SLEEP_MODE 0
#endif

/** Before sleeping, wait at most this long for TCP connections that are still closing (FIN
 *  exchange) to leave lwIP's active list. Connections in TIME_WAIT send nothing and are not
 *  waited for. */
#define DEMO_SLEEP_SETTLE_MAX_MS 3000

/** Before sleeping after a UDP send, wait at most this long for the chip to report the datagram
 *  sent (see rc_frames_sent()). */
#define DEMO_SLEEP_UDP_TX_MAX_MS 300

/** Length in bytes of every message's payload. */
#define DEMO_PAYLOAD_LEN 32

/** UDP messages go out as hand-built 802.3 + IPv4 + UDP frames through mmwlan_tx(), bypassing
 *  lwIP's UDP path (see send_udp_demo_packet_raw()). The frames are addressed to the access
 *  point's MAC, which routes them like any IP traffic; set it for your access point. */
#define AP_MAC_ADDR_OCTETS {0x94, 0x83, 0xC4, 0x82, 0x7C, 0x34}

/** Source UDP port of the raw frames (fixed: no socket assigns one). */
#define UDP_SOURCE_PORT 40000

#if defined(ENABLE_PWR_MEAS) && ENABLE_PWR_MEAS
/** Per-message transport marker for the current measurement, on two GPIOs: MM_DEBUG_0 (PA10,
 *  wired to the PPK2's D0 input) and MM_DEBUG_2 (PA0, wired to D1). The pair holds a 2-bit code
 *  for the duration of the send call (through the in-line report wait, when there is one) and is
 *  (0, 0) otherwise:
 *
 *    TCP (no report): D0=1 D1=0  (DEBUG_PROTO_TCP)
 *    UDP            : D0=0 D1=1  (DEBUG_PROTO_UDP; acknowledged UDP holds it over the whole
 *                                 exchange, retransmissions included)
 *    TCP + report   : D0=1 D1=1  (DEBUG_PROTO_TCP_REPLY)
 *    idle           : D0=0 D1=0
 *
 *  The pin widths alone cannot tell the transports apart (send() returns long before the radio
 *  is done), hence the code. MM_DEBUG_1 (PA9) is held high as a logic reference (app_init()). */
#define DEBUG_PROTO_UDP        MMHAL_DEBUG_PIN(2)
#define DEBUG_PROTO_TCP        MMHAL_DEBUG_PIN(0)
#define DEBUG_PROTO_TCP_REPLY  (MMHAL_DEBUG_PIN(0) | MMHAL_DEBUG_PIN(2))
#define DEBUG_PROTO_MASK       (MMHAL_DEBUG_PIN(0) | MMHAL_DEBUG_PIN(2))

#define SET_DEBUG_PROTO(code) mmhal_set_debug_pins(DEBUG_PROTO_MASK, (code))
#define CLEAR_DEBUG_PROTO() mmhal_set_debug_pins(DEBUG_PROTO_MASK, 0)
#else
#include "mmutils.h"
#define DEBUG_PROTO_UDP        0
#define DEBUG_PROTO_TCP        0
#define DEBUG_PROTO_TCP_REPLY  0
#define SET_DEBUG_PROTO(code) MM_UNUSED(code)
#define CLEAR_DEBUG_PROTO() ((void)0)
#endif

/**
 * Fills @p buf (of length @p len) with @p counter as a 4-byte big-endian sequence number,
 * followed by a fixed byte pattern. The receiver detects lost messages from gaps in the counter,
 * whichever transport carried them.
 */
static void fill_demo_payload(uint8_t *buf, size_t len, uint32_t counter)
{
    size_t index;
    uint32_t counter_be = htonl(counter);

    memcpy(buf, &counter_be, sizeof(counter_be));

    for (index = sizeof(counter_be); index < len; index++)
    {
        buf[index] = (uint8_t)('A' + (index % 26));
    }
}

/** One's-complement checksum over @p header (@p len bytes, even), as in the IPv4 header. */
static uint16_t ipv4_header_checksum(const uint8_t *header, uint16_t len)
{
    uint32_t sum = 0;
    uint16_t index;

    for (index = 0; index < len; index += 2)
    {
        sum += ((uint16_t)header[index] << 8) | header[index + 1];
    }

    while (sum >> 16)
    {
        sum = (sum & 0xFFFFU) + (sum >> 16);
    }

    return (uint16_t)~sum;
}

/**
 * Send a raw 802.3 + IPv4 + UDP frame carrying @p payload to the receiver through
 * @ref mmwlan_tx, bypassing lwIP. The source IP is the current DHCP address.
 *
 * @param own_mac  This device's MAC address (source of the 802.3 header).
 * @param seq      The message's sequence number, logged for matching with the receiver's log.
 *
 * @return true if the frame was handed to the transceiver.
 */
static bool send_udp_demo_packet_raw(const uint8_t *own_mac, const uint8_t *payload,
                                     uint16_t payload_len, uint32_t seq)
{
    enum mmwlan_status status;
    const uint8_t ap_mac[MMWLAN_MAC_ADDR_LEN] = AP_MAC_ADDR_OCTETS;
    struct mmipal_ip_config ip_config = MMIPAL_IP_CONFIG_DEFAULT;
    uint8_t source_ip[4] = {0};
    uint8_t target_ip[4];
    uint32_t target_ip_be = inet_addr(TCP_TARGET_IP_STR);
    const uint16_t udp_len = (uint16_t)(8U + payload_len);
    const uint16_t ip_total_len = (uint16_t)(20U + udp_len);

    memcpy(target_ip, &target_ip_be, sizeof(target_ip));

    if (mmipal_get_ip_config(&ip_config) == MMIPAL_SUCCESS)
    {
        uint32_t source_ip_be = inet_addr(ip_config.ip_addr);
        memcpy(source_ip, &source_ip_be, sizeof(source_ip));
    }

    uint8_t udp_packet[14 + 20 + 8 + DEMO_PAYLOAD_LEN] = {
        /* Ethernet header: DA = the access point, SA = ours, ethertype = IPv4. */
        ap_mac[0], ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5],
        own_mac[0], own_mac[1], own_mac[2], own_mac[3], own_mac[4], own_mac[5],
        0x08, 0x00,

        /* IPv4 header */
        0x45, 0x00,
        (uint8_t)(ip_total_len >> 8), (uint8_t)(ip_total_len & 0xff),
        0x00, 0x01,
        0x40, 0x00,
        64,
        17,
        0x00, 0x00,
        source_ip[0], source_ip[1], source_ip[2], source_ip[3],
        target_ip[0], target_ip[1], target_ip[2], target_ip[3],

        /* UDP header */
        (uint8_t)(UDP_SOURCE_PORT >> 8), (uint8_t)(UDP_SOURCE_PORT & 0xff),
        (uint8_t)(UDP_TARGET_PORT >> 8), (uint8_t)(UDP_TARGET_PORT & 0xff),
        (uint8_t)(udp_len >> 8), (uint8_t)(udp_len & 0xff),
        0x00, 0x00,
    };

    memcpy(&udp_packet[14 + 20 + 8], payload, payload_len);

    {
        const uint16_t ip_checksum = ipv4_header_checksum(&udp_packet[14], 20);
        udp_packet[24] = (uint8_t)(ip_checksum >> 8);
        udp_packet[25] = (uint8_t)(ip_checksum & 0xff);
    }

    status = mmwlan_tx(udp_packet, (unsigned)(14 + 20 + 8 + payload_len));
    if (status != MMWLAN_SUCCESS)
    {
        DEMO_LOG("UDP demo TX (raw frame) failed with status %d (seq=%lu)\n", status, seq);
        return false;
    }

    DEMO_LOG("UDP demo packet (raw frame) sent to %s:%u (%u bytes, seq=%lu)\n",
           TCP_TARGET_IP_STR, UDP_TARGET_PORT, payload_len, seq);
    return true;
}

#if DEMO_SLEEP_MODE
/** Frames the chip has reported sent, summed over the rate table (0 if unavailable). mmwlan has no
 *  per-frame TX-done callback; this count rises once a frame's TX status comes back. */
static uint32_t rc_frames_sent(void)
{
    struct mmwlan_rc_stats *stats = mmwlan_get_rc_stats();
    uint32_t sum = 0;

    if (stats != NULL)
    {
        for (uint32_t i = 0; i < stats->n_entries; i++)
        {
            sum += stats->total_sent[i];
        }
        mmwlan_free_rc_stats(stats);
    }
    return sum;
}

/** rc_frames_sent() just before the last UDP send, so sleep_until() can wait for that datagram
 *  to leave the radio: powering the chip down right after mmwlan_tx() dropped nearly every one. */
static uint32_t udp_rc_frames_before = 0;
static bool udp_pending_tx = false;
#endif

/** Hand one UDP datagram carrying @p payload to the radio as a raw frame, without touching the
 *  debug pins. */
static bool udp_tx(const uint8_t *payload, uint16_t payload_len, uint32_t seq)
{
    static uint8_t own_mac[MMWLAN_MAC_ADDR_LEN];
    static bool own_mac_ready = false;

    if (!own_mac_ready)
    {
        if (mmwlan_get_mac_addr(own_mac) != MMWLAN_SUCCESS)
        {
            DEMO_LOG("Failed to read own MAC address for raw UDP frame (seq=%lu)\n", seq);
            return false;
        }
        own_mac_ready = true;
    }
    return send_udp_demo_packet_raw(own_mac, payload, payload_len, seq);
}

/** Send @p payload as one UDP message, bracketed by the UDP debug-pin code. */
static bool send_udp_demo_packet(const uint8_t *payload, uint16_t payload_len, uint32_t seq)
{
#if DEMO_SLEEP_MODE
    udp_rc_frames_before = rc_frames_sent();
    udp_pending_tx = true;
#endif
    SET_DEBUG_PROTO(DEBUG_PROTO_UDP);
    bool success = udp_tx(payload, payload_len, seq);
    CLEAR_DEBUG_PROTO();
    return success;
}

/** Acknowledged UDP (run setting UDP_ACK, single-protocol runs): CoAP's confirmable-message
 *  transmission rules (RFC 7252, Section 4.2) with its default parameters. The first wait is drawn
 *  uniformly from [ACK_TIMEOUT, ACK_TIMEOUT * ACK_RANDOM_FACTOR] = [2, 3] s, every retransmission
 *  doubles it, and the message is given up after MAX_RETRANSMIT = 4 retransmissions. */
#define UDP_ACK_TIMEOUT_MS 2000
#define UDP_ACK_TIMEOUT_MAX_MS 3000
#define UDP_ACK_MAX_RETRANSMIT 4

/**
 * Send @p payload as acknowledged UDP: transmit, wait for the receiver's acknowledgment (the
 * message's 4-byte sequence number echoed back), and retransmit on timeout under the rules above.
 * @p sock must be bound to UDP_SOURCE_PORT, the source port of the raw frames, so that the
 * acknowledgment reaches it. The debug pins hold the UDP code over the whole exchange. An
 * acknowledgment of an earlier message is discarded.
 *
 * @return true if the message was acknowledged.
 */
static bool send_udp_ack_demo_packet(int sock, const uint8_t *payload, uint16_t payload_len,
                                     uint32_t seq)
{
    uint32_t timeout_ms = mmhal_random_u32(UDP_ACK_TIMEOUT_MS, UDP_ACK_TIMEOUT_MAX_MS);
    uint32_t start_ms = mmosal_get_time_ms();
    int attempt;

    SET_DEBUG_PROTO(DEBUG_PROTO_UDP);
    for (attempt = 0; attempt <= UDP_ACK_MAX_RETRANSMIT; attempt++)
    {
        uint32_t deadline_ms;

        udp_tx(payload, payload_len, seq);
        deadline_ms = mmosal_get_time_ms() + timeout_ms;
        while (true)
        {
            int32_t remaining_ms = (int32_t)(deadline_ms - mmosal_get_time_ms());
            struct timeval wait = {0};
            uint8_t ack[4];
            int received;

            if (remaining_ms <= 0)
            {
                break;
            }
            wait.tv_sec = remaining_ms / 1000;
            wait.tv_usec = (remaining_ms % 1000) * 1000;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof(wait));
            received = recv(sock, ack, sizeof(ack), 0);
            if (received == (int)sizeof(ack) && memcmp(ack, payload, sizeof(ack)) == 0)
            {
                DEMO_LOG("UDP ack for seq=%lu after %d transmission(s), %lu ms\n", seq,
                         attempt + 1, (unsigned long)(mmosal_get_time_ms() - start_ms));
                CLEAR_DEBUG_PROTO();
                return true;
            }
            if (received < 0)
            {
                break;      /* timed out (or failed): retransmit */
            }
            /* A late acknowledgment of an earlier message: keep waiting for this one's. */
        }
        timeout_ms *= 2;
    }
    DEMO_LOG("UDP ack for seq=%lu given up after %d transmissions, %lu ms\n", seq,
             UDP_ACK_MAX_RETRANSMIT + 1, (unsigned long)(mmosal_get_time_ms() - start_ms));
    CLEAR_DEBUG_PROTO();
    return false;
}

/**
 * Create a UDP socket connected to the receiver. The UDP messages themselves go out as raw
 * frames; the socket receives the acknowledgments of acknowledged UDP.
 *
 * @param bind_source_port  Bind the socket to UDP_SOURCE_PORT first, so that it receives the
 *                          acknowledgments (send_udp_ack_demo_packet()).
 *
 * @return The socket's file descriptor, or -1 on failure (logged).
 */
static int udp_connect(bool bind_source_port)
{
    struct sockaddr_in udp_dest_addr = {0};
    int sock;

    udp_dest_addr.sin_family = AF_INET;
    udp_dest_addr.sin_port = htons(UDP_TARGET_PORT);
    udp_dest_addr.sin_addr.s_addr = inet_addr(TCP_TARGET_IP_STR);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        DEMO_LOG("Failed to create UDP socket (errno %d)\n", errno);
        return -1;
    }

    if (bind_source_port)
    {
        struct sockaddr_in local_addr = {0};

        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(UDP_SOURCE_PORT);
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) != 0)
        {
            DEMO_LOG("UDP bind to port %u failed (errno %d)\n", UDP_SOURCE_PORT, errno);
            close(sock);
            return -1;
        }
    }

    if (connect(sock, (struct sockaddr *)&udp_dest_addr, sizeof(udp_dest_addr)) != 0)
    {
        DEMO_LOG("UDP connect to %s:%u failed (errno %d)\n",
               TCP_TARGET_IP_STR, UDP_TARGET_PORT, errno);
        close(sock);
        return -1;
    }

    DEMO_LOG("UDP socket connected to %s:%u\n", TCP_TARGET_IP_STR, UDP_TARGET_PORT);
    return sock;
}

/**
 * Send @p payload over a fresh TCP connection (handshake, data, teardown), preceded by the 1-byte
 * report request of the feedback protocol: @p expected_udp_count, the number of UDP messages sent
 * since the last report (0: no report wanted). With @p wait_for_reply the call then waits up to
 * TCP_REPLY_TIMEOUT_MS for the receiver's 1-byte report, the number of those it received.
 *
 * The return value says only whether the send() completed; whether a requested report arrived is
 * @p reply_received. A failed connect or send is logged, not fatal.
 *
 * @param[out] received_udp_count  The report, set when @p *reply_received is true.
 * @param[out] reply_received      Whether the report arrived within the in-line wait.
 * @param[out] held_sock           NULL to always close the socket. Otherwise set to -1, or, if the
 *                                 in-line wait timed out, to the still-open socket, which the
 *                                 caller then owns (see TCP_REPLY_HOLD_MS).
 * @param seq                      The message's sequence number, logged.
 *
 * @return true if the send() itself completed locally.
 */
static bool send_tcp_demo_packet(const uint8_t *payload, uint16_t payload_len,
                                 uint8_t expected_udp_count, uint8_t *received_udp_count,
                                 bool wait_for_reply, bool *reply_received, int *held_sock,
                                 uint32_t seq)
{
    struct sockaddr_in tcp_dest_addr = {0};
    struct timeval reply_timeout = {0};
    int tcp_sock;
    bool success = false;
    uint8_t tcp_frame[1 + DEMO_PAYLOAD_LEN];

    SET_DEBUG_PROTO(wait_for_reply ? DEBUG_PROTO_TCP_REPLY : DEBUG_PROTO_TCP);

    *reply_received = false;
    if (held_sock != NULL)
    {
        *held_sock = -1;
    }

    tcp_dest_addr.sin_family = AF_INET;
    tcp_dest_addr.sin_port = htons(TCP_TARGET_PORT);
    tcp_dest_addr.sin_addr.s_addr = inet_addr(TCP_TARGET_IP_STR);

    tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_sock < 0)
    {
        DEMO_LOG("Failed to create TCP socket (errno %d, seq=%lu)\n", errno, seq);
        CLEAR_DEBUG_PROTO();
        return false;
    }

    reply_timeout.tv_sec = TCP_REPLY_TIMEOUT_MS / 1000;
    reply_timeout.tv_usec = (TCP_REPLY_TIMEOUT_MS % 1000) * 1000;
    setsockopt(tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &reply_timeout, sizeof(reply_timeout));

    if (connect(tcp_sock, (struct sockaddr *)&tcp_dest_addr, sizeof(tcp_dest_addr)) == 0)
    {
        tcp_frame[0] = expected_udp_count;
        memcpy(&tcp_frame[1], payload, payload_len);

        if (send(tcp_sock, tcp_frame, (size_t)(1U + payload_len), 0) > 0)
        {
            success = true;

            if (!wait_for_reply)
            {
                DEMO_LOG("TCP demo packet sent to %s:%u (%u bytes, expected_udp=%u, seq=%lu)\n",
                       TCP_TARGET_IP_STR, TCP_TARGET_PORT, payload_len, expected_udp_count, seq);
            }
            else
            {
                uint8_t reply;
                int received = recv(tcp_sock, &reply, sizeof(reply), 0);

                if (received == (int)sizeof(reply))
                {
                    *received_udp_count = reply;
                    *reply_received = true;
                    DEMO_LOG("TCP demo packet sent to %s:%u (%u bytes, expected_udp=%u, seq=%lu); "
                           "server reports %u received\n",
                           TCP_TARGET_IP_STR, TCP_TARGET_PORT, payload_len, expected_udp_count,
                           seq, reply);
                }
                else if (held_sock != NULL && received < 0 &&
                         (errno == EAGAIN || errno == EWOULDBLOCK))
                {
                    /* Timed out, not errored: the report is most likely late. Hand the open
                     * socket to the caller instead of closing it. */
                    *held_sock = tcp_sock;
                    DEMO_LOG("TCP demo reply from %s:%u not in %u ms, holding socket open "
                           "(seq=%lu)\n", TCP_TARGET_IP_STR, TCP_TARGET_PORT,
                           (unsigned)TCP_REPLY_TIMEOUT_MS, seq);
                }
                else
                {
                    DEMO_LOG("TCP demo reply from %s:%u failed (errno %d, seq=%lu)\n",
                           TCP_TARGET_IP_STR, TCP_TARGET_PORT, errno, seq);
                }
            }
        }
        else
        {
            DEMO_LOG("TCP demo send to %s:%u failed (errno %d, seq=%lu)\n",
                   TCP_TARGET_IP_STR, TCP_TARGET_PORT, errno, seq);
        }
    }
    else
    {
        DEMO_LOG("TCP connect to %s:%u failed (errno %d, seq=%lu)\n",
               TCP_TARGET_IP_STR, TCP_TARGET_PORT, errno, seq);
    }
    if (held_sock == NULL || *held_sock < 0)
    {
        close(tcp_sock);
    }
    CLEAR_DEBUG_PROTO();
    return success;
}

#if DEMO_SLEEP_MODE
/**
 * Wait, up to DEMO_SLEEP_SETTLE_MAX_MS, until no TCP connection is left in lwIP's active list,
 * i.e. every connection opened so far has finished its exchange and teardown (or sits in
 * TIME_WAIT, which sends nothing). Returns how long it waited.
 */
static uint32_t wait_for_tcp_teardown(void)
{
    uint32_t start_ms = mmosal_get_time_ms();

    while (mmosal_get_time_ms() - start_ms < DEMO_SLEEP_SETTLE_MAX_MS)
    {
        bool busy;

        LOCK_TCPIP_CORE();
        busy = tcp_active_pcbs != NULL;
        UNLOCK_TCPIP_CORE();
        if (!busy)
        {
            break;
        }
        mmosal_task_sleep(10);
    }
    return mmosal_get_time_ms() - start_ms;
}

/**
 * Put the HaLow module into WNM sleep until @p wake_at_ms, then wake it. Logs how long entering
 * and leaving sleep took, so that the wake-up cost can be located in the current trace.
 */
static void sleep_until(uint32_t wake_at_ms)
{
    uint32_t settle_ms = wait_for_tcp_teardown();
    uint32_t t_udp = mmosal_get_time_ms();
    bool udp_confirmed = true;

    if (udp_pending_tx)
    {
        udp_pending_tx = false;
        udp_confirmed = false;
        while (mmosal_get_time_ms() - t_udp < DEMO_SLEEP_UDP_TX_MAX_MS)
        {
            if (rc_frames_sent() != udp_rc_frames_before)
            {
                udp_confirmed = true;
                break;
            }
            mmosal_task_sleep(2);
        }
    }
    uint32_t udp_wait_ms = mmosal_get_time_ms() - t_udp;
    uint32_t t0 = mmosal_get_time_ms();
    bool asleep = false;

    struct mmwlan_set_wnm_sleep_enabled_args wnm_args = MMWLAN_SET_WNM_SLEEP_ENABLED_ARGS_INIT;
    wnm_args.wnm_sleep_enabled = true;
    wnm_args.chip_powerdown_enabled = true;
    enum mmwlan_status status = mmwlan_set_wnm_sleep_enabled_ext(&wnm_args);
    asleep = status == MMWLAN_SUCCESS;
    if (!asleep)
    {
        /* The AP refused or does not support WNM sleep: the module stays in plain power save,
         * so this cycle measures mode 0, not mode 1. Loud, since it invalidates the capture. */
        DEMO_LOG("SLEEP: WNM sleep entry FAILED (status %d), module stays associated\n", status);
    }
    uint32_t enter_ms = mmosal_get_time_ms() - t0;

    int32_t remaining_ms = (int32_t)(wake_at_ms - mmosal_get_time_ms());
    if (remaining_ms > 0)
    {
        mmosal_task_sleep((uint32_t)remaining_ms);
    }

    uint32_t t1 = mmosal_get_time_ms();
    if (asleep)
    {
        wnm_args.wnm_sleep_enabled = false;
        status = mmwlan_set_wnm_sleep_enabled_ext(&wnm_args);
        if (status != MMWLAN_SUCCESS)
        {
            /* Per mmwlan.h the chip is awake regardless; a failed exit request usually means the
             * AP dropped the association, after which mmwlan re-associates on its own. */
            DEMO_LOG("SLEEP: WNM sleep exit FAILED (status %d)\n", status);
        }
    }
    DEMO_LOG("SLEEP: settle %lu ms, udp tx %lu ms%s, enter %lu ms, wake %lu ms\n",
             (unsigned long)settle_ms, (unsigned long)udp_wait_ms,
             udp_confirmed ? "" : " (NOT confirmed)", (unsigned long)enter_ms,
             (unsigned long)(mmosal_get_time_ms() - t1));
}
#endif

/**
 * Main entry point to the application. This will be invoked in a thread once operating system
 * and hardware initialization has completed.
 */
void app_init(void)
{
    DEMO_LOG("\n\nAdaptive Transport Example (Built " __DATE__ " " __TIME__ ")\n\n");

#if defined(ENABLE_PWR_MEAS) && ENABLE_PWR_MEAS
    /* Debug pin 1 (MM_DEBUG_1 / PA9) permanently high: a logic-high reference for the PPK2's
     * digital inputs (the devkit has no 3.3 V pin near the debug header). */
    mmhal_set_debug_pins(MMHAL_DEBUG_PIN(1), MMHAL_DEBUG_PIN(1));
#endif

    /* Initialize WLAN and IP stack using settings from the config store (see config.hjson), and
     * connect. Blocks until the link (association + DHCP lease) is up. */
    app_wlan_init();
    app_wlan_start();

    /* The run's settings from the receiver (see remote_config.h), before the controller is set
     * up, so that even its startup calibration runs under them. A failed or empty query leaves
     * these defaults. */
    struct remote_config remote_cfg = {
        .use_adaptive_transport = DEMO_USE_ADAPTIVE_TRANSPORT,
        .packet_use_tcp = DEMO_PACKET_USE_TCP,
        .tcp_wait_reply = DEMO_TCP_WAIT_REPLY,
        .atc_x = ATC_DEFAULT_X,
        .atc_L = ATC_DEFAULT_L,
        .atc_N0 = ATC_DEFAULT_N0,
        .atc_eps = ATC_DEFAULT_EPS,
        .atc_calibration_packets = ATC_DEFAULT_CALIBRATION_PACKETS,
        .atc_use_change_detection = ATC_DEFAULT_USE_CHANGE_DETECTION,
        .atc_debt_horizon = ATC_DEFAULT_DEBT_HORIZON,
        .atc_debt_per_class = ATC_DEFAULT_DEBT_PER_CLASS,
        .atc_debt_tcp = ATC_DEFAULT_DEBT_TCP_FAILURES,
        .atc_random_probe = ATC_DEFAULT_RANDOM_PROBE,
        .send_interval_ms = DEMO_SEND_INTERVAL_S * 1000U,
        .udp_ack = false,
        .mix_L_count = 0,
        .mix_share_count = 0,
    };
    remote_config_fetch(&remote_cfg, TCP_TARGET_IP_STR, REMOTE_CONFIG_TARGET_PORT,
                        REMOTE_CONFIG_TIMEOUT_MS);

    /* Mixed workload (remote_cfg.mix_L, see remote_config.h): cumulative class shares for the
     * per-message class draw in the loop below. Off unless both lists are present, equally long
     * and the shares sum to something positive. */
    int mix_classes = 0;
    double mix_cumulative[REMOTE_CONFIG_MAX_MIX_CLASSES];
    if (remote_cfg.mix_L_count > 0)
    {
        double share_sum = 0.0;
        int i;

        for (i = 0; i < remote_cfg.mix_share_count; i++)
        {
            share_sum += remote_cfg.mix_share[i];
            mix_cumulative[i] = share_sum;
        }
        if (remote_cfg.mix_share_count != remote_cfg.mix_L_count || share_sum <= 0.0 ||
            !remote_cfg.use_adaptive_transport)
        {
            DEMO_LOG("MIX: ignoring mixed workload (%d L values, %d shares, adaptive=%d) -- "
                   "every message uses L=%g\n", remote_cfg.mix_L_count,
                   remote_cfg.mix_share_count, (int)remote_cfg.use_adaptive_transport,
                   remote_cfg.atc_L);
        }
        else
        {
            mix_classes = remote_cfg.mix_L_count;
            for (i = 0; i < mix_classes; i++)
            {
                mix_cumulative[i] /= share_sum;
                DEMO_LOG("MIX: class %d L=%g share=%.4f\n", i, remote_cfg.mix_L[i],
                       remote_cfg.mix_share[i] / share_sum);
            }
        }
    }
    DEMO_LOG("Send interval %lu ms\n", (unsigned long)remote_cfg.send_interval_ms);

    uint8_t demo_payload[DEMO_PAYLOAD_LEN];
    uint32_t demo_packet_counter = 0;

    struct adaptive_transport_controller atc;
    atc_init(&atc, remote_cfg.atc_x, remote_cfg.atc_L, remote_cfg.atc_N0, remote_cfg.atc_eps,
             remote_cfg.atc_calibration_packets, remote_cfg.atc_use_change_detection);
    atc.debt_horizon = remote_cfg.atc_debt_horizon;
    atc.debt_per_class = remote_cfg.atc_debt_per_class;
    atc.debt_tcp_failures = remote_cfg.atc_debt_tcp;
    atc.random_probe = remote_cfg.atc_random_probe;
    DEMO_LOG("ATC: loss debt horizon=%g per_class=%d tcp_failures=%d\n", atc.debt_horizon,
           (int)atc.debt_per_class, (int)atc.debt_tcp_failures);
    DEMO_LOG("ATC: probe random_probe=%d (UDP floor 1/x=%.4f)\n", (int)atc.random_probe,
           1.0 / (double)atc.x);

    /* The socket of a TCP send whose report missed the in-line wait, held open until the report
     * arrives or TCP_REPLY_HOLD_MS passes (-1 if none), with that send's start time and sequence
     * number. */
    int held_reply_sock = -1;
    uint32_t held_reply_since_ms = 0;
    uint32_t held_reply_seq = 0;

    int udp_sock = udp_connect(remote_cfg.udp_ack);

    while (true)
    {
        uint32_t seq = demo_packet_counter;

        /* Resolve a held report before this message's decision: during the startup calibration
         * the controller keeps choosing TCP until the calibration report lands. */
        if (held_reply_sock >= 0)
        {
            uint8_t reply = 0;
            int received = recv(held_reply_sock, &reply, sizeof(reply), MSG_DONTWAIT);
            int recv_errno = errno;
            uint32_t held_ms = mmosal_get_time_ms() - held_reply_since_ms;
            bool still_waiting = received < 0 &&
                                 (recv_errno == EAGAIN || recv_errno == EWOULDBLOCK) &&
                                 held_ms < TCP_REPLY_HOLD_MS;

            if (!still_waiting)
            {
                bool reply_arrived = received == (int)sizeof(reply);

                if (reply_arrived)
                {
                    DEMO_LOG("TCP held reply for seq=%lu arrived after %lu ms: server reports %u "
                           "received\n", held_reply_seq, (unsigned long)held_ms, reply);
                }
                else
                {
                    DEMO_LOG("TCP held reply for seq=%lu given up after %lu ms (recv %d, "
                           "errno %d)\n", held_reply_seq, (unsigned long)held_ms, received,
                           recv_errno);
                }
                close(held_reply_sock);
                held_reply_sock = -1;
                atc_resolve_held_reply(&atc, reply_arrived, reply_arrived ? reply : 0);
            }
        }

        fill_demo_payload(demo_payload, sizeof(demo_payload), seq);
        demo_packet_counter++;

        if (remote_cfg.use_adaptive_transport)
        {
            /* Mixed workload: draw this message's class and plan it with that class's budget.
             * Logged per message so the analysis can attribute each sequence number's delivery
             * outcome (from the receiver's log) to its class. */
            if (mix_classes > 0)
            {
                double u = (double)mmhal_random_u32(0, UINT32_MAX) / ((double)UINT32_MAX + 1.0);
                int cls = 0;

                while (cls < mix_classes - 1 && u >= mix_cumulative[cls])
                {
                    cls++;
                }
                atc.L = remote_cfg.mix_L[cls];
                atc.debt_class = cls;      /* per-class loss debt, see debt_per_class */
                DEMO_LOG("MIX: seq=%lu class=%d L=%g\n", seq, cls, atc.L);
            }

            enum atc_protocol protocol = atc_next_protocol(&atc);
            bool success;
            int received_udp_count = 0;
            /* Whether this send asked for a report, and whether one arrived (see
             * atc_report_result()); both stay false for UDP. */
            bool reply_requested = false;
            bool reply_received = false;
            /* Set when this send's report missed the in-line wait and its socket is now held
             * open -- reported via atc_report_tcp_reply_held() instead. */
            bool reply_held = false;

            if (protocol == ATC_PROTOCOL_TCP)
            {
                int since_last_tcp = atc_udp_since_last_tcp(&atc);
                /* The request byte: the window's UDP count, capped at the byte's range. */
                uint8_t expected_udp_count = (uint8_t)(since_last_tcp > 255 ? 255 : since_last_tcp);
                uint8_t reply_received_count = 0;
                /* Never request a second report while one is held: the receiver would reset its
                 * window on whichever request arrived first, mixing up the two windows. The UDP
                 * count simply keeps accumulating until the held report is resolved. */
                bool want_reply = atc_reply_wanted(&atc) && held_reply_sock < 0;
                int held_sock = -1;
                uint32_t send_start_ms = mmosal_get_time_ms();

                reply_requested = want_reply;
                success = send_tcp_demo_packet(demo_payload, sizeof(demo_payload),
                                               want_reply ? expected_udp_count : 0,
                                               &reply_received_count, want_reply,
                                               &reply_received,
                                               /* a held socket cannot receive across a module
                                                * power-down (DEMO_SLEEP_MODE) */
                                               !DEMO_SLEEP_MODE ? &held_sock : NULL, seq);
                if (held_sock >= 0)
                {
                    held_reply_sock = held_sock;
                    held_reply_since_ms = send_start_ms;
                    held_reply_seq = seq;
                    reply_held = true;
                }
                received_udp_count = reply_received_count;
            }
            else
            {
                success = send_udp_demo_packet(demo_payload, sizeof(demo_payload), seq);
            }

            if (reply_held)
            {
                atc_report_tcp_reply_held(&atc);
            }
            else
            {
                atc_report_result(&atc, protocol, success, reply_requested, reply_received,
                                  received_udp_count);
            }
        }
        else if (remote_cfg.packet_use_tcp)
        {
            uint8_t reply_received_count;
            bool reply_received;
            /* A fixed-TCP run sends no UDP: the request byte only has to be nonzero for the
             * receiver to send a report when the run waits for one (tcp_wait_reply). */
            send_tcp_demo_packet(demo_payload, sizeof(demo_payload),
                                 remote_cfg.tcp_wait_reply ? 1 : 0, &reply_received_count,
                                 remote_cfg.tcp_wait_reply, &reply_received, NULL, seq);
        }
        else
        {
            if (udp_sock < 0)
            {
                udp_sock = udp_connect(remote_cfg.udp_ack);
            }
            if (udp_sock >= 0 && remote_cfg.udp_ack)
            {
                send_udp_ack_demo_packet(udp_sock, demo_payload, sizeof(demo_payload), seq);
            }
            else
            {
                send_udp_demo_packet(demo_payload, sizeof(demo_payload), seq);
            }
        }

#if DEMO_SLEEP_MODE
        sleep_until(mmosal_get_time_ms() + remote_cfg.send_interval_ms);
#else
        mmosal_task_sleep(remote_cfg.send_interval_ms);
#endif
    }
}

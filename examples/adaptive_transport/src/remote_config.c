/*
 * Copyright 2026 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/sockets.h"

#include "remote_config.h"

/** Logging sink -- see the identical definition/rationale in adaptive_transport.c. Duplicated
 *  here rather than shared via a header because each of the three .c files in this demo is
 *  expected to be independently readable, and the definition is one line. */
#define DEMO_LOG(...) printf(__VA_ARGS__)

/** Request line sent to the server to trigger a config response (see remote_config.h). */
#define REMOTE_CONFIG_REQUEST "GET_CONFIG\n"

/** Marks the end of a config/confirmation message body (see remote_config.h): a blank line. */
#define REMOTE_CONFIG_TERMINATOR "\n\n"

/** Upper bound on the response/confirmation body; comfortably larger than the ~14 short lines the
 *  documented key set produces (MIX_L/MIX_SHARE included), with room to spare for future keys. */
#define REMOTE_CONFIG_RECV_BUF_LEN 768

/** True if @p value looks like a "truthy" wire value ("1", "true"/"yes"/"on", case-insensitive).
 *  Written to accept either the plain "0"/"1" server.py emits or a human-typed alternative, since
 *  this buffer is also handy to poke at manually (e.g. with netcat) while setting up an
 *  experiment. */
static bool remote_config_parse_bool(const char *value)
{
    if (strcasecmp(value, "true") == 0 || strcasecmp(value, "yes") == 0 ||
        strcasecmp(value, "on") == 0)
    {
        return true;
    }
    return atoi(value) != 0;
}

/** Parse a comma-separated list of floats ("0.5,0.1,0.01"; empty for none) into @p out.
 *
 * @return The number of values stored, at most @p max (extra values are ignored, with a log line).
 */
static int remote_config_parse_list(const char *value, double *out, int max)
{
    char buf[96];
    char *saveptr = NULL;
    char *item;
    int count = 0;

    snprintf(buf, sizeof(buf), "%s", value);
    for (item = strtok_r(buf, ",", &saveptr); item != NULL; item = strtok_r(NULL, ",", &saveptr))
    {
        if (count == max)
        {
            DEMO_LOG("Remote config: ignoring list entries beyond the first %d\n", max);
            break;
        }
        out[count++] = atof(item);
    }
    return count;
}

/** Render @p count values of @p values into @p buf as a comma-separated "%g" list (empty for
 *  none) -- the inverse of remote_config_parse_list(), for the confirmation body. */
static const char *remote_config_format_list(const double *values, int count, char *buf,
                                             size_t buf_len)
{
    size_t used = 0;
    int i;

    buf[0] = '\0';
    for (i = 0; i < count && used < buf_len; i++)
    {
        used += (size_t)snprintf(buf + used, buf_len - used, "%s%g", i ? "," : "", values[i]);
    }
    return buf;
}

/**
 * Apply one "KEY=VALUE" line to @p cfg.
 *
 * @return true if @p line's key was recognized (and therefore applied), false if it was ignored.
 */
static bool remote_config_apply_line(struct remote_config *cfg, char *line)
{
    char *eq = strchr(line, '=');
    const char *key;
    const char *value;

    if (eq == NULL)
    {
        return false;
    }
    *eq = '\0';
    key = line;
    value = eq + 1;

    if (strcmp(key, "DEMO_USE_ADAPTIVE_TRANSPORT") == 0)
    {
        cfg->use_adaptive_transport = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "DEMO_PACKET_USE_TCP") == 0)
    {
        cfg->packet_use_tcp = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "DEMO_TCP_WAIT_REPLY") == 0)
    {
        cfg->tcp_wait_reply = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "ATC_X") == 0)
    {
        cfg->atc_x = atoi(value);
    }
    else if (strcmp(key, "ATC_L") == 0)
    {
        cfg->atc_L = atof(value);
    }
    else if (strcmp(key, "ATC_N0") == 0)
    {
        cfg->atc_N0 = atoi(value);
    }
    else if (strcmp(key, "ATC_EPS") == 0)
    {
        cfg->atc_eps = atof(value);
    }
    else if (strcmp(key, "ATC_CALIBRATION_PACKETS") == 0)
    {
        cfg->atc_calibration_packets = atoi(value);
    }
    else if (strcmp(key, "ATC_USE_CHANGE_DETECTION") == 0)
    {
        cfg->atc_use_change_detection = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "ATC_DEBT_R") == 0)
    {
        cfg->atc_debt_horizon = atof(value);
    }
    else if (strcmp(key, "ATC_DEBT_PER_CLASS") == 0)
    {
        cfg->atc_debt_per_class = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "ATC_DEBT_TCP") == 0)
    {
        cfg->atc_debt_tcp = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "ATC_RANDOM_PROBE") == 0)
    {
        cfg->atc_random_probe = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "SEND_INTERVAL_MS") == 0)
    {
        cfg->send_interval_ms = (uint32_t)strtoul(value, NULL, 10);
    }
    else if (strcmp(key, "UDP_ACK") == 0)
    {
        cfg->udp_ack = remote_config_parse_bool(value);
    }
    else if (strcmp(key, "MIX_L") == 0)
    {
        cfg->mix_L_count = remote_config_parse_list(value, cfg->mix_L,
                                                    REMOTE_CONFIG_MAX_MIX_CLASSES);
    }
    else if (strcmp(key, "MIX_SHARE") == 0)
    {
        cfg->mix_share_count = remote_config_parse_list(value, cfg->mix_share,
                                                        REMOTE_CONFIG_MAX_MIX_CLASSES);
    }
    else
    {
        DEMO_LOG("Remote config: ignoring unrecognized key '%s'\n", key);
        return false;
    }

    return true;
}

/** Parse @p body (a mutable, NUL-terminated buffer; modified in place by strtok_r) into @p cfg.
 *
 * @return true if at least one recognized "KEY=VALUE" line was found and applied.
 */
static bool remote_config_parse(struct remote_config *cfg, char *body)
{
    bool applied = false;
    char *saveptr = NULL;
    char *line = strtok_r(body, "\r\n", &saveptr);

    while (line != NULL)
    {
        if (remote_config_apply_line(cfg, line))
        {
            applied = true;
        }
        line = strtok_r(NULL, "\r\n", &saveptr);
    }

    return applied;
}

/**
 * Render @p cfg's current values into @p buf as the confirmation body sent back to the server
 * (see remote_config.h): every recognized key, in the same "KEY=VALUE" format the server sends,
 * terminated by a blank line.
 *
 * @return The number of bytes written (excluding the terminating NUL), or a negative value on
 *         formatting failure.
 */
static int remote_config_format(const struct remote_config *cfg, char *buf, size_t buf_len)
{
    char mix_L[64];
    char mix_share[64];

    return snprintf(buf, buf_len,
                    "DEMO_USE_ADAPTIVE_TRANSPORT=%d\n"
                    "DEMO_PACKET_USE_TCP=%d\n"
                    "DEMO_TCP_WAIT_REPLY=%d\n"
                    "ATC_X=%d\n"
                    "ATC_L=%g\n"
                    "ATC_N0=%d\n"
                    "ATC_EPS=%g\n"
                    "ATC_CALIBRATION_PACKETS=%d\n"
                    "ATC_USE_CHANGE_DETECTION=%d\n"
                    "ATC_DEBT_R=%g\n"
                    "ATC_DEBT_PER_CLASS=%d\n"
                    "ATC_DEBT_TCP=%d\n"
                    "ATC_RANDOM_PROBE=%d\n"
                    "SEND_INTERVAL_MS=%lu\n"
                    "UDP_ACK=%d\n"
                    "MIX_L=%s\n"
                    "MIX_SHARE=%s\n"
                    "\n",
                    (int)cfg->use_adaptive_transport, (int)cfg->packet_use_tcp,
                    (int)cfg->tcp_wait_reply, cfg->atc_x, cfg->atc_L, cfg->atc_N0, cfg->atc_eps,
                    cfg->atc_calibration_packets, (int)cfg->atc_use_change_detection,
                    cfg->atc_debt_horizon, (int)cfg->atc_debt_per_class, (int)cfg->atc_debt_tcp,
                    (int)cfg->atc_random_probe,
                    (unsigned long)cfg->send_interval_ms, (int)cfg->udp_ack,
                    remote_config_format_list(cfg->mix_L, cfg->mix_L_count, mix_L, sizeof(mix_L)),
                    remote_config_format_list(cfg->mix_share, cfg->mix_share_count, mix_share,
                                              sizeof(mix_share)));
}

bool remote_config_fetch(struct remote_config *cfg, const char *server_ip, uint16_t port,
                         uint32_t timeout_ms)
{
    struct sockaddr_in dest_addr = {0};
    struct timeval recv_timeout = {0};
    int sock;
    static char recv_buf[REMOTE_CONFIG_RECV_BUF_LEN];
    int total = 0;
    bool applied;

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr(server_ip);

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0)
    {
        DEMO_LOG("Remote config: failed to create socket (errno %d); using hard-coded defaults\n",
               errno);
        return false;
    }

    recv_timeout.tv_sec = timeout_ms / 1000;
    recv_timeout.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
    {
        DEMO_LOG("Remote config: connect to %s:%u failed (errno %d); using hard-coded defaults\n",
               server_ip, port, errno);
        close(sock);
        return false;
    }

    if (send(sock, REMOTE_CONFIG_REQUEST, strlen(REMOTE_CONFIG_REQUEST), 0) <= 0)
    {
        DEMO_LOG("Remote config: request send to %s:%u failed (errno %d); using hard-coded "
               "defaults\n", server_ip, port, errno);
        close(sock);
        return false;
    }

    /* Read until the blank-line terminator (see remote_config.h) shows up in what's been received
     * so far, the timeout above elapses on an individual recv() (returns -1), the server closes
     * the connection early (returns 0), or the buffer is full -- whichever comes first. The
     * connection is deliberately kept open by the server past the end of this message (it's
     * still waiting on the confirmation below), so EOF is not a valid end-of-message signal here
     * the way it is for the fire-and-forget demo packet sends elsewhere in this application. */
    while (total < (int)sizeof(recv_buf) - 1)
    {
        int received = recv(sock, recv_buf + total, sizeof(recv_buf) - 1 - (size_t)total, 0);
        if (received <= 0)
        {
            break;
        }
        total += received;
        recv_buf[total] = '\0';
        if (strstr(recv_buf, REMOTE_CONFIG_TERMINATOR) != NULL)
        {
            break;
        }
    }

    if (total <= 0)
    {
        DEMO_LOG("Remote config: no response from %s:%u; using hard-coded defaults\n", server_ip,
               port);
        close(sock);
        return false;
    }

    recv_buf[total] = '\0';
    applied = remote_config_parse(cfg, recv_buf);
    DEMO_LOG("Remote config: %s from %s:%u (%d bytes)\n",
           applied ? "applied server-provided overrides" : "no recognized keys in response",
           server_ip, port, total);

    if (applied)
    {
        /* Confirm the final, merged configuration back to the server (see remote_config.h) so its
         * log records what the device is actually about to run with -- then close. recv_buf's
         * received content has already been consumed by the parse above, so it's reused here
         * rather than adding a second stack/static buffer. */
        int confirm_len = remote_config_format(cfg, recv_buf, sizeof(recv_buf));

        /* snprintf() returns how much it *would* have written, which can exceed buf_len if the
         * output was truncated -- clamp so a future larger config can never turn into an
         * out-of-bounds send() length here. In practice the formatted body is well under the
         * buffer size, so this never actually truncates. */
        if (confirm_len > (int)sizeof(recv_buf) - 1)
        {
            confirm_len = (int)sizeof(recv_buf) - 1;
        }

        if (confirm_len > 0 && send(sock, recv_buf, (size_t)confirm_len, 0) <= 0)
        {
            DEMO_LOG("Remote config: failed to send confirmation to %s:%u (errno %d)\n", server_ip,
                   port, errno);
        }
    }

    close(sock);
    return applied;
}

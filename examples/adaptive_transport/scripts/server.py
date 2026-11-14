"""The receiver of the adaptive transport firmware (examples/adaptive_transport).

  UDP 64015   the device's UDP messages (with UDP_ACK: each acknowledged with its 4-byte sequence
              number)
  TCP 64015   the device's TCP messages: one frame per message, a request byte N followed by the
              32-byte payload; if N > 0, the reply is one byte, the number of UDP messages
              received since the previous reply (the delivery report)
  TCP 64016   the remote configuration a device fetches at boot (see remote_config.h)

Every event is logged as one JSON line to server_events.<start time>.log, which the analysis
reads. The sequence number at the start of each payload is used for this log only, never for the
reports.

    python3 server.py [--remote-config '{"ATC_L": 0.05, ...}']
"""
import argparse
import asyncio
import json
import logging
import struct
from datetime import datetime

UDP_PORT = 64015
TCP_PORT = 64015
CONFIG_PORT = 64016

# Timeout (seconds) for the configuration query's request line and for the device's confirmation.
CONFIG_QUERY_TIMEOUT = 5.0

# One TCP frame: the request byte and the 32-byte payload (DEMO_PAYLOAD_LEN in adaptive_transport.c).
# TCP has no message boundaries, so frames are read with readexactly().
TCP_FRAME_LEN = 33

# The run settings handed to a device that queries CONFIG_PORT at boot (see remote_config.h for the
# keys). run_power_experiment.py sets them per run through --remote-config.
REMOTE_CONFIG = {
    'DEMO_USE_ADAPTIVE_TRANSPORT': True,
    'DEMO_PACKET_USE_TCP': True,
    'DEMO_TCP_WAIT_REPLY': True,
    'ATC_X': 50,
    'ATC_L': 0.025,
    'ATC_N0': 50,
    'ATC_EPS': 1e-9,
    'ATC_CALIBRATION_PACKETS': 10,
    'ATC_USE_CHANGE_DETECTION': True,
    # Loss debt (firmware defaults): horizon in messages (0 = off), one account per class.
    'ATC_DEBT_R': 250,
    'ATC_DEBT_PER_CLASS': True,
    # Charge locally failed TCP sends to the loss debt (firmware default off).
    'ATC_DEBT_TCP': False,
    # Probe as a 1/ATC_X floor on the UDP probability (firmware default); False = the fixed cycle.
    'ATC_RANDOM_PROBE': True,
    'SEND_INTERVAL_MS': 1000,
    # Mixed workload: comma-separated per-class L and share; '' disables it.
    'MIX_L': '',
    'MIX_SHARE': '',
    # Acknowledged UDP (single-protocol UDP, default off). The receiver switches with it: every UDP
    # datagram is answered with its 4-byte sequence number, and a retransmission (the same sequence
    # number again) is acknowledged again but counted only once.
    'UDP_ACK': False,
}

DEFAULT_LOG_LEVEL = "INFO"

logger = logging.getLogger("adaptive_transport_server")

# Per-sender-IP trackers.
trackers = {}

# A fresh log file per start of server.py, so that one log is one session.
current_logfile = "server_events.{}.log".format(datetime.now().strftime('%Y%m%d%H%M%S'))


def log_event(event_type, ip, data):
    """Log event to file in JSON format for later analysis."""
    event = {
        'timestamp': datetime.now().isoformat(),
        'event_type': event_type,
        'ip': ip,
        'data': data,
    }
    with open(current_logfile, 'a') as f:
        f.write(json.dumps(event) + '\n')


def parse_seq(payload):
    """The leading 4-byte big-endian sequence number of a payload, or None if it is too short."""
    if len(payload) < 4:
        return None
    return struct.unpack('!I', payload[:4])[0]


class SequenceTracker:
    """Per sender: the UDP messages received since the last report (what a report carries), and,
    for the log only, gaps in the sequence numbers."""

    def __init__(self):
        self.highest_seq = None
        self.udp_received = 0
        self.tcp_received = 0
        self.out_of_order = 0
        self.lost_total = 0
        self.udp_received_since_last_tcp_reply = 0

    def record(self, seq, protocol):
        """Record a received message. Returns the gap (messages presumed lost immediately before
        it, from the sequence numbers), or 0."""
        gap = 0
        if self.highest_seq is None:
            self.highest_seq = seq
        elif seq > self.highest_seq:
            gap = seq - self.highest_seq - 1
            self.highest_seq = seq
        else:
            self.out_of_order += 1

        if protocol == 'udp':
            self.udp_received += 1
            self.udp_received_since_last_tcp_reply += 1
        else:
            self.tcp_received += 1

        self.lost_total += gap
        return gap

    def take_tcp_reply_received(self):
        """Return and reset the number of UDP messages received since the last report: the report.
        A count of messages, not of sequence numbers, which the protocol does not use.

        The raw count, not a lost count: the device closes a window when it sends a request, the
        receiver when the request arrives, and a request delayed by TCP retransmissions can arrive
        after UDP of the next window has. Those are then counted into the earlier window, which can
        exceed its expected count; the raw count keeps the excess, so the device can merge the two
        windows, whose combined count is exact (adaptive_transport_controller.c,
        atc_resolve_held_reply())."""
        received = self.udp_received_since_last_tcp_reply
        self.udp_received_since_last_tcp_reply = 0
        return received

    @property
    def total_received(self):
        return self.udp_received + self.tcp_received

    @property
    def loss_rate(self):
        total_sent = self.total_received + self.lost_total
        return (self.lost_total / total_sent) if total_sent > 0 else 0.0

    def summary(self):
        return (
            f"received={self.total_received} (udp={self.udp_received}, tcp={self.tcp_received}) "
            f"lost={self.lost_total} out_of_order={self.out_of_order} "
            f"loss_rate={self.loss_rate:.2%}"
        )


def get_tracker(ip):
    if ip not in trackers:
        trackers[ip] = SequenceTracker()
    return trackers[ip]


class UDPServerProtocol:
    def connection_made(self, transport):
        self.transport = transport  # for UDP_ACK's acknowledgments

    def datagram_received(self, data, addr):
        ip = addr[0]

        if REMOTE_CONFIG['UDP_ACK'] and len(data) >= 4:
            self.transport.sendto(data[:4], addr)
            ack_seq = parse_seq(data)
            if get_tracker(ip).highest_seq == ack_seq:
                # Stop-and-wait: a retransmission repeats the latest sequence number.
                logger.info("UDP from %s: seq=%d retransmission, acknowledged again", ip, ack_seq)
                log_event('udp_duplicate', ip, {'seq': ack_seq})
                return

        seq = parse_seq(data)
        if seq is None:
            logger.warning("UDP from %s: payload too short for a sequence number (%d bytes)",
                           ip, len(data))
            return

        tracker = get_tracker(ip)
        gap = tracker.record(seq, 'udp')
        if gap:
            logger.warning("UDP from %s: gap detected, %d packet(s) presumed lost before seq=%d",
                           ip, gap, seq)
        logger.info("UDP from %s: seq=%d payload_size=%d | %s",
                    ip, seq, len(data), tracker.summary())
        log_event('udp_received', ip, {
            'seq': seq,
            'payload_size': len(data),
            'gap': gap,
        })


async def handle_tcp(reader, writer):
    peername = writer.get_extra_info('peername')
    ip = peername[0]
    while True:
        # Exactly one frame per iteration: a plain read() can return several coalesced frames.
        try:
            data = await reader.readexactly(TCP_FRAME_LEN)
        except asyncio.IncompleteReadError:
            break

        # The request byte (the UDP messages the device sent since its last request; 0 = no report
        # wanted), then the payload.
        expected_udp_count = data[0]
        payload = data[1:]

        seq = parse_seq(payload)
        if seq is None:
            logger.warning("TCP from %s: payload too short for a sequence number (%d bytes)",
                           ip, len(payload))
            continue

        tracker = get_tracker(ip)
        gap = tracker.record(seq, 'tcp')
        if gap:
            logger.warning("TCP from %s: gap detected, %d packet(s) presumed lost before seq=%d",
                           ip, gap, seq)
        logger.info("TCP from %s: seq=%d expected_udp=%d payload_size=%d | %s",
                    ip, seq, expected_udp_count, len(payload), tracker.summary())
        # A report only when one is requested: it resets the window, and without a request the
        # device does not wait for one.
        send_reply = expected_udp_count > 0
        log_event('tcp_received', ip, {
            'seq': seq,
            'expected_udp': expected_udp_count,
            'payload_size': len(payload),
            'gap': gap,
            'replied': send_reply,
        })

        if send_reply:
            reply_received = tracker.take_tcp_reply_received()
            writer.write(min(reply_received, 255).to_bytes(1, 'big'))
            await writer.drain()

    writer.close()
    await writer.wait_closed()


def print_final_summary():
    """Per-sender statistics, printed and logged when the receiver stops."""
    print("\n=== Packet loss summary ===")
    print(f"(current session log: {current_logfile})")
    if not trackers:
        print("No packets received.")
    for ip, tracker in trackers.items():
        print(f"{ip}: {tracker.summary()}")
        log_event('summary', ip, {
            'received': tracker.total_received,
            'udp_received': tracker.udp_received,
            'tcp_received': tracker.tcp_received,
            'lost_total': tracker.lost_total,
            'out_of_order': tracker.out_of_order,
            'loss_rate': tracker.loss_rate,
        })
    print("============================\n")


def format_remote_config(config):
    """`config` in the wire format of remote_config.c: one KEY=VALUE line per entry (bools as 0/1),
    then a blank line."""
    def fmt(value):
        if isinstance(value, bool):
            return '1' if value else '0'
        return str(value)
    return "".join(f"{key}={fmt(value)}\n" for key, value in config.items()) + "\n"


def parse_remote_config_body(text):
    """Parse the device's KEY=VALUE confirmation into a dict of strings (for the log)."""
    result = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or '=' not in line:
            continue
        key, _, value = line.partition('=')
        result[key] = value
    return result


async def handle_config(reader, writer):
    """Serve one configuration query: read the request line, send REMOTE_CONFIG, and log the
    configuration the device confirms it applied."""
    peername = writer.get_extra_info('peername')
    ip = peername[0]

    try:
        request = await asyncio.wait_for(reader.readline(), timeout=CONFIG_QUERY_TIMEOUT)
    except asyncio.TimeoutError:
        logger.warning("Config query from %s: no request received within %.0fs, closing",
                       ip, CONFIG_QUERY_TIMEOUT)
        writer.close()
        await writer.wait_closed()
        return

    body = format_remote_config(REMOTE_CONFIG)
    writer.write(body.encode())
    await writer.drain()
    logger.info("Config query from %s: sent %d parameter(s)", ip, len(REMOTE_CONFIG))
    log_event('config_query', ip, {
        'request': request.decode(errors='replace').strip(),
        'config': REMOTE_CONFIG,
    })

    try:
        confirmation = await asyncio.wait_for(reader.readuntil(b'\n\n'),
                                              timeout=CONFIG_QUERY_TIMEOUT)
        effective = parse_remote_config_body(confirmation.decode(errors='replace'))
        logger.info("Config query from %s: device confirmed effective config: %s", ip, effective)
        log_event('config_confirmed', ip, {'effective_config': effective})
    except (asyncio.TimeoutError, asyncio.IncompleteReadError):
        logger.warning("Config query from %s: no confirmation received (device may not have "
                       "applied the remote config)", ip)

    writer.close()
    await writer.wait_closed()


async def main():
    loop = asyncio.get_running_loop()
    logger.info("Listening for UDP on port %d", UDP_PORT)
    await loop.create_datagram_endpoint(
        lambda: UDPServerProtocol(),  # type: ignore
        local_addr=('0.0.0.0', UDP_PORT)
    )
    logger.info("Listening for TCP on port %d", TCP_PORT)
    tcp_server = await asyncio.start_server(handle_tcp, '0.0.0.0', TCP_PORT)
    logger.info("Listening for remote config queries on port %d", CONFIG_PORT)
    config_server = await asyncio.start_server(handle_config, '0.0.0.0', CONFIG_PORT)
    async with tcp_server, config_server:
        await asyncio.gather(tcp_server.serve_forever(), config_server.serve_forever())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Receiver of the adaptive transport firmware")
    parser.add_argument('--log-level', default=DEFAULT_LOG_LEVEL,
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                        help=f"console log verbosity (default: {DEFAULT_LOG_LEVEL})")
    parser.add_argument('--remote-config', default=None,
                        help="JSON object overriding REMOTE_CONFIG entries for this run, e.g. "
                             '\'{"ATC_L": 0.05, "DEMO_PACKET_USE_TCP": false}\'. Unknown keys are '
                             "rejected.")
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level, format='%(asctime)s %(levelname)-8s %(message)s')

    if args.remote_config:
        try:
            remote_config_overrides = json.loads(args.remote_config)
        except json.JSONDecodeError as exc:
            parser.error(f"--remote-config: invalid JSON: {exc}")
        unknown_keys = set(remote_config_overrides) - set(REMOTE_CONFIG)
        if unknown_keys:
            parser.error(f"--remote-config: unknown key(s) {sorted(unknown_keys)} -- known keys "
                         f"are {sorted(REMOTE_CONFIG)}")
        REMOTE_CONFIG.update(remote_config_overrides)
        logger.info("Remote config overridden from --remote-config: %s", REMOTE_CONFIG)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        print_final_summary()

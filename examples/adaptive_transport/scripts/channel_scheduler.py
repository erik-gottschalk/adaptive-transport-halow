"""
channel_scheduler.py

Cycles the AP's artificial packet-loss rate (via nft rules, over SSH) through a schedule of loss
levels, logging every change with a timestamp so it can be cross-referenced against
server_events.log and the devkit's timestamped UART console log (see mm_common.ConsoleLogger)
when analyzing how the adaptive transport controller reacts to changing channel conditions.

Authenticates to the AP with a key by default -- AP_KEY, pointing at the same dedicated
halow_lab_ed25519 key used for the collection server (see run_power_experiment.py's --server-key/
--ap-key) -- since AP firmware upgrades have disabled password auth entirely ("Bad authentication
type; allowed types: ['publickey']"). A password (ap_credentials.py, see that file -- it's
gitignored via **/ap_credentials.py, specifically so it never ends up in git history) is still
supported as a fallback for an AP that still accepts one: set --key '' (or AP_KEY = None) to force
password auth instead.

Usage:
    python channel_scheduler.py --host 192.168.1.1
    python channel_scheduler.py --host 192.168.1.1 --once     # run the schedule once, don't loop
    python channel_scheduler.py --host 192.168.1.1 --key ~/.ssh/some_other_key
"""
import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

import paramiko

try:
    from ap_credentials import AP_PASSWORD
except ImportError:
    AP_PASSWORD = None

# --- Configuration -------------------------------------------------------------------------

# The address you SSH to for the AP (i.e. `ssh root@<AP_HOST>`). Override with --host instead of
# editing this if you'd rather not hardcode it.
AP_HOST = "192.168.0.140"
AP_USER = "root"

# Private key file for authenticating to the AP. Takes priority over AP_PASSWORD when set (the
# default) -- see the module docstring for why. Set to None (or pass --key '') to force password
# auth instead, e.g. against an AP that hasn't had passwords disabled.
AP_KEY = str(Path.home() / ".ssh" / "halow_lab_ed25519")

NFT_TABLE = "inet losstest"
NFT_CHAIN = "pre"
NFT_CHAIN_SPEC = "{ type filter hook prerouting priority -300 ; }"

# Bursty (Gilbert-Elliott) loss instead of the nft rule's independent drops: BURST > 0 is the mean
# loss-run length in packets. The simple Gilbert model -- every packet in the bad state lost, none
# in the good -- with good->bad p = q / ((1 - q) * BURST) and bad->good r = 1 / BURST, so the
# long-run loss stays q (sim_opt.c's BURST, per packet instead of per second). Implemented with
# netem's gemodel on an IFB device that the HaLow and WAN ingress traffic to/from SERVER_IP is
# redirected through -- dropped on arrival at the AP, like the nft rule, but with ONE Markov chain
# shared by both directions, so a bad period hits uplink and downlink alike. Matching SERVER_IP
# keeps unrelated traffic (the office LAN on wan, SSH to the AP itself) from advancing the chain.
# Needs kmod-netem, kmod-ifb and tc-tiny on the AP (installed 2026-10-27).
BURST = 0
SERVER_IP = None
HALOW_IFACE = "wlan0"
WAN_IFACE = "wan"
IFB_IFACE = "ifb0"

# (duration_seconds, loss_percent) pairs, applied in order and repeated indefinitely (unless
# --once is passed), when the script runs on its own. run_power_experiment.py does not use it: it
# sets each phase's loss itself (set_loss_rate()).
SCHEDULE = [
    (10000, 20),
]

LOGFILE = "channel_conditions.log"
DEFAULT_LOG_LEVEL = "INFO"

logger = logging.getLogger("channel_scheduler")


def log_event(event_type, data):
    """Log event to file in JSON format, matching server.py's log_event() (same field names,
    same isoformat() timestamp) so entries from both scripts can be merged/sorted by timestamp
    during analysis."""
    event = {
        'timestamp': datetime.now().isoformat(),
        'event_type': event_type,
        'data': data,
    }
    with open(LOGFILE, 'a') as f:
        f.write(json.dumps(event) + '\n')


def run_remote_command(command, timeout=20):
    """Runs `command` on the AP over SSH -- key auth (AP_KEY) if set, password (AP_PASSWORD)
    otherwise -- returning (exit_status, stderr_text). Raises on connection failure -- callers
    should catch broadly, since paramiko surfaces auth/network failures as various exception
    types."""
    client = paramiko.SSHClient()
    # Lab/bench device with no stable known_hosts entry -- auto-accept its host key rather than
    # fail on an unknown/changed key. Equivalent to `ssh -o StrictHostKeyChecking=no`.
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        if AP_KEY:
            client.connect(AP_HOST, username=AP_USER, key_filename=str(Path(AP_KEY).expanduser()),
                           timeout=10, look_for_keys=False, allow_agent=False)
        else:
            client.connect(AP_HOST, username=AP_USER, password=AP_PASSWORD, timeout=10,
                           look_for_keys=False, allow_agent=False)
        _, stdout, stderr = client.exec_command(command, timeout=timeout)
        exit_status = stdout.channel.recv_exit_status()
        stderr_text = stderr.read().decode(errors='replace')
        return exit_status, stderr_text
    finally:
        client.close()


def set_loss_rate(percent):
    """Applies the given artificial packet loss rate (0-99) on the AP over SSH, replacing
    whatever rule was previously in the chain. percent=0 just flushes the chain (no drop rule ->
    no artificial loss). Returns True on success.

    Always (re-)ensures the table/chain exist first via `nft add table`/`nft add chain` -- unlike
    `nft create`, `add` is a no-op if the table/chain is already there, so this costs nothing on
    the common path. It matters because an AP reboot flushes its entire nftables ruleset (table
    and chain included, not just the rules in it), which otherwise makes the `nft flush chain ...`
    below fail with "No such file or directory" the first time this runs after any AP restart.
    """
    ensure_table_cmd = f"nft add table {NFT_TABLE}"
    ensure_chain_cmd = f"nft add chain {NFT_TABLE} {NFT_CHAIN} '{NFT_CHAIN_SPEC}'"
    flush_cmd = f"nft flush chain {NFT_TABLE} {NFT_CHAIN}"
    nft_cmd = f"{ensure_table_cmd} && {ensure_chain_cmd} && {flush_cmd}"
    # Every call starts from no loss of either kind, so switching between the nft and the
    # Gilbert-Elliott mode never leaves the other one active.
    remote_cmd = f"{ge_teardown_cmd()} ; {nft_cmd}"
    if percent > 0 and BURST > 0:
        remote_cmd += f" && {ge_setup_cmd(percent / 100.0)}"
    elif percent > 0:
        # The '<' has to be backslash-escaped for the remote shell (same as when typed
        # interactively at the AP's prompt), since it's otherwise parsed as I/O redirection.
        add_cmd = (f"nft add rule {NFT_TABLE} {NFT_CHAIN} meta l4proto {{ tcp, udp }} "
                   f"numgen random mod 100 \\< {percent} drop")
        remote_cmd += f" && {add_cmd}"

    logger.debug("Running on %s@%s: %s", AP_USER, AP_HOST, remote_cmd)
    try:
        exit_status, stderr_text = run_remote_command(remote_cmd)
    except Exception as e:  # noqa: BLE001 -- paramiko raises many distinct exception types here
        logger.error("SSH connection/command failed setting loss rate to %d%%: %s", percent, e)
        log_event('loss_rate_change_failed', {'requested_percent': percent, 'error': str(e)})
        return False

    if exit_status != 0:
        logger.error("Failed to set loss rate to %d%%: %s", percent, stderr_text.strip())
        log_event('loss_rate_change_failed', {
            'requested_percent': percent,
            'error': stderr_text.strip(),
        })
        return False

    if percent > 0 and BURST > 0:
        logger.info("Channel condition set: %d%% artificial loss in runs of %g packets",
                    percent, BURST)
        log_event('loss_rate_changed', {'percent': percent, 'burst': BURST})
    else:
        logger.info("Channel condition set: %d%% artificial loss", percent)
        log_event('loss_rate_changed', {'percent': percent})
    return True


def ge_params(q, burst):
    """Gilbert transition probabilities (p good->bad, r bad->good) for long-run loss q in runs
    of mean length burst packets."""
    return q / ((1.0 - q) * burst), 1.0 / burst


def ge_teardown_cmd():
    """Removes the Gilbert-Elliott setup; a no-op (and harmless without tc) if it isn't there."""
    return (f"{{ tc qdisc del dev {HALOW_IFACE} ingress ; tc qdisc del dev {WAN_IFACE} ingress ; "
            f"tc qdisc del dev {IFB_IFACE} root ; }} 2>/dev/null ; true")


def ge_setup_cmd(q):
    if not SERVER_IP:
        raise ValueError("Gilbert-Elliott loss needs SERVER_IP (the collection server's address)")
    p, r = ge_params(q, BURST)
    cmds = [f"{{ ip link show {IFB_IFACE} >/dev/null 2>&1 || ip link add {IFB_IFACE} type ifb ; }}",
            f"ip link set {IFB_IFACE} up",
            f"tc qdisc add dev {IFB_IFACE} root netem loss gemodel {100 * p:.4f}% {100 * r:.4f}%"]
    for dev, direction in ((HALOW_IFACE, "dst"), (WAN_IFACE, "src")):
        cmds.append(f"tc qdisc add dev {dev} ingress")
        for proto in (6, 17):                # tcp, udp -- as the nft rule's l4proto set
            cmds.append(f"tc filter add dev {dev} parent ffff: protocol ip prio 1 u32 "
                        f"match ip {direction} {SERVER_IP}/32 match ip protocol {proto} 0xff "
                        f"action mirred egress redirect dev {IFB_IFACE}")
    return " && ".join(cmds)


def run_schedule(schedule, repeat):
    cycle = 0
    while True:
        cycle += 1
        logger.info("=== Schedule cycle %d ===", cycle)
        log_event('schedule_cycle_start', {'cycle': cycle})
        for duration_s, percent in schedule:
            if not set_loss_rate(percent):
                logger.warning("Continuing schedule despite failed rate change")
            logger.info("Holding %d%% loss for %ds", percent, duration_s)
            time.sleep(duration_s)
        if not repeat:
            break


def main():
    global AP_HOST, AP_USER, AP_KEY

    parser = argparse.ArgumentParser(
        description="Cycle AP artificial packet loss through a schedule, logging each change "
                    "for later cross-reference with server_events.log."
    )
    parser.add_argument('--host', default=AP_HOST, help=f"AP SSH host (default: {AP_HOST})")
    parser.add_argument('--user', default=AP_USER, help=f"AP SSH user (default: {AP_USER})")
    parser.add_argument('--key', default=AP_KEY,
                        help=f"private key for AP auth, takes priority over ap_credentials.py's "
                             f"password (default: {AP_KEY}); pass --key '' to force password auth")
    parser.add_argument('--log-level', default=DEFAULT_LOG_LEVEL,
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                        help=f"console log verbosity (default: {DEFAULT_LOG_LEVEL})")
    parser.add_argument('--once', action='store_true',
                        help="run through the schedule once instead of repeating indefinitely")
    parser.add_argument('--no-reset-on-exit', dest='reset_on_exit', action='store_false',
                        default=True,
                        help="don't reset loss rate to 0%% on exit (default: reset)")
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level, format='%(asctime)s %(levelname)-8s %(message)s')

    AP_HOST = args.host
    AP_USER = args.user
    AP_KEY = args.key or None

    if AP_HOST == "CHANGE_ME":
        logger.error("AP_HOST is not configured -- set it at the top of this script, or pass "
                     "--host <address>")
        sys.exit(1)

    if AP_KEY:
        if not Path(AP_KEY).expanduser().exists():
            logger.error("AP key not found: %s -- pass --key <path>, or --key '' to fall back to "
                         "ap_credentials.py's password", AP_KEY)
            sys.exit(1)
    else:
        if AP_PASSWORD is None:
            logger.error("no AP_KEY set and ap_credentials.py not found -- create it next to "
                         "this script with AP_PASSWORD = \"your-password-here\" (it's gitignored "
                         "so it's safe to leave your real password there), or pass --key <path> "
                         "for key auth instead")
            sys.exit(1)
        if AP_PASSWORD == "CHANGE_ME":
            logger.error("AP_PASSWORD in ap_credentials.py is still the placeholder -- fill in "
                         "your real password")
            sys.exit(1)

    logger.info("Target: %s@%s, schedule: %s, repeat=%s", AP_USER, AP_HOST, SCHEDULE, not args.once)

    try:
        run_schedule(SCHEDULE, repeat=not args.once)
    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        if args.reset_on_exit:
            logger.info("Resetting channel to 0%% loss on exit")
            set_loss_rate(0)


if __name__ == '__main__':
    main()

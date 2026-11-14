"""
mm_common.py

Shared plumbing for run_power_experiment.py: the receiver's SSH lifecycle (RemoteServer), the
device reset over SWD (OpenOCD/STLink, reset_device()), the UART console capture (ConsoleLogger),
the AP loss schedule format, and the default controller parameters.
"""
import datetime as dt
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
APP_DIR = HERE.parent                              # examples/adaptive_transport
FRAMEWORK_ROOT = HERE.parents[2] / "framework"      # mm-iot-sdk/framework

CHANNEL_LOG = HERE / "channel_conditions.log"
EVAL = HERE / "eval"

DEFAULT_PLATFORM = "mm-mm8108-ekh05"
DEFAULT_AP_HOST = "192.168.0.140"
DEFAULT_SERVER_DIR = "~/adapt"
DEFAULT_CONSOLE_BAUD = 115200

# The controller's default parameters (adaptive_transport_controller.h, ATC_DEFAULT_*).
BASELINE_ATC = {"x": "50", "L": "0.025", "N0": "50", "eps": "1e-9",
                "calibration_packets": "10", "use_change_detection": "true"}


def log(msg, level="info"):
    stamp = dt.datetime.now().strftime("%H:%M:%S")
    prefix = {"info": "  ", "step": "==", "warn": "!!", "err": "XX"}[level]
    print(f"[{stamp}] {prefix} {msg}", flush=True)


def git_info():
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=HERE,
                                capture_output=True, text=True).stdout.strip() or "?"
        status = subprocess.run(["git", "status", "--porcelain"], cwd=HERE,
                                capture_output=True, text=True).stdout.strip()
        return commit, ("dirty" if status else "clean")
    except Exception:
        return "?", "?"


# ---------------------------------------------------------------------------------------------
# AP loss schedule
# ---------------------------------------------------------------------------------------------


def parse_schedule(text):
    """'600:20,3000:1' -> [(600, 20), (3000, 1)]"""
    out = []
    for part in text.split(","):
        secs, pct = part.split(":")
        out.append((int(secs), int(pct)))
    if not out:
        raise ValueError("empty schedule")
    return out


# ---------------------------------------------------------------------------------------------
# DUT reset (OpenOCD/STLink)
# ---------------------------------------------------------------------------------------------


def openocd_cfg_path(platform):
    return FRAMEWORK_ROOT / "src" / "platforms" / platform / "openocd.cfg"


# Fallback locations for the openocd binary, tried if it's not on PATH -- see openocd_binary()'s
# docstring for why PATH alone isn't reliable here.
OPENOCD_FALLBACK_PATHS = [
    "/opt/morse/xpack-openocd-0.12.0-2/bin/openocd",
]


def openocd_binary():
    """Resolve the openocd executable to run: the OPENOCD environment variable if set, else
    whatever 'openocd' resolves to on PATH, else OPENOCD_FALLBACK_PATHS. Needed because on at
    least one dev machine, openocd's bin directory is only added to PATH by an interactive-shell
    guarded block in .bashrc ('case $- in *i*) ... ;; esac') -- a script invoked non-interactively
    (a wrapper .sh run from cron, an IDE task runner, etc.) inherits a PATH without it even though
    `which openocd` works fine at an interactive prompt."""
    import os
    import shutil
    env = os.environ.get("OPENOCD")
    if env:
        return env
    found = shutil.which("openocd")
    if found:
        return found
    for candidate in OPENOCD_FALLBACK_PATHS:
        if Path(candidate).exists():
            return candidate
    return "openocd"          # nothing found -- let subprocess raise a clear error


def reset_device(openocd_cfg, stlink_serial=None, dry_run=False, timeout=30):
    """Restart the already-flashed firmware via OpenOCD/STLink SRST, without touching flash --
    the equivalent of the Makefile's `reprogram` target's trailing `... reset exit`, minus the
    `program`/`verify` steps. The Power Profiler only measures (it does not power the devkit, see
    run_power_experiment.py's module docstring) and the console UART has no reset line, so this is
    how each capture starts from a fresh boot.
    """
    cmd = [openocd_binary()]
    if stlink_serial:
        cmd += ["-c", f"set SERIAL_NO {stlink_serial}"]
    cmd += ["-f", str(openocd_cfg), "-c", "init", "-c", "reset run", "-c", "exit"]
    log("resetting DUT: " + " ".join(cmd))
    if dry_run:
        return
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(f"openocd reset failed (exit {r.returncode}):\n{r.stderr.strip()}")


# ---------------------------------------------------------------------------------------------
# UART console capture
# ---------------------------------------------------------------------------------------------


class ConsoleLogger(threading.Thread):
    """Capture the devkit's UART console to a file, each line prefixed with an ISO timestamp (the
    format the analysis parses). Resetting the device is a separate OpenOCD/STLink step (see
    reset_device()) done by the caller.

    Start this *before* calling reset_device() so the port is already open and draining when the
    firmware's boot banner appears -- opening the port after reset risks missing the first lines.
    """

    def __init__(self, port, baud, path):
        super().__init__(daemon=True)
        self.port, self.baud, self.path = port, baud, path
        self._stop_evt = threading.Event()
        self.lines = 0
        self.error = None

    def run(self):
        import serial
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser, \
                    open(self.path, "w", encoding="utf-8", errors="replace") as fh:
                ser.reset_input_buffer()
                buf = b""
                while not self._stop_evt.is_set():
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("utf-8", "replace").rstrip("\r")
                        ts = dt.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
                        fh.write(f"{ts} {line}\n")
                        self.lines += 1
                    fh.flush()
        except Exception as exc:                      # surfaced by the caller, not swallowed
            self.error = exc

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=5)


# ---------------------------------------------------------------------------------------------
# remote server
# ---------------------------------------------------------------------------------------------


class RemoteServer:
    """server.py running on the receiver host, controlled over SSH."""

    def __init__(self, host, user, key, directory, python="python3", port=22, dry_run=False):
        self.host, self.user, self.key = host, user, key
        self.dir, self.python, self.port = directory, python, port
        self.dry_run = dry_run
        self.client = None
        self.pid = None
        self.logname = "server_events.log"
        self.start_epoch = 0

    def _reconnect_if_dropped(self):
        transport = self.client.get_transport() if self.client else None
        if transport is not None and transport.is_active():
            return
        log("SSH session is no longer active -- reconnecting", "warn")
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
        self.connect()

    def _run(self, cmd, check=True, timeout=30):
        log(f"ssh {self.user}@{self.host}: {cmd}")
        if self.dry_run:
            return 0, "", ""
        if self.client is None:
            raise RuntimeError("connect() must be called before running remote commands")
        self._reconnect_if_dropped()
        stdin, out, _ = self.client.exec_command(cmd, timeout=timeout)
        stdin.close()
        chan = out.channel
        deadline = time.time() + timeout
        while not chan.exit_status_ready():
            if time.time() > deadline:
                chan.close()
                raise TimeoutError(
                    f"remote command still running after {timeout}s: {cmd}\n"
                    "If it backgrounds a process, redirect its stdin from /dev/null too -- "
                    "otherwise the process inherits the SSH channel and it never closes.")
            time.sleep(0.1)
        rc = chan.recv_exit_status()

        def drain(ready, recv, first_wait=1.5):
            data = b""
            deadline = time.time() + first_wait
            while len(data) < (1 << 20):
                if ready():
                    chunk = recv(65536)
                    if not chunk:
                        break
                    data += chunk
                elif data or time.time() > deadline:
                    break
                else:
                    time.sleep(0.05)
            return data.decode(errors="replace")

        so = drain(chan.recv_ready, chan.recv)
        se = drain(chan.recv_stderr_ready, chan.recv_stderr, first_wait=0.15)
        chan.close()
        if check and rc != 0:
            raise RuntimeError(f"remote command failed (rc={rc}): {cmd}\n{se.strip()}")
        return rc, so, se

    def connect(self):
        log(f"connecting to server {self.user}@{self.host}", "step")
        if self.dry_run:
            return
        import paramiko
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(self.host, port=self.port, username=self.user,
                            key_filename=str(Path(self.key).expanduser()), timeout=20,
                            look_for_keys=False, allow_agent=False)
        transport = self.client.get_transport()
        if transport is not None:
            transport.set_keepalive(30)

    def start(self, remote_config=None):
        import json
        import shlex
        stamp = dt.datetime.now().strftime("%Y%m%d%H%M%S")
        _, epoch, _ = self._run("date +%s")
        self.start_epoch = int(epoch.strip()) if not self.dry_run else 0
        self._run(f"cd {self.dir} && [ -f {self.logname} ] && "
                  f"mv {self.logname} server_events.pre{stamp}.log || true", check=False)
        # A server.py left over from a previous crashed/interrupted run can still hold the UDP
        # port, making the fresh process below die immediately with "Address already in use" --
        # clear any leftovers before binding a new one.
        self._run("pkill -9 -f 'server\\.py' 2>/dev/null || true", check=False)
        time.sleep(0.5)
        remote_config_arg = ""
        if remote_config is not None:
            remote_config_arg = " --remote-config " + shlex.quote(json.dumps(remote_config))
            log(f"remote config for devices querying this server: {remote_config}")
        _, out, _ = self._run(
            f"cd {self.dir} && nohup {self.python} -u server.py{remote_config_arg} "
            f"> server_stdout.{stamp}.log 2>&1 < /dev/null & echo $!")
        self.pid = out.strip() if not self.dry_run else "<pid>"
        log(f"server.py started, pid {self.pid}")
        time.sleep(2)
        rc, _, _ = self._run(f"kill -0 {self.pid}", check=False)
        if rc != 0 and not self.dry_run:
            _, tail, _ = self._run(f"cd {self.dir} && tail -20 server_stdout.{stamp}.log",
                                   check=False)
            raise RuntimeError(f"server.py exited immediately:\n{tail}")

    def stop(self):
        log("stopping server.py", "step")
        if self.pid:
            self._run(f"kill -INT {self.pid} 2>/dev/null || true", check=False)
            time.sleep(3)
            self._run(f"kill -9 {self.pid} 2>/dev/null || true", check=False)
        else:
            log("no PID recorded (start() did not complete) -- falling back to pkill", "warn")
        self._run("pkill -INT -f 'server\\.py' 2>/dev/null || true", check=False)
        time.sleep(2)
        self._run("pkill -9 -f 'server\\.py' 2>/dev/null || true", check=False)
        _, out, _ = self._run("pgrep -af 'server\\.py' || echo none", check=False)
        log(f"remaining server.py processes: {out.strip() or 'none'}")

    def fetch_log(self, dest):
        log("locating the server log(s) for this run", "step")
        if self.dry_run:
            dest.write_text("(dry-run placeholder)\n", encoding="utf-8")
            return 0
        _, listing, _ = self._run(
            f"cd {self.dir} && for f in server_events*.log; do "
            f"[ -f \"$f\" ] && echo \"$(stat -c %Y \"$f\") $f\"; done", check=False)
        mine = []
        for line in listing.strip().splitlines():
            try:
                mtime, name = line.split(None, 1)
                mtime = int(mtime)
            except ValueError:
                continue
            if ".pre" in name:
                continue
            if mtime >= self.start_epoch - 5:
                mine.append((mtime, name.strip()))
        mine.sort()
        if not mine:
            raise RuntimeError(f"no server log written since the run started in {self.dir}")
        log("  " + ", ".join(f"{n} (mtime +{m - self.start_epoch}s)" for m, n in mine))

        sftp = self.client.open_sftp()
        try:
            def pull(name, target):
                remote = f"{self.dir.rstrip('/')}/{name}"
                if remote.startswith("~"):
                    remote = remote.replace("~", ".", 1)
                sftp.get(remote, str(target))

            pull(mine[-1][1], dest)
            log(f"  run log: {mine[-1][1]} -> {dest.name}")
            for i, (_, name) in enumerate(mine[:-1]):
                side = dest.with_name(f"server_events.prereset{i or ''}.log")
                pull(name, side)
                log(f"  pre-reset traffic: {name} -> {side.name}")
        finally:
            sftp.close()
        return dest.stat().st_size

    def close(self):
        if self.client:
            self.client.close()

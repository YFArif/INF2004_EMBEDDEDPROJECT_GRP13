"""S1 laptop dashboard: START, STOP and RESET, their acks, the heartbeat
and the robot's telemetry (S2 motion, S3 line, S4 IMU, S5 state).

Serves one page on http://127.0.0.1:8765/ (this machine only) and pushes
live updates to it with server-sent events. It reaches the broker through
mosquitto_sub (one for the whole run) and mosquitto_pub (one per command),
and reuses the message code that is already tested: cmd_rtt.command_bytes()
packs the commands (S1-U06 helper) and s1_decode decodes everything the
robot sends (S1-U11).

Mission commands only, never teleoperation. Commands go to
<prefix>/cmd/dash at QoS 1 and are never retained: lwIP does not report the
retain flag, so the robot would run a retained command again after every
reconnect (docs/MQTT.md section 1.1). An ack with result 0 means "queued
for S5", not carried out: S5's verdict is on the robot's console only
until the event channel is built (the S5 panel shows the state change).

The telemetry panels read two messages on <prefix>/telemetry: the 2 Hz
telemetry frame (type 1) and the 5 Hz live message (type 6), both defined
in docs/S1_binary_frame.md. A panel greys out when its messages stop.

The broker host, port and prefix come from app_program/subsystem1/
mqtt_config.h (git-ignored) or from --host, --port and --prefix. The page,
the log and the console never show the broker host or the login: every
text passes through one scrubber first, and a login reaches the mosquitto
clients in a mode-0600 options file, never as an argument.

Run from the repository root (docs/S1_dashboard.md is the full guide):

    python3 -I -B testing/subsystem1/s1_dashboard.py
    python3 -I -B testing/subsystem1/s1_dashboard.py --simulate
    python3 -I -B testing/subsystem1/s1_dashboard.py --check

--simulate needs no broker: a fake car answers with S5's command rules
and telemetry_task's acks, heartbeat, telemetry and live message.
--check is test S1-U16 part A;
it exits 0 PASS, 1 FAIL, 2 INCOMPLETE (steps not built yet).
"""

import argparse
import collections
import contextlib
import datetime
import heapq
import http.client
import http.server
import io
import json
import math
import os
import queue
import random
import re
import secrets
import shutil
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

# python3 -I leaves the script's folder off sys.path, so both folders are
# added by name: cmd_rtt.py sits next to this file, s1_decode.py in the
# subsystem's dashboard folder.
sys.path.insert(0, os.path.join(REPO, "app_program", "subsystem1",
                                "dashboard"))
sys.path.insert(0, HERE)

import cmd_rtt  # noqa: E402  (needs the path set-up above)
import s1_decode  # noqa: E402

DEFAULT_CONFIG = os.path.join(REPO, "app_program", "subsystem1",
                              "mqtt_config.h")
BIND_ADDRESS = "127.0.0.1"      # Never configurable: the page sends commands.
HTTP_PORT = 8765
ACK_TIMEOUT_S = 2.0
HB_LIMIT_S = 3.0                # The light turns red after this long.
BROKER_LIMIT_S = 12.0           # Two $SYS intervals of 5 s, plus 2 s.
TICK_S = 0.25
KEEPALIVE_S = 15.0
CMD_SUFFIX = "cmd/dash"
LOG_KEEP = 200
COMMANDS_KEEP = 20
RECENT_KEEP_S = 30.0            # How long a late or duplicate ack is known.
TIMELINE_KEEP = 100             # S3 timeline entries kept for a new page.
IMU_KEEP = 120                  # Pitch and roll points: 60 s at 2 Hz.
# The largest believable change of an S3 counter between two live messages
# (200 ms apart). A bigger one, or a live seq gap above SEQ_GAP_MAX, means
# the robot restarted or many messages were lost, so the timeline starts
# again instead of showing a burst.
# NOTE: A counter that was close below its wrap reads as a small change when
# a restart sets it to 0: rejects 13 to 15 give 3, 2 or 1 (barcodes, junctions
# and losts: 252 to 255). The seq gap catches that unless the live seq was
# also within SEQ_GAP_MAX of its own wrap. Then one wrong timeline entry can
# appear before the heartbeat shows the restart.
COUNTER_JUMP_MAX = {"barcodes": 4, "junctions": 4, "losts": 4, "rejects": 3}
SEQ_GAP_MAX = 25                # About 5 s of live messages.
TEL_DROP_MAX = 255              # tel_drop is 8 bits and clamps here.
RETAINED_TEST_SUFFIX = "cmd/test"
SUB_FORMAT = "@Y-@m-@dT@H:@M:@S.@N %r %t %x"
LABELS = {0: "queued", 1: "rejected"}
LOOPBACK = ("127.0.0.1", "localhost", "::1")
PAGE_CSP = ("default-src 'none'; script-src 'unsafe-inline'; "
            "style-src 'unsafe-inline'; connect-src 'self'; img-src 'self'; "
            "frame-ancestors 'none'; base-uri 'none'; form-action 'none'")

CONFIG_PATTERNS = {
    "host": r'MQTT_LINK_CFG_BROKER_HOST\s+"([^"]*)"',
    "port": r'MQTT_LINK_CFG_BROKER_PORT\s+\(?\s*(\d+)\s*[uU]?\s*\)?',
    "prefix": r'MQTT_LINK_CFG_TOPIC_PREFIX\s+"([^"]*)"',
    "user": r'MQTT_LINK_CFG_USERNAME\s+"([^"]*)"',
    "password": r'MQTT_LINK_CFG_PASSWORD\s+"([^"]*)"',
}


def now_text():
    """This machine's local time, ISO, to the millisecond."""
    return datetime.datetime.now().isoformat(timespec="milliseconds")


def age_ms(now, then):
    """Milliseconds since a monotonic time stamp, or None if never."""
    if then is None:
        return None
    return int((now - then) * 1000.0)


def tel_drop_text(value):
    """tel_drop for the page: at its clamp, the true count may be higher."""
    if value >= TEL_DROP_MAX:
        return "%d or more" % TEL_DROP_MAX
    return str(value)


def read_config(path):
    """Return the broker settings found in mqtt_config.h (never printed)."""
    found = {}
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return found
    for key, pattern in CONFIG_PATTERNS.items():
        match = re.search(r"^\s*#\s*define\s+" + pattern, text, re.MULTILINE)
        if match is not None:
            value = match.group(1)
            found[key] = int(value) if key == "port" else value
    return found


class Settings:
    """Broker settings for one run. host, user and password are secret."""

    def __init__(self, host, port, prefix, user, password, source):
        self.host = host
        self.port = port
        self.prefix = prefix
        self.user = user
        self.password = password
        self.source = source


def settings_from(args):
    """Merge the command line over mqtt_config.h; exit on a bad value."""
    cfg = read_config(args.config)
    host = args.host if args.host else cfg.get("host")
    port = args.port if args.port else cfg.get("port", 1883)
    prefix = args.prefix if args.prefix else cfg.get("prefix")
    if args.simulate:
        source = "simulate"
    elif args.host:
        source = "command line"
    else:
        source = "mqtt_config.h"
    if not args.simulate and not host:
        sys.exit("s1_dashboard: no broker host: create "
                 "app_program/subsystem1/mqtt_config.h from "
                 "mqtt_config.example.h, or give --host")
    if prefix is None:
        prefix = "robot" if args.simulate else None
    if not prefix:
        sys.exit("s1_dashboard: no topic prefix: set "
                 "MQTT_LINK_CFG_TOPIC_PREFIX or give --prefix")
    if (("#" in prefix) or ("+" in prefix) or prefix.startswith("/")
            or prefix.endswith("/")):
        sys.exit("s1_dashboard: the prefix must not hold # or + or start "
                 "or end with /")
    if not 1 <= port <= 65535:
        sys.exit("s1_dashboard: the port must be 1 to 65535")
    return Settings(host, port, prefix, cfg.get("user", ""),
                    cfg.get("password", ""), source)


class Scrubber:
    """Replace the broker host and the login with placeholders.

    A loopback host is left alone: it says nothing about the network, and
    the dashboard's own address 127.0.0.1 must stay readable.

    NOTE: Scrub the values, then serialize. Replacing text in serialized JSON
    would turn a number that holds a digit-only login into invalid JSON.
    """

    def __init__(self, settings):
        pairs = [(settings.host, "<broker>"), (settings.user, "<user>"),
                 (settings.password, "<password>")]
        self._pairs = sorted([(s, tag) for s, tag in pairs
                              if s and (s not in LOOPBACK)],
                             key=lambda pair: -len(pair[0]))

    def text(self, value):
        for secret, tag in self._pairs:
            value = value.replace(secret, tag)
        return value

    def obj(self, value):
        """Scrub every string, keys included, inside a JSON-like value."""
        if isinstance(value, str):
            return self.text(value)
        if isinstance(value, dict):
            return {self.obj(key): self.obj(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self.obj(item) for item in value]
        return value


def write_options(settings, opts_dir):
    """Put the login in a mode-0600 mosquitto options file, or return None.

    The clients read it with -o, so the login never shows in the argument
    list (ps). The caller removes the file when the run ends.
    """
    if not (settings.user or settings.password):
        return None
    handle, path = tempfile.mkstemp(prefix="s1dash-opts-", dir=opts_dir)
    os.fchmod(handle, stat.S_IRUSR | stat.S_IWUSR)
    lines = []
    if settings.user:
        lines.append("-u " + settings.user)
    if settings.password:
        lines.append("-P " + settings.password)
    os.write(handle, ("\n".join(lines) + "\n").encode("utf-8"))
    os.close(handle)
    return path


def broker_args(settings, opts_path):
    argv = ["-h", settings.host or "127.0.0.1", "-p", str(settings.port)]
    if opts_path is not None:
        argv += ["-o", opts_path]
    return argv


def pub_args(settings, opts_path):
    """mosquitto_pub for one command: QoS 1, payload on stdin, no retain."""
    return (["mosquitto_pub"] + broker_args(settings, opts_path)
            + ["-q", "1", "-t", settings.prefix + "/" + CMD_SUFFIX, "-s"])


def check_no_retain(argv):
    """Refuse to run a publish that would leave a retained command."""
    if ("-r" in argv) or ("--retain" in argv):
        raise RuntimeError("refusing to publish a retained command")


def sub_topics(prefix):
    return [prefix + "/status", prefix + "/telemetry", prefix + "/ack",
            prefix + "/event", prefix + "/cmd/#", "$SYS/broker/uptime"]


def sub_args(settings, opts_path, b_v5):
    argv = (["mosquitto_sub"] + broker_args(settings, opts_path)
            + ["-q", "1", "-I", "s1dash-"])
    if b_v5:
        # MQTT v5 keeps the retain flag on a message that arrives while we
        # listen; MQTT 3.1.1 clears it (section 3.3.1.3).
        argv += ["-V", "mqttv5", "--retain-as-published"]
    for topic in sub_topics(settings.prefix):
        argv += ["-t", topic]
    return argv + ["-F", SUB_FORMAT]


def banner_supported():
    """True if this mosquitto_sub knows -V mqttv5 --retain-as-published."""
    try:
        done = subprocess.run(["mosquitto_sub", "-V", "mqttv5",
                               "--retain-as-published", "--help"],
                              capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return False
    # A known option set prints the help text; an unknown one, "Error:".
    return "Error:" not in (done.stdout + done.stderr)


def parse_sub_line(line):
    """Split one SUB_FORMAT line into (when, retained, topic, payload)."""
    parts = line.rstrip("\r\n").split(" ")
    if len(parts) < 4:
        return None
    stamp, retained, hex_text = parts[0], parts[1], parts[-1]
    topic = " ".join(parts[2:-1])
    try:
        # @N gives nanoseconds; datetime keeps microseconds.
        when = datetime.datetime.strptime(stamp[:26], "%Y-%m-%dT%H:%M:%S.%f")
        data = bytes.fromhex(hex_text)
    except ValueError:
        return None
    if (retained not in ("0", "1")) or (not topic):
        return None
    return when, retained == "1", topic, data


def sub_line(b_retained, topic, payload):
    """Format one message exactly as mosquitto_sub -F SUB_FORMAT does."""
    now = datetime.datetime.now()
    return "%s%06d000 %d %s %s\n" % (now.strftime("%Y-%m-%dT%H:%M:%S."),
                                     now.microsecond, 1 if b_retained else 0,
                                     topic, payload.hex())


def pack(msg_type, values):
    """Build a payload from field values: s1_decode.decode() in reverse."""
    length = s1_decode.LENGTHS[msg_type]
    total = 8 * length
    value = ((s1_decode.LAYOUT_VERSION << 4) | msg_type) << (total - 8)
    for name, offset, width, kind in s1_decode.FIELDS[msg_type]:
        raw = values.get(name, 0)
        if kind == s1_decode.SIGNED:
            raw += 1 << (width - 1)
        elif kind == s1_decode.FLAG:
            raw = 1 if raw else 0
        elif kind == s1_decode.NEGATED:
            raw = -raw
        elif kind == s1_decode.BYTE_LIST:
            raw = int.from_bytes(bytes(raw), "big")
        value |= (raw & ((1 << width) - 1)) << (total - offset - width)
    return value.to_bytes(length, "big")


class LinkError(Exception):
    """A command that did not reach the broker (text already scrubbed)."""


class MosquittoLink:
    """The real broker: one mosquitto_sub, one mosquitto_pub per command."""

    def __init__(self, settings, scrub, opts_dir, b_v5):
        self._settings = settings
        self._scrub = scrub
        self._opts_path = write_options(settings, opts_dir)
        self._b_v5 = b_v5
        self._stop = threading.Event()
        self._proc = None
        self.on_line = None
        self.on_error = None
        self.on_sim = None
        self.on_banner = None

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        delay = 1.0
        while not self._stop.is_set():
            argv = sub_args(self._settings, self._opts_path, self._b_v5)
            began = time.monotonic()
            try:
                proc = subprocess.Popen(argv, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True,
                                        errors="replace")
            except OSError as err:
                self.on_error(self._scrub.text(str(err)))
                self._stop.wait(delay)
                continue
            self._proc = proc
            for line in proc.stdout:
                self.on_line(line)
            proc.wait()
            detail = proc.stderr.read().strip()
            if self._stop.is_set():
                break
            self.on_error(self._scrub.text(
                "mosquitto_sub ended (exit %d)%s"
                % (proc.returncode, (": " + detail) if detail else "")))
            if self._b_v5 and ("protocol version" in detail.lower()):
                # A broker without MQTT v5: carry on without the banner.
                self._b_v5 = False
                self.on_banner("the broker refused MQTT v5 (mosquitto 2.0 "
                               "or later is needed)")
            if time.monotonic() - began > 30.0:
                delay = 1.0
            self._stop.wait(delay)
            delay = min(delay * 2.0, 8.0)

    def publish(self, payload):
        argv = pub_args(self._settings, self._opts_path)
        check_no_retain(argv)
        try:
            done = subprocess.run(argv, input=payload, capture_output=True,
                                  timeout=5)
        except (OSError, subprocess.TimeoutExpired) as err:
            raise LinkError(self._scrub.text(str(err)))
        if done.returncode != 0:
            detail = done.stderr.decode("utf-8", "replace").strip()
            raise LinkError(self._scrub.text(
                "mosquitto_pub exit %d: %s" % (done.returncode, detail)))

    def sim(self, action):
        return False

    def published_copy(self):
        return []

    def stop(self):
        self._stop.set()
        if (self._proc is not None) and (self._proc.poll() is None):
            self._proc.terminate()
        remove_quietly(self._opts_path)


def remove_quietly(path):
    if path is not None:
        try:
            os.remove(path)
        except OSError:
            pass


class FakeCar:
    """--simulate: a car and a broker in one, with no network at all.

    Lines go to the dashboard in the exact mosquitto_sub format, so the
    real parser is used. The command rules copy s5_mission_command()
    (app_program/subsystem5/s5_mission.c); the acks and the heartbeat copy
    telemetry_task (ack result 0 = queued for S5, a 1 Hz heartbeat with
    rssi 0). It also sends the 2 Hz telemetry frame and the 5 Hz live
    message with made-up but plausible values (a car that follows a line,
    reads a barcode every 7 s, takes a junction every 11 s, loses the line
    every 29 s and crosses a hump every 17 s while it is running), so every
    panel can be seen working. After each 5 s of running its heartbeat also
    reports 3 more skipped messages in tel_drop, which clamps at 255 as the
    robot's does. The fake car's own state is also in the
    simulator panel; a real car's state is in the S5 panel.
    """

    CAL_S = 3.0
    RUNNING = "LINE_FOLLOWING"
    TEL_S = 0.5                 # s1_telem.h: S1_TELEM_TEL_PERIOD_MSEC.
    LIVE_S = 0.2                # s1_telem.h: S1_TELEM_LIVE_PERIOD_MSEC.
    TRACK_MM = 140.0            # Made-up wheel track, for the heading.
    COUNTS_PER_MM = 3.1         # Made-up encoder scale.
    CTRL_INDEX = {"CALIBRATING": 1, "READY": 2, "LINE_FOLLOWING": 3,
                  "FAULT": 9, "FINISHED": 10}
    SCRIPT = (("barcode", 7.0), ("junction", 11.0), ("reject", 19.0),
              ("lost", 29.0), ("hump", 17.0), ("skip", 5.0))
    SKIPS_PER_FIRE = 3
    HUMP_S = 3.0
    BARCODE_LETTERS = "ABCD"

    def __init__(self, settings, scrub, opts_dir, b_v5):
        self._settings = settings
        self._scrub = scrub
        self._opts_path = write_options(settings, opts_dir)
        self._b_v5 = b_v5
        self._next_mode = None
        self._rng = random.Random(16)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._timeline = []
        self._count = 0
        self.published = []
        self.on_line = None
        self.on_error = None
        self.on_sim = None
        prefix = settings.prefix
        self._topic = {"status": prefix + "/status",
                       "telemetry": prefix + "/telemetry",
                       "ack": prefix + "/ack",
                       "cmd": prefix + "/" + CMD_SUFFIX}
        self._state = "CALIBRATING"
        self._t0 = time.monotonic()
        self._next_hb = self._t0 + 1.0
        self._next_sys = self._t0 + 5.0
        self._hb_seq = 0
        self._uptime0 = 12
        self._connect_count = 1
        self._cmd_reject = 0
        self._b_connected = True
        self._b_hb_paused = False
        self._model_lock = threading.Lock()
        self._tel_seq = 0
        self._live_seq = 0
        self._next_tel = self._t0 + 0.3
        self._next_live = self._t0 + 0.3
        self._last_model = None
        self._run_s = 0.0
        self._due = dict(self.SCRIPT)
        self._hump_left = 0.0
        self._hump_count = 0
        self._hold = {"junction": 0.0, "lost": 0.0, "impact": 0.0}
        self._dist_mm = 0.0
        self._heading_deg = 0.0
        self._enc = [0.0, 0.0]
        self._tel_drop = 0
        self._s3 = {"barcodes": 0, "rejects": 0, "junctions": 0, "losts": 0}
        self._bc = {"letter": s1_decode.LETTER_NONE, "cmd": 0,
                    "b_rev": False, "turn": 0}
        self._view = {}

    def start(self):
        self._t0 = time.monotonic()
        self._next_hb = self._t0 + 1.5
        self._next_sys = self._t0 + 5.0
        self._next_tel = self._t0 + 0.3
        self._next_live = self._t0 + 0.3
        self.on_sim(self._state, "fake car console: power-up, CALIBRATING")
        self.on_line(sub_line(True, self._topic["status"], b"online"))
        self.on_line(sub_line(True, "$SYS/broker/uptime", b"1 seconds"))
        threading.Thread(target=self._loop, daemon=True).start()

    def _after(self, delay, func, *args):
        with self._lock:
            self._count += 1
            heapq.heappush(self._timeline, (time.monotonic() + delay,
                                            self._count, func, args))

    def _loop(self):
        while not self._stop.wait(0.01):
            now = time.monotonic()
            due = []
            with self._lock:
                while self._timeline and (self._timeline[0][0] <= now):
                    due.append(heapq.heappop(self._timeline))
            for _, _, func, args in due:
                func(*args)
            self._step(now)

    def _step(self, now):
        if (self._state == "CALIBRATING") and (now - self._t0 >= self.CAL_S):
            self._state = "READY"
            self.on_sim("READY", "fake car console: calibrated, READY")
        if now >= self._next_sys:
            self._next_sys = now + 5.0
            uptime = "%d seconds" % int(now - self._t0)
            self.on_line(sub_line(False, "$SYS/broker/uptime",
                                  uptime.encode("ascii")))
        if now >= self._next_hb:
            self._next_hb = max(self._next_hb + 1.0, now + 0.5)
            self._heartbeat(now)
        self._advance(now)
        if self._b_connected and (now >= self._next_live):
            self._next_live = max(self._next_live + self.LIVE_S, now + 0.1)
            self._send_live()
        if self._b_connected and (now >= self._next_tel):
            self._next_tel = max(self._next_tel + self.TEL_S, now + 0.25)
            self._send_telemetry()

    def _advance(self, now):
        """Move the made-up car on to time `now` and work out the values
        both messages send (the S2, S3, S4 and S5 fields of section 3A)."""
        with self._model_lock:
            dt = 0.0 if self._last_model is None else min(
                now - self._last_model, 0.5)
            self._last_model = now
            for key in self._hold:
                self._hold[key] = max(0.0, self._hold[key] - dt)
            b_run = self._state == self.RUNNING
            if b_run:
                self._run_s += dt
                for name, period in self.SCRIPT:
                    if self._run_s >= self._due[name]:
                        self._due[name] += period
                        self._fire(name)
            hump_before = self._hump_left
            self._hump_left = max(0.0, self._hump_left - dt)
            if (hump_before > 0.0) and (self._hump_left == 0.0):
                self._hump_count += 1
            self._view = self._values(dt, b_run)

    def _values(self, dt, b_run):
        """The fake car's current numbers (model lock held)."""
        run = self._run_s
        b_lost = self._hold["lost"] > 0.0
        b_junction = self._hold["junction"] > 0.0
        b_fault = self._state == "FAULT"
        steer = 0
        target = 0
        if b_run and (not b_lost):
            target = 80 if b_junction else 200
            if not b_junction:
                steer = int(60.0 * math.sin(0.8 * run))
        measured = 0.0
        if b_run and (not b_lost):
            measured = target + 12.0 * math.sin(2.3 * run)
        speed_l = int(measured - steer / 2.0)
        speed_r = int(measured + steer / 2.0)
        self._dist_mm = min(self._dist_mm + measured * dt, 32767.0)
        self._heading_deg += math.degrees((speed_r - speed_l)
                                          / self.TRACK_MM) * dt
        self._heading_deg = (self._heading_deg + 180.0) % 360.0 - 180.0
        self._enc[0] += speed_l * dt * self.COUNTS_PER_MM
        self._enc[1] += speed_r * dt * self.COUNTS_PER_MM
        pitch = 1.5 * math.sin(2.0 * run) if b_run else 0.0
        roll = 2.0 * math.sin(0.7 * run) if b_run else 0.0
        hump_state = 0
        if self._hump_left > 0.0:
            phase = self.HUMP_S - self._hump_left
            half = self.HUMP_S / 2.0
            if phase < half:
                hump_state = 1
                pitch += 12.0 * math.sin(math.pi * phase / half)
            else:
                hump_state = 2
                pitch -= 12.0 * math.sin(math.pi * (phase - half) / half)
        b_impact = self._hold["impact"] > 0.0
        if b_fault:
            motion = 4
        elif b_junction:
            motion = 3
        elif b_run:
            motion = 1
        else:
            motion = 0
        if b_impact:
            klass = 6
        elif hump_state == 1:
            klass = 4
        elif hump_state == 2:
            klass = 5
        elif b_junction:
            klass = 3
        elif b_run:
            klass = 1
        else:
            klass = 0
        if self._state == "CALIBRATING":
            line = 5
        elif b_lost:
            line = 2
        elif b_junction:
            line = 1
        elif b_run:
            line = 0
        else:
            line = 3
        outer = int(10.0 + 15.0 * abs(math.sin(0.9 * run))) if b_run else 0
        centre = int(200.0 + 30.0 * math.sin(1.7 * run)) if b_run else 0
        if b_lost:
            outer, centre = 0, 0
        return {
            "motion": motion, "b_estop": b_fault, "cmd_speed": target,
            "speed_l": speed_l, "speed_r": speed_r, "steer": steer,
            "heading_hdeg": int(round(self._heading_deg * 2.0)),
            "dist": int(self._dist_mm),
            "turn_dps": int(math.degrees((speed_r - speed_l)
                                         / self.TRACK_MM)),
            "pitch_hdeg": max(-64, min(63, int(round(pitch * 2.0)))),
            "roll_hdeg": max(-64, min(63, int(round(roll * 2.0)))),
            "hump_state": hump_state, "klass": klass, "line": line,
            "outer": min(outer, 255), "centre": min(centre, 255),
            "b_overshoot": outer > 22,
            "ctrl": self.CTRL_INDEX.get(self._state, 0),
            "fault_mask": 1 if b_fault else 0,
            "b_cal_done": self._state != "CALIBRATING"}

    def _fire(self, name):
        """One scripted or simulator-button event (model lock held)."""
        if name == "barcode":
            number = self._s3["barcodes"]
            letter = self.BARCODE_LETTERS[number % len(self.BARCODE_LETTERS)]
            self._bc = {"letter": s1_decode.CODE39.index(letter),
                        "cmd": 1 + (number % 4),
                        "b_rev": (number % 5) == 4, "turn": self._bc["turn"]}
            self._s3["barcodes"] += 1
        elif name == "reject":
            self._s3["rejects"] += 1
        elif name == "junction":
            self._bc["turn"] = (self._s3["junctions"] + 1) % 4
            self._s3["junctions"] += 1
            self._hold["junction"] = 0.6
        elif name == "lost":
            self._s3["losts"] += 1
            self._hold["lost"] = 0.8
        elif name == "hump":
            self._hump_left = self.HUMP_S
        elif name == "impact":
            self._hold["impact"] = 1.0
        elif name == "skip":
            self._tel_drop = min(self._tel_drop + self.SKIPS_PER_FIRE, 255)
        else:
            pass

    def _send_telemetry(self):
        with self._model_lock:
            view = dict(self._view)
            bc = dict(self._bc)
            seq = self._tel_seq
            self._tel_seq += 1
            enc = [int(self._enc[0]), int(self._enc[1])]
            humps = self._hump_count
        if not view:
            return
        values = {
            "seq": seq & 0xFF, "ctrl_state": view["ctrl"],
            "motion_state": view["motion"],
            "fault_mask": view["fault_mask"], "line_state": view["line"],
            "bc_letter": bc["letter"], "bc_cmd": 0,
            "b_bc_dir": bc["b_rev"],
            "speed_l_mms": view["speed_l"], "speed_r_mms": view["speed_r"],
            "count_l": enc[0], "count_r": enc[1], "dist_mm": view["dist"],
            "pitch_hdeg": view["pitch_hdeg"],
            "roll_hdeg": view["roll_hdeg"],
            "hump_state": view["hump_state"],
            "peak_mm": 25 if humps > 0 else 0,
            "hump_count": min(humps, 15), "motion_class": view["klass"],
            "turn_dps": view["turn_dps"], "b_imu_ok": True,
            "b_cal_done": view["b_cal_done"]}
        self.on_line(sub_line(False, self._topic["telemetry"],
                              pack(s1_decode.TELEMETRY, values)))

    def _send_live(self):
        with self._model_lock:
            view = dict(self._view)
            bc = dict(self._bc)
            counts = dict(self._s3)
            seq = self._live_seq
            self._live_seq += 1
        if not view:
            return
        values = {
            "seq": seq & 0xFF, "motion_state": view["motion"],
            "b_estop": view["b_estop"], "cmd_speed_mms": view["cmd_speed"],
            "heading_hdeg": view["heading_hdeg"], "line_state": view["line"],
            "steer_dv_mms": view["steer"], "outer_ir": view["outer"],
            "centre_ir": view["centre"],
            "b_overshoot": view["b_overshoot"],
            "bc_letter": bc["letter"], "bc_cmd": bc["cmd"],
            "b_bc_rev": bc["b_rev"], "junc_turn": bc["turn"],
            "rejects": counts["rejects"] & 0xF,
            "junctions": counts["junctions"] & 0xFF,
            "losts": counts["losts"] & 0xFF,
            "barcodes": counts["barcodes"] & 0xFF}
        self.on_line(sub_line(False, self._topic["telemetry"],
                              pack(s1_decode.LIVE, values)))

    def _heartbeat(self, now):
        values = {"seq": self._hb_seq & 0xFF,
                  "uptime_s": (self._uptime0 + int(now - self._t0)) & 0xFFFF,
                  "rssi_dbm": 0,
                  "connect_count": self._connect_count & 0xFF,
                  "tel_drop": self._tel_drop, "evt_drop": 0,
                  "cmd_reject": min(self._cmd_reject, 255)}
        if not self._b_connected:
            return
        # A paused heartbeat is a lost one: seq still moves on.
        self._hb_seq += 1
        if not self._b_hb_paused:
            self.on_line(sub_line(False, self._topic["telemetry"],
                                  pack(s1_decode.HEARTBEAT, values)))

    def publish(self, payload):
        argv = pub_args(self._settings, self._opts_path)
        check_no_retain(argv)
        with self._lock:
            self.published.append(list(argv))
        # The broker hands the command back to our own cmd/# subscription.
        self._after(0.005, self._deliver, self._topic["cmd"], payload)
        self._after(0.010, self._take_command, payload)

    def _deliver(self, topic, payload):
        self.on_line(sub_line(False, topic, payload))

    def _deliver_live_retained(self, topic, payload):
        """A retained publish reaching a subscriber that is listening:
        flagged retained only under MQTT v5 retain-as-published."""
        self.on_line(sub_line(self._b_v5, topic, payload))

    def _take_command(self, payload):
        """telemetry_task and S5 handling one command from the broker."""
        if not self._b_connected:
            return
        if len(payload) > 2:
            # The driver drops it before the codec: no ack, counted.
            self._cmd_reject += 1
            return
        try:
            msg = s1_decode.decode(payload)
        except s1_decode.DecodeError:
            msg = None
        if (msg is None) or (msg["msg_type"] != s1_decode.COMMAND):
            opcode, cmd_id, b_valid = 0, 0, False
        else:
            opcode, cmd_id, b_valid = msg["opcode"], msg["cmd_id"], msg["valid"]
        mode, self._next_mode = self._next_mode, None
        if b_valid and (mode != "reject"):
            result = 0
            before = self._state
            b_ok = self._mission(opcode)
            self.on_sim(self._state, "fake car console: EVT 0x65 a=%d b=%d"
                        " (%s -> %s)" % (opcode, 1 if b_ok else 0, before,
                                         self._state))
        else:
            result = 1
            self._cmd_reject += 1
            self.on_sim(self._state, "fake car console: command %s refused "
                        "by telemetry_task (%s)" % (
                            payload.hex() or "(empty)",
                            "post failure" if b_valid else "codec"))
        ack = pack(s1_decode.ACK, {"opcode": opcode, "cmd_id": cmd_id,
                                   "result": result})
        if mode == "drop":
            self.on_sim(self._state, "simulator: this ack is dropped")
            return
        delay = 3.0 if mode == "delay" else self._rng.uniform(0.03, 0.12)
        self._after(delay, self._deliver, self._topic["ack"], ack)
        if mode == "dup":
            self._after(delay + 0.05, self._deliver, self._topic["ack"], ack)

    def _mission(self, opcode):
        """The rules of s5_mission_command(): True if S5 would accept."""
        b_ok = False
        if opcode == 1:
            if self._state == "READY":
                self._state = self.RUNNING
                b_ok = True
            elif self._state == "FINISHED":
                self._state = "READY"
                b_ok = True
        elif opcode == 2:
            # Always accepted; only a running car has anything to stop.
            if self._state == self.RUNNING:
                self._state = "FINISHED"
            b_ok = True
        elif opcode == 3:
            if self._state in ("FAULT", "FINISHED"):
                self._state = "READY"
                b_ok = True
        return b_ok

    def sim(self, action):
        """Apply one simulator action; False if the action is unknown."""
        if action == "hb_pause":
            self._b_hb_paused = True
        elif action == "hb_resume":
            self._b_hb_paused = False
            self._next_hb = time.monotonic()
        elif action in ("reject_next", "drop_next", "delay_next",
                        "dup_next"):
            self._next_mode = action[:-len("_next")]
        elif action in ("offline", "online"):
            self._set_connected(action == "online")
        elif action in ("fault", "gp20"):
            self._car_action(action)
        elif action in ("s3_barcode", "s3_reject", "s3_junction", "s3_lost",
                        "s4_impact", "s4_hump"):
            with self._model_lock:
                self._fire(action[3:])
        elif action in ("retained_cmd", "retained_clear"):
            # Another client publishes on <prefix>/cmd/test with retain: a
            # RESET with cmd_id 42, or the empty message that clears it.
            # The robot, subscribed, takes it like any other command.
            payload = (cmd_rtt.command_bytes(3, 42)
                       if action == "retained_cmd" else b"")
            self._deliver_live_retained(
                self._settings.prefix + "/" + RETAINED_TEST_SUFFIX, payload)
            self._after(0.010, self._take_command, payload)
        elif action == "fake_error":
            # For S1-U16 A12: an error line holding every secret, left
            # unscrubbed here so that the dashboard's own scrubber is tested.
            self.on_error(
                "Error: connection to %s:%d refused for user %s, password %s"
                % (self._settings.host, self._settings.port,
                   self._settings.user, self._settings.password))
        else:
            return False
        return True

    def _set_connected(self, b_online):
        """Lose or regain the broker session (hotspot off, then on)."""
        if b_online and (not self._b_connected):
            self._b_connected = True
            self._connect_count += 1
            self._deliver_live_retained(self._topic["status"], b"online")
            self._next_hb = time.monotonic() + 0.5
        elif (not b_online) and self._b_connected:
            self._b_connected = False
            # The broker publishes the last will once the session is gone.
            self._after(0.5, self._deliver_live_retained,
                        self._topic["status"], b"offline")
        else:
            pass

    def _car_action(self, action):
        """A stall fault while running, or a GP20 (Button 1) press."""
        before = self._state
        if action == "fault":
            if self._state == self.RUNNING:
                self._state = "FAULT"
                text = "stall fault"
            else:
                text = "no fault: faults latch only while running"
        else:
            # Button 1 toggles: STOP while running, START otherwise.
            opcode = 2 if self._state == self.RUNNING else 1
            b_ok = self._mission(opcode)
            text = "GP20 press, %s %s" % (s1_decode.OPCODE_NAMES[opcode],
                                          "done" if b_ok else "refused")
        self.on_sim(self._state, "fake car console: %s (%s -> %s)"
                    % (text, before, self._state))

    def published_copy(self):
        with self._lock:
            return [list(argv) for argv in self.published]

    def stop(self):
        self._stop.set()
        remove_quietly(self._opts_path)


class Dashboard:
    """All state, the 3 s heartbeat rule and the ack matching.

    Every change goes out as one server-sent event, scrubbed, to every page
    and to the --log file. The lock guards everything below.
    """

    def __init__(self, settings, scrub, mode, ack_timeout, log_path,
                 banner_why=None):
        self.banner = "ok" if banner_why is None else "unavailable"
        self.banner_why = banner_why
        self.retained = collections.OrderedDict()
        self.recent = {}
        self.settings = settings
        self.scrub = scrub
        self.mode = mode
        self.ack_timeout = ack_timeout
        self.link = None
        self.lock = threading.RLock()
        self.clients = []
        self.event_id = 0
        self.session = s1_decode.Session()
        self.status = {"value": "unknown", "retained": None}
        self.status_mono = None
        self.heartbeat = None
        self.hb_mono = None
        self.lost_total = 0
        self.light = {"light": "grey", "reason": "no status from the robot"}
        self.broker = {"state": "down"}
        self.broker_mono = None
        self.controls = {"start": False, "reset": False, "stop": True,
                         "start_why": "no status from the robot",
                         "reset_why": "no status from the robot"}
        self.commands = collections.deque(maxlen=COMMANDS_KEEP)
        self.pending = {}
        self.next_id = 0
        self.cmd_count = 0
        self.log = collections.deque(maxlen=LOG_KEEP)
        self.telemetry = None
        self.tel_mono = None
        self.tel_lost_total = 0
        self.live = None
        self.live_lost_total = 0
        self.live_mono = None
        self.s3_timeline = collections.deque(maxlen=TIMELINE_KEEP)
        self.s3_tally = {}
        self.s3_rejected = 0
        self.imu_history = collections.deque(maxlen=IMU_KEEP)
        self.impact_count = 0
        self.impact_at = None
        self.b_impact_prev = False
        self.car_state = None
        self.log_file = None
        if log_path:
            self.log_file = open(log_path, "a", encoding="utf-8")

    def attach(self, link):
        self.link = link
        link.on_line = self.on_line
        link.on_error = self.on_link_error
        link.on_sim = self.on_sim
        link.on_banner = self.on_banner_off

    # -- events ----------------------------------------------------------

    def emit(self, name, data):
        """Send one event to every page and the log file (lock held)."""
        data = self.scrub.obj(dict(data))
        data["t"] = now_text()
        self.event_id += 1
        text = json.dumps(data, sort_keys=True)
        if "log" in data:
            self.log.append({"t": data["t"], "kind": log_kind(name, data),
                             "text": data["log"]})
            print(self.scrub.text("%s  %s" % (data["t"][11:], data["log"])),
                  flush=True)
        if self.log_file is not None:
            self.log_file.write(json.dumps(
                {"id": self.event_id, "event": name, "data": data},
                sort_keys=True) + "\n")
            self.log_file.flush()
        message = "id: %d\nevent: %s\ndata: %s\n\n" % (self.event_id, name,
                                                         text)
        for client in list(self.clients):
            client.put(message)

    def subscribe(self):
        """Register a page: its first event is the snapshot."""
        client = queue.Queue()
        with self.lock:
            text = json.dumps(self.snapshot(), sort_keys=True)
            client.put("id: %d\nevent: snapshot\ndata: %s\n\n"
                       % (self.event_id, text))
            self.clients.append(client)
        return client

    def unsubscribe(self, client):
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)

    def snapshot(self):
        now = time.monotonic()
        age = None
        if self.hb_mono is not None:
            age = int((now - self.hb_mono) * 1000.0)
        snap = {"mode": self.mode, "prefix": self.settings.prefix,
                "port": self.settings.port,
                "config_source": self.settings.source,
                "broker": self.broker, "status": self.status,
                "link": self.light, "heartbeat": self.heartbeat,
                "hb_age_ms": age, "controls": self.controls,
                "commands": [public(rec) for rec in self.commands],
                "log": list(self.log), "telemetry": self.telemetry_view(),
                "tel_age_ms": age_ms(now, self.tel_mono),
                "live": self.live,
                "live_age_ms": age_ms(now, self.live_mono),
                "s3_timeline": list(self.s3_timeline),
                "s3_tally": self.s3_tally,
                "s3_rejected": self.s3_rejected,
                "imu_history": list(self.imu_history),
                "car_state": self.car_state, "banner": self.banner,
                "banner_why": self.banner_why,
                "retained_cmd": [dict(e) for e in self.retained.values()],
                "t": now_text()}
        return self.scrub.obj(snap)

    def public_state(self):
        with self.lock:
            state = self.snapshot()
            if self.mode == "simulate":
                state["publish_args"] = self.scrub.obj(
                    self.link.published_copy())
            return state

    # -- from the link ---------------------------------------------------

    def on_line(self, line):
        parsed = parse_sub_line(line)
        with self.lock:
            if parsed is None:
                self.emit("note", {"log": "unreadable line from "
                                          "mosquitto_sub"})
                return
            when, b_retained, topic, data = parsed
            self.broker_mono = time.monotonic()
            prefix = self.settings.prefix + "/"
            if topic.startswith(prefix):
                self.dispatch(topic[len(prefix):], when, b_retained, data)
            self.update_derived()

    def dispatch(self, suffix, when, b_retained, data):
        if suffix == "status":
            self.on_status(b_retained, data)
        elif suffix == "telemetry":
            self.on_telemetry(suffix, data)
        elif suffix == "ack":
            self.on_ack(when, data)
        elif (suffix == "cmd") or suffix.startswith("cmd/"):
            self.on_cmd_seen(suffix, b_retained, data)
        elif suffix == "event":
            self.on_event(suffix, data)
        else:
            pass

    def on_link_error(self, text):
        with self.lock:
            self.broker_mono = None
            self.update_derived()
            self.emit("broker", {"state": self.broker["state"],
                                 "detail": text, "log": "broker: " + text})

    def on_sim(self, car_state, text):
        with self.lock:
            self.car_state = car_state
            self.emit("sim", {"car_state": car_state, "log": text})

    def on_banner_off(self, why):
        with self.lock:
            if self.banner == "ok":
                self.banner = "unavailable"
                self.banner_why = why
                self.retained.clear()
                self.emit("banner", {"banner": self.banner, "why": why,
                                     "log": "retained-command banner "
                                            "unavailable: " + why})

    def on_status(self, b_retained, data):
        text = data.decode("ascii", "replace")
        if text in ("online", "offline"):
            value = text
        elif not text:
            value = "unknown"
        else:
            value = "other"
        self.status = {"value": value, "retained": b_retained}
        self.status_mono = time.monotonic()
        self.emit("status", {"value": value, "retained": b_retained,
                             "log": "robot status: %s%s" % (
                                 value, " (retained)" if b_retained else "")})

    def on_telemetry(self, suffix, data):
        try:
            msg = self.session.feed(data)
        except s1_decode.DecodeError as err:
            self.decode_error(suffix, err)
            return
        if msg["msg_type"] == s1_decode.HEARTBEAT:
            self.on_heartbeat(msg)
        elif msg["msg_type"] == s1_decode.TELEMETRY:
            self.on_frame(msg)
        elif msg["msg_type"] == s1_decode.LIVE:
            self.on_live(msg)
        else:
            self.emit("note", {"log": "%s message on %s ignored"
                                      % (msg["type"], suffix)})

    def telemetry_view(self):
        """The last telemetry frame plus what the dashboard adds: how many
        IMPACT flags it has seen and when the last one arrived."""
        if self.telemetry is None:
            return None
        view = dict(self.telemetry)
        view["impact_count"] = self.impact_count
        view["impact_at"] = self.impact_at
        view["lost_total"] = self.tel_lost_total
        return view

    def on_frame(self, msg):
        """A 2 Hz telemetry frame: S2 odometry, S4 IMU, S5 state."""
        self.telemetry = s1_decode.describe(msg)
        self.tel_mono = time.monotonic()
        if msg["lost_before"] <= SEQ_GAP_MAX:
            self.tel_lost_total += msg["lost_before"]
        self.imu_history.append([self.telemetry["pitch_deg"],
                                 self.telemetry["roll_deg"]])
        b_impact = self.telemetry["b_impact"]
        b_new = b_impact and (not self.b_impact_prev)
        self.b_impact_prev = b_impact
        if b_new:
            # Count the rising edge only; S4's flag may stay up for a while.
            self.impact_count += 1
            self.impact_at = now_text()
        view = self.telemetry_view()
        if b_new:
            view["log"] = "Lifted/overturned reported by S4 (number %d)" % (
                self.impact_count)
        self.emit("telemetry", view)

    def on_live(self, msg):
        """A 5 Hz live message: S2 command and heading, S3 line state."""
        self.live = s1_decode.describe(msg)
        self.live_mono = time.monotonic()
        if msg["lost_before"] <= SEQ_GAP_MAX:
            self.live_lost_total += msg["lost_before"]
        self.live["lost_total"] = self.live_lost_total
        entries = self.live_entries(msg)
        self.emit("live", self.live)
        if entries:
            self.emit("s3_events", {"entries": entries,
                                    "tally": self.s3_tally,
                                    "rejected": self.s3_rejected})

    def add_timeline(self, kind, text):
        """Append one entry to the S3 timeline and return it."""
        entry = {"t": now_text(), "kind": kind, "text": text}
        self.s3_timeline.append(entry)
        return entry

    def live_entries(self, msg):
        """Timeline entries for the S3 counters that moved since the
        previous live message; also updates the letter tally.

        S3 sends counters, not events, so each entry is dated by when the
        message arrived (200 ms resolution). Only the latest letter is
        known, so a second barcode inside one interval is counted under "?".
        """
        deltas = {name: msg["d_" + name]
                  for name, _ in s1_decode.Session.LIVE_COUNTERS}
        if ((msg["lost_before"] > SEQ_GAP_MAX)
                or any(deltas[name] > COUNTER_JUMP_MAX[name]
                       for name in deltas)):
            self.session.rebase_live()
            return [self.add_timeline(
                "note", "S3 counters or live messages jumped: the robot "
                        "restarted or messages were lost, counting again "
                        "from here")]
        entries = []
        shown = self.live
        if deltas["barcodes"] > 0:
            symbol = shown["bc_char"] or "?"
            self.s3_tally[symbol] = self.s3_tally.get(symbol, 0) + 1
            extra = deltas["barcodes"] - 1
            if extra > 0:
                self.s3_tally["?"] = self.s3_tally.get("?", 0) + extra
            text = "barcode %s accepted: %s, read %s" % (
                symbol, shown["bc_cmd_name"],
                "backwards" if shown["b_bc_rev"] else "forwards")
            if extra > 0:
                text += " (and %d more, letters not seen)" % extra
            entries.append(self.add_timeline("barcode", text))
        if deltas["rejects"] > 0:
            self.s3_rejected += deltas["rejects"]
            entries.append(self.add_timeline(
                "reject", "barcode rejected (%d)" % deltas["rejects"]))
        if deltas["junctions"] > 0:
            entries.append(self.add_timeline(
                "junction", "junction: turn %s%s" % (
                    shown["junc_name"],
                    "" if deltas["junctions"] == 1
                    else " (%d junctions, last turn)" % deltas["junctions"])))
        if deltas["losts"] > 0:
            entries.append(self.add_timeline(
                "lost", "line lost%s" % (
                    "" if deltas["losts"] == 1
                    else " (%d times)" % deltas["losts"])))
        return entries

    def on_heartbeat(self, msg):
        notes = []
        old = self.heartbeat
        if old is not None:
            if msg["uptime_s"] < old["uptime_s"]:
                notes.append("uptime went back: the robot restarted")
                self.session.rebase_robot()
                # feed() compared this seq with the previous run's.
                msg["lost_before"] = 0
                self.emit("s3_events", {
                    "entries": [self.add_timeline(
                        "note", "the robot restarted: S3 counts again "
                                "from 0")],
                    "tally": self.s3_tally, "rejected": self.s3_rejected})
            if msg["connect_count"] != old["connect_count"]:
                notes.append("connect_count %d -> %d: the robot reconnected"
                             % (old["connect_count"], msg["connect_count"]))
        self.lost_total += msg["lost_before"]
        rssi = msg["rssi_dbm"]
        self.heartbeat = {
            "seq": msg["seq"], "lost_before": msg["lost_before"],
            "lost_total": self.lost_total, "uptime_s": msg["uptime_s"],
            "rssi_dbm": rssi,
            "rssi_text": "not measured" if rssi == 0 else "%d dBm" % rssi,
            "connect_count": msg["connect_count"],
            "tel_drop": msg["tel_drop"], "evt_drop": msg["evt_drop"],
            "tel_drop_text": tel_drop_text(msg["tel_drop"]),
            "cmd_reject": msg["cmd_reject"]}
        self.hb_mono = time.monotonic()
        data = dict(self.heartbeat)
        if notes:
            data["log"] = "heartbeat: " + "; ".join(notes)
        self.emit("heartbeat", data)

    def on_ack(self, when, data):
        try:
            msg = s1_decode.decode(data)
        except s1_decode.DecodeError as err:
            self.decode_error("ack", err)
            return
        if msg["msg_type"] != s1_decode.ACK:
            self.emit("note", {"log": "%s message on ack ignored"
                                      % msg["type"]})
            return
        cmd_id, opcode, result = msg["cmd_id"], msg["opcode"], msg["result"]
        name = s1_decode.OPCODE_NAMES[opcode]
        label = LABELS.get(result, "reserved %d" % result)
        rec = self.pending.get(cmd_id)
        old = self.recent.get(cmd_id)
        data = {"cmd_id": cmd_id, "op": name, "result": result,
                "label": label, "rtt_ms": None, "match": "unmatched"}
        if (rec is not None) and (rec["opcode"] == opcode):
            del self.pending[cmd_id]
            self.resolve(rec, label, when, "matched", data)
            data["log"] = "ack %s id %d: %s, %.1f ms" % (name, cmd_id, label,
                                                         data["rtt_ms"])
        elif ((old is not None) and (old["opcode"] == opcode)
              and (old["state"] == "no ack")):
            self.resolve(old, label + " (late)", when, "late", data)
            data["log"] = ("late ack %s id %d: %s, %.1f ms (after the "
                           "%.1f s limit)" % (name, cmd_id, label,
                                              data["rtt_ms"],
                                              self.ack_timeout))
        elif (old is not None) and (old["opcode"] == opcode):
            data["match"] = "duplicate"
            data["log"] = ("duplicate ack %s id %d: %s (already matched)"
                           % (name, cmd_id, label))
        else:
            data["log"] = ("ack %s id %d: %s, not matched to a command "
                           "from this dashboard" % (name, cmd_id, label))
        self.emit("ack", data)

    def resolve(self, rec, state, when, match, data):
        """Close a command with its ack; keep it for late or duplicates."""
        rtt = round((when - rec["sent_wall"]).total_seconds() * 1000.0, 1)
        rec["state"] = state
        rec["rtt_ms"] = rtt
        rec["resolved_mono"] = time.monotonic()
        self.recent[rec["cmd_id"]] = rec
        data.update({"n": rec["n"], "state": state, "rtt_ms": rtt,
                     "match": match})

    def on_cmd_seen(self, suffix, b_retained, data):
        now = time.monotonic()
        own = (suffix == CMD_SUFFIX) and any(
            (rec["hex"] == data.hex())
            and (now - rec["sent_mono"] <= self.ack_timeout + 2.0)
            for rec in self.commands)
        event = {"topic_suffix": suffix, "hex": data.hex(),
                 "retained": b_retained, "own": own}
        if b_retained and (self.banner == "ok"):
            self.on_retained(suffix, data)
        elif not own:
            event["log"] = "command from another client on %s: %s" % (
                suffix, data.hex() or "(empty)")
        else:
            pass
        self.emit("cmd_seen", event)

    def on_retained(self, suffix, data):
        """A retained message on <prefix>/cmd/...: raise or clear the
        banner. Only trusted with MQTT v5 retain-as-published."""
        entry = self.retained.get(suffix)
        if data:
            entry = {"topic_suffix": suffix, "hex": data.hex(),
                     "at": now_text(), "cleared": False, "cleared_at": None}
            self.retained[suffix] = entry
            text = ("WARNING: a retained command is stored on %s (%s): the "
                    "robot would run it again after every reconnect"
                    % (suffix, data.hex()))
        elif (entry is not None) and (not entry["cleared"]):
            entry["cleared"] = True
            entry["cleared_at"] = now_text()
            text = "the retained command on %s is cleared" % suffix
        else:
            return
        event = dict(entry)
        event["log"] = text
        self.emit("retained_cmd", event)

    def on_event(self, suffix, data):
        try:
            msg = s1_decode.decode(data)
        except s1_decode.DecodeError as err:
            self.decode_error(suffix, err)
            return
        self.emit("event", {"code": msg.get("code"), "args": msg.get("args"),
                            "t_ms": msg.get("t_ms"),
                            "log": "event code %s" % msg.get("code")})

    def decode_error(self, suffix, err):
        self.emit("decode_error", {"topic_suffix": suffix,
                                   "reason": err.reason,
                                   "detail": err.detail,
                                   "log": "could not decode a message on %s:"
                                          " %s" % (suffix, err.detail)})

    # -- commands --------------------------------------------------------

    def send(self, op_name):
        """Publish one command. Returns (HTTP status, reply)."""
        opcode = cmd_rtt.OPCODES[op_name]
        name = s1_decode.OPCODE_NAMES[opcode]
        with self.lock:
            self.update_derived()
            if (op_name != "stop") and (not self.controls[op_name]):
                why = self.controls[op_name + "_why"]
                self.emit("note", {"log": "%s not sent: %s" % (name, why)})
                return 409, {"error": "%s not sent: %s" % (name, why)}
            cmd_id = self.next_id
            self.next_id = (cmd_id + 1) % 64
            self.recent.pop(cmd_id, None)
            old = self.pending.pop(cmd_id, None)
            if old is not None:
                old["state"] = "superseded"
                self.emit("note", {"log": "id %d reused before its ack"
                                          % cmd_id})
            payload = cmd_rtt.command_bytes(opcode, cmd_id)
            self.cmd_count += 1
            rec = {"n": self.cmd_count, "cmd_id": cmd_id, "op": name,
                   "opcode": opcode, "hex": payload.hex(),
                   "sent_at": now_text(), "state": "pending",
                   "rtt_ms": None, "sent_mono": time.monotonic(),
                   "sent_wall": datetime.datetime.now()}
            self.pending[cmd_id] = rec
            self.commands.append(rec)
            data = public(rec)
            data["log"] = "sent %s id %d (%s)" % (name, cmd_id, payload.hex())
            self.emit("cmd", data)
            self.update_derived()
        try:
            self.link.publish(payload)
        except LinkError as err:
            with self.lock:
                if self.pending.get(cmd_id) is rec:
                    del self.pending[cmd_id]
                rec["state"] = "publish failed"
                self.emit("cmd_failed", {"n": rec["n"], "cmd_id": cmd_id,
                                         "op": name, "state": rec["state"],
                                         "log": "%s id %d not published: %s"
                                                % (name, cmd_id, err)})
                self.update_derived()
            return 502, {"error": "not published: %s" % err}
        return 200, {"cmd_id": cmd_id, "n": rec["n"]}

    def sim(self, action):
        if self.mode != "simulate":
            return 404, {"error": "not in simulate mode"}
        if not self.link.sim(action):
            return 400, {"error": "unknown simulator action"}
        with self.lock:
            self.emit("sim", {"car_state": self.car_state,
                              "log": "simulator: %s" % action})
        return 200, {"ok": True}

    # -- every TICK_S ----------------------------------------------------

    def tick(self):
        with self.lock:
            now = time.monotonic()
            for cmd_id, rec in list(self.pending.items()):
                if now - rec["sent_mono"] >= self.ack_timeout:
                    del self.pending[cmd_id]
                    rec["state"] = "no ack"
                    rec["resolved_mono"] = now
                    self.recent[cmd_id] = rec
                    self.emit("no_ack", {"n": rec["n"], "cmd_id": cmd_id,
                                         "op": rec["op"], "state": "no ack",
                                         "log": "no ack for %s id %d within "
                                                "%.1f s" % (rec["op"], cmd_id,
                                                            self.ack_timeout)})
            for cmd_id, rec in list(self.recent.items()):
                if now - rec["resolved_mono"] > RECENT_KEEP_S:
                    del self.recent[cmd_id]
            self.update_derived()

    def robot_light(self, now):
        value = self.status["value"]
        if value == "offline":
            return "red", "offline (status topic)"
        if value != "online":
            return "grey", "no status from the robot"
        if (self.hb_mono is not None) and (now - self.hb_mono <= HB_LIMIT_S):
            return "green", "online, heartbeat arriving"
        since = max(t for t in (self.hb_mono, self.status_mono)
                    if t is not None)
        if now - since > HB_LIMIT_S:
            return "red", "online, but no heartbeat for more than 3 s"
        return "grey", "online, waiting for a heartbeat"

    def update_derived(self):
        """The broker light, the robot light and the buttons (lock held)."""
        now = time.monotonic()
        b_up = ((self.broker_mono is not None)
                and (now - self.broker_mono <= BROKER_LIMIT_S))
        state = "up" if b_up else "down"
        if state != self.broker["state"]:
            self.broker = {"state": state}
            self.emit("broker", {"state": state, "log": "broker: %s" % (
                "connected" if b_up else
                "no message for %.0f s" % BROKER_LIMIT_S)})
        light, reason = self.robot_light(now)
        if (light, reason) != (self.light["light"], self.light["reason"]):
            self.light = {"light": light, "reason": reason}
            self.emit("link", {"light": light, "reason": reason,
                               "log": "robot light %s: %s" % (light, reason)})
        self.update_controls(light == "green", reason)

    def update_controls(self, b_ready, reason):
        """Enable START and RESET, each with its reason when it is not.

        STOP is not decided here: "stop" is the constant True, and send()
        never refuses a STOP.
        """
        waiting = {rec["opcode"] for rec in self.pending.values()}
        start_why = button_why(b_ready, reason,
                               cmd_rtt.OPCODES["start"] in waiting, "START")
        reset_why = button_why(b_ready, reason,
                               cmd_rtt.OPCODES["reset"] in waiting, "RESET")
        controls = {"start": not start_why, "reset": not reset_why,
                    "stop": True, "start_why": start_why,
                    "reset_why": reset_why}
        if controls != self.controls:
            self.controls = controls
            self.emit("controls", controls)

    def close(self):
        if self.log_file is not None:
            self.log_file.close()


def public(rec):
    """A command record without its internal clock fields."""
    return {key: rec[key] for key in ("n", "cmd_id", "op", "hex", "sent_at",
                                      "state", "rtt_ms")}


def button_why(b_ready, reason, b_waiting, name):
    """Why START or RESET is disabled, or "" when it is enabled."""
    if not b_ready:
        return reason
    if b_waiting:
        return "waiting for the ack of the last %s" % name
    return ""


def log_kind(name, data):
    """The event log's colour class; an ack goes by its result."""
    if name == "ack":
        return "ack_queued" if data.get("result") == 0 else "ack_other"
    return name


def report_error(scrub, where, err):
    """Print one line for an unexpected error: its type and scrubbed text,
    never a traceback (an exception's text can hold the broker host or the
    login)."""
    print(scrub.text("s1_dashboard: %s: %s: %s"
                     % (where, type(err).__name__, err)),
          file=sys.stderr, flush=True)


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        """One scrubbed line instead of socketserver's traceback."""
        report_error(self.dash.scrub, "request error", sys.exc_info()[1])


class Handler(http.server.BaseHTTPRequestHandler):
    """The page, the event stream and the two POST endpoints."""

    server_version = "s1_dashboard"
    sys_version = ""

    def log_message(self, fmt, *args):
        """Requests are not printed: the console shows the event log."""

    def _host_ok(self):
        return self.headers.get("Host", "") in self.server.allowed_hosts

    def _reply(self, status, body, content_type, csp=None):
        data = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if csp is not None:
            self.send_header("Content-Security-Policy", csp)
        self.end_headers()
        self.wfile.write(data)

    def _json(self, status, obj):
        self._reply(status, json.dumps(obj), "application/json")

    def do_GET(self):
        dash = self.server.dash
        path = self.path.split("?", 1)[0]
        if not self._host_ok():
            self._json(403, {"error": "wrong Host header"})
        elif path == "/":
            page = PAGE.replace("__TOKEN__", json.dumps(self.server.token))
            self._reply(200, page, "text/html; charset=utf-8", PAGE_CSP)
        elif path == "/events":
            self._events(dash)
        elif path == "/api/state":
            self._json(200, dash.public_state())
        else:
            self._json(404, {"error": "not found"})

    def _events(self, dash):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        client = dash.subscribe()
        try:
            self.wfile.write(b"retry: 2000\n\n")
            while True:
                try:
                    message = client.get(timeout=KEEPALIVE_S)
                except queue.Empty:
                    message = ": keepalive\n\n"
                self.wfile.write(message.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            dash.unsubscribe(client)

    def do_POST(self):
        dash = self.server.dash
        path = self.path.split("?", 1)[0]
        token = self.headers.get("X-S1-Token", "")
        kind = self.headers.get("Content-Type", "").split(";")[0].strip()
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = -1
        if not self._host_ok():
            self._json(403, {"error": "wrong Host header"})
        elif not secrets.compare_digest(token.encode("utf-8", "replace"),
                                        self.server.token.encode("ascii")):
            self._json(403, {"error": "missing or wrong page token"})
        elif kind != "application/json":
            self._json(415, {"error": "JSON only"})
        elif not 0 <= length <= 1024:
            self._json(413, {"error": "body too large"})
        else:
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                body = None
            if not isinstance(body, dict):
                self._json(400, {"error": "bad JSON"})
            elif path == "/api/cmd":
                op = body.get("op")
                if (not isinstance(op, str)) or (op not in cmd_rtt.OPCODES):
                    self._json(400, {"error": "op must be start, stop or "
                                              "reset"})
                else:
                    self._json(*dash.send(op))
            elif path == "/api/sim":
                self._json(*dash.sim(body.get("action")))
            else:
                self._json(404, {"error": "not found"})


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>S1 dashboard</title>
<style>
:root {
  --bg: #f4f5f7; --panel: #ffffff; --text: #1c2230; --muted: #5d6677;
  --line: #d8dce4; --green: #1e9e57; --red: #d23f3f; --grey: #9aa2b1;
  --amber: #d79a00; --stop: #c62828; --queued: #3a6ea5;
  --btn: #2c3e57; --btn-edge: #0f1b2b;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #13161b; --panel: #1c2027; --text: #e5e8ee; --muted: #9aa2b1;
    --line: #2d333d; --queued: #7fa6d6; --btn: #3b5373;
    --btn-edge: #a9bbd1;
  }
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--bg); color: var(--text);
  font: 15px/1.45 -apple-system, BlinkMacSystemFont, system-ui, sans-serif;
}
header {
  display: flex; flex-wrap: wrap; gap: 8px 20px; align-items: center;
  padding: 10px 16px; background: var(--panel);
  border-bottom: 1px solid var(--line);
}
header strong { font-size: 17px; }
code { font-size: 14px; }
.muted { color: var(--muted); }
.badge { padding: 2px 8px; border-radius: 6px; font-weight: 700; }
.badge.sim { background: var(--amber); color: #000; }
.badge.live { background: var(--green); color: #fff; }
main {
  display: grid; grid-template-columns: minmax(280px, 1fr)
  minmax(320px, 1.15fr); gap: 16px; padding: 16px;
}
.full { grid-column: 1 / -1; }
@media (max-width: 820px) { main { grid-template-columns: 1fr; } }
.panel {
  background: var(--panel); border: 1px solid var(--line);
  border-radius: 10px; padding: 12px 14px; margin-bottom: 16px;
}
h2 { font-size: 15px; margin: 0 0 8px; }
#stop {
  width: 100%; height: 40vh; min-height: 220px; border: 0;
  border-radius: 14px; background: var(--stop); color: #fff;
  font-size: 60px; font-weight: 800; letter-spacing: 6px; cursor: pointer;
}
#stop:active { filter: brightness(0.85); }
.pair {
  display: grid; grid-template-columns: 1fr 1fr; gap: 12px;
  margin-top: 12px;
}
.pair button {
  width: 100%; height: 64px; font-size: 21px; font-weight: 700;
  border-radius: 10px; border: 2px solid var(--btn-edge);
  background: var(--btn); color: #fff; cursor: pointer;
}
.pair button:disabled { opacity: 0.4; cursor: not-allowed; }
.reason {
  min-height: 2.6em; margin-top: 4px; font-size: 13px;
  color: var(--muted);
}
.note { margin: -4px 0 8px; font-size: 13px; }
#why { min-height: 1.4em; margin: 8px 2px 14px; }
.light {
  display: inline-block; width: 14px; height: 14px; border-radius: 50%;
  background: var(--grey); vertical-align: -2px;
}
.light.big { width: 30px; height: 30px; vertical-align: middle; }
.light.green { background: var(--green); }
.light.red { background: var(--red); }
.bigline { font-size: 19px; font-weight: 600; margin-bottom: 10px; }
table { width: 100%; border-collapse: collapse; }
td, th {
  text-align: left; padding: 4px 6px; border-bottom: 1px solid var(--line);
  font-variant-numeric: tabular-nums;
}
.kv td:first-child { color: var(--muted); width: 45%; }
.bad { color: var(--red); font-weight: 700; }
.st-queued, .log-ack_queued { color: var(--queued); }
.log-ack_other { color: var(--red); }
.st-rejected, .st-no-ack, .st-publish-failed { color: var(--red); }
ul { margin: 0; padding-left: 20px; }
li { margin-bottom: 4px; }
#log {
  max-height: 38vh; overflow: auto; font-size: 13px;
  font-family: ui-monospace, Menlo, monospace;
}
#log div { padding: 1px 0; border-bottom: 1px dotted var(--line); }
.log-no_ack, .log-cmd_failed, .log-decode_error { color: var(--red); }
.log-sim, .log-cmd_seen { color: var(--muted); }
.log-telemetry { color: var(--red); font-weight: 700; }
.log-retained_cmd { color: var(--red); font-weight: 700; }
.sim-buttons button { margin: 4px 6px 0 0; }
.tgrid {
  display: grid; gap: 16px; grid-column: 1 / -1;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
}
.tgrid .panel { margin-bottom: 0; }
.age { font-size: 12px; font-weight: 400; color: var(--muted); }
.panel.stale .vals { opacity: 0.4; }
.panel.stale .age { color: var(--red); font-weight: 700; }
.flag {
  display: inline-block; padding: 1px 8px; border-radius: 6px;
  font-size: 14px; font-weight: 700; background: var(--red); color: #fff;
  vertical-align: 2px;
}
.flag[hidden] { display: none; }
.impact {
  margin: 2px 0 6px; padding: 12px 8px; text-align: center;
  border: 2px solid var(--line); border-radius: 10px; font-size: 34px;
  font-weight: 800; letter-spacing: 4px; color: var(--muted);
}
.impact.hot { background: var(--red); border-color: var(--red); color: #fff; }
.letter {
  display: inline-block; min-width: 1.4em; margin-left: 10px;
  padding: 0 6px; text-align: center; border: 2px solid var(--line);
  border-radius: 8px; font-size: 26px; font-weight: 800;
}
.bar {
  height: 10px; margin-top: 2px; border: 1px solid var(--line);
  border-radius: 5px; overflow: hidden;
}
.bar i { display: block; width: 0; height: 100%; background: var(--queued); }
.tally { display: flex; flex-wrap: wrap; gap: 6px; margin: 4px 0; }
.tally span {
  padding: 1px 8px; border: 1px solid var(--line); border-radius: 6px;
  font-variant-numeric: tabular-nums;
}
#timeline {
  max-height: 180px; overflow: auto; font-size: 13px;
  font-family: ui-monospace, Menlo, monospace;
}
#timeline div { padding: 1px 0; border-bottom: 1px dotted var(--line); }
.tl-reject, .tl-lost { color: var(--red); }
.tl-junction { color: var(--queued); }
.tl-note { color: var(--muted); }
#imu {
  width: 100%; height: 90px; margin-top: 8px; border: 1px solid var(--line);
  border-radius: 6px;
}
#imu line { stroke: var(--line); stroke-width: 1; }
#imu polyline {
  fill: none; stroke-width: 1.6; vector-effect: non-scaling-stroke;
}
#g-pitch { stroke: var(--queued); }
#g-roll { stroke: var(--amber); stroke-dasharray: 5 3; }
#banner { display: none; padding: 10px 16px; font-weight: 600; }
#banner.warn { display: block; background: var(--red); color: #fff; }
#banner.done { display: block; background: var(--amber); color: #000; }
</style>
</head>
<body>
<header>
  <strong>S1 dashboard</strong>
  <span id="mode" class="badge">...</span>
  <span>prefix <code id="prefix"></code></span>
  <span>port <code id="port"></code></span>
  <span class="muted">broker from <span id="source"></span></span>
  <span><span id="broker-light" class="light"></span>
    broker <span id="broker-text">waiting</span></span>
  <span id="banner-off" class="muted"></span>
  <span id="stream" class="bad"></span>
</header>
<div id="banner"></div>
<main>
<section>
  <button id="stop" type="button" title="The Escape key also sends STOP"
    >STOP</button>
  <div class="pair">
    <div>
      <button id="start" type="button" disabled>START</button>
      <div id="start-why" class="reason"></div>
    </div>
    <div>
      <button id="reset" type="button" disabled>RESET</button>
      <div id="reset-why" class="reason"></div>
    </div>
  </div>
  <p id="why" class="muted"></p>
  <div class="panel">
    <h2>Last commands</h2>
    <p class="note muted">queued = sent to the robot's controller, not
      confirmed</p>
    <table>
      <thead><tr><th>id</th><th>command</th><th>sent</th><th>result</th>
        <th>round trip</th></tr></thead>
      <tbody id="cmds"></tbody>
    </table>
  </div>
</section>
<section>
  <div class="panel">
    <h2>Robot link</h2>
    <div class="bigline"><span id="robot-light" class="light big"></span>
      <span id="robot-text">no status from the robot</span></div>
    <table class="kv">
      <tr><td>heartbeat age</td><td id="age">none yet</td></tr>
      <tr><td>uptime</td><td id="uptime">-</td></tr>
      <tr><td>connect_count</td><td id="connect">-</td></tr>
      <tr><td>cmd_reject</td><td id="reject">-</td></tr>
      <tr><td>rssi</td><td id="rssi">-</td></tr>
      <tr><td>tel_drop / evt_drop</td><td id="drops">-</td></tr>
      <tr><td>heartbeats lost</td><td id="lost">-</td></tr>
      <tr><td>telemetry / live lost</td><td id="msg-lost">-</td></tr>
    </table>
  </div>
  <div class="panel">
    <h2>Operator rules</h2>
    <ul>
      <li>First test with the wheels off the ground.</li>
      <li>START runs the robot only from READY. In FINISHED, START only
        returns to READY; the next START runs.</li>
      <li>START is refused while running, while calibrating (up to 3 s
        after power-up, or after Button 2) and in FAULT (send RESET
        first).</li>
      <li>STOP is always accepted: it ends a run in FINISHED, and does
        nothing when the robot is not running. This button, the Escape
        key and the GP20 button all stop the robot.</li>
      <li>RESET works only from FAULT or FINISHED, back to READY with the
        motors braked.</li>
      <li>After a GP20 start the robot is running, so a dashboard START
        is refused; STOP still works. GP20 held for 2 s is RESET.</li>
      <li>TEST=1 images ignore START, STOP and RESET until an S5 test
        step arms the controller.</li>
      <li>"queued" means S5 received the command, not that it acted.
        S5's verdict is on the robot's console only (EVT 0x65 a=opcode
        b=1 or 0) until the event channel is built; the S5 panel shows
        the state change. "rejected" means S1 refused the command or
        could not hand it to S5.</li>
      <li>A command sent while the robot is offline is lost: no ack.</li>
    </ul>
  </div>
  <div class="panel" id="sim-panel" hidden>
    <h2>Simulator</h2>
    <div>fake car state: <b id="car-state">-</b>
      <span class="muted">(a real car's state is in the S5 panel)</span>
    </div>
    <div class="sim-buttons">
      <button type="button" data-sim="hb_pause">Pause heartbeat</button>
      <button type="button" data-sim="hb_resume">Resume heartbeat</button>
      <button type="button" data-sim="offline">Go offline</button>
      <button type="button" data-sim="online">Come back online</button>
      <button type="button" data-sim="fault">Stall fault</button>
      <button type="button" data-sim="gp20">GP20 press</button>
      <button type="button" data-sim="reject_next">Reject next</button>
      <button type="button" data-sim="drop_next">Drop next ack</button>
      <button type="button" data-sim="delay_next">Delay next ack 3 s
        </button>
      <button type="button" data-sim="dup_next">Send next ack twice
        </button>
      <button type="button" data-sim="retained_cmd">Retained command
        </button>
      <button type="button" data-sim="retained_clear">Clear retained
        </button>
      <button type="button" data-sim="s4_impact">S4 lifted/overturned</button>
      <button type="button" data-sim="s4_hump">S4 hump</button>
      <button type="button" data-sim="s3_barcode">S3 barcode</button>
      <button type="button" data-sim="s3_reject">S3 barcode rejected
        </button>
      <button type="button" data-sim="s3_junction">S3 junction</button>
      <button type="button" data-sim="s3_lost">S3 line lost</button>
    </div>
  </div>
</section>
<section class="tgrid">
  <div class="panel" id="p-s2">
    <h2>S2 motion <span class="age" id="age-s2"></span></h2>
    <div class="vals">
      <div class="bigline"><span id="s2-state">-</span>
        <span id="s2-estop" class="flag" hidden>E-STOP</span>
        <span id="s2-stall" class="flag" hidden>STALL</span></div>
      <table class="kv">
        <tr><td>commanded speed</td><td id="s2-cmd">-</td></tr>
        <tr><td>measured speed</td><td id="s2-speed">-</td></tr>
        <tr><td>left / right wheel</td><td id="s2-wheels">-</td></tr>
        <tr><td>distance</td><td id="s2-dist">-</td></tr>
        <tr><td>heading (wheel odometry)</td><td id="s2-head">-</td></tr>
        <tr><td>fault bits</td><td id="s2-faults">-</td></tr>
      </table>
    </div>
  </div>
  <div class="panel" id="p-s3">
    <h2>S3 line and barcodes <span class="age" id="age-s3"></span></h2>
    <div class="vals">
      <div class="bigline"><span id="s3-state">-</span>
        <span id="s3-letter" class="letter">-</span></div>
      <table class="kv">
        <tr><td>outer sensors</td><td><span id="s3-outer">-</span>
          <div class="bar"><i id="s3-outer-bar"></i></div></td></tr>
        <tr><td>centre sensors</td><td><span id="s3-centre">-</span>
          <div class="bar"><i id="s3-centre-bar"></i></div></td></tr>
        <tr><td>steering</td><td id="s3-steer">-</td></tr>
        <tr><td>OUTER rule</td><td id="s3-over">-</td></tr>
        <tr><td>last barcode</td><td id="s3-bc">none yet</td></tr>
        <tr><td>last junction</td><td id="s3-junc">none yet</td></tr>
        <tr><td>robot counts</td><td id="s3-counts">-</td></tr>
      </table>
      <h2>Barcodes accepted, by letter</h2>
      <div class="tally" id="s3-tally">none yet</div>
      <div class="muted">rejected: <b id="s3-rejected">0</b> (S3 sends a
        count, not the letter)</div>
      <h2>Timeline</h2>
      <div id="timeline"></div>
    </div>
  </div>
  <div class="panel" id="p-s4">
    <h2>S4 motion sensor <span class="age" id="age-s4"></span></h2>
    <div class="vals">
      <div class="impact" id="s4-impact">OK</div>
      <div class="muted" id="s4-impacts">lift/overturn stops: 0</div>
      <table class="kv">
        <tr><td>pitch / roll</td><td id="s4-angles">-</td></tr>
        <tr><td>turn rate</td><td id="s4-turn">-</td></tr>
        <tr><td>motion class</td><td id="s4-class">-</td></tr>
        <tr><td>hump</td><td id="s4-hump">-</td></tr>
        <tr><td>humps / peak height</td><td id="s4-humps">-</td></tr>
        <tr><td>IMU</td><td id="s4-imu">-</td></tr>
      </table>
      <svg id="imu" viewBox="0 0 240 90" preserveAspectRatio="none"
        role="img" aria-label="pitch and roll, last 60 seconds">
        <line x1="0" y1="45" x2="240" y2="45"></line>
        <polyline id="g-pitch" points=""></polyline>
        <polyline id="g-roll" points=""></polyline>
      </svg>
      <div class="note muted">pitch (solid) and roll (dashed), last 60 s,
        scale +/-32 degrees</div>
    </div>
  </div>
  <div class="panel" id="p-s5">
    <h2>S5 controller <span class="age" id="age-s5"></span></h2>
    <div class="vals">
      <div class="bigline" id="s5-state">-</div>
      <table class="kv">
        <tr><td>calibration</td><td id="s5-cal">-</td></tr>
        <tr><td>fault bits</td><td id="s5-faults">-</td></tr>
        <tr><td>closest obstacle</td><td id="s5-closest">-</td></tr>
        <tr><td>gap width</td><td id="s5-width">-</td></tr>
        <tr><td>clear left / right</td><td id="s5-clear">-</td></tr>
      </table>
    </div>
  </div>
</section>
<section class="panel full">
  <h2>Event log</h2>
  <div id="log"></div>
</section>
</main>
<script>
"use strict";
const TOKEN = __TOKEN__;
const el = (id) => document.getElementById(id);
const rows = new Map();
let hbBase = null;
let lastControls = {start: false, reset: false, start_why: "",
                    reset_why: ""};

function post(path, body) {
  return fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json", "X-S1-Token": TOKEN},
    body: JSON.stringify(body)
  }).then((r) => r.json().then((j) => ({status: r.status, body: j})))
    .catch((e) => ({status: 0, body: {error: String(e)}}));
}

function setControls(c) {
  lastControls = c;
  el("start").disabled = !c.start;
  el("reset").disabled = !c.reset;
  el("start-why").textContent = c.start ? "" : (c.start_why || "");
  el("reset-why").textContent = c.reset ? "" : (c.reset_why || "");
}

function send(op) {
  post("/api/cmd", {op: op}).then((r) => {
    el("why").textContent = "";
    if (r.status !== 200) {
      setControls(lastControls);
      el("why").textContent = (r.body && r.body.error) ? r.body.error
        : "not sent (HTTP " + r.status + ")";
    }
  });
}

el("stop").addEventListener("click", () => send("stop"));
el("start").addEventListener("click", () => {
  el("start").disabled = true;
  send("start");
});
el("reset").addEventListener("click", () => {
  el("reset").disabled = true;
  send("reset");
});
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    e.preventDefault();
    send("stop");
  }
});
document.querySelectorAll("[data-sim]").forEach((b) => {
  b.addEventListener("click", () => post("/api/sim",
                                         {action: b.dataset.sim}));
});

function setLight(id, color, big) {
  el(id).className = "light" + (big ? " big " : " ") + color;
}

function setBroker(b) {
  const up = b && b.state === "up";
  setLight("broker-light", up ? "green" : "red", false);
  el("broker-text").textContent = up ? "connected" : "not connected";
}

function setLink(l) {
  setLight("robot-light", l.light, true);
  el("robot-text").textContent = l.reason;
}

function uptime(s) {
  const h = Math.floor(s / 3600);
  const m = Math.floor(s / 60) % 60;
  const x = s % 60;
  return h + ":" + String(m).padStart(2, "0") + ":"
    + String(x).padStart(2, "0");
}

function setHeartbeat(h, ageMs) {
  if (!h) {
    hbBase = null;
    return;
  }
  hbBase = performance.now() - ageMs;
  el("uptime").textContent = uptime(h.uptime_s);
  el("connect").textContent = h.connect_count;
  el("reject").textContent = h.cmd_reject;
  el("rssi").textContent = h.rssi_text;
  el("drops").textContent = h.tel_drop_text + " / " + h.evt_drop;
  el("lost").textContent = h.lost_total;
}

setInterval(() => {
  if (hbBase === null) {
    el("age").textContent = "none yet";
    el("age").className = "";
    return;
  }
  const age = (performance.now() - hbBase) / 1000;
  el("age").textContent = age.toFixed(1) + " s";
  el("age").className = age > 3 ? "bad" : "";
}, 200);

function upsertRow(c) {
  let tr = rows.get(c.n);
  if (!tr) {
    tr = document.createElement("tr");
    for (let i = 0; i < 5; i++) {
      tr.appendChild(document.createElement("td"));
    }
    el("cmds").prepend(tr);
    rows.set(c.n, tr);
    while (el("cmds").children.length > 20) {
      const last = el("cmds").lastElementChild;
      rows.forEach((v, k) => { if (v === last) { rows.delete(k); } });
      last.remove();
    }
  }
  const td = tr.children;
  if (c.cmd_id !== undefined) { td[0].textContent = c.cmd_id; }
  if (c.op) { td[1].textContent = c.op; }
  if (c.sent_at) { td[2].textContent = c.sent_at.slice(11); }
  if (c.state) {
    const key = c.state.startsWith("queued") ? "queued"
      : c.state.startsWith("rejected") ? "rejected"
      : c.state.replace(/\s+/g, "-");
    td[3].textContent = c.state;
    td[3].className = "st-" + key;
  }
  if (typeof c.rtt_ms === "number") {
    td[4].textContent = c.rtt_ms.toFixed(1) + " ms";
  }
}

function addLog(kind, d) {
  if (!d || !d.log) { return; }
  const div = document.createElement("div");
  div.className = "log-" + kind;
  div.textContent = (d.t || "").slice(11) + "  " + d.log;
  el("log").prepend(div);
  while (el("log").children.length > 300) {
    el("log").lastElementChild.remove();
  }
}

const STALE_S = 2;
const IMPACT_HOLD_S = 3;
const IMU_KEEP = 120;
const feedAt = {live: null, tel: null};
const PANELS = [["p-s2", ["live", "tel"], "age-s2"],
                ["p-s3", ["live"], "age-s3"],
                ["p-s4", ["tel"], "age-s4"],
                ["p-s5", ["tel"], "age-s5"]];
let impactAt = null;
let imu = [];
let liveSeen = false;
const msgLost = {tel: 0, live: 0};

function markFeed(feed, ageMs) {
  feedAt[feed] = (ageMs === null || ageMs === undefined) ? null
    : performance.now() - ageMs;
}

function text(id, value) { el(id).textContent = value; }

function num(v, unit, digits) {
  return (typeof v === "number") ? v.toFixed(digits || 0) + (unit ? " " + unit : "")
    : "-";
}

function list(names) { return names.length ? names.join(", ") : "none"; }

function paintAges() {
  const now = performance.now();
  PANELS.forEach(([panel, feeds, ageId]) => {
    const marks = feeds.map((f) => feedAt[f]);
    let stale = true;
    let label = "no data yet";
    if (marks.every((m) => m !== null)) {
      const age = (now - Math.min(...marks)) / 1000;
      stale = age > STALE_S;
      label = stale ? "no data for " + age.toFixed(1) + " s"
        : "updated " + age.toFixed(1) + " s ago";
    }
    el(panel).classList.toggle("stale", stale);
    text(ageId, label);
  });
  const hot = impactAt !== null && (now - impactAt) / 1000 < IMPACT_HOLD_S;
  el("s4-impact").className = hot ? "impact hot" : "impact";
  text("s4-impact", hot ? "LIFTED / OVERTURNED" : "OK");
}
setInterval(paintAges, 200);

function drawImu() {
  const x = (i) => 240 - (imu.length - 1 - i) * 2;
  const y = (v) => 45 - Math.max(-32, Math.min(32, v)) * 45 / 32;
  [["g-pitch", 0], ["g-roll", 1]].forEach(([id, k]) => {
    el(id).setAttribute("points", imu.map((p, i) =>
      x(i).toFixed(1) + "," + y(p[k]).toFixed(1)).join(" "));
  });
}

function setMsgLost() {
  text("msg-lost", msgLost.tel + " / " + msgLost.live);
}

function setTelemetry(t, ageMs) {
  if (!t) { return; }
  markFeed("tel", ageMs);
  msgLost.tel = t.lost_total;
  setMsgLost();
  if (!liveSeen) { text("s2-state", t.motion_name); }
  text("s2-speed", num(t.speed_mms, "mm/s"));
  text("s2-wheels", num(t.speed_l_mms) + " / " + num(t.speed_r_mms)
       + " mm/s");
  text("s2-dist", num(t.dist_mm, "mm") + " (" + num(t.dist_mm / 1000, "m", 2)
       + ")");
  text("s2-faults", list(t.faults));
  el("s2-faults").className = t.faults.length ? "bad" : "";
  el("s2-stall").hidden = !t.faults.includes("stall");
  el("s4-impacts").textContent = "lift/overturn stops: " + t.impact_count
    + (t.impact_at ? " (last at " + t.impact_at.slice(11, 19) + ")" : "");
  if (t.b_impact) { impactAt = performance.now() - (ageMs || 0); }
  text("s4-angles", num(t.pitch_deg, "°", 1) + " / "
       + num(t.roll_deg, "°", 1));
  text("s4-turn", num(t.turn_dps, "deg/s"));
  text("s4-class", t.class_name);
  text("s4-hump", t.hump_name);
  text("s4-humps", t.hump_count + " / " + num(t.peak_mm, "mm"));
  text("s4-imu", t.b_imu_ok ? "ok" : "NOT OK");
  el("s4-imu").className = t.b_imu_ok ? "" : "bad";
  text("s5-state", t.ctrl_name);
  text("s5-cal", t.b_cal_done ? "done" : "not done");
  text("s5-faults", list(t.faults));
  el("s5-faults").className = t.faults.length ? "bad" : "";
  text("s5-closest", t.closest_mm ? num(t.closest_mm, "mm") + " at "
       + num(t.angle_idx * 7.5, "\u00b0", 1) + " (servo)" : "none");
  text("s5-width", t.width_mm ? num(t.width_mm, "mm") : "-");
  text("s5-clear", (t.width_mm || t.clear_l_mm || t.clear_r_mm)
       ? num(t.clear_l_mm) + " / " + num(t.clear_r_mm) + " mm" : "-");
}

function setLive(l, ageMs) {
  if (!l) { return; }
  markFeed("live", ageMs);
  msgLost.live = l.lost_total;
  setMsgLost();
  liveSeen = true;
  text("s2-state", l.motion_name);
  el("s2-estop").hidden = !l.b_estop;
  text("s2-cmd", num(l.cmd_speed_mms, "mm/s"));
  text("s2-head", num(l.heading_deg, "°", 1));
  text("s3-state", l.line_name);
  el("s3-state").className = (l.line_name === "LOST") ? "bad" : "";
  text("s3-letter", l.bc_char || "-");
  text("s3-outer", num(l.outer_ir) + " of 255");
  el("s3-outer-bar").style.width = (l.outer_ir * 100 / 255) + "%";
  text("s3-centre", num(l.centre_ir) + " of 255");
  el("s3-centre-bar").style.width = (l.centre_ir * 100 / 255) + "%";
  text("s3-steer", num(l.steer_dv_mms, "mm/s"));
  text("s3-over", l.b_overshoot ? "on" : "off");
  text("s3-bc", l.bc_char ? l.bc_char + ": " + l.bc_cmd_name + ", read "
       + (l.b_bc_rev ? "backwards" : "forwards") : "none yet");
  text("s3-junc", l.junctions ? "turn " + l.junc_name : "none yet");
  text("s3-counts", "barcodes " + l.barcodes + ", rejected " + l.rejects
       + " (wraps at 16), junctions " + l.junctions + ", lost " + l.losts);
}

function addTimeline(entry) {
  const div = document.createElement("div");
  div.className = "tl-" + entry.kind;
  div.textContent = entry.t.slice(11, 21) + "  " + entry.text;
  el("timeline").prepend(div);
  while (el("timeline").children.length > 100) {
    el("timeline").lastElementChild.remove();
  }
}

function setTally(tally, rejected) {
  const box = el("s3-tally");
  box.textContent = "";
  const symbols = Object.keys(tally).sort();
  if (symbols.length === 0) { box.textContent = "none yet"; }
  symbols.forEach((k) => {
    const chip = document.createElement("span");
    chip.textContent = k + " × " + tally[k];
    box.appendChild(chip);
  });
  text("s3-rejected", rejected);
}

const banner = {state: "ok", why: "", entries: new Map()};

function renderBanner() {
  const box = el("banner");
  const all = Array.from(banner.entries.values());
  const open = all.filter((e) => !e.cleared);
  el("banner-off").textContent = banner.state === "ok" ? ""
    : "retained-command banner unavailable: " + banner.why;
  if ((banner.state !== "ok") || (all.length === 0)) {
    box.className = "";
    box.textContent = "";
  } else if (open.length > 0) {
    box.className = "warn";
    box.textContent = "WARNING: a retained command is stored on "
      + open.map((e) => el("prefix").textContent + "/" + e.topic_suffix
                 + " (" + e.hex + ", since " + e.at.slice(11) + ")")
        .join(", ")
      + ". The robot would run it again after every reconnect. Clear it:"
      + " docs/S1_dashboard.md, \"Clearing a retained command\".";
  } else {
    box.className = "done";
    box.textContent = "Retained command cleared: "
      + all.map((e) => e.topic_suffix + " at " + e.cleared_at.slice(11))
        .join(", ") + ".";
  }
}

function applySnapshot(s) {
  const sim = s.mode === "simulate";
  el("mode").textContent = sim ? "SIMULATED (no robot)" : "LIVE";
  el("mode").className = "badge " + (sim ? "sim" : "live");
  el("prefix").textContent = s.prefix;
  el("port").textContent = s.port;
  el("source").textContent = s.config_source;
  setBroker(s.broker);
  setLink(s.link);
  setControls(s.controls);
  setHeartbeat(s.heartbeat, s.hb_age_ms || 0);
  el("cmds").textContent = "";
  rows.clear();
  s.commands.forEach(upsertRow);
  el("log").textContent = "";
  s.log.forEach((e) => addLog(e.kind, {t: e.t, log: e.text}));
  el("sim-panel").hidden = !sim;
  el("car-state").textContent = s.car_state || "-";
  liveSeen = false;
  imu = s.imu_history.slice();
  drawImu();
  setTelemetry(s.telemetry, s.tel_age_ms);
  setLive(s.live, s.live_age_ms);
  if (!s.telemetry) { markFeed("tel", null); }
  if (!s.live) { markFeed("live", null); }
  el("timeline").textContent = "";
  s.s3_timeline.forEach(addTimeline);
  setTally(s.s3_tally, s.s3_rejected);
  paintAges();
  banner.state = s.banner;
  banner.why = s.banner_why || "";
  banner.entries = new Map(s.retained_cmd.map((e) => [e.topic_suffix, e]));
  renderBanner();
}

const es = new EventSource("/events");
es.onopen = () => { el("stream").textContent = ""; };
es.onerror = () => {
  el("stream").textContent = "dashboard server not reachable: retrying";
};
function on(name, fn) {
  es.addEventListener(name, (ev) => fn(JSON.parse(ev.data)));
}
on("snapshot", applySnapshot);
on("broker", (d) => { setBroker(d); addLog("broker", d); });
on("status", (d) => addLog("status", d));
on("link", (d) => { setLink(d); addLog("link", d); });
on("heartbeat", (d) => { setHeartbeat(d, 0); addLog("heartbeat", d); });
on("controls", setControls);
on("cmd", (d) => { upsertRow(d); addLog("cmd", d); });
on("ack", (d) => {
  if (d.n !== undefined) { upsertRow(d); }
  addLog(d.result === 0 ? "ack_queued" : "ack_other", d);
});
on("no_ack", (d) => { upsertRow(d); addLog("no_ack", d); });
on("cmd_failed", (d) => { upsertRow(d); addLog("cmd_failed", d); });
on("cmd_seen", (d) => addLog("cmd_seen", d));
on("decode_error", (d) => addLog("decode_error", d));
on("telemetry", (d) => {
  setTelemetry(d, 0);
  addLog("telemetry", d);
  imu.push([d.pitch_deg, d.roll_deg]);
  if (imu.length > IMU_KEEP) { imu.shift(); }
  drawImu();
  paintAges();
});
on("live", (d) => { setLive(d, 0); paintAges(); });
on("s3_events", (d) => {
  d.entries.forEach(addTimeline);
  setTally(d.tally, d.rejected);
});
on("event", (d) => addLog("event", d));
on("note", (d) => addLog("note", d));
on("retained_cmd", (d) => {
  banner.entries.set(d.topic_suffix, d);
  renderBanner();
  addLog("retained_cmd", d);
});
on("banner", (d) => {
  banner.state = d.banner;
  banner.why = d.why;
  banner.entries.clear();
  renderBanner();
  addLog("banner", d);
});
on("sim", (d) => {
  el("car-state").textContent = d.car_state || "-";
  addLog("sim", d);
});
</script>
</body>
</html>
"""


def make_server(dash, http_port):
    """The HTTP server on 127.0.0.1, with a fresh page token."""
    server = Server((BIND_ADDRESS, http_port), Handler)
    port = server.server_address[1]
    server.dash = dash
    server.token = secrets.token_hex(16)
    server.allowed_hosts = {"%s:%d" % (BIND_ADDRESS, port),
                            "localhost:%d" % port}
    return server


def serve(args):
    """Run the dashboard until Ctrl-C.

    An unexpected error, in this thread or another, is printed as one
    scrubbed line with no traceback (report_error()).
    """
    settings = settings_from(args)
    scrub = Scrubber(settings)

    def on_thread_error(hook):
        report_error(scrub, "thread error", hook.exc_value)

    threading.excepthook = on_thread_error
    try:
        return run_server(args, settings, scrub)
    except Exception as err:  # The top-level handler: one scrubbed line.
        report_error(scrub, "stopped", err)
        return 1


def run_server(args, settings, scrub):
    mode = "simulate" if args.simulate else "live"
    if not args.simulate:
        for tool in ("mosquitto_sub", "mosquitto_pub"):
            if shutil.which(tool) is None:
                sys.exit("s1_dashboard: %s is not on the PATH "
                         "(brew install mosquitto)" % tool)
    if args.banner_off:
        banner_why = "switched off with --banner-off"
    elif (not args.simulate) and (not banner_supported()):
        banner_why = ("this mosquitto_sub lacks -V mqttv5 or "
                      "--retain-as-published (mosquitto 2.0 or later)")
    else:
        banner_why = None
    dash = Dashboard(settings, scrub, mode, args.ack_timeout, args.log,
                     banner_why)
    if args.simulate:
        link = FakeCar(settings, scrub, args.opts_dir, banner_why is None)
    else:
        link = MosquittoLink(settings, scrub, args.opts_dir,
                             banner_why is None)
    dash.attach(link)
    server = make_server(dash, args.http_port)
    port = server.server_address[1]
    stop = threading.Event()

    def ticker():
        while not stop.wait(TICK_S):
            dash.tick()

    def on_term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    print(scrub.text("s1_dashboard: %s mode, broker from %s, port %d, "
                     "prefix %s" % (mode, settings.source, settings.port,
                                    settings.prefix)), flush=True)
    print("s1_dashboard: retained-command banner %s"
          % ("on" if banner_why is None else "unavailable: " + banner_why),
          flush=True)
    print("s1_dashboard: open http://%s:%d/ (Ctrl-C to quit)"
          % (BIND_ADDRESS, port), flush=True)
    link.start()
    threading.Thread(target=ticker, daemon=True).start()
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        link.stop()
        server.server_close()
        dash.close()
        print("s1_dashboard: closed", flush=True)
    return 0


# -- S1-U16 part A: --check ---------------------------------------------

FAKE_HOST = "203.0.113.77"      # TEST-NET-3: a documentation address.
FAKE_USER = "u-s1u16-login"
FAKE_PASSWORD = "p-s1u16-secret"
FAKE_DIGITS = "1234"            # A login of digits only (A12).
FAKE_PORT = 1884
FAKE_PREFIX = "s1u16"
ALL_STEPS = ["A%d" % number for number in range(1, 15)]
# Measured between client receipts: allow 0.1 s of jitter below the 3 s limit.
A8_RED_MIN_S = 2.9
A8_RED_MAX_S = 3.5


def http_call(port, method, path, body=None, headers=None, host=None):
    """One request to the dashboard; returns (status, text)."""
    conn = http.client.HTTPConnection(BIND_ADDRESS, port, timeout=10)
    sent = {"Host": host if host else "%s:%d" % (BIND_ADDRESS, port)}
    data = None
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        sent["Content-Type"] = "application/json"
    sent.update(headers or {})
    conn.request(method, path, body=data, headers=sent)
    resp = conn.getresponse()
    text = resp.read().decode("utf-8", "replace")
    conn.close()
    return resp.status, text


def page_headers(port):
    """The response headers of GET /, names in lower case (A1)."""
    conn = http.client.HTTPConnection(BIND_ADDRESS, port, timeout=10)
    conn.request("GET", "/", headers={"Host": "%s:%d" % (BIND_ADDRESS,
                                                          port)})
    resp = conn.getresponse()
    resp.read()
    headers = {name.lower(): value for name, value in resp.getheaders()}
    conn.close()
    return headers


class Stream:
    """One event-stream connection, read on its own thread."""

    def __init__(self, port):
        self.events = []
        self.raw = bytearray()
        self.cond = threading.Condition()
        self._port = port
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        conn = http.client.HTTPConnection(BIND_ADDRESS, self._port,
                                          timeout=120)
        conn.request("GET", "/events",
                     headers={"Host": "%s:%d" % (BIND_ADDRESS, self._port)})
        resp = conn.getresponse()
        name, data = None, []
        for line in iter(resp.readline, b""):
            text = line.decode("utf-8", "replace").rstrip("\n")
            with self.cond:
                self.raw.extend(line)
                if text.startswith("event: "):
                    name = text[7:]
                elif text.startswith("data: "):
                    data.append(text[6:])
                elif (text == "") and (name is not None):
                    self.events.append((time.monotonic(), name,
                                        json.loads("".join(data))))
                    name, data = None, []
                    self.cond.notify_all()

    def mark(self):
        with self.cond:
            return len(self.events)

    def wait_for(self, name, pred=None, timeout=3.0, since=0):
        """Return the first (index, time, data) that matches, or None."""
        deadline = time.monotonic() + timeout
        with self.cond:
            while True:
                for index in range(since, len(self.events)):
                    when, event, data = self.events[index]
                    if (event == name) and ((pred is None) or pred(data)):
                        return index, when, data
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self.cond.wait(left)

    def all_of(self, name, since=0):
        with self.cond:
            return [data for _, event, data in self.events[since:]
                    if event == name]


class CheckRun:
    """The dashboard under test: a child process in simulate mode."""

    def __init__(self, tmpdir, extra):
        self.tmpdir = tmpdir
        self.config = os.path.join(tmpdir, "mqtt_config.h")
        self.log = os.path.join(tmpdir, "dashboard_log.jsonl")
        self.out = os.path.join(tmpdir, "stdout.txt")
        self.err = os.path.join(tmpdir, "stderr.txt")
        with open(self.config, "w", encoding="utf-8") as handle:
            handle.write(
                "#define MQTT_LINK_CFG_SET               1\n"
                "#define MQTT_LINK_CFG_BROKER_HOST       \"%s\"\n"
                "#define MQTT_LINK_CFG_BROKER_PORT       %dU\n"
                "#define MQTT_LINK_CFG_CLIENT_ID         \"s1u16-car\"\n"
                "#define MQTT_LINK_CFG_TOPIC_PREFIX      \"%s\"\n"
                "#define MQTT_LINK_CFG_USERNAME          \"%s\"\n"
                "#define MQTT_LINK_CFG_PASSWORD          \"%s\"\n"
                % (FAKE_HOST, FAKE_PORT, FAKE_PREFIX, FAKE_USER,
                   FAKE_PASSWORD))
        argv = [sys.executable, "-I", "-B", os.path.abspath(__file__),
                "--simulate", "--config", self.config, "--http-port", "0",
                "--log", self.log, "--opts-dir", tmpdir] + extra
        self._out = open(self.out, "w", encoding="utf-8")
        self._err = open(self.err, "w", encoding="utf-8")
        self.proc = subprocess.Popen(argv, stdout=self._out,
                                     stderr=self._err)
        self.port = None
        deadline = time.monotonic() + 15.0
        while (self.port is None) and (time.monotonic() < deadline):
            time.sleep(0.05)
            with open(self.out, encoding="utf-8") as handle:
                match = re.search(r"http://127\.0\.0\.1:(\d+)/",
                                  handle.read())
            if match is not None:
                self.port = int(match.group(1))
        if self.port is None:
            self.stop()
            tail = self.read(self.err).strip().splitlines()[-3:]
            for secret in (FAKE_HOST, FAKE_USER, FAKE_PASSWORD):
                tail = [line.replace(secret, "<secret>") for line in tail]
            raise RuntimeError("the dashboard did not start: %s"
                               % " / ".join(tail))

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._out.close()
        self._err.close()

    def read(self, path):
        try:
            with open(path, encoding="utf-8", errors="replace") as handle:
                return handle.read()
        except OSError:
            return ""


def other_ipv4_addresses():
    """This machine's non-loopback IPv4 addresses (never printed)."""
    found = set()
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # A UDP connect sends nothing; it only picks the outgoing address.
        probe.connect(("192.0.2.1", 9))
        found.add(probe.getsockname()[0])
        probe.close()
    except OSError:
        pass
    return sorted(addr for addr in found
                  if (not addr.startswith("127.")) and (addr != "0.0.0.0"))


def post_json(ctx, path, body):
    status, text = http_call(ctx.port, "POST", path, body,
                             {"X-S1-Token": ctx.token})
    try:
        reply = json.loads(text)
    except ValueError:
        reply = {}
    return status, reply


def check_a1(ctx):
    status_page, _ = http_call(ctx.port, "GET", "/")
    others = other_ipv4_addresses()
    refused = 0
    for addr in others:
        try:
            socket.create_connection((addr, ctx.port), timeout=1.0).close()
        except OSError:
            refused += 1
    status_host, _ = http_call(ctx.port, "GET", "/",
                               host="dashboard.example:%d" % ctx.port)
    status_none, _ = http_call(ctx.port, "POST", "/api/cmd", {"op": "stop"})
    status_bad, _ = http_call(ctx.port, "POST", "/api/cmd", {"op": "stop"},
                              {"X-S1-Token": "0" * 32})
    status_kind, _ = http_call(ctx.port, "POST", "/api/cmd", None,
                               {"X-S1-Token": ctx.token,
                                "Content-Type": "text/plain"})
    status_list, _ = http_call(ctx.port, "POST", "/api/cmd",
                               {"op": ["start"]}, {"X-S1-Token": ctx.token})
    status_number, _ = http_call(ctx.port, "POST", "/api/cmd", {"op": 5},
                                 {"X-S1-Token": ctx.token})
    headers = page_headers(ctx.port)
    b_headers = ((headers.get("content-security-policy") == PAGE_CSP)
                 and (headers.get("x-content-type-options") == "nosniff")
                 and (headers.get("cache-control") == "no-store"))
    status_live = ctx.live["sim_status"]
    if others:
        cross = "refused on %d of %d other IPv4 addresses" % (refused,
                                                             len(others))
    else:
        cross = "no other IPv4 address up, not cross-checked"
    ok = ((status_page == 200) and (refused == len(others))
          and (status_host == 403) and (status_none == 403)
          and (status_bad == 403) and (status_kind == 415)
          and (status_list == 400) and (status_number == 400) and b_headers
          and (status_live == 404))
    return ok, ("page %d on 127.0.0.1; %s; wrong Host %d; no token %d; "
                "wrong token %d; text/plain %d; op as a list %d, as a number "
                "%d; page headers CSP, nosniff and no-store: %s; /api/sim in "
                "live mode %d"
                % (status_page, cross, status_host, status_none, status_bad,
                   status_kind, status_list, status_number, b_headers,
                   status_live))


def check_a2(ctx):
    tag = re.search(r'<button[^>]*id="stop"[^>]*>', ctx.page)
    b_tag = (tag is not None) and ("disabled" not in tag.group(0))
    b_script = re.search(r'"stop"\)\.disabled|\bstop\.disabled',
                         ctx.page) is None
    controls = ctx.stream.all_of("controls")
    snapshot = ctx.stream.events[0][2].get("controls", {})
    b_events = (len(controls) >= 2) and all(
        c.get("stop") is True for c in controls + [snapshot])
    return (b_tag and b_script and b_events,
            "STOP tag without disabled: %s; no script line disables it: %s; "
            "stop true in all %d controls events and the snapshot: %s"
            % (b_tag, b_script, len(controls), b_events))


def check_a3(ctx):
    first = ctx.stream.events[0][1] if ctx.stream.events else None
    snap = ctx.stream.events[0][2] if ctx.stream.events else {}
    status = snap.get("status", {})
    b_online = (status.get("value") == "online") or (ctx.stream.wait_for(
        "status", lambda d: d["value"] == "online", 3.0) is not None)
    hb = ctx.stream.wait_for("heartbeat", None, 4.0)
    green = ctx.stream.wait_for("link", lambda d: d["light"] == "green",
                                5.0)
    delay = None
    if (hb is not None) and (green is not None):
        delay = green[1] - hb[1]
    b_green = (delay is not None) and (0.0 <= delay <= 1.5)
    b_rssi = (hb is not None) and (hb[2].get("rssi_text") == "not measured")
    ok = (first == "snapshot") and b_online and b_green and b_rssi
    return ok, ("first event %s; status online (retained %s): %s; light "
                "green %s s after the first heartbeat; rssi text %r"
                % (first, status.get("retained"), b_online,
                   "%.3f" % delay if delay is not None else "never",
                   hb[2].get("rssi_text") if hb else None))


def check_a4(ctx):
    ready = ctx.stream.wait_for("sim", lambda d: d["car_state"] == "READY",
                                6.0)
    since = ctx.stream.mark()
    status1, reply1 = post_json(ctx, "/api/cmd", {"op": "start"})
    status2, _ = post_json(ctx, "/api/cmd", {"op": "start"})
    off = ctx.stream.wait_for("controls", lambda d: d["start"] is False,
                              1.0, since)
    ack = ctx.stream.wait_for("ack", lambda d: d["cmd_id"] == 0, 2.5, since)
    back = None
    if ack is not None:
        back = ctx.stream.wait_for("controls", lambda d: d["start"] is True,
                                   2.5, ack[0])
    data = ack[2] if ack else {}
    ok = ((ready is not None) and (status1 == 200)
          and (reply1.get("cmd_id") == 0) and (status2 == 409)
          and (off is not None) and (back is not None)
          and (data.get("match") == "matched")
          and (data.get("label") == "queued")
          and isinstance(data.get("rtt_ms"), float)
          and (data.get("rtt_ms") > 0.0))
    return ok, ("fake car READY: %s; START %d, cmd_id %s; second START %d; "
                "START disabled then enabled: %s; ack %s, %s, %s ms"
                % (ready is not None, status1, reply1.get("cmd_id"), status2,
                   (off is not None) and (back is not None),
                   data.get("match"), data.get("label"), data.get("rtt_ms")))


def check_a5(ctx):
    since = ctx.stream.mark()
    ids, bad, replies = [], 0, []
    for _ in range(70):
        status, reply = post_json(ctx, "/api/cmd", {"op": "stop"})
        number = reply.get("n")
        replies.append((number, reply.get("cmd_id")))
        ack = ctx.stream.wait_for("ack", lambda d, n=number: d.get("n") == n,
                                  2.5, since)
        if ((status != 200) or (ack is None)
                or (ack[2]["cmd_id"] != reply.get("cmd_id"))
                or (ack[2]["match"] != "matched")):
            bad += 1
        ids.append(reply.get("cmd_id"))
    expected = list(range(1, 64)) + list(range(0, 7))
    unmatched = sum(1 for d in ctx.stream.all_of("ack", since)
                    if d["match"] != "matched")
    missing = len(ctx.stream.all_of("no_ack", since))
    crossed = cross_check_ids(ctx, since, replies)
    ok = (ids == expected) and (bad == 0) and (unmatched == 0) and (
        missing == 0) and (crossed == 70)
    return ok, ("cmd_ids %s..%s then %s..%s as expected: %s; acks matched "
                "to their own id: %d of 70; unmatched %d; no ack %d; payload "
                "and cmd event agree with the reply's cmd_id: %d of 70"
                % (ids[0], ids[62], ids[63], ids[69], ids == expected,
                   70 - bad, unmatched, missing, crossed))


def cross_check_ids(ctx, since, replies):
    """Count the STOPs whose cmd event and published payload both carry
    the cmd_id the server replied with (A5)."""
    events = {d["n"]: d for d in ctx.stream.all_of("cmd", since)}
    crossed = 0
    for number, cmd_id in replies:
        event = events.get(number)
        if event is None:
            continue
        msg = s1_decode.decode(bytes.fromhex(event["hex"]))
        if ((event["cmd_id"] == cmd_id) and (event["op"] == "STOP")
                and (msg["cmd_id"] == cmd_id)
                and (msg["opcode"] == cmd_rtt.OPCODES["stop"])):
            crossed += 1
    return crossed


def check_a8(ctx):
    since = ctx.stream.mark()
    post_json(ctx, "/api/sim", {"action": "hb_pause"})
    red = ctx.stream.wait_for("link", lambda d: d["light"] == "red", 6.0,
                              since)
    delay, b_controls, b_stop = None, False, False
    if red is not None:
        beats = [when for when, event, _ in ctx.stream.events[:red[0]]
                 if event == "heartbeat"]
        delay = red[1] - beats[-1] if beats else None
        _, text = http_call(ctx.port, "GET", "/api/state")
        state = json.loads(text)
        controls = state["controls"]
        b_controls = ((controls["start"] is False)
                      and (controls["reset"] is False)
                      and (controls["stop"] is True))
        before = len(state["publish_args"])
        status, _ = post_json(ctx, "/api/cmd", {"op": "stop"})
        _, text = http_call(ctx.port, "GET", "/api/state")
        b_stop = (status == 200) and (
            len(json.loads(text)["publish_args"]) == before + 1)
    post_json(ctx, "/api/sim", {"action": "hb_resume"})
    green = ctx.stream.wait_for("link", lambda d: d["light"] == "green",
                                3.0, red[0] if red else since)
    ok = ((delay is not None) and (A8_RED_MIN_S <= delay <= A8_RED_MAX_S)
          and b_controls and b_stop and (green is not None))
    return ok, ("light red %s s after the last heartbeat; START and RESET "
                "disabled, STOP enabled: %s; STOP published while red: %s; "
                "green after the resume: %s"
                % ("%.3f" % delay if delay is not None else "never",
                   b_controls, b_stop, green is not None))


def check_a6(ctx):
    post_json(ctx, "/api/sim", {"action": "reject_next"})
    since = ctx.stream.mark()
    status, reply = post_json(ctx, "/api/cmd", {"op": "start"})
    ack = ctx.stream.wait_for("ack", lambda d: d.get("n") == reply.get("n"),
                              2.5, since)
    data = ack[2] if ack else {}
    ok = ((status == 200) and (data.get("match") == "matched")
          and (data.get("result") == 1) and (data.get("label") == "rejected"))
    return ok, ("START %d; ack %s, result %s, label %s"
                % (status, data.get("match"), data.get("result"),
                   data.get("label")))


def check_a7(ctx):
    post_json(ctx, "/api/sim", {"action": "drop_next"})
    since = ctx.stream.mark()
    sent = time.monotonic()
    _, reply = post_json(ctx, "/api/cmd", {"op": "stop"})
    gone = ctx.stream.wait_for("no_ack", lambda d: d["n"] == reply.get("n"),
                               3.0, since)
    wait = (gone[1] - sent) if gone else None
    b_drop = (wait is not None) and (1.7 <= wait <= 2.3)

    post_json(ctx, "/api/sim", {"action": "delay_next"})
    since = ctx.stream.mark()
    _, reply = post_json(ctx, "/api/cmd", {"op": "stop"})
    number = reply.get("n")
    gone = ctx.stream.wait_for("no_ack", lambda d: d["n"] == number, 3.0,
                               since)
    late = ctx.stream.wait_for("ack", lambda d: (d.get("n") == number) and (
        d["match"] == "late"), 3.0, gone[0] if gone else since)
    data = late[2] if late else {}
    b_late = ((gone is not None) and (data.get("label") == "queued")
              and (data.get("state") == "queued (late)")
              and (data.get("rtt_ms", 0.0) >= 2000.0))

    post_json(ctx, "/api/sim", {"action": "dup_next"})
    since = ctx.stream.mark()
    _, reply = post_json(ctx, "/api/cmd", {"op": "stop"})
    first = ctx.stream.wait_for("ack", lambda d: d.get("n") == reply.get(
        "n"), 2.5, since)
    second = None
    if first is not None:
        second = ctx.stream.wait_for(
            "ack", lambda d: (d["match"] == "duplicate") and (
                d["cmd_id"] == reply.get("cmd_id")), 2.5, first[0] + 1)
    b_dup = ((first is not None) and (first[2]["match"] == "matched")
             and (second is not None) and (second[2]["label"] == "queued"))
    return b_drop and b_late and b_dup, (
        "dropped ack: no_ack after %s s; delayed ack: no_ack, then %s %s "
        "%s ms; doubled ack: first %s, second %s"
        % ("%.2f" % wait if wait is not None else "never", data.get("match"),
           data.get("state"), data.get("rtt_ms"),
           first[2]["match"] if first else None,
           second[2]["match"] if second else None))


def check_a9(ctx):
    since = ctx.stream.mark()
    post_json(ctx, "/api/sim", {"action": "offline"})
    off = ctx.stream.wait_for("status", lambda d: d["value"] == "offline",
                              3.0, since)
    red = ctx.stream.wait_for("link", lambda d: (d["light"] == "red") and (
        "offline" in d["reason"]), 3.0, since)
    since = ctx.stream.mark()
    post_json(ctx, "/api/sim", {"action": "online"})
    on = ctx.stream.wait_for("status", lambda d: d["value"] == "online", 3.0,
                             since)
    green = ctx.stream.wait_for("link", lambda d: d["light"] == "green", 3.0,
                                since)
    count = ctx.stream.wait_for("heartbeat", lambda d: "connect_count 1 -> 2"
                                in d.get("log", ""), 3.0, since)
    b_logged = (off is not None) and bool(off[2].get("log"))
    ok = ((off is not None) and (red is not None) and b_logged
          and (on is not None) and (green is not None) and (count is not None))
    return ok, ("offline status and red light: %s, logged: %s; online and "
                "green again: %s; connect_count 1 -> 2 logged: %s"
                % ((off is not None) and (red is not None), b_logged,
                   (on is not None) and (green is not None),
                   count is not None))


def check_a10_live(ctx):
    """A10's first half: the banner raised, kept and cleared."""
    since = ctx.stream.mark()
    post_json(ctx, "/api/sim", {"action": "retained_cmd"})
    raised = ctx.stream.wait_for("retained_cmd", lambda d: (
        d["topic_suffix"] == RETAINED_TEST_SUFFIX) and (not d["cleared"]),
        2.0, since)
    second = Stream(ctx.port)
    snap = second.wait_for("snapshot", None, 3.0)
    entries = snap[2].get("retained_cmd", []) if snap else []
    b_kept = (snap is not None) and (snap[2].get("banner") == "ok") and any(
        (e["topic_suffix"] == RETAINED_TEST_SUFFIX) and (not e["cleared"])
        for e in entries)
    post_json(ctx, "/api/sim", {"action": "retained_clear"})
    cleared = ctx.stream.wait_for("retained_cmd", lambda d: d["cleared"],
                                  2.0, since)
    ok = (raised is not None) and b_kept and (cleared is not None)
    return ok, ("banner raised: %s; in a new connection's snapshot: %s; "
                "cleared: %s" % (raised is not None, b_kept,
                                 cleared is not None))


def check_a10_off(tmpdir):
    """A10's second half: a run with --banner-off."""
    folder = os.path.join(tmpdir, "banner_off")
    os.mkdir(folder)
    run = CheckRun(folder, ["--banner-off"])
    try:
        ctx = CheckContext(run)
        snap = ctx.stream.wait_for("snapshot", None, 5.0)
        b_off = (snap is not None) and (
            snap[2].get("banner") == "unavailable")
        post_json(ctx, "/api/sim", {"action": "retained_cmd"})
        time.sleep(1.0)
        _, text = http_call(ctx.port, "GET", "/api/state")
        b_none = (not ctx.stream.all_of("retained_cmd")) and (
            json.loads(text).get("retained_cmd") == [])
    finally:
        run.stop()
    return b_off and b_none, ("with --banner-off: banner unavailable: %s; "
                              "no banner for a retained command: %s"
                              % (b_off, b_none))


class _StubLink:
    """A link that publishes nothing (A13's in-process dashboard)."""

    def publish(self, payload):
        return None


def golden_ack():
    """Feed golden vector ACK 158500 to an in-process dashboard."""
    settings = Settings(None, 1883, "golden", "", "", "check")
    dash = Dashboard(settings, Scrubber(settings), "live", 2.0, None)
    dash.attach(_StubLink())
    client = dash.subscribe()
    dash.next_id = 5
    dash.send("stop")
    dash.on_line(sub_line(False, "golden/ack", bytes.fromhex("158500")))
    found = {}
    while not client.empty():
        message = client.get()
        if "\nevent: ack\n" in message:
            found = json.loads(message.split("\ndata: ", 1)[1])
    dash.close()
    return found


def live_probe():
    """An in-process dashboard in live mode with no broker (A1, A12): it
    must answer /api/sim with 404, and turn an exception in a request
    handler into one scrubbed line."""
    settings = Settings(FAKE_HOST, FAKE_PORT, FAKE_PREFIX, FAKE_USER,
                        FAKE_PASSWORD, "check")
    dash = Dashboard(settings, Scrubber(settings), "live", ACK_TIMEOUT_S,
                     None)
    dash.attach(_StubLink())
    server = make_server(dash, 0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever,
                     kwargs={"poll_interval": 0.1}, daemon=True).start()
    status, _ = http_call(port, "POST", "/api/sim", {"action": "hb_pause"},
                          {"X-S1-Token": server.token})
    sink = io.StringIO()
    with contextlib.redirect_stderr(sink):
        try:
            raise OSError("connect to %s:%d as %s with %s failed"
                          % (FAKE_HOST, FAKE_PORT, FAKE_USER, FAKE_PASSWORD))
        except OSError:
            server.handle_error(None, (BIND_ADDRESS, 0))
    server.shutdown()
    server.server_close()
    dash.close()
    return {"sim_status": status, "error_text": sink.getvalue()}


def digit_password_probe(tmpdir):
    """A login of digits only (A12): a number that holds it must stay a
    number, every event and log line must stay valid JSON, and the login
    must still be masked where it appears as text."""
    settings = Settings(None, 1883, "probe", "", FAKE_DIGITS, "check")
    log_path = os.path.join(tmpdir, "digits_log.jsonl")
    dash = Dashboard(settings, Scrubber(settings), "live", ACK_TIMEOUT_S,
                     log_path)
    dash.attach(_StubLink())
    client = dash.subscribe()
    with contextlib.redirect_stdout(io.StringIO()):
        dash.on_line(sub_line(False, "probe/telemetry",
                              pack(s1_decode.TELEMETRY, {"count_l": 12345})))
        dash.on_link_error("refused, password %s" % FAKE_DIGITS)
    second = dash.subscribe()
    messages = []
    for queued in (client, second):
        while not queued.empty():
            messages.append(queued.get())
    dash.close()
    found = {"json": True, "number": False, "masked": False}
    try:
        with open(log_path, encoding="utf-8") as handle:
            lines = handle.read().splitlines()
        rows = [json.loads(line)["data"] for line in lines]
        rows += [json.loads(message.split("\ndata: ", 1)[1])
                 for message in messages]
    except ValueError:
        found["json"] = False
        return found
    found["number"] = any(row.get("count_l") == 12345 for row in rows)
    logs = [row.get("log", "") for row in rows if "refused" in row.get(
        "log", "")]
    found["masked"] = bool(logs) and all(
        ("<password>" in text) and (FAKE_DIGITS not in text) for text in logs)
    return found


def check_a13(ctx):
    sink = io.StringIO()
    with contextlib.redirect_stdout(sink):
        b_self = cmd_rtt.run_selftest() == 0
        golden = golden_ack()
    b_golden = ((golden.get("op") == "STOP") and (golden.get("cmd_id") == 5)
                and (golden.get("label") == "queued")
                and (golden.get("match") == "matched"))
    labels = [d["label"] for d in ctx.stream.all_of("ack")]
    b_labels = bool(labels) and all(
        (label in ("queued", "rejected")) or label.startswith("reserved ")
        for label in labels)
    texts = [json.dumps(d).lower() for name in ("cmd", "ack")
             for d in ctx.stream.all_of(name)]
    words = [w for w in ("started", "stopped", "accepted")
             if any(w in text for text in texts)]
    ok = b_self and b_golden and b_labels and (not words)
    return ok, ("cmd_rtt self-test: %s; golden ACK 158500: %s id %s %s; "
                "%d ack labels all queued, rejected or reserved: %s; "
                "forbidden words in %d cmd and ack events: %s"
                % ("PASS" if b_self else "FAIL", golden.get("op"),
                   golden.get("cmd_id"), golden.get("label"), len(labels),
                   b_labels, len(texts), ", ".join(words) or "none"))


def probe_dash():
    """An in-process dashboard in live mode, and a way to feed it."""
    settings = Settings(None, 1883, "probe", "", "", "check")
    dash = Dashboard(settings, Scrubber(settings), "live", ACK_TIMEOUT_S,
                     None)
    dash.attach(_StubLink())

    def feed(msg_type, values):
        dash.on_line(sub_line(False, "probe/telemetry",
                              pack(msg_type, values)))

    return dash, feed


def tel_drop_snapshot_probe():
    """What a newly loaded page receives for tel_drop: connect to the real
    event stream of an in-process dashboard at each value and read the
    heartbeat in its first event (A14)."""
    dash, feed = probe_dash()
    server = make_server(dash, 0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever,
                     kwargs={"poll_interval": 0.1}, daemon=True).start()
    found = {}
    for value in (254, 255):
        feed(s1_decode.HEARTBEAT, {"seq": value, "uptime_s": value,
                                   "tel_drop": value})
        snap = Stream(port).wait_for("snapshot", None, 3.0)
        beat = snap[2].get("heartbeat") if snap else None
        found[value] = beat if beat else {}
    server.shutdown()
    server.server_close()
    dash.close()
    return ((found[254].get("tel_drop_text") == "254")
            and (found[254].get("tel_drop") == 254)
            and (found[255].get("tel_drop_text") == "255 or more")
            and (found[255].get("tel_drop") == 255))


def telemetry_probe():
    """Feed crafted live, telemetry and heartbeat messages to a dashboard
    and report what the S3 timeline, the letter tally and the IMPACT count
    did (A14). No broker and no page are involved."""
    dash, feed = probe_dash()
    letter = {name: s1_decode.CODE39.index(name) for name in "CD"}
    base = {"letter": letter["C"], "barcodes": 5, "rejects": 3,
            "junctions": 2, "losts": 1, "turn": 0}

    def live(**changes):
        now = dict(base, **changes)
        feed(s1_decode.LIVE, {
            "seq": now.setdefault("seq", 0), "bc_letter": now["letter"],
            "bc_cmd": 1, "junc_turn": now["turn"],
            "barcodes": now["barcodes"], "rejects": now["rejects"],
            "junctions": now["junctions"], "losts": now["losts"]})
        base.update(now)

    def kinds():
        return [entry["kind"] for entry in dash.s3_timeline]

    with contextlib.redirect_stdout(io.StringIO()):
        live()
        b_first = len(dash.s3_timeline) == 0
        live(barcodes=6, letter=letter["C"])
        live(barcodes=8, letter=letter["D"])
        b_tally = dash.s3_tally == {"C": 1, "D": 1, "?": 1}
        live(rejects=4)
        live(junctions=3, turn=2)
        live(losts=3)
        texts = [entry["text"] for entry in dash.s3_timeline]
        b_kinds = kinds() == ["barcode", "barcode", "reject", "junction",
                              "lost"]
        b_texts = (("barcode C accepted" in texts[0])
                   and ("and 1 more" in texts[1])
                   and ("turn right" in texts[3])
                   and ("2 times" in texts[4]) and (dash.s3_rejected == 1))
        before = len(dash.s3_timeline)
        live(barcodes=106)
        b_jump = ((kinds()[before:] == ["note"])
                  and (dash.s3_tally == {"C": 1, "D": 1, "?": 1}))
        live(barcodes=107)
        b_rebase = len(dash.s3_timeline) == before + 1
        for uptime in (100, 5):
            feed(s1_decode.HEARTBEAT, {"seq": uptime, "uptime_s": uptime})
        b_hb_restart = dash.lost_total == 0
        for seq in (7, 10):
            feed(s1_decode.HEARTBEAT, {"seq": seq, "uptime_s": seq})
        b_hb_gap = dash.lost_total == 2
        feed(s1_decode.HEARTBEAT, {"seq": 11, "uptime_s": 11, "tel_drop": 254})
        b_below = dash.heartbeat["tel_drop_text"] == "254"
        feed(s1_decode.HEARTBEAT, {"seq": 12, "uptime_s": 12, "tel_drop": 255})
        b_clamped = ((dash.heartbeat["tel_drop_text"] == "255 or more")
                     and (dash.heartbeat["tel_drop"] == 255))
        live(barcodes=0, rejects=0, junctions=0, losts=0)
        b_restart = ((kinds()[-1] == "note")
                     and ("restarted" in dash.s3_timeline[-1]["text"])
                     and (len(dash.s3_timeline) == before + 2))
        for klass in (6, 6, 0, 6):
            feed(s1_decode.TELEMETRY, {"motion_class": klass})
        live_lost, tel_lost = dash.live_lost_total, dash.tel_lost_total
        live(seq=1)
        live(seq=4)
        for seq in (20, 24):
            feed(s1_decode.TELEMETRY, {"seq": seq})
        b_lost = ((dash.live_lost_total - live_lost == 2)
                  and (dash.tel_lost_total - tel_lost == 3 + 19))
        before = len(dash.s3_timeline)
        live(seq=100, barcodes=base["barcodes"] + 1)
        feed(s1_decode.TELEMETRY, {"seq": 200})
        b_gap = ((kinds()[before:] == ["note"])
                 and (dash.live_lost_total - live_lost == 2)
                 and (dash.tel_lost_total - tel_lost == 3 + 19))
        low, feed_low = probe_dash()
        feed_low(s1_decode.LIVE, {"seq": 1, "rejects": 12, "bc_letter": 63})
        feed_low(s1_decode.LIVE, {"seq": 0, "rejects": 0, "bc_letter": 63})
        b_low = ((low.s3_rejected == 0)
                 and ([entry["kind"] for entry in low.s3_timeline]
                      == ["note"]))
    b_impact = dash.impact_count == 2
    return {"first": b_first, "tally": b_tally, "kinds": b_kinds,
            "texts": b_texts, "jump": b_jump, "rebase": b_rebase,
            "restart": b_restart, "impact": b_impact, "lost": b_lost,
            "hb_restart": b_hb_restart, "hb_gap": b_hb_gap, "seq_gap": b_gap,
            "low_counter": b_low, "tel_drop_clamp": b_below and b_clamped,
            "snapshot_text": tel_drop_snapshot_probe()}


def check_a14(ctx):
    """The telemetry panels: both messages reach the page at their rates,
    the IMPACT flag counts once per rising edge, S3's counters become
    timeline entries and a letter tally, and the odd cases (a counter
    jump, a robot restart) restart the timeline instead of faking events."""
    since = ctx.stream.mark()
    time.sleep(2.0)
    tel = len(ctx.stream.all_of("telemetry", since))
    live = len(ctx.stream.all_of("live", since))
    b_rates = (3 <= tel <= 6) and (8 <= live <= 13)
    _, text = http_call(ctx.port, "GET", "/api/state")
    state = json.loads(text)
    b_state = (bool(state["telemetry"]) and bool(state["live"])
               and bool(state["imu_history"])
               and (state["tel_age_ms"] is not None)
               and (state["tel_age_ms"] < 2000)
               and (state["live_age_ms"] is not None)
               and (state["live_age_ms"] < 2000))
    count = state["telemetry"]["impact_count"] if state["telemetry"] else -1
    mark = ctx.stream.mark()
    post_json(ctx, "/api/sim", {"action": "s4_impact"})
    hit = ctx.stream.wait_for(
        "telemetry", lambda d: d.get("b_impact") and ("Lifted/overturned" in d.get(
            "log", "")), 3.0, mark)
    time.sleep(1.5)
    _, text = http_call(ctx.port, "GET", "/api/state")
    after = json.loads(text)["telemetry"]["impact_count"]
    b_impact = (hit is not None) and (after == count + 1) and (
        hit[2]["class_name"] == "LIFTED_OVERTURNED")
    mark = ctx.stream.mark()
    for action in ("s3_barcode", "s3_reject", "s3_junction", "s3_lost"):
        post_json(ctx, "/api/sim", {"action": action})
        time.sleep(0.5)
    entries = [entry for data in ctx.stream.all_of("s3_events", mark)
               for entry in data["entries"]]
    seen = {entry["kind"] for entry in entries}
    last = ctx.stream.all_of("s3_events", mark)
    b_s3 = ({"barcode", "reject", "junction", "lost"} <= seen) and bool(
        last) and (last[-1]["rejected"] >= 1) and (
        sum(last[-1]["tally"].values()) >= 1)
    ids = ("p-s2", "p-s3", "p-s4", "p-s5", "s4-impact", "g-pitch", "g-roll",
           "timeline", "s3-tally")
    b_page = all(('id="%s"' % name) in ctx.page for name in ids) and (
        "Stage 4b)" not in ctx.page) and ("h.tel_drop_text" in ctx.page)
    probe = telemetry_probe()
    b_probe = all(probe.values())
    drop_ok, drop_text = check_tel_drop_shown(ctx)
    ok = (b_rates and b_state and b_impact and b_s3 and b_page and b_probe
          and drop_ok)
    return ok, ("%d telemetry and %d live events in 2 s (about 4 and 10): "
                "%s; state holds both with ages under 2 s: %s; one S4 impact "
                "counted once (%d -> %d): %s; S3 sim events give barcode, "
                "reject, junction and lost entries and a tally: %s; page "
                "has the four panels: %s; %s; in-process probe %s"
                % (tel, live, b_rates, b_state, count, after, b_impact,
                   b_s3, b_page, drop_text, ", ".join(
                       "%s %s" % (name, "ok" if good else "FAIL")
                       for name, good in probe.items())))


def check_tel_drop_shown(ctx):
    """The fake car's tel_drop is 0 until it has run for 5 s, then above 0
    in a heartbeat, in /api/state and in a new page's snapshot."""
    _, text = http_call(ctx.port, "GET", "/api/state")
    before = json.loads(text)["heartbeat"]["tel_drop"]
    mark = ctx.stream.mark()
    b_running = False
    # In FINISHED the first START only returns to READY, so allow a second.
    for _ in range(3):
        post_json(ctx, "/api/cmd", {"op": "start"})
        time.sleep(0.4)
        _, text = http_call(ctx.port, "GET", "/api/state")
        if json.loads(text)["car_state"] == FakeCar.RUNNING:
            b_running = True
            break
    beat = ctx.stream.wait_for("heartbeat", lambda d: d["tel_drop"] > 0, 8.0,
                               mark)
    _, text = http_call(ctx.port, "GET", "/api/state")
    in_state = json.loads(text)["heartbeat"]["tel_drop"]
    snap = Stream(ctx.port).wait_for("snapshot", None, 3.0)
    in_snapshot = snap[2]["heartbeat"]["tel_drop"] if snap else None
    text_snapshot = snap[2]["heartbeat"]["tel_drop_text"] if snap else None
    ok = (b_running and (before == 0) and (beat is not None)
          and (in_state > 0) and (in_snapshot is not None)
          and (in_snapshot > 0) and (text_snapshot == str(in_snapshot)))
    return ok, ("tel_drop %d before the run, then %s in a heartbeat, %d in "
                "/api/state, %s (text %r) in a new page's snapshot: %s"
                % (before, beat[2]["tel_drop"] if beat else "never", in_state,
                   in_snapshot, text_snapshot, ok))


def options_files(tmpdir):
    return [name for name in os.listdir(tmpdir)
            if name.startswith("s1dash-opts-")]


def check_options_live(ctx):
    """A11's options-file half while the dashboard runs."""
    names = options_files(ctx.run.tmpdir)
    if len(names) != 1:
        return False, "%d options files" % len(names)
    path = os.path.join(ctx.run.tmpdir, names[0])
    mode = stat.S_IMODE(os.stat(path).st_mode)
    text = ctx.run.read(path)
    b_lines = ("-u %s\n" % FAKE_USER in text) and (
        "-P %s\n" % FAKE_PASSWORD in text)
    return (mode == 0o600) and b_lines, (
        "one options file, mode %o, holding the login: %s" % (mode, b_lines))


def check_a11(ctx, args_list, options_live):
    topic = "%s/%s" % (FAKE_PREFIX, CMD_SUFFIX)
    bad = 0
    for argv in args_list:
        pairs = list(zip(argv, argv[1:]))
        if (("-q", "1") not in pairs) or (("-t", topic) not in pairs) or (
                "-s" not in argv) or ("-r" in argv) or (
                "--retain" in argv) or ("-u" in argv) or ("-P" in argv) or (
                "-o" not in argv) or (("-h", "<broker>") not in pairs):
            bad += 1
    b_gone = not options_files(ctx.run.tmpdir)
    ok = bool(args_list) and (bad == 0) and options_live[0] and b_gone
    return ok, ("%d publishes, %d with a wrong argument (QoS 1, topic %s, "
                "-s, no -r or --retain, login only through -o, host "
                "scrubbed); %s; removed after exit: %s"
                % (len(args_list), bad, topic, options_live[1], b_gone))


def check_a12(ctx, state_text):
    texts = {"page": ctx.page, "stream": ctx.stream.raw.decode(
        "utf-8", "replace"), "api_state": state_text,
             "log": ctx.run.read(ctx.run.log),
             "stdout": ctx.run.read(ctx.run.out),
             "stderr": ctx.run.read(ctx.run.err),
             "handler_error": ctx.live["error_text"]}
    hits = sum(text.count(secret) for text in texts.values()
               for secret in (FAKE_HOST, FAKE_USER, FAKE_PASSWORD))
    errors = [d.get("log", "") for d in ctx.stream.all_of("broker")
              if "refused for user" in d.get("log", "")]
    b_masked = bool(errors) and all(
        ("<broker>" in line) and ("<user>" in line)
        and ("<password>" in line) for line in errors)
    snap = ctx.stream.events[0][2]
    b_config = (snap.get("port") == FAKE_PORT) and (
        snap.get("prefix") == FAKE_PREFIX)
    line = ctx.live["error_text"]
    b_handler = ((line.count("\n") == 1) and ("Traceback" not in line)
                 and ("<broker>" in line) and ("<user>" in line)
                 and ("<password>" in line))
    b_digits = all(ctx.digits.values())
    ok = (hits == 0) and b_masked and b_config and b_handler and b_digits
    return ok, ("%d hits of the host, user or password in the page, the "
                "stream, /api/state, the log, stdout, stderr and a request "
                "handler's error; error line masked: %s; handler error one "
                "masked line, no traceback: %s; port and prefix from the "
                "file: %s; digit-only login: %s"
                % (hits, b_masked, b_handler, b_config, ", ".join(
                    "%s %s" % (name, "ok" if good else "FAIL")
                    for name, good in ctx.digits.items())))


class CheckContext:
    def __init__(self, run):
        self.run = run
        self.port = run.port
        _, self.page = http_call(run.port, "GET", "/")
        match = re.search(r'const TOKEN = "([0-9a-f]+)";', self.page)
        self.token = match.group(1) if match else ""
        self.stream = Stream(run.port)


def run_check(args):
    print("S1-U16 part A: s1_dashboard.py --check (simulate mode, no "
          "broker)", flush=True)
    results = {}

    def record(step, outcome):
        results[step] = outcome
        print("%-3s %s: %s" % (step, "PASS" if outcome[0] else "FAIL",
                               outcome[1]), flush=True)

    tmpdir = tempfile.mkdtemp(prefix="s1u16-")
    run = None
    try:
        run = CheckRun(tmpdir, [])
        ctx = CheckContext(run)
        if ctx.stream.wait_for("snapshot", None, 5.0) is None:
            raise RuntimeError("no event stream")
        ctx.live = live_probe()
        ctx.digits = digit_password_probe(tmpdir)
        record("A1", check_a1(ctx))
        options_live = check_options_live(ctx)
        record("A3", check_a3(ctx))
        record("A4", check_a4(ctx))
        record("A5", check_a5(ctx))
        record("A6", check_a6(ctx))
        record("A7", check_a7(ctx))
        record("A8", check_a8(ctx))
        record("A9", check_a9(ctx))
        banner_live = check_a10_live(ctx)
        post_json(ctx, "/api/sim", {"action": "fake_error"})
        ctx.stream.wait_for("broker", lambda d: "refused for user"
                            in d.get("log", ""), 3.0)
        _, state_text = http_call(ctx.port, "GET", "/api/state")
        args_list = json.loads(state_text).get("publish_args", [])
        record("A2", check_a2(ctx))
        record("A13", check_a13(ctx))
        record("A14", check_a14(ctx))
        run.stop()
        record("A11", check_a11(ctx, args_list, options_live))
        record("A12", check_a12(ctx, state_text))
        banner_off = check_a10_off(tmpdir)
        record("A10", (banner_live[0] and banner_off[0],
                       banner_live[1] + "; " + banner_off[1]))
    except (RuntimeError, OSError, ValueError, KeyError, TypeError) as err:
        print("check stopped: %s" % err, flush=True)
    finally:
        if run is not None:
            run.stop()
        shutil.rmtree(tmpdir, ignore_errors=True)
    return summarise(results)


def summarise(results):
    ran = [step for step in ALL_STEPS if step in results]
    missing = [step for step in ALL_STEPS if step not in results]
    passed = all(results[step][0] for step in ran)
    print("steps run: %s" % (" ".join(ran) or "none"))
    if missing:
        print("not run (not built yet, or the check stopped): %s"
              % " ".join(missing))
    if not passed:
        verdict, code = "FAIL", 1
    elif missing:
        verdict, code = "INCOMPLETE", 2
    else:
        verdict, code = "PASS", 0
    print("S1-U16 part A: %s" % verdict, flush=True)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=DEFAULT_CONFIG,
                        help="mqtt_config.h to read the broker from")
    parser.add_argument("--host", help="broker host (overrides the file)")
    parser.add_argument("--port", type=int, help="broker port")
    parser.add_argument("--prefix", help="topic prefix")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT,
                        help="port of the page on 127.0.0.1")
    parser.add_argument("--log", help="append every event to this file "
                        "(JSON lines)")
    parser.add_argument("--ack-timeout", type=float, default=ACK_TIMEOUT_S,
                        help="seconds to wait for an ack")
    parser.add_argument("--simulate", action="store_true",
                        help="no broker: a fake car answers")
    parser.add_argument("--banner-off", action="store_true",
                        help="run without the retained-command banner")
    parser.add_argument("--check", action="store_true",
                        help="run test S1-U16 part A")
    parser.add_argument("--opts-dir", help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.check:
            return run_check(args)
        return serve(args)
    except Exception as err:
        # No scrubber may exist yet, so only the type is printed.
        print("s1_dashboard: stopped: %s" % type(err).__name__,
              file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())

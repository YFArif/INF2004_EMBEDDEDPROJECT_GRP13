"""S1-U12: the dashboard decoder fed by a local mosquitto broker.

Runs steps 4 and 5 of docs/MQTT.md section 12 on the dashboard side only,
through a mosquitto broker that this script starts on 127.0.0.1 and stops
again. The robot is not involved.

Stages; the run stops at the first stage that fails:
  1  Broker: start mosquitto with a generated configuration (listener on
     127.0.0.1 only, anonymous, no persistence, full log); lsof must show
     that every socket it listens on is 127.0.0.1:PORT.
  2  Stream: mosquitto_sub -q 1 -t s1test/stream -F '%x' (no -C) piped
     into s1_decode.py --describe --session, while the 18 golden vectors
     from golden.py are published in document order, one
     mosquitto_pub -q 1 -s each; the subscriber is stopped with SIGINT
     once it has acknowledged all 18. Its output must equal decoding the
     same 18 hex lines directly with the same options: 18 lines, 0 error
     lines, subscriber and decoder exit status 0. A second subscriber with
     the same options records the raw output, to report how messages are
     separated (reported, not checked).
  3  STOP: mosquitto_sub -q 1 -t '<prefix>/cmd/#' -C 2 -F '%x' piped into
     s1_decode.py --describe, while 14 85 (golden vector CMD-1) is
     published with -q 1 to <prefix>/cmd and then to <prefix>/cmd/ui.
     Both must decode as STOP, id 5, and the broker log must show each
     one received and sent on to the subscriber at QoS 1.
  4  The broker is stopped; nothing may listen on the port afterwards.
The full broker log is printed at the end.

Run from the repository root:

    python3 -I -B testing/subsystem1/broker_test.py

To keep the evidence, as testing/README.md asks:

    python3 -I -B testing/subsystem1/broker_test.py \
        | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_broker.txt

The last line is "S1-U12: PASS (...)" with exit status 0, "S1-U12: FAIL
(...)" with exit status 1, or "S1-U12: NOT RUN (...)" with exit status 2
when a program it needs is missing or the port is already in use. Nothing
in the repository is changed: Python runs with -B, and the broker's
configuration and log go to a temporary folder that is removed at the
end. Commands are printed with repository-relative paths and program
names only, so a saved log holds no home-folder path.

Environment: MOSQUITTO, MOSQUITTO_SUB, MOSQUITTO_PUB (the programs; by
default found on PATH -- Homebrew installs mosquitto itself in its sbin
folder), S1U12_PORT (default 1883), TMPDIR.
"""
import difflib
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DECODER_REL = "app_program/subsystem1/dashboard/s1_decode.py"
GOLDEN_REL = "testing/subsystem1/golden.py"
DOC_REL = "docs/S1_binary_frame.md"

HOST = "127.0.0.1"
PORT = os.environ.get("S1U12_PORT", "1883")
# The example prefix of mqtt_config.example.h. The broker is local, so the
# prefix only has to match between publisher and subscriber.
PREFIX = "robocar/car1"
STREAM_TOPIC = "s1test/stream"
STOP_BYTES = b"\x14\x85"            # golden vector CMD-1: STOP, id 5
STOP_TOPICS = (PREFIX + "/cmd", PREFIX + "/cmd/ui")
GOLDEN_COUNT = 18
WAIT_SECONDS = 10
SETTLE_SECONDS = 0.5

TOOLS = (("mosquitto", "MOSQUITTO"), ("mosquitto_sub", "MOSQUITTO_SUB"),
         ("mosquitto_pub", "MOSQUITTO_PUB"))
INSTALL = ("install mosquitto (macOS: brew install mosquitto; Debian or "
           "Ubuntu: apt install mosquitto mosquitto-clients) and put it on "
           "PATH, or set %s to the program")


class NotRun(Exception):
    """Something the test needs is missing: exit status 2."""


class Failed(Exception):
    """A stage failed: exit status 1."""


def wait_for(test, seconds=WAIT_SECONDS):
    """Poll test() every 50 ms; True as soon as it holds."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if test():
            return True
        time.sleep(0.05)
    return False


def listening():
    """True if something accepts TCP connections on HOST:PORT."""
    with socket.socket() as s:
        s.settimeout(0.2)
        return 0 == s.connect_ex((HOST, int(PORT)))


def parse(line):
    """One decoder output line as a dict; a line that is not JSON counts
    as an error line."""
    try:
        return json.loads(line)
    except ValueError:
        return {"error": "not JSON", "detail": line}


def git(*args):
    """Output of a git command in the repository, or "" if it fails."""
    r = subprocess.run(["git"] + list(args), cwd=ROOT, capture_output=True,
                       text=True)
    return r.stdout.strip() if 0 == r.returncode else ""


def find_tools():
    """Map each mosquitto program name to its path, or raise NotRun."""
    tools = {}
    for name, env in TOOLS:
        path = os.environ.get(env) or shutil.which(name)
        if (not path) or (not os.access(path, os.X_OK)):
            raise NotRun("%s not found: %s" % (name, INSTALL % env))
        tools[name] = path
    if not shutil.which("lsof"):
        raise NotRun("lsof not found; it checks that the broker listens on "
                     "127.0.0.1 only")
    return tools


def header(tools):
    """Print the date, commit, and the Python and client versions."""
    print("S1-U12 broker_test.py, %s"
          % time.strftime("%Y-%m-%d %H:%M:%S %Z"))
    commit = git("rev-parse", "--short", "HEAD") or "unknown"
    dirty = [line for line in git("status", "--porcelain", "--",
                                  "app_program/subsystem1/dashboard",
                                  "testing/subsystem1").splitlines()
             if not (line.startswith("?? ") and ("logs/" in line))]
    if dirty:
        commit += " (tested files have uncommitted changes)"
    print("commit: " + commit)
    print("python: Python " + sys.version.split()[0])
    for name in ("mosquitto_sub", "mosquitto_pub"):
        r = subprocess.run([tools[name], "--help"], capture_output=True,
                           text=True)
        text = r.stdout + r.stderr
        line = next((l for l in text.splitlines() if " version " in l), "")
        print("%s: %s" % (name, line.strip() or "version not printed"))


def golden_vectors():
    """The 18 golden vectors from golden.py, as (name, hex) pairs."""
    r = subprocess.run([sys.executable, "-I", "-B", str(ROOT / GOLDEN_REL),
                        str(ROOT / DOC_REL)], capture_output=True, text=True)
    if 0 != r.returncode:
        raise Failed("2: golden.py exit status %d" % r.returncode)
    vectors = [(f[0], f[2].strip()) for f in
               (line.split("|") for line in r.stdout.splitlines())]
    if GOLDEN_COUNT != len(vectors):
        raise Failed("2: golden.py gave %d vectors, expected %d"
                     % (len(vectors), GOLDEN_COUNT))
    return vectors


def log_text(work):
    """The broker log so far."""
    return (work / "broker.log").read_text(errors="replace")


def start_broker(tools, work):
    """Stage 1: start mosquitto on HOST:PORT only and check its sockets."""
    conf = work / "broker.conf"
    conf.write_text("listener %s %s\nallow_anonymous true\n"
                    "persistence false\nlog_dest stdout\nlog_type all\n"
                    % (PORT, HOST))
    with open(work / "broker.log", "w") as log:
        broker = subprocess.Popen([tools["mosquitto"], "-c", str(conf)],
                                  stdout=log, stderr=subprocess.STDOUT)
    print("== stage 1: broker")
    print("configuration: listener %s %s, allow_anonymous true, "
          "persistence false, log_type all" % (PORT, HOST))
    if not wait_for(listening):
        raise Failed("1: the broker did not listen on %s:%s" % (HOST, PORT))
    out = subprocess.run(["lsof", "-nP", "-a", "-p", str(broker.pid),
                          "-iTCP", "-sTCP:LISTEN"], capture_output=True,
                         text=True).stdout
    sockets = [line.split()[-2] for line in out.splitlines()[1:]]
    print("broker listens on: " + (", ".join(sockets) or "nothing"))
    if not sockets:
        raise Failed("1: lsof showed no listening socket for the broker")
    if any(s != "%s:%s" % (HOST, PORT) for s in sockets):
        raise Failed("1: the broker listens on more than %s:%s"
                     % (HOST, PORT))
    print("stage 1: ok")
    return broker


def stop_subscriber(proc, name):
    """Stop a subscriber as Ctrl-C would; its exit status."""
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    try:
        return proc.wait(timeout=WAIT_SECONDS)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        raise Failed("2: %s did not stop on SIGINT" % name)


def stage_stream(tools, work, vectors, procs):
    """Stage 2: the 18 vectors through the broker, against a direct decode."""
    print("== stage 2: stream")
    sub = [tools["mosquitto_sub"], "-h", HOST, "-p", PORT, "-q", "1", "-t",
           STREAM_TOPIC, "-F", "%x"]
    decode = [sys.executable, "-I", "-B", str(ROOT / DECODER_REL),
              "--describe", "--session"]
    print("subscriber: mosquitto_sub -h %s -p %s -q 1 -t '%s' -F '%%x' | "
          "python3 %s --describe --session" % (HOST, PORT, STREAM_TOPIC,
                                               DECODER_REL))
    print("publisher, once per vector: mosquitto_pub -h %s -p %s -q 1 "
          "-t '%s' -s (payload in binary on stdin)" % (HOST, PORT,
                                                       STREAM_TOPIC))
    with open(work / "stream.json", "w") as out_a, \
            open(work / "raw.txt", "wb") as out_b:
        sub_a = subprocess.Popen(sub + ["-i", "s1u12-stream"],
                                 stdout=subprocess.PIPE)
        procs.append(sub_a)
        dec = subprocess.Popen(decode, stdin=sub_a.stdout, stdout=out_a,
                               stderr=subprocess.PIPE)
        procs.append(dec)
        sub_a.stdout.close()
        sub_b = subprocess.Popen(sub + ["-i", "s1u12-raw"], stdout=out_b)
        procs.append(sub_b)
        if not wait_for(lambda: 2 <= log_text(work).count(
                "Received SUBSCRIBE")):
            raise Failed("2: the two subscriptions did not reach the broker")
        for number, (name, hexes) in enumerate(vectors, 1):
            r = subprocess.run([tools["mosquitto_pub"], "-h", HOST, "-p",
                                PORT, "-q", "1", "-i", "s1u12-pub-%02d"
                                % number, "-t", STREAM_TOPIC, "-s"],
                               input=bytes.fromhex(hexes),
                               capture_output=True)
            if 0 != r.returncode:
                raise Failed("2: publishing %s: mosquitto_pub exit status "
                             "%d" % (name, r.returncode))
        for client in ("s1u12-stream", "s1u12-raw"):
            if not wait_for(lambda: len(vectors) <= log_text(work).count(
                    "Received PUBACK from " + client + " ")):
                raise Failed("2: %s did not acknowledge all %d messages"
                             % (client, len(vectors)))
        time.sleep(SETTLE_SECONDS)
        rc_a = stop_subscriber(sub_a, "the subscriber")
        rc_b = stop_subscriber(sub_b, "the raw subscriber")
        try:
            _, err = dec.communicate(timeout=WAIT_SECONDS)
        except subprocess.TimeoutExpired:
            raise Failed("2: the decoder did not finish")
    log = log_text(work)
    print("published %d; broker: Received PUBLISH %d, Sending PUBLISH %d, "
          "Received PUBACK %d" % (len(vectors),
                                  log.count("Received PUBLISH"),
                                  log.count("Sending PUBLISH"),
                                  log.count("Received PUBACK")))
    print("exit status: subscriber %d, decoder %d, raw subscriber %d"
          % (rc_a, dec.returncode, rc_b))
    if err.strip():
        print("decoder stderr: " + err.decode(errors="replace").strip())

    stream = (work / "stream.json").read_text().splitlines()
    direct = subprocess.run(decode, input="\n".join(h for _, h in vectors)
                            + "\n", capture_output=True, text=True)
    direct_lines = direct.stdout.splitlines()
    errors = sum(1 for line in stream if "error" in parse(line))
    print("lines: stream %d, direct %d; error lines in the stream: %d"
          % (len(stream), len(direct_lines), errors))
    diff = list(difflib.unified_diff(direct_lines, stream, "direct",
                                     "stream", lineterm=""))
    print("diff direct stream: " + ("no differences" if not diff else ""))
    for line in diff[:40]:
        print("  " + line)

    raw = (work / "raw.txt").read_bytes()
    want = "".join(h.replace(" ", "").lower() + "\n" for _, h in vectors)
    print("raw -F '%%x' output: %d bytes, %d newlines, %d carriage "
          "returns; %s" % (len(raw), raw.count(b"\n"), raw.count(b"\r"),
                           "the 18 payloads in lowercase hex, one per line"
                           if raw == want.encode() else
                           "NOT one payload per line"))

    if ((0 != rc_a) or (0 != dec.returncode) or (0 != direct.returncode)
            or (GOLDEN_COUNT != len(stream)) or (0 != errors) or diff):
        raise Failed("2: stream decode differs from the direct decode, or "
                     "an exit status is not 0")
    print("stage 2: ok")
    return len(stream)


def stage_stop(tools, work, procs):
    """Stage 3: STOP to <prefix>/cmd and <prefix>/cmd/ui, on cmd/#."""
    print("== stage 3: STOP")
    filt = PREFIX + "/cmd/#"
    print("subscriber: mosquitto_sub -h %s -p %s -q 1 -t '%s' -C 2 -F '%%x' "
          "| python3 %s --describe" % (HOST, PORT, filt, DECODER_REL))
    before = log_text(work).count("Received SUBSCRIBE from s1u12-cmd")
    sub = subprocess.Popen([tools["mosquitto_sub"], "-h", HOST, "-p", PORT,
                            "-q", "1", "-i", "s1u12-cmd", "-t", filt, "-C",
                            "2", "-F", "%x"], stdout=subprocess.PIPE)
    procs.append(sub)
    dec = subprocess.Popen([sys.executable, "-I", "-B",
                            str(ROOT / DECODER_REL), "--describe"],
                           stdin=sub.stdout, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, text=True)
    procs.append(dec)
    sub.stdout.close()
    if not wait_for(lambda: before < log_text(work).count(
            "Received SUBSCRIBE from s1u12-cmd")):
        raise Failed("3: the subscription did not reach the broker")
    for number, topic in enumerate(STOP_TOPICS, 1):
        print("publisher: printf '\\024\\205' | mosquitto_pub -h %s -p %s "
              "-q 1 -t '%s' -s" % (HOST, PORT, topic))
        r = subprocess.run([tools["mosquitto_pub"], "-h", HOST, "-p", PORT,
                            "-q", "1", "-i", "s1u12-stop-%d" % number, "-t",
                            topic, "-s"], input=STOP_BYTES,
                           capture_output=True)
        if 0 != r.returncode:
            raise Failed("3: mosquitto_pub exit status %d" % r.returncode)
    try:
        out, err = dec.communicate(timeout=WAIT_SECONDS)
        rc_sub = sub.wait(timeout=WAIT_SECONDS)
    except subprocess.TimeoutExpired:
        raise Failed("3: the subscriber did not receive both messages")
    print("exit status: subscriber %d, decoder %d" % (rc_sub, dec.returncode))
    for line in out.splitlines():
        print("decoded: " + line)
    if err.strip():
        print("decoder stderr: " + err.strip())
    decoded = [parse(line) for line in out.splitlines()]
    good = [d for d in decoded
            if ("COMMAND" == d.get("type")) and ("STOP" == d.get(
                "opcode_name")) and (5 == d.get("cmd_id"))
            and (True is d.get("valid"))]

    log = log_text(work)
    received = re.findall(r"Received PUBLISH from s1u12-stop-\d \(d\d, "
                          r"q(\d), r\d, m\d+, '([^']*)'", log)
    sent = re.findall(r"Sending PUBLISH to s1u12-cmd \(d\d, q(\d), r\d, "
                      r"m\d+, '([^']*)'", log)
    for what, rows in (("received", received), ("sent on", sent)):
        print("broker %s: %s" % (what, ", ".join(
            "'%s' at QoS %s" % (t, q) for q, t in rows) or "nothing"))
    expect = [("1", t) for t in STOP_TOPICS]
    if ((0 != rc_sub) or (0 != dec.returncode) or (2 != len(good))
            or (2 != len(decoded)) or (expect != received)
            or (expect != sent)):
        raise Failed("3: STOP not received and decoded twice at QoS 1")
    print("stage 3: ok")


def stop_broker(broker):
    """Stage 4: stop the broker; True if the port is free afterwards."""
    broker.terminate()
    try:
        broker.wait(timeout=WAIT_SECONDS)
    except subprocess.TimeoutExpired:
        broker.kill()
        broker.wait()
    free = not listening()
    print("== stage 4: broker stopped, exit status %d; %s:%s %s"
          % (broker.returncode, HOST, PORT,
             "free" if free else "STILL IN USE"))
    return free


def main():
    """Run the stages; print the last line; return the exit status."""
    broker = None
    work = None
    procs = []
    status, code, detail = "FAIL", 1, "did not finish"
    try:
        for rel in (DECODER_REL, GOLDEN_REL, DOC_REL):
            if not (ROOT / rel).is_file():
                raise NotRun("%s is missing" % rel)
        tools = find_tools()
        header(tools)
        if listening():
            raise NotRun("something already listens on %s:%s; stop it or "
                         "set S1U12_PORT" % (HOST, PORT))
        vectors = golden_vectors()
        work = Path(tempfile.mkdtemp(prefix="s1u12."))
        broker = start_broker(tools, work)
        count = stage_stream(tools, work, vectors, procs)
        stage_stop(tools, work, procs)
        status, code = "PASS", 0
        detail = ("%d/%d vectors decoded through the broker as directly, "
                  "0 error lines; STOP id 5 from %s and %s received on "
                  "%s/cmd/# at QoS 1" % (count, GOLDEN_COUNT,
                                         STOP_TOPICS[0], STOP_TOPICS[1],
                                         PREFIX))
    except NotRun as e:
        status, code, detail = "NOT RUN", 2, str(e)
    except Failed as e:
        status, code, detail = "FAIL", 1, "stage " + str(e)
    except Exception as e:  # noqa: BLE001 (report it, then clean up)
        traceback.print_exc(file=sys.stdout)
        status, code, detail = "FAIL", 1, "unexpected error: %r" % e
    finally:
        for proc in procs:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        if broker is not None:
            if (not stop_broker(broker)) and (0 == code):
                status, code, detail = "FAIL", 1, "stage 4: port in use"
            print("== broker log")
            sys.stdout.write(log_text(work))
        if work is not None:
            shutil.rmtree(work, ignore_errors=True)
    if 0 == code:
        detail += "; broker on %s:%s only, stopped" % (HOST, PORT)
    print("S1-U12: %s (%s)" % (status, detail))
    return code


if __name__ == "__main__":
    sys.exit(main())

"""S1-U06 helper: dashboard-side round trip of START, STOP and RESET.

Publishes 2-byte COMMAND messages (docs/S1_binary_frame.md section 5) to
<prefix>/cmd/dash with mosquitto_pub -q 1, never retained, and matches
each one with the ACK that comes back on <prefix>/ack with the same opcode
and cmd_id. The round trip runs from just before mosquitto_pub starts to
the time mosquitto_sub stamped the ACK, both on this machine's clock, so
it includes mosquitto_pub's own start-up and is on the pessimistic side.

An ACK with result 0 means "queued for S5", not "carried out" (Stage 4a,
docs/MQTT.md): vehicle_ctrl_task decides later, and its verdict is only on
the robot's console.

Modes:
  timing (default)  --op stop --count 50: one command every --gap seconds,
                    cmd_id 0, 1, 2 ... (at most 64 so every id is unique);
                    prints every trial, then min, median, p95 and max, and
                    the NFR15 check: at least 95 % of the trials acked
                    within --limit-ms (a missing ACK counts as a miss).
  raw               --raw 1409: publish these hex bytes once and print the
                    ACK received within --wait seconds, or that none came.
  selftest          --selftest: check this script's own packing and
                    parsing against golden vector CMD-1 (14 85).

The ACKs are collected by one mosquitto_sub for the whole run and read
after it is stopped, so pipe buffering cannot delay a time stamp.

Run from the repository root, with the broker the robot uses:

    python3 -I -B testing/subsystem1/cmd_rtt.py --prefix robot --op stop

Needs mosquitto_sub and mosquitto_pub on the PATH. It prints no address.
"""

import argparse
import datetime
import shutil
import signal
import statistics
import subprocess
import sys
import time

HDR_COMMAND = 0x14          # Version 1, type 4.
HDR_ACK = 0x15              # Version 1, type 5.
OPCODES = {"start": 1, "stop": 2, "reset": 3}
NAMES = {0: "INVALID", 1: "START", 2: "STOP", 3: "RESET"}
RESULTS = {0: "queued", 1: "rejected"}
STAMP_FORMAT = "@Y-@m-@dT@H:@M:@S.@N %x"


def command_bytes(opcode, cmd_id):
    """Pack a COMMAND: header, then opcode (2 bits) and cmd_id (6 bits)."""
    return bytes([HDR_COMMAND, ((opcode & 0x03) << 6) | (cmd_id & 0x3F)])


def parse_ack(hex_text):
    """Return (opcode, cmd_id, result) for a valid 3-byte ACK, else None."""
    try:
        data = bytes.fromhex(hex_text)
    except ValueError:
        return None
    if len(data) != 3 or data[0] != HDR_ACK:
        return None
    return (data[1] >> 6, data[1] & 0x3F, data[2] >> 6)


def parse_line(line):
    """Split one mosquitto_sub line into (time stamp, ack) or None."""
    parts = line.strip().split(" ")
    if len(parts) != 2:
        return None
    stamp, hex_text = parts
    try:
        # @N gives nanoseconds; datetime keeps microseconds.
        when = datetime.datetime.strptime(stamp[:26], "%Y-%m-%dT%H:%M:%S.%f")
    except ValueError:
        return None
    ack = parse_ack(hex_text)
    return None if ack is None else (when, ack)


def start_subscriber(args):
    cmd = ["mosquitto_sub", "-h", args.host, "-p", str(args.port), "-q", "1",
           "-t", args.prefix + "/ack", "-F", STAMP_FORMAT]
    sub = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, text=True)
    time.sleep(args.settle)
    if sub.poll() is not None:
        sys.exit("cmd_rtt: mosquitto_sub could not start: "
                 + sub.stderr.read().strip())
    return sub


def stop_subscriber(sub):
    sub.send_signal(signal.SIGINT)
    out, _ = sub.communicate(timeout=10)
    acks = []
    for line in out.splitlines():
        parsed = parse_line(line)
        if parsed is not None:
            acks.append(parsed)
    return acks


def publish(args, payload):
    cmd = ["mosquitto_pub", "-h", args.host, "-p", str(args.port), "-q", "1",
           "-t", args.prefix + "/cmd/dash", "-s"]
    subprocess.run(cmd, input=payload, check=True)


def run_timing(args):
    opcode = OPCODES[args.op]
    sub = start_subscriber(args)
    sent = []
    for trial in range(args.count):
        cmd_id = trial % 64
        sent.append((trial, cmd_id, datetime.datetime.now()))
        publish(args, command_bytes(opcode, cmd_id))
        time.sleep(args.gap)
    time.sleep(args.wait)
    acks = stop_subscriber(sub)

    first = {}
    duplicates = 0
    for when, (ack_op, ack_id, result) in acks:
        if ack_op != opcode:
            continue
        if ack_id in first:
            duplicates += 1
        else:
            first[ack_id] = (when, result)

    rtts = []
    under = 0
    queued = 0
    for trial, cmd_id, t_sent in sent:
        if cmd_id not in first:
            print("trial %2d id %2d %s: no ack" % (trial, cmd_id,
                                                  NAMES[opcode]))
            continue
        when, result = first[cmd_id]
        rtt_ms = (when - t_sent).total_seconds() * 1000.0
        rtts.append(rtt_ms)
        if rtt_ms < args.limit_ms:
            under += 1
        if result == 0:
            queued += 1
        print("trial %2d id %2d %s: %7.1f ms, %s"
              % (trial, cmd_id, NAMES[opcode], rtt_ms,
                 RESULTS.get(result, "reserved %d" % result)))

    print("sent %d, acked %d (queued %d), missing %d, duplicate acks %d"
          % (len(sent), len(rtts), queued, len(sent) - len(rtts),
             duplicates))
    if rtts:
        ordered = sorted(rtts)
        p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
        print("round trip ms: min %.1f, median %.1f, p95 %.1f, max %.1f"
              % (ordered[0], statistics.median(ordered), p95, ordered[-1]))
    share = 100.0 * under / len(sent)
    verdict = "PASS" if share >= 95.0 else "FAIL"
    print("NFR15: %d of %d under %d ms (%.1f %%): %s"
          % (under, len(sent), args.limit_ms, share, verdict))
    return 0 if verdict == "PASS" else 1


def run_raw(args):
    payload = bytes.fromhex(args.raw)
    sub = start_subscriber(args)
    t_sent = datetime.datetime.now()
    publish(args, payload)
    time.sleep(args.wait)
    acks = stop_subscriber(sub)
    if not acks:
        print("raw %s: no ack within %.1f s" % (payload.hex(), args.wait))
        return 1
    for when, (ack_op, ack_id, result) in acks:
        print("raw %s: ack opcode %d (%s), id %d, %s, after %.1f ms"
              % (payload.hex(), ack_op, NAMES[ack_op], ack_id,
                 RESULTS.get(result, "reserved %d" % result),
                 (when - t_sent).total_seconds() * 1000.0))
    return 0


def run_selftest():
    ok = (command_bytes(2, 5) == bytes([0x14, 0x85])
          and parse_ack("158500") == (2, 5, 0)
          and parse_ack("150040") == (0, 0, 1)
          and parse_ack("1485") is None
          and parse_line("2026-11-01T10:00:00.123456789 158500")
          == (datetime.datetime(2026, 11, 1, 10, 0, 0, 123456), (2, 5, 0)))
    print("cmd_rtt selftest: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--prefix", default="robot")
    parser.add_argument("--op", choices=sorted(OPCODES), default="stop")
    parser.add_argument("--count", type=int, default=50)
    parser.add_argument("--gap", type=float, default=0.5)
    parser.add_argument("--wait", type=float, default=2.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--limit-ms", type=int, default=300)
    parser.add_argument("--raw")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return run_selftest()
    for tool in ("mosquitto_sub", "mosquitto_pub"):
        if shutil.which(tool) is None:
            sys.exit("cmd_rtt: %s is not on the PATH" % tool)
    if not 1 <= args.count <= 64:
        sys.exit("cmd_rtt: --count must be 1 to 64 (cmd_id is 6 bits)")
    if args.raw is not None:
        return run_raw(args)
    return run_timing(args)


if __name__ == "__main__":
    sys.exit(main())

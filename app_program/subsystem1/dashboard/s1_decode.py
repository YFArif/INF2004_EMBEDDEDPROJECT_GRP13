"""Dashboard decoder for the S1 binary MQTT messages.

The layout is docs/S1_binary_frame.md (layout version 1). decode() turns
one MQTT payload into a dict whose field names and values are exactly
what msg_codec_get_*() in app_program/subsystem1/msg_codec.c returns, so
the robot, the C test and the dashboard use one name per field. It
accepts and rejects the same payloads as the C decoder: the length must
be exact for the type, the version must be 1 and the type 1 to 5.
Padding bits are ignored (rule 5 of section 2).

Use from Python:

    import s1_decode
    msg = s1_decode.decode(payload)        # raises DecodeError if bad
    shown = s1_decode.describe(msg)        # adds display-only fields

Use from the command line, one payload per line as hex on stdin (spaces
or colons between bytes are allowed, '#' starts a comment):

    mosquitto_sub -t 'robot/#' -F '%x' | python3 s1_decode.py --describe

Each line is printed as one JSON object. A rejected line is printed as
{"line": n, "error": reason, "detail": text}. The exit status is 0 if
every line decoded, 1 if any line was rejected and 2 for bad usage.

Test: S1-U11, testing/subsystem1/run_decoder_test.sh.
"""
import json
import re
import sys

LAYOUT_VERSION = 1

TELEMETRY = 1
HEARTBEAT = 2
EVENT = 3
COMMAND = 4
ACK = 5

TYPE_NAMES = {
    TELEMETRY: "TELEMETRY",
    HEARTBEAT: "HEARTBEAT",
    EVENT: "EVENT",
    COMMAND: "COMMAND",
    ACK: "ACK",
}

# Exact payload length of each message type, in bytes.
LENGTHS = {TELEMETRY: 25, HEARTBEAT: 9, EVENT: 7, COMMAND: 2, ACK: 3}

# How a stored field becomes a value.
UNSIGNED = "unsigned"      # as stored
SIGNED = "signed"          # offset binary: stored - 2^(width - 1)
FLAG = "flag"              # 0 or 1 -> False or True
NEGATED = "negated"        # stored -RSSI -> RSSI in dBm
BYTE_LIST = "byte_list"    # event args -> list of bytes, first byte first

# (name, bit offset, width, kind) for every field after the header byte,
# from sections 3 to 6 of the layout document. Names are the msg_codec.h
# struct member names.
FIELDS = {
    TELEMETRY: (
        ("seq", 8, 8, UNSIGNED),
        ("ctrl_state", 16, 4, UNSIGNED),
        ("motion_state", 20, 3, UNSIGNED),
        ("fault_mask", 23, 5, UNSIGNED),
        ("line_state", 28, 3, UNSIGNED),
        ("bc_letter", 31, 6, UNSIGNED),
        ("bc_cmd", 37, 2, UNSIGNED),
        ("b_bc_dir", 39, 1, FLAG),
        ("speed_l_mms", 40, 10, SIGNED),
        ("speed_r_mms", 50, 10, SIGNED),
        ("count_l", 60, 16, UNSIGNED),
        ("count_r", 76, 16, UNSIGNED),
        ("dist_mm", 92, 16, SIGNED),
        ("pitch_hdeg", 108, 7, SIGNED),
        ("roll_hdeg", 115, 7, SIGNED),
        ("hump_state", 122, 2, UNSIGNED),
        ("peak_mm", 124, 8, UNSIGNED),
        ("hump_count", 132, 4, UNSIGNED),
        ("motion_class", 136, 3, UNSIGNED),
        ("turn_dps", 139, 10, SIGNED),
        ("b_imu_ok", 149, 1, FLAG),
        ("b_cal_done", 150, 1, FLAG),
        ("closest_mm", 151, 10, UNSIGNED),
        ("angle_idx", 161, 5, UNSIGNED),
        ("width_mm", 166, 10, UNSIGNED),
        ("clear_l_mm", 176, 10, UNSIGNED),
        ("clear_r_mm", 186, 10, UNSIGNED),
    ),
    HEARTBEAT: (
        ("seq", 8, 8, UNSIGNED),
        ("uptime_s", 16, 16, UNSIGNED),
        ("rssi_dbm", 32, 7, NEGATED),
        ("connect_count", 39, 8, UNSIGNED),
        ("tel_drop", 47, 8, UNSIGNED),
        ("evt_drop", 55, 8, UNSIGNED),
        ("cmd_reject", 63, 8, UNSIGNED),
    ),
    EVENT: (
        ("code", 8, 8, UNSIGNED),
        ("args", 16, 16, BYTE_LIST),
        ("t_ms", 32, 24, UNSIGNED),
    ),
    COMMAND: (
        ("opcode", 8, 2, UNSIGNED),
        ("cmd_id", 10, 6, UNSIGNED),
    ),
    ACK: (
        ("opcode", 8, 2, UNSIGNED),
        ("cmd_id", 10, 6, UNSIGNED),
        ("result", 16, 2, UNSIGNED),
    ),
}

OP_INVALID = 0
OPCODE_NAMES = {0: "INVALID", 1: "START", 2: "STOP", 3: "RESET"}
RESULT_NAMES = {0: "accepted", 1: "rejected"}

# Section 3.1: bit n of fault_mask is fault n. Bits 1 and 3 are not
# defined yet, so describe() shows them only by number.
FAULT_NAMES = {0: "stall", 2: "imu", 4: "wifi"}

USAGE = "usage: s1_decode.py [--describe] [--session] < hex_lines"


class DecodeError(ValueError):
    """A payload the robot's own decoder would reject.

    reason is one of "empty", "version", "type" or "length".
    """

    def __init__(self, reason, detail):
        ValueError.__init__(self, "%s: %s" % (reason, detail))
        self.reason = reason
        self.detail = detail


def _convert(raw, width, kind):
    """Turn a stored field into its value."""
    if kind == SIGNED:
        return raw - (1 << (width - 1))
    if kind == FLAG:
        return raw == 1
    if kind == NEGATED:
        return -raw
    if kind == BYTE_LIST:
        return list(raw.to_bytes(width // 8, "big"))
    return raw


def decode(data):
    """Decode one payload (bytes) into a dict, or raise DecodeError."""
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("decode() needs bytes, not %s" % type(data).__name__)
    if len(data) == 0:
        raise DecodeError("empty", "no bytes")
    version = data[0] >> 4
    msg_type = data[0] & 0x0F
    if version != LAYOUT_VERSION:
        raise DecodeError("version", "version %d, expected %d%s" % (
            version, LAYOUT_VERSION,
            " (0 is never valid)" if version == 0 else ""))
    if msg_type not in LENGTHS:
        raise DecodeError("type", "type %d is not 1 to 5" % msg_type)
    if len(data) != LENGTHS[msg_type]:
        raise DecodeError("length", "%d bytes; %s needs exactly %d" % (
            len(data), TYPE_NAMES[msg_type], LENGTHS[msg_type]))

    # The payload is one bit stream, MSB first (rule 1 of section 2): as a
    # big-endian integer, a field at bit offset o of width w sits w bits
    # below bit (total - o) of that integer.
    value = int.from_bytes(data, "big")
    total = 8 * len(data)
    msg = {"version": version, "msg_type": msg_type,
           "type": TYPE_NAMES[msg_type]}
    for name, offset, width, kind in FIELDS[msg_type]:
        raw = (value >> (total - offset - width)) & ((1 << width) - 1)
        msg[name] = _convert(raw, width, kind)
    if msg_type == COMMAND:
        # Same as msg_codec_get_command(): opcode 0 is decoded, so the ack
        # can echo it, but the command is not valid.
        msg["valid"] = msg["opcode"] != OP_INVALID
    return msg


def describe(msg):
    """Return a copy of a decoded message with display-only fields added.

    Only names and units the layout document defines are added. Controller,
    motion, line and hump states, barcode commands, motion classes and
    event codes stay numbers until their owners define them.
    """
    shown = dict(msg)
    if msg["msg_type"] == TELEMETRY:
        shown["pitch_deg"] = msg["pitch_hdeg"] / 2.0
        shown["roll_deg"] = msg["roll_hdeg"] / 2.0
        shown["speed_mms"] = (msg["speed_l_mms"] + msg["speed_r_mms"]) / 2.0
        shown["faults"] = [FAULT_NAMES.get(bit, "bit%d" % bit)
                           for bit in range(5)
                           if (msg["fault_mask"] >> bit) & 1]
    elif msg["msg_type"] in (COMMAND, ACK):
        shown["opcode_name"] = OPCODE_NAMES[msg["opcode"]]
        if msg["msg_type"] == ACK:
            shown["result_name"] = RESULT_NAMES.get(
                msg["result"], "reserved %d" % msg["result"])
    return shown


def unwrap16(prev_full, low16):
    """Rebuild a full encoder count from the 16 bits sent.

    Same rule as unwrap16() in testing/subsystem1/codec_test_main.c
    (section 10.1): the count is taken to have moved forward by the
    difference of the low 16 bits, modulo 65536.
    """
    return prev_full + ((low16 - (prev_full & 0xFFFF)) & 0xFFFF)


class Session:
    """State for one robot: full encoder counts and lost frames.

    feed() decodes a payload like decode() and adds, for telemetry and
    heartbeat, lost_before (frames missing since the previous one of that
    type, from seq modulo 256) and duplicate (same seq as the previous
    one); for telemetry also count_l_full, count_r_full and
    counts_anchored (False on the first telemetry frame, whose full counts
    are only the 16 bits received).
    """

    def __init__(self):
        self._last_seq = {}
        self._full = {}

    def feed(self, data):
        msg = decode(data)
        msg_type = msg["msg_type"]
        if msg_type in (TELEMETRY, HEARTBEAT):
            prev = self._last_seq.get(msg_type)
            msg["duplicate"] = (prev is not None) and (msg["seq"] == prev)
            if (prev is None) or msg["duplicate"]:
                msg["lost_before"] = 0
            else:
                msg["lost_before"] = (msg["seq"] - prev - 1) % 256
            self._last_seq[msg_type] = msg["seq"]
        if msg_type == TELEMETRY:
            msg["counts_anchored"] = "count_l" in self._full
            for name in ("count_l", "count_r"):
                prev_full = self._full.get(name)
                if prev_full is None:
                    full = msg[name]
                else:
                    full = unwrap16(prev_full, msg[name])
                self._full[name] = full
                msg[name + "_full"] = full
        return msg


def parse_hex(line):
    """Turn one line of hex digits into bytes, or raise ValueError."""
    text = re.sub(r"[\s:]", "", line)
    if (len(text) % 2 != 0) or not re.fullmatch(r"[0-9A-Fa-f]*", text):
        raise ValueError("not an even number of hex digits")
    return bytes.fromhex(text)


def main(argv):
    options = argv[1:]
    if any(opt not in ("--describe", "--session") for opt in options):
        print(USAGE, file=sys.stderr)
        return 2
    session = Session() if "--session" in options else None
    status = 0
    for number, raw_line in enumerate(sys.stdin, 1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        try:
            data = parse_hex(line)
            msg = session.feed(data) if session else decode(data)
            out = describe(msg) if "--describe" in options else msg
        except DecodeError as err:
            out = {"line": number, "error": err.reason, "detail": err.detail}
            status = 1
        except ValueError as err:
            out = {"line": number, "error": "hex", "detail": str(err)}
            status = 1
        print(json.dumps(out, sort_keys=True))
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv))

"""Dashboard decoder for the S1 binary MQTT messages.

The layout is docs/S1_binary_frame.md (layout version 1). decode() turns
one MQTT payload into a dict whose field names and values are exactly
what msg_codec_get_*() in app_program/subsystem1/msg_codec.c returns, so
the robot, the C test and the dashboard use one name per field. It
accepts and rejects the same payloads as the C decoder: the length must
be exact for the type, the version must be 1 and the type 1 to 7.
Padding bits are ignored (rule 5 of section 2). A MAP (type 7) decodes to
its four raw letter indices, 0 to 63, which are not judged: S3 is the
authority on which maps are valid.

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
LIVE = 6
MAP = 7

TYPE_NAMES = {
    TELEMETRY: "TELEMETRY",
    HEARTBEAT: "HEARTBEAT",
    EVENT: "EVENT",
    COMMAND: "COMMAND",
    ACK: "ACK",
    LIVE: "LIVE",
    MAP: "MAP",
}

# Exact payload length of each message type, in bytes.
LENGTHS = {TELEMETRY: 25, HEARTBEAT: 9, EVENT: 7, COMMAND: 2, ACK: 3,
           LIVE: 14, MAP: 5}

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
    LIVE: (
        ("seq", 8, 8, UNSIGNED),
        ("motion_state", 16, 3, UNSIGNED),
        ("b_estop", 19, 1, FLAG),
        ("cmd_speed_mms", 20, 10, SIGNED),
        ("heading_hdeg", 30, 10, SIGNED),
        ("line_state", 40, 3, UNSIGNED),
        ("steer_dv_mms", 43, 10, SIGNED),
        ("outer_ir", 53, 8, UNSIGNED),
        ("centre_ir", 61, 8, UNSIGNED),
        ("b_overshoot", 69, 1, FLAG),
        ("bc_letter", 70, 6, UNSIGNED),
        ("bc_cmd", 76, 3, UNSIGNED),
        ("b_bc_rev", 79, 1, FLAG),
        ("junc_turn", 80, 2, UNSIGNED),
        ("rejects", 82, 4, UNSIGNED),
        ("junctions", 86, 8, UNSIGNED),
        ("losts", 94, 8, UNSIGNED),
        ("barcodes", 102, 8, UNSIGNED),
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
    MAP: (
        ("cmd_id", 8, 6, UNSIGNED),
        ("letter_left", 14, 6, UNSIGNED),
        ("letter_right", 20, 6, UNSIGNED),
        ("letter_straight", 26, 6, UNSIGNED),
        ("letter_uturn", 32, 6, UNSIGNED),
    ),
}

OP_INVALID = 0
OPCODE_NAMES = {0: "INVALID", 1: "START", 2: "STOP", 3: "RESET"}
# Result 0 means the command was queued for S5, not that S5 carried it out.
RESULT_NAMES = {0: "queued", 1: "rejected"}
# An ack's opcode 0 has two uses (section 5.1): it answers a MAP, whose id
# the dashboard keeps from 1 to 63, or a command the codec refused, whose
# id is echoed or, when it could not be read, 0. So an ack with opcode 0
# and an id above 0 is named MAP, and one with id 0 INVALID. For a MAP,
# result 0 means S3 applied the map.
ACK_OP_MAP = 0
MAP_RESULT_NAMES = {0: "applied", 1: "refused"}
# The four letters of a MAP, in wire order, and the command each one is.
MAP_SLOTS = (("letter_left", "LEFT"), ("letter_right", "RIGHT"),
             ("letter_straight", "STRAIGHT"), ("letter_uturn", "U-TURN"))

# Section 3.1: bit n of fault_mask is fault n, as S5 numbers them
# (S5_FAULT_xxx in subsystem5.h).
FAULT_NAMES = {0: "stall", 1: "encoder", 2: "imu", 3: "no echo", 4: "wifi"}

# Names the owners' code defines. A value outside a table is shown as its
# number (see name_of()), so a state that is added later still displays.
# S2: subsystem2_state_t.
MOTION_NAMES = {0: "IDLE", 1: "FOLLOWING", 2: "MOVING", 3: "TURNING",
                4: "ESTOP"}
# S3: s3_line_state_t.
LINE_NAMES = {0: "FOLLOW", 1: "JUNCTION", 2: "LOST", 3: "DISABLED",
              4: "SEARCH", 5: "UNCAL"}
# S3: s3_cmd_t, copied with no remap (a renumbering by S3 changes this).
BC_CMD_NAMES = {0: "none", 1: "left", 2: "right", 3: "straight",
                4: "u-turn"}
# S3: S3_JUNC_xxx.
JUNC_NAMES = {0: "straight", 1: "left", 2: "right", 3: "u-turn"}
# S4: subsystem4_hump_t and subsystem4_class_t.
HUMP_NAMES = {0: "flat", 1: "climbing", 2: "descending"}
CLASS_NAMES = {0: "stationary", 1: "moving", 2: "accelerating",
               3: "turning", 4: "climbing hump", 5: "descending hump",
               6: "LIFTED_OVERTURNED"}
CLASS_IMPACT = 6
# S5: subsystem5_ctrl_state_t.
CTRL_NAMES = {0: "INIT", 1: "CALIBRATING", 2: "READY",
              3: "LINE_FOLLOWING", 4: "APPROACH_JUNCTION",
              5: "EXECUTING_COMMAND", 6: "REACQUIRE_LINE",
              7: "LINE_RECOVERY", 8: "OBSTACLE_AVOIDANCE", 9: "FAULT",
              10: "FINISHED"}

# S3's Code 39 index (s3_code39.c): the symbol at each index. 63 = none.
CODE39 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%"
LETTER_NONE = 63

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
        raise DecodeError("type", "type %d is not 1 to 7" % msg_type)
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


def name_of(table, value):
    """The name of a value, or its number when the table lacks it."""
    return table.get(value, str(value))


def letter_of(index):
    """The barcode symbol for S3's Code 39 index; '' when none yet."""
    if 0 <= index < len(CODE39):
        return CODE39[index]
    return ""


def code39_char(index):
    """S3's s3_code39_char(): the symbol for an index, '?' above 43."""
    if 0 <= index < len(CODE39):
        return CODE39[index]
    return "?"


def map_letters(msg):
    """The four letters of a decoded MAP as S3 will read them, e.g.
    "DCBA"; '?' for an index above 43."""
    return "".join(code39_char(msg[name]) for name, _ in MAP_SLOTS)


def map_text(letters):
    """'LEFT=D RIGHT=C STRAIGHT=B U-TURN=A' for the letters "DCBA", the
    order of S3's own console line."""
    return " ".join("%s=%s" % (slot, letter)
                    for (_, slot), letter in zip(MAP_SLOTS, letters))


def ack_is_map(msg):
    """True for an ack that answers a MAP: opcode 0 and an id above 0."""
    return (msg["opcode"] == ACK_OP_MAP) and (msg["cmd_id"] != 0)


def describe(msg):
    """Return a copy of a decoded message with display-only fields added.

    Only names and units the owners' code defines are added (the tables
    above). Event codes stay numbers until S5's table is agreed.
    """
    shown = dict(msg)
    if msg["msg_type"] == TELEMETRY:
        shown["ctrl_name"] = name_of(CTRL_NAMES, msg["ctrl_state"])
        shown["motion_name"] = name_of(MOTION_NAMES, msg["motion_state"])
        shown["line_name"] = name_of(LINE_NAMES, msg["line_state"])
        shown["bc_char"] = letter_of(msg["bc_letter"])
        shown["hump_name"] = name_of(HUMP_NAMES, msg["hump_state"])
        shown["class_name"] = name_of(CLASS_NAMES, msg["motion_class"])
        shown["b_impact"] = msg["motion_class"] == CLASS_IMPACT
        shown["pitch_deg"] = msg["pitch_hdeg"] / 2.0
        shown["roll_deg"] = msg["roll_hdeg"] / 2.0
        shown["speed_mms"] = (msg["speed_l_mms"] + msg["speed_r_mms"]) / 2.0
        shown["faults"] = [FAULT_NAMES.get(bit, "bit%d" % bit)
                           for bit in range(5)
                           if (msg["fault_mask"] >> bit) & 1]
    elif msg["msg_type"] == LIVE:
        shown["motion_name"] = name_of(MOTION_NAMES, msg["motion_state"])
        shown["heading_deg"] = msg["heading_hdeg"] / 2.0
        shown["line_name"] = name_of(LINE_NAMES, msg["line_state"])
        shown["bc_char"] = letter_of(msg["bc_letter"])
        shown["bc_cmd_name"] = name_of(BC_CMD_NAMES, msg["bc_cmd"])
        shown["junc_name"] = name_of(JUNC_NAMES, msg["junc_turn"])
    elif (msg["msg_type"] == ACK) and ack_is_map(msg):
        shown["opcode_name"] = "MAP"
        shown["result_name"] = MAP_RESULT_NAMES.get(
            msg["result"], "reserved %d" % msg["result"])
    elif msg["msg_type"] in (COMMAND, ACK):
        shown["opcode_name"] = OPCODE_NAMES[msg["opcode"]]
        if msg["msg_type"] == ACK:
            shown["result_name"] = RESULT_NAMES.get(
                msg["result"], "reserved %d" % msg["result"])
    elif msg["msg_type"] == MAP:
        shown["letters"] = map_letters(msg)
        shown["map_text"] = map_text(shown["letters"])
    else:
        pass
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
    heartbeat and live, lost_before (frames missing since the previous one
    of that type, from seq modulo 256) and duplicate (same seq as the
    previous one); for telemetry also count_l_full, count_r_full and
    counts_anchored (False on the first telemetry frame, whose full counts
    are only the 16 bits received); for live also d_barcodes, d_junctions,
    d_losts and d_rejects, how many of each S3 counted since the previous
    live message (0 on the first one), taken modulo the counter's range.
    """

    LIVE_COUNTERS = (("barcodes", 256), ("junctions", 256), ("losts", 256),
                     ("rejects", 16))

    def __init__(self):
        self._last_seq = {}
        self._full = {}

    def rebase_robot(self):
        """Forget every baseline, for when the robot restarted: its seq
        numbers, encoder counts and S3 counters all start again, so the next
        message of each type counts as the first one."""
        self._last_seq.clear()
        self._full.clear()

    def rebase_live(self):
        """Forget the live counters' baseline, for example when the robot
        restarted and S3 counts from 0 again: the next live message counts
        as the first one (d_ values 0, lost_before 0)."""
        self._last_seq.pop(LIVE, None)
        for name, _ in self.LIVE_COUNTERS:
            self._full.pop("live_" + name, None)

    def feed(self, data):
        msg = decode(data)
        msg_type = msg["msg_type"]
        if msg_type in (TELEMETRY, HEARTBEAT, LIVE):
            prev = self._last_seq.get(msg_type)
            msg["duplicate"] = (prev is not None) and (msg["seq"] == prev)
            if (prev is None) or msg["duplicate"]:
                msg["lost_before"] = 0
            else:
                msg["lost_before"] = (msg["seq"] - prev - 1) % 256
            self._last_seq[msg_type] = msg["seq"]
        if msg_type == LIVE:
            for name, modulus in self.LIVE_COUNTERS:
                key = "live_" + name
                prev_count = self._full.get(key)
                if prev_count is None:
                    msg["d_" + name] = 0
                else:
                    msg["d_" + name] = (msg[name] - prev_count) % modulus
                self._full[key] = msg[name]
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

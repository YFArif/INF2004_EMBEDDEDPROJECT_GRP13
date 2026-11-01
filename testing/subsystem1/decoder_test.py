"""S1-U11: host test of the dashboard decoder.

Tests app_program/subsystem1/dashboard/s1_decode.py against the layout in
docs/S1_binary_frame.md. The 18 golden vectors are not copied here: their
bytes and the values msg_codec_get_*() must return ("out") are read from
testing/subsystem1/codec_vectors.c, the file the C test S1-U05 uses, and
golden.py --check (stage 1 of run_decoder_test.sh) confirms that file
still agrees with the document. The decoder must give exactly those
values, under the same field names.

Run from the repository root (run_decoder_test.sh does this, then runs
the mutations):

    python3 -I -B testing/subsystem1/decoder_test.py

S1_DECODE_DIR, if set, is the folder s1_decode.py is loaded from; the
runner points it at mutated copies.
"""
import ast
import json
import os
import re
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DECODER_DIR = Path(os.environ.get(
    "S1_DECODE_DIR", str(ROOT / "app_program" / "subsystem1" / "dashboard")))
VECTORS_C = ROOT / "testing" / "subsystem1" / "codec_vectors.c"
LAYOUT_H = ROOT / "app_program" / "subsystem1" / "msg_layout.h"

sys.path.insert(0, str(DECODER_DIR))
import s1_decode as dec  # noqa: E402 (the path is set just above)

GOLDEN_COUNT = 18

# The C arrays in codec_vectors.c and the message type of each.
ARRAYS = (("g_tel", dec.TELEMETRY), ("g_hb", dec.HEARTBEAT),
          ("g_evt", dec.EVENT), ("g_cmd", dec.COMMAND), ("g_ack", dec.ACK))

TOKEN = re.compile(r'\s*(?:("(?:[^"\\]|\\.)*")|(\.[A-Za-z_]\w*)|([{}=,])'
                   r'|(-?(?:0[xX][0-9A-Fa-f]+|\d+))[uU]?|(true|false))')


def c_tokens(text):
    """Split a C initializer into tokens."""
    tokens = []
    pos = 0
    while pos < len(text):
        if not text[pos:].strip():
            break
        m = TOKEN.match(text, pos)
        if not m:
            raise ValueError("cannot parse C near %r" % text[pos:pos + 30])
        tokens.append(next(g for g in m.groups() if g is not None))
        pos = m.end()
    return tokens


def c_value(tokens, i):
    """Parse one initializer value starting at tokens[i]."""
    tok = tokens[i]
    if tok == "{":
        i += 1
        designated = {}
        plain = []
        while tokens[i] != "}":
            if tokens[i].startswith("."):
                name = tokens[i][1:]
                assert tokens[i + 1] == "=", tokens[i:i + 2]
                designated[name], i = c_value(tokens, i + 2)
            else:
                value, i = c_value(tokens, i)
                plain.append(value)
            if tokens[i] == ",":
                i += 1
        assert not (designated and plain)
        return (designated if designated else plain), i + 1
    if tok.startswith('"'):
        return tok[1:-1], i + 1
    if tok in ("true", "false"):
        return tok == "true", i + 1
    return int(tok, 0), i + 1


def load_vectors():
    """[(label, msg_type, bytes, expected fields, entry)] from the C file."""
    text = VECTORS_C.read_text(encoding="utf-8")
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = re.sub(r"//[^\n]*", " ", text)
    vectors = []
    for array, msg_type in ARRAYS:
        start = text.index("{", text.index(array + "[] ="))
        depth = 0
        for end in range(start, len(text)):
            depth += {"{": 1, "}": -1}.get(text[end], 0)
            if depth == 0:
                break
        entries, _ = c_value(c_tokens(text[start:end + 1]), 0)
        for entry in entries:
            out = dict(entry["out"])
            if "event" in out:
                # msg_codec_event_t nests code and args in .event.
                out.update(out.pop("event"))
            vectors.append((entry["p_label"], msg_type,
                            bytes(entry["bytes"]), out, entry))
    return vectors


VECTORS = load_vectors()


def first_vector(msg_type):
    return next(v for v in VECTORS if v[1] == msg_type)


def vector(label):
    return next(v for v in VECTORS if v[0] == label)


def set_field(data, offset, width, value):
    """Return data with one field replaced (MSB-first bit stream)."""
    total = 8 * len(data)
    n = int.from_bytes(data, "big")
    shift = total - offset - width
    n &= ~(((1 << width) - 1) << shift)
    n |= value << shift
    return n.to_bytes(len(data), "big")


def field_offset(msg_type, name):
    return next(f for f in dec.FIELDS[msg_type] if f[0] == name)


class GoldenVectors(unittest.TestCase):
    """Every golden vector decodes to exactly what the C decoder returns."""

    def test_count(self):
        labels = [v[0] for v in VECTORS]
        self.assertEqual(len(labels), GOLDEN_COUNT)
        self.assertEqual(len(set(labels)), GOLDEN_COUNT)

    def test_decode(self):
        for label, msg_type, data, out, entry in VECTORS:
            with self.subTest(label=label):
                msg = dec.decode(data)
                self.assertEqual(msg["version"], 1)
                self.assertEqual(msg["msg_type"], msg_type)
                names = [f[0] for f in dec.FIELDS[msg_type]]
                self.assertEqual(sorted(names), sorted(out))
                self.assertEqual({k: msg[k] for k in names}, out)
                for name in names:
                    if name.startswith("b_"):
                        self.assertIs(type(msg[name]), bool, name)
                if msg_type == dec.COMMAND:
                    self.assertEqual(msg["valid"], entry["b_get_ok"])

    def test_bytearray(self):
        data = vector("TEL-1")[2]
        self.assertEqual(dec.decode(bytearray(data)), dec.decode(data))


class Malformed(unittest.TestCase):
    """The decoder rejects what msg_codec's header_ok() rejects."""

    def reason(self, data):
        with self.assertRaises(dec.DecodeError) as caught:
            dec.decode(data)
        return caught.exception.reason

    def test_length(self):
        for msg_type in dec.LENGTHS:
            label, _, data, _, _ = first_vector(msg_type)
            with self.subTest(label=label, case="one byte short"):
                self.assertEqual(self.reason(data[:-1]), "length")
            with self.subTest(label=label, case="one byte long"):
                self.assertEqual(self.reason(data + b"\x00"), "length")

    def test_version(self):
        data = vector("TEL-1")[2]
        for version in (0, 2, 15):
            with self.subTest(version=version):
                bad = bytes([(version << 4) | (data[0] & 0x0F)]) + data[1:]
                self.assertEqual(self.reason(bad), "version")

    def test_type(self):
        data = vector("TEL-1")[2]
        for msg_type in (0, 6, 15):
            with self.subTest(msg_type=msg_type):
                bad = bytes([0x10 | msg_type]) + data[1:]
                self.assertEqual(self.reason(bad), "type")

    def test_empty(self):
        self.assertEqual(self.reason(b""), "empty")

    def test_not_bytes(self):
        for value in ("11 07", [0x11, 0x07], None):
            with self.subTest(value=value):
                with self.assertRaises(TypeError):
                    dec.decode(value)


class Padding(unittest.TestCase):
    """Padding bits are ignored (rule 5 of section 2)."""

    def test_padding_ignored(self):
        for label, offset, width in (("TEL-1", 196, 4), ("HB-1", 71, 1),
                                     ("ACK-1", 18, 6)):
            with self.subTest(label=label):
                data = vector(label)[2]
                padded = set_field(data, offset, width, (1 << width) - 1)
                self.assertNotEqual(padded, data)
                self.assertEqual(dec.decode(padded), dec.decode(data))


class Reconstruction(unittest.TestCase):
    """Full encoder counts, with the rule of section 10.1."""

    def test_rule(self):
        # The two reconstruction rows of docs/S1_binary_frame.md 10.1.
        self.assertEqual(dec.unwrap16(65535, 0), 65536)
        self.assertEqual(dec.unwrap16(0x0001FFF0, 0x0010), 0x00020010)

    def test_session(self):
        base = vector("TEL-1")[2]
        _, off_l, width, _ = field_offset(dec.TELEMETRY, "count_l")
        session = dec.Session()
        first = session.feed(set_field(base, off_l, width, 0xFFF0))
        self.assertFalse(first["counts_anchored"])
        self.assertEqual(first["count_l_full"], 0xFFF0)
        self.assertEqual(first["count_r_full"], 65535)
        second = session.feed(set_field(base, off_l, width, 0x0010))
        self.assertTrue(second["counts_anchored"])
        self.assertEqual(second["count_l_full"], 0x00010010)
        self.assertEqual(second["count_r_full"], 65535)


class LostFrames(unittest.TestCase):
    """Lost frames from seq, per message type."""

    def frame(self, label, seq):
        msg_type = vector(label)[1]
        _, offset, width, _ = field_offset(msg_type, "seq")
        return set_field(vector(label)[2], offset, width, seq)

    def test_seq_gaps(self):
        session = dec.Session()
        self.assertEqual(session.feed(self.frame("TEL-1", 255))
                         ["lost_before"], 0)
        self.assertEqual(session.feed(self.frame("TEL-1", 0))
                         ["lost_before"], 0)
        self.assertEqual(session.feed(self.frame("TEL-1", 3))
                         ["lost_before"], 2)
        again = session.feed(self.frame("TEL-1", 3))
        self.assertTrue(again["duplicate"])
        self.assertEqual(again["lost_before"], 0)
        # Heartbeats keep their own counter.
        self.assertEqual(session.feed(self.frame("HB-1", 42))
                         ["lost_before"], 0)
        self.assertEqual(session.feed(self.frame("HB-1", 45))
                         ["lost_before"], 2)


class Describe(unittest.TestCase):
    """Display-only fields."""

    def test_telemetry(self):
        msg = dec.decode(vector("TEL-1")[2])
        shown = dec.describe(msg)
        self.assertEqual(shown["pitch_deg"], -1.0)
        self.assertEqual(shown["roll_deg"], 0.5)
        self.assertEqual(shown["speed_mms"], 90.0)
        self.assertEqual(shown["faults"], ["imu"])
        self.assertNotIn("faults", msg)

    def test_fault_bits(self):
        expected = ("stall", "bit1", "imu", "bit3", "wifi")
        for bit, name in enumerate(expected):
            with self.subTest(bit=bit):
                msg = dec.decode(vector("TEL-F%d" % bit)[2])
                self.assertEqual(dec.describe(msg)["faults"], [name])

    def test_command_and_ack(self):
        for label, key, name in (("CMD-1", "opcode_name", "STOP"),
                                 ("CMD-2", "opcode_name", "START"),
                                 ("CMD-3", "opcode_name", "RESET"),
                                 ("CMD-4", "opcode_name", "INVALID"),
                                 ("ACK-1", "result_name", "accepted"),
                                 ("ACK-2", "result_name", "rejected")):
            with self.subTest(label=label):
                shown = dec.describe(dec.decode(vector(label)[2]))
                self.assertEqual(shown[key], name)


class LayoutHeader(unittest.TestCase):
    """The decoder's field table matches msg_layout.h."""

    PREFIX = {
        dec.TELEMETRY: {
            "seq": "TEL_SEQ", "ctrl_state": "TEL_CTRL",
            "motion_state": "TEL_MOTION", "fault_mask": "TEL_FAULT",
            "line_state": "TEL_LINE", "bc_letter": "TEL_BC_LETTER",
            "bc_cmd": "TEL_BC_CMD", "b_bc_dir": "TEL_BC_DIR",
            "speed_l_mms": "TEL_SPEED_L", "speed_r_mms": "TEL_SPEED_R",
            "count_l": "TEL_COUNT_L", "count_r": "TEL_COUNT_R",
            "dist_mm": "TEL_DIST", "pitch_hdeg": "TEL_PITCH",
            "roll_hdeg": "TEL_ROLL", "hump_state": "TEL_HUMP_ST",
            "peak_mm": "TEL_PEAK", "hump_count": "TEL_HUMP_CNT",
            "motion_class": "TEL_MCLASS", "turn_dps": "TEL_TURN",
            "b_imu_ok": "TEL_IMU_OK", "b_cal_done": "TEL_CAL_DONE",
            "closest_mm": "TEL_CLOSEST", "angle_idx": "TEL_ANGLE",
            "width_mm": "TEL_WIDTH", "clear_l_mm": "TEL_CLEAR_L",
            "clear_r_mm": "TEL_CLEAR_R",
        },
        dec.HEARTBEAT: {
            "seq": "HB_SEQ", "uptime_s": "HB_UPTIME", "rssi_dbm": "HB_RSSI",
            "connect_count": "HB_CONNECT", "tel_drop": "HB_TEL_DROP",
            "evt_drop": "HB_EVT_DROP", "cmd_reject": "HB_CMD_REJ",
        },
        dec.EVENT: {"code": "EVT_CODE", "args": "EVT_ARGS",
                    "t_ms": "EVT_TIME"},
        dec.COMMAND: {"opcode": "CMD_OPCODE", "cmd_id": "CMD_ID"},
        dec.ACK: {"opcode": "ACK_OPCODE", "cmd_id": "ACK_ID",
                  "result": "ACK_RESULT"},
    }
    SHORT = {dec.TELEMETRY: "TEL", dec.HEARTBEAT: "HB", dec.EVENT: "EVT",
             dec.COMMAND: "CMD", dec.ACK: "ACK"}

    @classmethod
    def setUpClass(cls):
        text = LAYOUT_H.read_text(encoding="utf-8")
        text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
        cls.defs = {}
        for m in re.finditer(r"^#define[ \t]+(MSG_\w+)[ \t]+(\S[^\n]*?)"
                             r"[ \t]*$", text, re.M):
            cls.defs[m.group(1)] = m.group(2)

    def macro(self, name):
        expr = re.sub(r"\b(0[xX][0-9A-Fa-f]+|\d+)[uU]\b", r"\1",
                      self.defs[name])

        def walk(node):
            if isinstance(node, ast.Constant) and isinstance(node.value, int):
                return node.value
            if isinstance(node, ast.Name):
                return self.macro(node.id)
            if isinstance(node, ast.BinOp):
                ops = {ast.Add: int.__add__, ast.Sub: int.__sub__,
                       ast.Mult: int.__mul__}
                return ops[type(node.op)](walk(node.left), walk(node.right))
            raise ValueError("unexpected %s in %s" % (ast.dump(node), name))

        return walk(ast.parse(expr, mode="eval").body)

    def test_version_types_lengths(self):
        self.assertEqual(dec.LAYOUT_VERSION, self.macro("MSG_LAYOUT_VERSION"))
        for msg_type, name in dec.TYPE_NAMES.items():
            with self.subTest(msg_type=name):
                self.assertEqual(msg_type, self.macro("MSG_TYPE_" + name))
                self.assertEqual(dec.LENGTHS[msg_type], self.macro(
                    "MSG_%s_BYTES" % self.SHORT[msg_type]))

    def test_fields(self):
        for msg_type, fields in dec.FIELDS.items():
            names = self.PREFIX[msg_type]
            self.assertEqual(sorted(f[0] for f in fields), sorted(names))
            for name, offset, width, _ in fields:
                with self.subTest(field=name):
                    prefix = "MSG_" + names[name]
                    self.assertEqual(offset, self.macro(prefix + "_POS"))
                    self.assertEqual(width, self.macro(prefix + "_BITS"))
            last = max(offset + width for _, offset, width, _ in fields)
            self.assertEqual(last, self.macro(
                "MSG_%s_DATA_BITS" % self.SHORT[msg_type]))


class CommandLine(unittest.TestCase):
    """s1_decode.py reads hex lines from stdin and prints JSON lines."""

    def run_cli(self, text, *options):
        script = DECODER_DIR / "s1_decode.py"
        return subprocess.run(
            [sys.executable, "-I", "-B", str(script)] + list(options),
            input=text, capture_output=True, text=True, timeout=60)

    def test_golden_lines(self):
        lines = ["# the 18 golden vectors", ""]
        lines += [data.hex(" ").upper() for _, _, data, _, _ in VECTORS]
        lines[-1] = lines[-1].replace(" ", ":")
        result = self.run_cli("\n".join(lines) + "\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        out = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(out), GOLDEN_COUNT)
        for (label, _, data, _, _), msg in zip(VECTORS, out):
            with self.subTest(label=label):
                self.assertEqual(msg, dec.decode(data))

    def test_bad_lines(self):
        result = self.run_cli("1\nzz\n11 07\n01 07\n14 85\n")
        self.assertEqual(result.returncode, 1)
        out = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([o.get("error") for o in out],
                         ["hex", "hex", "length", "version", None])
        self.assertEqual([o.get("line") for o in out[:4]], [1, 2, 3, 4])

    def test_options(self):
        tel = vector("TEL-1")[2].hex()
        result = self.run_cli(tel + "\n" + tel + "\n", "--describe",
                              "--session")
        self.assertEqual(result.returncode, 0, result.stderr)
        out = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(out[0]["pitch_deg"], -1.0)
        self.assertTrue(out[1]["duplicate"])

    def test_usage(self):
        result = self.run_cli("", "--bogus")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main(verbosity=2)

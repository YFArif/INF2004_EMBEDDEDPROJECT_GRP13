"""Golden vectors for docs/S1_binary_frame.md, built without the C codec.

Field widths and offsets are read from the document's own tables. Each
value is turned into a string of '0'/'1' characters, MSB first, and the
strings are joined; nothing here shares code with msg_codec.c, so it is an
independent check of both the codec and the document.

Usage, from the repository root:

    python3 testing/subsystem1/golden.py docs/S1_binary_frame.md
        Print every vector as label|values|hex (the 10.2 table).

    python3 testing/subsystem1/golden.py --check docs/S1_binary_frame.md \
        testing/subsystem1/codec_vectors.c
        Compare three copies of each vector: this script's own packing, the
        hex in the document's 10.2 table and the bytes in codec_vectors.c.
        Exit 0 only if all three agree for all 25 vectors (21, plus the
        4 MAP vectors of S1-U24).

TEL-INDEP is a second independent calculation from the section 3 table;
its bytes in the document were not produced by this script, which only
has to reproduce them. run_codec_test.sh runs the --check mode as part of
S1-U05.
"""
import re
import sys

USAGE = ("usage: golden.py DOC\n"
         "       golden.py --check DOC VECTORS_C")
ARGS = sys.argv[1:]
CHECK = (len(ARGS) == 3) and (ARGS[0] == "--check")
if not (CHECK or (len(ARGS) == 1 and not ARGS[0].startswith("-"))):
    print(USAGE)
    sys.exit(2)
DOC = ARGS[1] if CHECK else ARGS[0]
VERSION = 1
GOLDEN_COUNT = 25


def table(section_start, section_end, name_col, bits_col, off_col):
    """Return [(name, bits, offset)] from the first table in a section."""
    text = open(DOC, encoding="utf-8").read()
    body = text.split(section_start, 1)[1].split(section_end, 1)[0]
    rows = []
    for line in body.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) > max(bits_col, off_col) and cells[bits_col].isdigit() \
                and cells[off_col].isdigit():
            rows.append((cells[name_col], int(cells[bits_col]),
                         int(cells[off_col])))
    return rows


def pack(rows, values, msg_type, total_bytes):
    bits = ""
    for name, width, offset in rows:
        assert len(bits) == offset, (name, len(bits), offset)
        if name.startswith("version"):
            field = format(VERSION, "04b") + format(msg_type, "04b")
        elif name == "padding":
            field = "0" * width
        else:
            v = values[name]
            assert 0 <= v < (1 << width), (name, v)
            field = format(v, "0%db" % width)
        assert len(field) == width
        bits += field
    pad = total_bytes * 8 - len(bits)
    assert 0 <= pad < 8
    bits += "0" * pad
    return " ".join("%02X" % int(bits[i:i + 8], 2)
                    for i in range(0, len(bits), 8))


def off(v, width):
    """Offset binary for a signed value, checked to be in range."""
    half = 1 << (width - 1)
    assert -half <= v < half, v
    return v + half


def hdeg(cdeg):
    """Section 3.3: limit, round half away from zero, clamp."""
    lim = max(-4000, min(4000, cdeg))
    h = (abs(lim) + 25) // 50
    h = -h if lim < 0 else h
    return max(-64, min(63, h))


tel = table("## 3. TELEMETRY", "### 3.1", 1, 4, 5)
live = table("## 3A. LIVE", "### 3A.1", 0, 2, 3)
hb = table("## 4. HEARTBEAT", "## 5. COMMAND", 0, 2, 3)
cmd = table("COMMAND:", "MAP, the barcode letter map", 0, 2, 3)
mp = table("MAP, the barcode letter map", "ACK, published", 0, 2, 3)
ack = table("ACK, published", "### 5.1", 0, 2, 3)
evt = table("EVENT message on the wire:", "## 7.", 0, 2, 3)


def telemetry(seq, ctrl, motion, fault, line, letter, bccmd, bcdir, sl, sr,
              cl, cr, dist, pitch_cdeg, roll_cdeg, hst, peak, hcnt, mcls,
              turn, imu, cal, closest, angle, width, cll, clr):
    v = {
        "seq": seq % 256, "ctrl_state": ctrl, "motion_state": motion,
        "fault_mask": fault, "line_state": line, "bc_letter": letter,
        "bc_cmd": bccmd, "bc_dir": bcdir,
        "speed_l_mms": off(sl, 10), "speed_r_mms": off(sr, 10),
        "count_l": cl % 65536, "count_r": cr % 65536,
        "dist_mm": off(dist, 16),
        "pitch": off(hdeg(pitch_cdeg), 7), "roll": off(hdeg(roll_cdeg), 7),
        "hump_state": hst, "peak_mm": peak, "hump_count": hcnt,
        "motion_class": mcls, "turn_rate": off(turn, 10),
        "imu_ok": imu, "cal_done": cal, "closest_mm": closest,
        "angle_idx": angle, "width_mm": width, "clear_l_mm": cll,
        "clear_r_mm": clr,
    }
    return pack(tel, v, 1, 25)


out = []
out.append(("TEL-1 typical",
            "seq 7, ctrl 3, motion 1, fault 0x04 (IMU), line 0, letter 11, "
            "bc_cmd 2, bc_dir 1, speed_l +300, speed_r -120, count_l 70000 "
            "(sent 4464), count_r 0x0001FFFF (sent 65535), dist -1234, "
            "pitch -75 cdeg (-2 hdeg), roll +25 cdeg (+1 hdeg), hump_state 2, "
            "peak 37, hump_count 3, class 4, turn -45, imu_ok 1, cal_done 1, "
            "closest 350, angle 12, width 210, clear_l 400, clear_r 1023",
            telemetry(7, 3, 1, 0x04, 0, 11, 2, 1, 300, -120, 70000,
                      0x0001FFFF, -1234, -75, 25, 2, 37, 3, 4, -45, 1, 1,
                      350, 12, 210, 400, 1023)))
out.append(("TEL-2 all minimum",
            "every unsigned field 0, flags 0, every signed field at its "
            "minimum (speeds -512, dist -32768, pitch/roll -64 hdeg, "
            "turn -512)",
            telemetry(0, 0, 0, 0, 0, 0, 0, 0, -512, -512, 0, 0, -32768,
                      -3200, -3200, 0, 0, 0, 0, -512, 0, 0, 0, 0, 0, 0, 0)))
out.append(("TEL-3 all maximum",
            "every field at its largest stored value (speeds 511, dist "
            "32767, pitch/roll +63 hdeg, turn 511, counts 65535)",
            telemetry(255, 15, 7, 31, 7, 63, 3, 1, 511, 511, 65535, 65535,
                      32767, 3150, 3150, 3, 255, 15, 7, 511, 1, 1, 1023, 31,
                      1023, 1023, 1023)))
for n, name in enumerate(("motor stall", "encoder fault", "IMU fault",
                           "no echo", "WiFi loss")):
    out.append(("TEL-F%d" % n,
                "TEL-2 with only fault_mask 0x%02X set (flg_fault bit %d, "
                "%s; wire offset %d)" % (1 << n, n, name, 27 - n),
                telemetry(0, 0, 0, 1 << n, 0, 0, 0, 0, -512, -512, 0, 0,
                          -32768, -3200, -3200, 0, 0, 0, 0, -512, 0, 0, 0,
                          0, 0, 0, 0)))
out.append(("TEL-INDEP",
            "a second independent calculation from the section 3 table: "
            "seq 165, ctrl 9, motion 5, fault 0x01 (stall), line 6, "
            "letter 37, bc_cmd 2, bc_dir 1, speed_l -400, speed_r +400, "
            "count_l 0x00071234 (sent 0x1234), count_r 0x0009ABCD (sent "
            "0xABCD), dist -1500, pitch +75 cdeg (+2 hdeg), roll -25 cdeg "
            "(-1 hdeg), hump_state 2, peak 60, hump_count 3, class 4, "
            "turn -90, imu_ok 1, cal_done 0, closest 1500 (clamped to 1023), "
            "angle 12, width 250, clear_l 0, clear_r 1023",
            telemetry(165, 9, 5, 0x01, 6, 37, 2, 1, -400, 400, 0x00071234,
                      0x0009ABCD, -1500, 75, -25, 2, 60, 3, 4, -90, 1, 0,
                      1023, 12, 250, 0, 1023)))
def heading_hdeg(cdeg):
    """Section 3A.1: limit to +/-18000, round half away from zero."""
    lim = max(-18000, min(18000, cdeg))
    h = (abs(lim) + 25) // 50
    return -h if lim < 0 else h


def live_values(seq, motion, estop, cmd_speed, heading_h, line, steer,
                outer, centre, overshoot, letter, bccmd, rev, junc, rejects,
                junctions, losts, barcodes):
    return {
        "seq": seq % 256, "motion_state": motion, "estop": estop,
        "cmd_speed_mms": off(cmd_speed, 10),
        "heading": off(heading_h, 10),
        "line_state": line, "steer_dv": off(steer, 10),
        "n_outer": outer, "n_centre": centre, "overshoot": overshoot,
        "bc_letter": letter, "bc_cmd": bccmd, "bc_rev": rev,
        "junc_turn": junc, "rejects": rejects % 16,
        "junctions": junctions % 256, "losts": losts % 256,
        "barcodes": barcodes % 256,
    }


out.append(("LIV-1 typical",
            "seq 9, motion 1 (FOLLOWING), estop 0, cmd_speed +300, heading "
            "+4575 cdeg (+92 hdeg, rounds up), line_state 1, steer_dv -87, "
            "n_outer 12, n_centre 230, overshoot 1, bc_letter 11 ('B'), "
            "bc_cmd 2 (RIGHT), bc_rev 1, junc_turn 2, rejects 3, junctions "
            "300 (sent 44), losts 1, barcodes 5",
            pack(live, live_values(9, 1, 0, 300, heading_hdeg(4575), 1, -87,
                                   12, 230, 1, 11, 2, 1, 2, 3, 300, 1, 5),
                 6, 14)))
out.append(("LIV-2 all minimum",
            "every unsigned field 0, flags 0, every signed field at its "
            "minimum (cmd_speed -512, heading -512 hdeg, steer_dv -512)",
            pack(live, live_values(0, 0, 0, -512, -512, 0, -512, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 0, 0), 6, 14)))
out.append(("LIV-3 all maximum",
            "every field at its largest stored value (cmd_speed 511, "
            "heading 511 hdeg, steer_dv 511, bc_letter 63, bc_cmd 7)",
            pack(live, live_values(255, 7, 1, 511, 511, 7, 511, 255, 255,
                                   1, 63, 7, 1, 3, 15, 255, 255, 255), 6,
                 14)))
out.append(("HB-1",
            "seq 42, uptime 1234 s, rssi -67 dBm (sent 67), connect 3, "
            "tel_drop 1, evt_drop 0, cmd_reject 2",
            pack(hb, {"seq": 42, "uptime_s": 1234, "rssi_neg": 67,
                      "connect_count": 3, "tel_drop": 1, "evt_drop": 0,
                      "cmd_reject": 2}, 2, 9)))
out.append(("EVT-1",
            "code 0x21, args 0x25 0x30, t_ms 123456",
            pack(evt, {"code": 0x21, "args": 0x2530, "t_ms": 123456},
                 3, 7)))
for label, desc, opc, cid in (("CMD-1", "STOP, id 5", 2, 5),
                              ("CMD-2", "START, id 0", 1, 0),
                              ("CMD-3", "RESET, id 63", 3, 63),
                              ("CMD-4", "opcode 0 (rejected), id 5", 0, 5)):
    out.append((label, desc, pack(cmd, {"opcode": opc, "cmd_id": cid}, 4,
                                  2)))
for label, desc, opc, cid, res in (
        ("ACK-1", "accepted, answers CMD-1 (STOP id 5)", 2, 5, 0),
        ("ACK-2", "rejected, answers CMD-4 (opcode 0, id 5 echoed)", 0, 5,
         1),
        ("ACK-3", "rejected, command unreadable (wrong length, version or "
         "type)", 0, 0, 1)):
    out.append((label, desc, pack(ack, {"opcode": opc, "cmd_id": cid,
                                        "result": res}, 5, 3)))

# MAP letters are S3's Code 39 indices (s3_code39.c), written out here by
# hand rather than looked up, so this stays independent of the decoder.
for label, desc, cid, letters in (
        ("MAP-1", '"DCBA", id 9: letters D 13, C 12, B 11, A 10', 9,
         (13, 12, 11, 10)),
        ("MAP-2", "all minimum: id 0, every letter 0 ('0')", 0, (0, 0, 0, 0)),
        ("MAP-3", "all maximum: id 63, every letter 63 (not a Code 39 "
         "index; decoded, then refused by S3)", 63, (63, 63, 63, 63)),
        ("MAP-4", "\"09-Z\", id 33: letters 0, 9, 36 ('-'), 35 ('Z')", 33,
         (0, 9, 36, 35))):
    out.append((label, desc, pack(mp, {
        "cmd_id": cid, "letter_left": letters[0],
        "letter_right": letters[1], "letter_straight": letters[2],
        "letter_uturn": letters[3]}, 7, 5)))


def short_id(label):
    """'TEL-1 typical' -> 'TEL-1'."""
    return label.split()[0]


def doc_vectors():
    """Hex of every row of the document's 10.2 table, by short ID."""
    text = open(DOC, encoding="utf-8").read()
    body = text.split("### 10.2 Golden vectors", 1)[1].split("\n## ", 1)[0]
    found = {}
    for line in body.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 3 and re.fullmatch(r"`[0-9A-F ]+`", cells[-1]):
            found[short_id(cells[0])] = cells[-1].strip("`")
    return found


def c_vectors(path):
    """Bytes of every vector in codec_vectors.c, by label."""
    text = open(path, encoding="utf-8").read()
    found = {}
    pattern = r'\.p_label\s*=\s*"([^"]+)".*?\.bytes\s*=\s*\{([^}]*)\}'
    for label, body in re.findall(pattern, text, re.S):
        values = re.findall(r"0x([0-9A-Fa-f]{2})U", body)
        found[label] = " ".join(v.upper() for v in values)
    return found


if not CHECK:
    for label, desc, hexs in out:
        print("%s|%s|%s" % (label, desc, hexs))
    sys.exit(0)

mine = {short_id(label): hexs for label, _, hexs in out}
doc = doc_vectors()
cvec = c_vectors(ARGS[2])
failures = 0
for name, counted in (("golden.py", mine), ("document 10.2", doc),
                      ("codec_vectors.c", cvec)):
    if len(counted) != GOLDEN_COUNT:
        print("FAIL %s has %d vectors, expected %d"
              % (name, len(counted), GOLDEN_COUNT))
        failures += 1
for vid in sorted(set(mine) | set(doc) | set(cvec)):
    a, b, c = mine.get(vid), doc.get(vid), cvec.get(vid)
    if not (a is not None and a == b == c):
        print("FAIL %s: golden.py %s | document %s | codec_vectors.c %s"
              % (vid, a, b, c))
        failures += 1
if failures:
    print("golden.py --check: %d disagreement(s)" % failures)
    sys.exit(1)
print("golden.py --check: %d/%d vectors agree (golden.py, document 10.2, "
      "codec_vectors.c)" % (len(mine), GOLDEN_COUNT))
sys.exit(0)

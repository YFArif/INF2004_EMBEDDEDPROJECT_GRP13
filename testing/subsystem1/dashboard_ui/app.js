"use strict";
// The server fills the s1-data block of index.html: the page token and
// the Code 39 table. It is data, not script (script-src is 'self').
const S1_DATA = JSON.parse(document.getElementById("s1-data").textContent);
const TOKEN = S1_DATA.token;
const CODE39 = S1_DATA.code39;
const MAP_IDS = ["map-left", "map-right", "map-straight", "map-uturn"];
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
function mapCheck(letters) {
  if (letters.length !== 4) { return "Fill in all four boxes."; }
  for (const ch of letters) {
    if ((CODE39.indexOf(ch) < 0) || (ch === "*")) {
      return "\"" + ch + "\" is not a Code 39 letter S3 takes.";
    }
  }
  if (new Set(letters).size !== 4) { return "Each letter may be used once."; }
  return "";
}

function setMap(m) {
  el("map-status").textContent = m.status;
  el("map-status").className = "st-" + m.state;
}

MAP_IDS.forEach((id) => {
  el(id).addEventListener("input", (e) => {
    e.target.value = e.target.value.toUpperCase();
  });
});
el("map-set").addEventListener("click", () => {
  const letters = MAP_IDS.map((id) => el(id).value).join("");
  const why = mapCheck(letters);
  el("map-why").textContent = why;
  if (why) { return; }
  post("/api/map", {letters: letters}).then((r) => {
    if (r.status !== 200) {
      el("map-why").textContent = (r.body && r.body.error) ? r.body.error
        : "not sent (HTTP " + r.status + ")";
    } else if (r.body.offline) {
      el("map-why").textContent = "The robot is offline: the map is kept "
        + "and sent when it comes online.";
    }
  });
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
  setMap(s.map);
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
on("map", (d) => { setMap(d); addLog("map", d); });
on("map_ack", (d) => addLog("map_ack", d));

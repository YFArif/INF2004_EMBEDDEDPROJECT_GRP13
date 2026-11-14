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
el("stop-mini").addEventListener("click", () => send("stop"));
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
  addLatest(d);
  const div = document.createElement("div");
  div.className = "log-" + kind;
  div.textContent = (d.t || "").slice(11) + "  " + d.log;
  el("log").prepend(div);
  while (el("log").children.length > 300) {
    el("log").lastElementChild.remove();
  }
}

// -- Appearance: Auto follows the system; Light and Dark are remembered.
// Every localStorage access is wrapped: the page works without it.
const THEME_KEY = "s1-theme";

function readTheme() {
  try {
    const saved = localStorage.getItem(THEME_KEY);
    return (saved === "light" || saved === "dark") ? saved : "auto";
  } catch (e) {
    return "auto";
  }
}

function applyTheme(theme, save) {
  if (theme === "auto") {
    document.documentElement.removeAttribute("data-theme");
  } else {
    document.documentElement.setAttribute("data-theme", theme);
  }
  document.querySelectorAll("[data-theme-set]").forEach((b) => {
    b.setAttribute("aria-pressed", String(b.dataset.themeSet === theme));
  });
  if (save) {
    try { localStorage.setItem(THEME_KEY, theme); } catch (e) { /* none */ }
  }
}

document.querySelectorAll("[data-theme-set]").forEach((b) => {
  b.addEventListener("click", () => applyTheme(b.dataset.themeSet, true));
});
applyTheme(readTheme(), false);

// -- Which font is in use. The system font keywords resolve to SF Pro on
// Apple devices only. A test string is measured in "monospace", then in
// the keywords with monospace as the fallback: the same width means the
// keywords did not resolve, so the page falls through to Inter and needs
// Apple's tracking values (html.font-inter in tokens.css).
function fontPath() {
  const probe = document.createElement("span");
  probe.textContent = "mmmmmmmmmmlliiWW00";
  probe.style.position = "absolute";
  probe.style.visibility = "hidden";
  probe.style.whiteSpace = "nowrap";
  probe.style.fontSize = "72px";
  document.body.appendChild(probe);
  probe.style.fontFamily = "monospace";
  const mono = probe.getBoundingClientRect().width;
  probe.style.fontFamily = "-apple-system, BlinkMacSystemFont, monospace";
  const keywords = probe.getBoundingClientRect().width;
  probe.remove();
  return (Math.abs(keywords - mono) > 0.5) ? "system" : "inter";
}
document.documentElement.classList.add("font-" + fontPath());

// -- Developer drawer: closed by default. It is not modal, and the Escape
// key stays STOP, so it has its own Close button.
function setDrawer(open, target) {
  el("dev-drawer").hidden = !open;
  el("dev-open").setAttribute("aria-expanded", String(open));
  if (open) {
    (target || el("dev-close")).focus();
    if (target) { target.scrollIntoView({block: "start"}); }
  } else {
    el("dev-open").focus();
  }
}
el("dev-open").addEventListener("click",
                                () => setDrawer(el("dev-drawer").hidden));
el("dev-close").addEventListener("click", () => setDrawer(false));
el("open-log").addEventListener("click", () => {
  el("log-panel").tabIndex = -1;
  setDrawer(true, el("log-panel"));
});

// -- Latest events: the three newest lines of the event log.
const LATEST_KEEP = 3;
const latest = [];

function addLatest(d) {
  latest.unshift(d);
  latest.length = Math.min(latest.length, LATEST_KEEP);
  const box = el("latest");
  box.textContent = "";
  latest.forEach((entry) => {
    const row = document.createElement("div");
    const when = document.createElement("span");
    when.className = "when";
    when.textContent = (entry.t || "").slice(11, 19);
    row.appendChild(when);
    row.appendChild(document.createTextNode(entry.log));
    row.title = entry.log;
    box.appendChild(row);
  });
}

// -- Message rates in the header: messages received in the last 5 s.
const RATE_WINDOW_S = 5;
const seen = {tel: [], live: []};

function rateOf(times, now) {
  while (times.length && (now - times[0]) > RATE_WINDOW_S * 1000) {
    times.shift();
  }
  return (times.length / RATE_WINDOW_S).toFixed(1);
}

function paintRates(now) {
  text("rate-tel", "Telemetry " + rateOf(seen.tel, now) + " Hz");
  text("rate-live", "Live " + rateOf(seen.live, now) + " Hz");
  const marks = [feedAt.live, feedAt.tel].filter((m) => m !== null);
  text("last-frame", marks.length ? "Last frame "
       + ((now - Math.max(...marks)) / 1000).toFixed(1) + " s ago"
       : "No frame yet");
}

// -- The summary cards and their derived words. docs/S1_dashboard.md
// gives each rule; the raw value is in each word's tooltip.
const STEER_STRAIGHT_MMS = 10;    // 5 % of S3_STEER_MAX_MMS (200 mm/s).
const STEER_SLIGHT_MMS = 100;     // Half of S3_STEER_MAX_MMS.
const OBSTACLE_NEAR_MM = 350;     // S5's EV_OBSTACLE_NEAR: under 35 cm.
const CTRL_WORDS = {
  INIT: "Starting up", CALIBRATING: "Calibrating", READY: "Ready",
  LINE_FOLLOWING: "Following line",
  APPROACH_JUNCTION: "Approaching junction",
  EXECUTING_COMMAND: "Turning at junction",
  REACQUIRE_LINE: "Finding the line again",
  LINE_RECOVERY: "Recovering the line",
  OBSTACLE_AVOIDANCE: "Avoiding obstacle", FAULT: "Fault",
  FINISHED: "Finished"
};
const MOTION_WORDS = {
  IDLE: "Stopped", FOLLOWING: "Cruising", MOVING: "Cruising",
  TURNING: "Turning", ESTOP: "E-stop"
};
const LINE_WORDS = {
  JUNCTION: "At junction", LOST: "Lost", DISABLED: "Off",
  SEARCH: "Searching", UNCAL: "Not ready"
};
const HUMP_WORDS = {flat: "None", climbing: "Climbing",
                    descending: "Descending"};

function word(id, value, raw, bad) {
  el(id).textContent = value;
  el(id).title = raw;
  el(id).classList.toggle("bad", Boolean(bad));
}

function signed(v, digits) {
  if (typeof v !== "number") { return "-"; }
  return (v > 0 ? "+" : "") + v.toFixed(digits);
}

function steerWord(dv) {
  const size = Math.abs(dv);
  if (size <= STEER_STRAIGHT_MMS) { return "Straight"; }
  return (size <= STEER_SLIGHT_MMS ? "Slight " : "Hard ")
    + (dv > 0 ? "right" : "left");
}

// A number with its unit, or "-" with the unit hidden.
function measure(id, value) {
  text(id, value || "-");
  el(id).nextElementSibling.hidden = !value;
}

function capital(s) { return s.charAt(0).toUpperCase() + s.slice(1); }

function cardsTelemetry(t) {
  text("m-left", num(t.speed_l_mms));
  text("m-right", num(t.speed_r_mms));
  text("m-dist", num(t.dist_mm / 1000, "", 2));
  el("m-dist").title = "dist_mm " + t.dist_mm;
  text("a-pitch", signed(t.pitch_deg, 1));
  text("a-roll", signed(t.roll_deg, 1));
  word("a-hump", HUMP_WORDS[t.hump_name] || capital(String(t.hump_name)),
       "hump_state " + t.hump_name + ", hump_count " + t.hump_count
       + ", peak_mm " + t.peak_mm, false);
  word("a-imu", !t.b_imu_ok ? "Fault"
       : (t.b_cal_done ? "Calibrated" : "Not calibrated"),
       "imu_ok " + (t.b_imu_ok ? 1 : 0) + ", cal_done "
       + (t.b_cal_done ? 1 : 0), !t.b_imu_ok);
  const ctrl = CTRL_WORDS[t.ctrl_name] || String(t.ctrl_name);
  text("chip-ctrl-text", ctrl);
  el("chip-ctrl").title = "ctrl_state " + t.ctrl_name;
  text("stage-state", ctrl);
  const near = t.closest_mm > 0 && t.closest_mm <= OBSTACLE_NEAR_MM;
  el("chip-obstacle").hidden = !near;
  text("chip-obstacle-text", "Obstacle ahead, " + t.closest_mm + " mm");
  el("chip-faults").hidden = t.faults.length === 0;
  text("chip-faults", "Fault: " + t.faults.join(", "));
  text("o-closest", t.closest_mm ? num(t.closest_mm) : "No echo");
  el("o-closest-unit").hidden = !t.closest_mm;
  el("o-closest").title = "closest_mm " + t.closest_mm + ", angle_idx "
    + t.angle_idx + " (" + (t.angle_idx * 7.5).toFixed(1) + " deg servo)";
  const scanned = t.width_mm || t.clear_l_mm || t.clear_r_mm;
  measure("o-width", t.width_mm ? num(t.width_mm) : "");
  measure("o-left", scanned ? num(t.clear_l_mm) : "");
  measure("o-right", scanned ? num(t.clear_r_mm) : "");
}

function cardsLive(l) {
  text("m-head", num(l.heading_deg, "", 1));
  el("m-head").title = "Positive is left of the heading at power-up";
  word("chip-motion", MOTION_WORDS[l.motion_name] || String(l.motion_name),
       "motion_state " + l.motion_name + ", cmd_speed_mms "
       + l.cmd_speed_mms, false);
  el("chip-estop").hidden = !l.b_estop;
  const follow = l.line_name === "FOLLOW";
  word("l-line", follow
       ? (Math.abs(l.steer_dv_mms) <= STEER_STRAIGHT_MMS ? "Centred"
          : "Correcting")
       : (LINE_WORDS[l.line_name] || String(l.line_name)),
       "line_state " + l.line_name + ", centre " + l.centre_ir
       + ", outer " + l.outer_ir, l.line_name === "LOST");
  word("l-steer", steerWord(l.steer_dv_mms),
       "steer_dv " + l.steer_dv_mms + " mm/s (positive is right)", false);
  word("l-bc", l.bc_char || "None", l.bc_char
       ? "bc_cmd " + l.bc_cmd_name + ", read "
         + (l.b_bc_rev ? "backwards" : "forwards") : "bc_letter 63", false);
  word("l-junc", l.junctions ? capital(l.junc_name) : "None yet",
       "junc_turn " + l.junc_name + ", junctions " + l.junctions, false);
}

const STALE_S = 2;
const IMPACT_HOLD_S = 3;
const IMU_KEEP = 120;
const feedAt = {live: null, tel: null};
const PANELS = [["p-s2", ["live", "tel"], "age-s2"],
                ["p-s3", ["live"], "age-s3"],
                ["p-s4", ["tel"], "age-s4"],
                ["p-s5", ["tel"], "age-s5"],
                ["c-view", ["live", "tel"], "age-view"],
                ["c-motion", ["live", "tel"], "age-motion"],
                ["c-att", ["tel"], "age-att"],
                ["c-line", ["live"], "age-line"],
                ["c-obs", ["tel"], "age-obs"]];
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
  el("view-impact").hidden = !hot;
  paintRates(now);
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
  cardsTelemetry(t);
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
  cardsLive(l);
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
  latest.length = 0;
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
  seen.tel.push(performance.now());
  setTelemetry(d, 0);
  addLog("telemetry", d);
  imu.push([d.pitch_deg, d.roll_deg]);
  if (imu.length > IMU_KEEP) { imu.shift(); }
  drawImu();
  paintAges();
});
on("live", (d) => {
  seen.live.push(performance.now());
  setLive(d, 0);
  paintAges();
});
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

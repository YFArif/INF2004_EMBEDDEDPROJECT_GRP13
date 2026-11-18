// The 3D view of the car (S1 dashboard, stage 2).
//
// A stylised model built from boxes and cylinders, driven only by what the
// robot sends: the LIVE message (5 Hz) and the TELEMETRY frame (2 Hz), as
// app.js decodes them. Nothing here is on the wire that the page does not
// also show as a number.
//
// The car's position is an ESTIMATE (dead reckoning, see Estimate below).
// No track is drawn ahead of the car: the track is not in the data.
//
// Every colour comes from tokens.css (the --scene-* tokens).
import * as THREE from "/vendor/three/three.module.js";

// -- The car, in millimetres. MEASURED values come from the firmware.
// PLACEHOLDER values are read off photos and wait for a ruler.
const CAR = {
  trackMm: 112,            // MEASURED: S2_TRACK_MM, tyre centre to centre.
  wheelDiaMm: 66.5,        // MEASURED: rolling diameter (subsystem2.h).
  axleToCasterMm: 77.5,    // MEASURED: imu_task.c, a ruler reading by S4.
  tyreWidthMm: 26,         // PLACEHOLDER
  hubDiaMm: 44,            // PLACEHOLDER
  casterDiaMm: 14,         // PLACEHOLDER
  chassisLengthMm: 165,    // PLACEHOLDER
  chassisWidthMm: 84,      // PLACEHOLDER
  chassisDepthMm: 22,      // PLACEHOLDER: the folded side walls.
  chassisTopMm: 44,        // PLACEHOLDER: top of the plate above the floor.
  axleFromRearMm: 52,      // PLACEHOLDER: rear edge of the plate to the axle.
  batteryMm: [72, 62, 38], // PLACEHOLDER: length, width, height.
  batteryAheadMm: 8,       // PLACEHOLDER: its centre ahead of the axle.
  boardMm: [88, 74],       // PLACEHOLDER: Robo Pico length and width.
  boardTopMm: 108,         // PLACEHOLDER: top of the board above the floor.
  boardAheadMm: 12,        // PLACEHOLDER: its centre ahead of the axle.
  picoMm: [51, 21],        // PLACEHOLDER: the Pico W's nominal size.
  sonarAheadMm: 66,        // PLACEHOLDER: servo axis ahead of the axle.
  sonarHeightMm: 66,       // PLACEHOLDER: centre of the two transducers.
  sonarBoardMm: [45, 20],  // PLACEHOLDER: HC-SR04 nominal width and height.
  sonarEyeDiaMm: 16,       // PLACEHOLDER
  irAheadMm: 112,          // PLACEHOLDER: front IR modules ahead of the axle.
  irSpacingMm: 24,         // PLACEHOLDER: between the two front modules.
  irModuleMm: [14, 32],    // PLACEHOLDER: width and length.
  irHeightMm: 14,          // PLACEHOLDER
  sideIrAheadMm: 84,       // PLACEHOLDER: the third module, on the left arm.
  sideIrLeftMm: 62,        // PLACEHOLDER: left of the centre line.
  imuBarMm: [112, 14],     // PLACEHOLDER: the clear bar's width and depth.
  imuBarTopMm: 128,        // PLACEHOLDER
  imuBarAheadMm: -34,      // PLACEHOLDER: behind the axle.
  servoCentreDeg: 90,      // PLACEHOLDER: servo angle that points ahead.
  beamDeg: 30,             // PLACEHOLDER: ultrasonic beam, full width.
  coneNoEchoMm: 400,       // PLACEHOLDER: cone length drawn with no echo.
  obstacleMm: [60, 60, 20] // PLACEHOLDER: block width (if not sent), height,
                           // depth.
};

const MM = 0.001;                 // The scene is in metres.
const PIXEL_RATIO_MAX = 1.5;
const EASE_S = 0.35;              // Pitch, roll and wheel speed smoothing.
const POSE_EASE_S = 0.25;         // Position and heading smoothing.
const TRAIL_STEP_MM = 10;         // A trail point every 10 mm travelled.
const TRAIL_MAX = 4000;           // About 40 m of trail.
const TRAIL_WIDTH_MM = 12;
const DIST_CLAMP_MM = 32767;      // dist_mm stops here (frame document).
const SPEED_MAX_MMS = 511;        // The speed fields' limit.
const GAP_MAX_S = 2.5;            // A longer gap between frames: no step.
const SHAKE_S = 0.6;
const LABEL_EVERY_MS = 3000;      // The canvas's aria-label refresh.
const SETTLE = 1e-4;

const bus = window.s1Bus;
const byId = (id) => document.getElementById(id);
const canvas = byId("scene");
const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)");

// -- Colours from tokens.css. A hidden element resolves var() and
// light-dark() for the theme in use.
const swatch = document.createElement("span");
swatch.hidden = true;
document.body.appendChild(swatch);

function token(name) {
  swatch.style.color = "var(" + name + ")";
  const parts = getComputedStyle(swatch).color.match(/[\d.]+/g);
  return new THREE.Color().setRGB(parts[0] / 255, parts[1] / 255,
                                  parts[2] / 255, THREE.SRGBColorSpace);
}

const painted = [];               // [material, token name]

function paint(name, options) {
  const material = new THREE.MeshStandardMaterial(Object.assign(
    {roughness: 0.7, metalness: 0.05}, options || {}));
  painted.push([material, name]);
  return material;
}

// -- Geometry helpers. Car frame: ahead, left and up, in millimetres, from
// the floor under the middle of the axle. Scene frame: ahead is -Z, left
// is -X, up is +Y.
function at(object, ahead, left, up) {
  object.position.set(-left * MM, up * MM, -ahead * MM);
  return object;
}

function box(length, width, height, material) {
  return new THREE.Mesh(new THREE.BoxGeometry(width * MM, height * MM,
                                              length * MM), material);
}

function drum(diameter, width, material, segments) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(
    diameter * MM / 2, diameter * MM / 2, width * MM, segments || 32),
    material);
  mesh.rotation.z = Math.PI / 2;          // Axis along left-right.
  return mesh;
}

function post(height, base, ahead, left, material) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(
    2.5 * MM, 2.5 * MM, height * MM, 8), material);
  return at(mesh, ahead, left, base + height / 2);
}

// -- The model.
function buildWheel(mats) {
  const wheel = new THREE.Group();
  wheel.add(drum(CAR.wheelDiaMm, CAR.tyreWidthMm, mats.tyre));
  wheel.add(drum(CAR.hubDiaMm, CAR.tyreWidthMm + 1, mats.hub));
  // Six dark holes, as on the real hub, so that the spin can be seen.
  for (let i = 0; i < 6; i++) {
    const hole = drum(7, CAR.tyreWidthMm + 2, mats.black, 12);
    const angle = i * Math.PI / 3;
    hole.position.set(0, Math.sin(angle) * CAR.hubDiaMm * 0.3 * MM,
                      Math.cos(angle) * CAR.hubDiaMm * 0.3 * MM);
    wheel.add(hole);
  }
  return wheel;
}

function buildIr(mats, lamp) {
  const module = new THREE.Group();
  module.add(box(CAR.irModuleMm[1], CAR.irModuleMm[0], 2, mats.bracket));
  const led = box(5, 6, 2.5, lamp);
  led.position.set(0, 2 * MM, 6 * MM);
  module.add(led);
  return module;
}

function buildCar(mats) {
  const car = new THREE.Group();
  const radius = CAR.wheelDiaMm / 2;
  const half = CAR.chassisLengthMm / 2;
  const middle = half - CAR.axleFromRearMm;   // Plate centre, ahead of axle.
  const plateTop = CAR.chassisTopMm;

  // Blue chassis: the plate and its two folded side walls.
  car.add(at(box(CAR.chassisLengthMm, CAR.chassisWidthMm, 3, mats.chassis),
             middle, 0, plateTop - 1.5));
  [-1, 1].forEach((side) => {
    car.add(at(box(CAR.chassisLengthMm, 2.5, CAR.chassisDepthMm,
                   mats.chassis),
               middle, side * (CAR.chassisWidthMm / 2 - 1.25),
               plateTop - CAR.chassisDepthMm / 2));
  });

  // Two black tyres with white hubs, and the ball caster ahead.
  const wheels = [-1, 1].map((side) => {
    const wheel = buildWheel(mats);
    car.add(at(wheel, 0, -side * CAR.trackMm / 2, radius));
    return wheel;                             // [left, right]
  });
  const ball = new THREE.Mesh(new THREE.SphereGeometry(
    CAR.casterDiaMm * MM / 2, 16, 12), mats.metal);
  car.add(at(ball, CAR.axleToCasterMm, 0, CAR.casterDiaMm / 2));
  car.add(post(plateTop - CAR.chassisDepthMm - CAR.casterDiaMm + 8,
               CAR.casterDiaMm - 4, CAR.axleToCasterMm, 0, mats.metal));

  // Black battery block on the plate.
  car.add(at(box(CAR.batteryMm[0], CAR.batteryMm[1], CAR.batteryMm[2],
                 mats.black),
             CAR.batteryAheadMm, 0, plateTop + CAR.batteryMm[2] / 2));

  // Purple Robo Pico board on four standoffs, with the Pico W on it.
  const boardBase = CAR.boardTopMm - 2;
  car.add(at(box(CAR.boardMm[0], CAR.boardMm[1], 2, mats.board),
             CAR.boardAheadMm, 0, CAR.boardTopMm - 1));
  [-1, 1].forEach((fore) => [-1, 1].forEach((side) => {
    car.add(post(boardBase - plateTop, plateTop,
                 CAR.boardAheadMm + fore * (CAR.boardMm[0] / 2 - 5),
                 side * (CAR.boardMm[1] / 2 - 5), mats.black));
  }));
  car.add(at(box(CAR.picoMm[1], CAR.picoMm[0], 3, mats.pico),
             CAR.boardAheadMm - 4, 0, CAR.boardTopMm + 1.5));

  // The IMU on a clear bar on top, on two posts.
  car.add(at(box(CAR.imuBarMm[1], CAR.imuBarMm[0], 3, mats.acrylic),
             CAR.imuBarAheadMm, 0, CAR.imuBarTopMm - 1.5));
  car.add(at(box(14, 16, 3, mats.bracket),
             CAR.imuBarAheadMm, 0, CAR.imuBarTopMm + 1.5));
  [-1, 1].forEach((side) => {
    car.add(post(CAR.imuBarTopMm - 3 - CAR.boardTopMm, CAR.boardTopMm,
                 CAR.imuBarAheadMm, side * (CAR.boardMm[1] / 2 - 5),
                 mats.black));
  });

  // Ultrasonic sensor on a servo bracket at the front. The head turns
  // with the servo angle.
  car.add(at(box(22, 12, CAR.sonarHeightMm - plateTop - 12, mats.bracket),
             CAR.sonarAheadMm, 0,
             plateTop + (CAR.sonarHeightMm - plateTop - 12) / 2));
  const head = new THREE.Group();
  head.add(at(box(2, CAR.sonarBoardMm[0], CAR.sonarBoardMm[1],
                  mats.bracket), 8, 0, 0));
  [-1, 1].forEach((side) => {
    const eye = drum(CAR.sonarEyeDiaMm, 12, mats.metal, 20);
    eye.rotation.set(Math.PI / 2, 0, 0);    // Axis along ahead.
    head.add(at(eye, 14, side * 13, 0));
  });
  car.add(at(head, CAR.sonarAheadMm, 0, CAR.sonarHeightMm));

  // Two IR modules on a front bracket, with the line-state bar on it.
  const lineBar = box(4, CAR.chassisWidthMm, 4, mats.lineBar);
  car.add(at(lineBar, CAR.irAheadMm - 12, 0, CAR.irHeightMm + 8));
  car.add(at(box(34, 3, 10, mats.bracket), CAR.irAheadMm - 26,
             CAR.chassisWidthMm / 2 - 2, CAR.irHeightMm + 8));
  car.add(at(box(34, 3, 10, mats.bracket), CAR.irAheadMm - 26,
             -(CAR.chassisWidthMm / 2 - 2), CAR.irHeightMm + 8));
  [-1, 1].forEach((side) => {
    car.add(at(buildIr(mats, mats.irCentre), CAR.irAheadMm,
               side * CAR.irSpacingMm / 2, CAR.irHeightMm));
  });

  // A clear acrylic arm on the car's left with the third IR module.
  car.add(at(box(14, CAR.sideIrLeftMm - CAR.chassisWidthMm / 2 + 16, 3,
                 mats.acrylic),
             CAR.sideIrAheadMm,
             (CAR.sideIrLeftMm + CAR.chassisWidthMm / 2) / 2 - 4,
             CAR.irHeightMm + 4));
  car.add(at(buildIr(mats, mats.irOuter), CAR.sideIrAheadMm,
             CAR.sideIrLeftMm, CAR.irHeightMm));

  return {car: car, wheels: wheels, head: head};
}

// A baked blob shadow: one soft dark disc under the car. No real-time
// shadows are computed.
function buildShadow() {
  const size = 128;
  const bitmap = document.createElement("canvas");
  bitmap.width = bitmap.height = size;
  const context = bitmap.getContext("2d");
  const fade = context.createRadialGradient(size / 2, size / 2, 4,
                                            size / 2, size / 2, size / 2);
  // An alpha map: light is solid, dark is clear.
  fade.addColorStop(0, token("--scene-light").getStyle());
  fade.addColorStop(1, token("--scene-shadow").getStyle());
  context.fillStyle = fade;
  context.fillRect(0, 0, size, size);
  const material = new THREE.MeshBasicMaterial({
    alphaMap: new THREE.CanvasTexture(bitmap), transparent: true,
    opacity: 0.32, depthWrite: false});
  painted.push([material, "--scene-shadow"]);
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.26, 0.3), material);
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.set(0, 0.0015, -0.03);
  return mesh;
}

// The ultrasonic beam, flat on the floor: a sector that points ahead.
function buildCone() {
  const half = CAR.beamDeg * Math.PI / 360;
  const geometry = new THREE.CircleGeometry(1, 20, Math.PI / 2 - half,
                                            2 * half);
  geometry.rotateX(-Math.PI / 2);
  const material = new THREE.MeshBasicMaterial({
    transparent: true, opacity: 0.22, depthWrite: false,
    side: THREE.DoubleSide});
  painted.push([material, "--scene-cone"]);
  const mesh = new THREE.Mesh(geometry, material);
  mesh.position.y = 0.003;
  return mesh;
}

// The estimated trail: a flat ribbon behind the car.
class Trail {
  constructor() {
    this.points = [];
    this.positions = new Float32Array(TRAIL_MAX * 2 * 3);
    const index = [];
    for (let i = 0; i < TRAIL_MAX - 1; i++) {
      const a = i * 2;
      index.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position",
                          new THREE.BufferAttribute(this.positions, 3));
    geometry.setIndex(index);
    geometry.setDrawRange(0, 0);
    const material = new THREE.MeshBasicMaterial({
      transparent: true, opacity: 0.7, depthWrite: false,
      side: THREE.DoubleSide});
    painted.push([material, "--scene-trail"]);
    this.mesh = new THREE.Mesh(geometry, material);
    this.mesh.frustumCulled = false;
  }

  clear() {
    this.points.length = 0;
    this.mesh.geometry.setDrawRange(0, 0);
  }

  add(x, z) {
    const last = this.points[this.points.length - 1];
    if (last && Math.hypot(x - last[0], z - last[1]) < TRAIL_STEP_MM * MM) {
      return;
    }
    if (this.points.length === TRAIL_MAX) { this.points.shift(); }
    this.points.push([x, z]);
    const half = TRAIL_WIDTH_MM * MM / 2;
    const count = this.points.length;
    // Only the last two cross-sections change, unless the list shifted.
    const from = (count === TRAIL_MAX) ? 0 : Math.max(0, count - 2);
    for (let i = from; i < count; i++) {
      const here = this.points[i];
      const next = this.points[Math.min(i + 1, count - 1)];
      const prev = this.points[Math.max(i - 1, 0)];
      const dx = next[0] - prev[0];
      const dz = next[1] - prev[1];
      const size = Math.hypot(dx, dz) || 1;
      const nx = -dz / size * half;
      const nz = dx / size * half;
      this.positions.set([here[0] + nx, 0.002, here[1] + nz,
                          here[0] - nx, 0.002, here[1] - nz], i * 6);
    }
    this.mesh.geometry.attributes.position.needsUpdate = true;
    this.mesh.geometry.setDrawRange(0, Math.max(0, count - 1) * 6);
  }
}

// -- The estimate (dead reckoning). The robot sends no position. The
// page adds up the change of dist_mm between two TELEMETRY frames (2 Hz)
// along the mean of the LIVE heading (5 Hz) at the two frames. Wheel slip,
// lost frames and the half-degree heading steps all add up, so it drifts.
// It starts again at the origin when the robot restarts (its uptime goes
// down) and when Clear trail is pressed. A page reload also starts it
// again: the estimate lives in this page only.
class Estimate {
  constructor() { this.reset(); }

  reset() {
    this.x = 0;                 // Metres, scene frame.
    this.z = 0;
    this.heading = 0;           // Radians, positive is left of the start.
    this.lastDist = null;
    this.lastAt = null;
    this.lastHeading = 0;
    this.travelled = 0;         // Metres added up since the reset.
  }

  onHeading(deg) { this.heading = deg * Math.PI / 180; }

  // Returns true when the position moved.
  onTelemetry(msg, at) {
    const dist = msg.dist_mm;
    let moved = false;
    if ((this.lastDist !== null) && (typeof dist === "number")) {
      const dt = (at - this.lastAt) / 1000;
      let step = dist - this.lastDist;
      if (Math.abs(dist) >= DIST_CLAMP_MM) {
        // dist_mm is clamped, not wrapped: fall back to speed x time.
        step = (msg.speed_mms || 0) * dt;
      }
      const limit = SPEED_MAX_MMS * Math.max(dt, 0.5) * 1.5 + 50;
      if ((dt > 0) && (dt <= GAP_MAX_S) && (Math.abs(step) <= limit)) {
        const mid = this.lastHeading
          + angleDiff(this.heading, this.lastHeading) / 2;
        this.x += -Math.sin(mid) * step * MM;
        this.z += -Math.cos(mid) * step * MM;
        this.travelled += Math.abs(step) * MM;
        moved = step !== 0;
      }
    }
    this.lastDist = (typeof dist === "number") ? dist : null;
    this.lastAt = at;
    this.lastHeading = this.heading;
    return moved;
  }
}

function angleDiff(a, b) {
  return Math.atan2(Math.sin(a - b), Math.cos(a - b));
}

function ease(current, target, dt, seconds) {
  return current + (target - current) * (1 - Math.exp(-dt / seconds));
}

// -- The view.
function start() {
  const renderer = new THREE.WebGLRenderer({canvas: canvas, antialias: true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1,
                                  PIXEL_RATIO_MAX));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color();
  scene.fog = new THREE.Fog(0, 1.6, 4.5);
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20);

  scene.add(new THREE.HemisphereLight(token("--scene-light"),
                                      token("--scene-light-ground"), 2.2));
  const sun = new THREE.DirectionalLight(token("--scene-light"), 1.6);
  sun.position.set(0.4, 1, 0.3);
  scene.add(sun);

  const floorMaterial = new THREE.MeshBasicMaterial();
  painted.push([floorMaterial, "--scene-floor"]);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(12, 12),
                               floorMaterial);
  floor.rotation.x = -Math.PI / 2;
  scene.add(floor);
  const grid = new THREE.GridHelper(12, 120);    // 100 mm squares.
  grid.position.y = 0.001;
  grid.material.transparent = true;
  grid.material.opacity = 0.7;
  painted.push([grid.material, "--scene-grid"]);
  grid.material.vertexColors = false;
  grid.material.needsUpdate = true;
  scene.add(grid);

  const mats = {
    chassis: paint("--scene-chassis", {metalness: 0.4, roughness: 0.45}),
    bracket: paint("--scene-bracket"),
    board: paint("--scene-board"),
    pico: paint("--scene-pico"),
    tyre: paint("--scene-tyre", {roughness: 0.95}),
    hub: paint("--scene-hub"),
    black: paint("--scene-black"),
    metal: paint("--scene-metal", {metalness: 0.7, roughness: 0.35}),
    acrylic: paint("--scene-acrylic", {transparent: true, opacity: 0.4,
                                       roughness: 0.1}),
    lineBar: new THREE.MeshBasicMaterial(),
    irCentre: paint("--scene-black"),
    irOuter: paint("--scene-black")
  };
  const model = buildCar(mats);
  const yaw = new THREE.Group();      // Position and heading.
  const tilt = new THREE.Group();     // Pitch and roll.
  tilt.add(model.car);
  yaw.add(tilt);
  yaw.add(buildShadow());
  const beam = new THREE.Group();     // Cone and obstacle, at the sensor.
  const cone = buildCone();
  const obstacleMaterial = paint("--scene-obstacle");
  const obstacle = box(CAR.obstacleMm[2], CAR.obstacleMm[0],
                       CAR.obstacleMm[1], obstacleMaterial);
  obstacle.visible = false;
  beam.add(cone);
  beam.add(obstacle);
  beam.position.set(0, 0, -CAR.sonarAheadMm * MM);
  yaw.add(beam);
  scene.add(yaw);
  const trail = new Trail();
  scene.add(trail.mesh);

  const estimate = new Estimate();
  const want = {pitch: 0, roll: 0, speedL: 0, speedR: 0, servo: 0,
                telL: 0, telR: 0};
  const now = {x: 0, z: 0, heading: 0, pitch: 0, roll: 0, speedL: 0,
               speedR: 0, servo: 0, camHeading: 0};
  const state = {cam: "chase", b_hold: false, b_impact: false, b_beam: false,
                 shakeAt: null,
                 lineName: null, irCentre: 0, irOuter: 0, closest: 0,
                 width: 0, lastTel: null, lastLive: null, uptime: null,
                 b_dirty: true, frames: 0, lastFrame: null, raf: null,
                 fpsText: ""};

  function repaint() {
    painted.forEach(([material, name]) => material.color.copy(token(name)));
    scene.background.copy(token("--scene-bg"));
    scene.fog.color.copy(scene.background);
    paintLine();
    wake();
  }

  function paintLine() {
    const name = {FOLLOW: "--scene-line-ok", LOST: "--scene-line-lost",
                  JUNCTION: "--scene-line-turn",
                  SEARCH: "--scene-line-search"}[state.lineName]
      || "--scene-line-off";
    mats.lineBar.color.copy(token(name));
    // The IR modules' lamps brighten with the readings (0 white, 255
    // black line).
    const lit = token("--scene-line-ok");
    mats.irCentre.emissive.copy(lit).multiplyScalar(state.irCentre / 255);
    mats.irOuter.emissive.copy(lit).multiplyScalar(state.irOuter / 255);
  }

  function resize() {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if ((width === 0) || (height === 0)) { return; }
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    wake();
  }

  // -- Inputs.
  function onLive(entry) {
    const msg = entry.msg;
    state.lastLive = msg;
    estimate.onHeading(msg.heading_deg);
    // The E-stop, or a commanded speed of 0 outside a turn: the wheels
    // are eased to rest from this 5 Hz message, without waiting for the
    // next 2 Hz frame. A turn on the spot (motion_state 3, TURNING) also
    // has a commanded speed of 0, the mean of the two wheels, while both
    // wheels run: there the measured speeds keep driving the spin.
    state.b_hold = Boolean(msg.b_estop)
      || ((msg.cmd_speed_mms === 0) && (msg.motion_name !== "TURNING"));
    applySpeeds();
    if ((msg.line_name !== state.lineName) || (msg.centre_ir !== state.irCentre)
        || (msg.outer_ir !== state.irOuter)) {
      state.lineName = msg.line_name;
      state.irCentre = msg.centre_ir || 0;
      state.irOuter = msg.outer_ir || 0;
      paintLine();
    }
    wake();
  }

  function applySpeeds() {
    want.speedL = state.b_hold ? 0 : want.telL;
    want.speedR = state.b_hold ? 0 : want.telR;
  }

  function onTelemetry(entry) {
    const msg = entry.msg;
    state.lastTel = msg;
    want.telL = (msg.speed_l_mms || 0) * MM;      // Measured, 2 Hz.
    want.telR = (msg.speed_r_mms || 0) * MM;
    applySpeeds();
    want.pitch = (msg.pitch_deg || 0) * Math.PI / 180;
    want.roll = (msg.roll_deg || 0) * Math.PI / 180;
    state.closest = msg.closest_mm || 0;
    // angle_idx is 0 until S5 has pinged, and 0 would draw the beam to the
    // car's right. So the beam and the sensor's turn wait for the first
    // frame with an echo (closest_mm above 0).
    if (state.closest > 0) { showBeam(true); }
    if (state.b_beam) {
      want.servo = ((msg.angle_idx || 0) * 7.5 - CAR.servoCentreDeg)
        * Math.PI / 180;
    }
    state.width = msg.width_mm || 0;
    estimate.onTelemetry(msg, entry.at);
    // Class 6 (LIFTED) or 7 (OVERTURNED) stays for as long as the stop is
    // latched. One short shake marks the moment it appears.
    const impact = msg.class_name === "LIFTED"
      || msg.class_name === "OVERTURNED";
    if (impact && !state.b_impact && !reduceMotion.matches) {
      state.shakeAt = performance.now();
    }
    state.b_impact = impact;
    wake();
  }

  function onHeartbeat(msg) {
    if ((state.uptime !== null) && (typeof msg.uptime_s === "number")
        && (msg.uptime_s < state.uptime)) {
      clearEstimate();            // The robot restarted.
      showBeam(false);
    }
    if (typeof msg.uptime_s === "number") { state.uptime = msg.uptime_s; }
  }

  function showBeam(b_show) {
    state.b_beam = b_show;
    cone.visible = b_show;
    byId("cone-note").hidden = !b_show;
    if (!b_show) { want.servo = 0; }
  }

  function clearEstimate() {
    const heading = estimate.heading;
    estimate.reset();
    estimate.heading = heading;
    now.x = now.z = 0;
    trail.clear();
    wake();
  }

  // -- Drawing.
  function placeCamera(dt) {
    now.camHeading += angleDiff(now.heading, now.camHeading)
      * (1 - Math.exp(-dt / 0.35));
    const sin = Math.sin(now.camHeading);
    const cos = Math.cos(now.camHeading);
    const ahead = (d) => [now.x - sin * d, now.z - cos * d];
    if (state.cam === "top") {
      camera.up.set(0, 0, -1);            // The start heading points up.
      camera.position.set(now.x, 0.95, now.z);
      camera.lookAt(now.x, 0, now.z);
    } else if (state.cam === "side") {
      camera.up.set(0, 1, 0);             // From the car's right.
      camera.position.set(now.x + cos * 0.62, 0.16, now.z - sin * 0.62);
      camera.lookAt(now.x, 0.05, now.z);
    } else {
      const back = ahead(-0.4);
      const front = ahead(0.14);
      camera.up.set(0, 1, 0);
      camera.position.set(back[0] + cos * 0.12, 0.27, back[1] - sin * 0.12);
      camera.lookAt(front[0], 0.04, front[1]);
    }
  }

  function placeObstacle() {
    const label = byId("obstacle-label");
    const length = state.closest > 0 ? state.closest : CAR.coneNoEchoMm;
    cone.scale.set(length * MM, 1, length * MM);
    cone.material.opacity = state.closest > 0 ? 0.26 : 0.12;
    obstacle.visible = state.closest > 0;
    if (!obstacle.visible) {
      label.hidden = true;
      return;
    }
    const width = state.width > 0 ? state.width : CAR.obstacleMm[0];
    obstacle.scale.set(width / CAR.obstacleMm[0], 1, 1);
    obstacle.position.set(0, CAR.obstacleMm[1] * MM / 2,
                          -(state.closest + CAR.obstacleMm[2] / 2) * MM);
    // The chip sits over the block, clamped to the view.
    const spot = obstacle.getWorldPosition(new THREE.Vector3());
    spot.y += CAR.obstacleMm[1] * MM;
    spot.project(camera);
    const width_px = canvas.clientWidth;
    const height_px = canvas.clientHeight;
    label.textContent = "Obstacle " + state.closest + " mm";
    label.hidden = spot.z > 1;
    const left = Math.max(8, Math.min(width_px - label.offsetWidth - 8,
      (spot.x + 1) / 2 * width_px - label.offsetWidth / 2));
    const top = Math.max(8, Math.min(height_px - 100,
      (1 - spot.y) / 2 * height_px - label.offsetHeight - 6));
    label.style.transform = "translate(" + left.toFixed(0) + "px, "
      + top.toFixed(0) + "px)";
  }

  function frame(time) {
    state.raf = null;
    state.b_dirty = false;
    const dt = Math.min(0.1, state.lastFrame === null ? 0.016
                        : (time - state.lastFrame) / 1000);
    state.lastFrame = time;
    const before = [now.x, now.z, now.heading, now.pitch, now.roll,
                    now.speedL, now.speedR, now.servo, now.camHeading];
    now.x = ease(now.x, estimate.x, dt, POSE_EASE_S);
    now.z = ease(now.z, estimate.z, dt, POSE_EASE_S);
    now.heading += angleDiff(estimate.heading, now.heading)
      * (1 - Math.exp(-dt / POSE_EASE_S));
    // Pitch and roll arrive at 2 Hz, so the body lags by up to 0.5 s
    // behind the car. That is expected.
    now.pitch = ease(now.pitch, want.pitch, dt, EASE_S);
    now.roll = ease(now.roll, want.roll, dt, EASE_S);
    now.speedL = ease(now.speedL, want.speedL, dt, EASE_S);
    now.speedR = ease(now.speedR, want.speedR, dt, EASE_S);
    now.servo = ease(now.servo, want.servo, dt, 0.15);
    const radius = CAR.wheelDiaMm * MM / 2;
    model.wheels[0].rotation.x -= now.speedL / radius * dt;
    model.wheels[1].rotation.x -= now.speedR / radius * dt;
    yaw.position.set(now.x, 0, now.z);
    yaw.rotation.y = now.heading;
    tilt.rotation.x = now.pitch;          // Positive is nose up.
    tilt.rotation.z = -now.roll;          // Positive is left side up.
    let shaking = false;
    if (state.shakeAt !== null) {
      const age = (time - state.shakeAt) / 1000;
      shaking = age < SHAKE_S;
      if (shaking) {
        tilt.rotation.z += Math.sin(age * 60) * 0.06 * (1 - age / SHAKE_S);
      } else {
        state.shakeAt = null;
      }
    }
    model.head.rotation.y = now.servo;
    beam.rotation.y = now.servo;
    trail.add(now.x, now.z);
    placeCamera(dt);
    yaw.updateMatrixWorld(true);
    camera.updateMatrixWorld(true);
    placeObstacle();
    grid.position.x = Math.round(now.x * 10) / 10;
    grid.position.z = Math.round(now.z * 10) / 10;
    floor.position.set(grid.position.x, 0, grid.position.z);
    renderer.render(scene, camera);
    state.frames += 1;
    const after = [now.x, now.z, now.heading, now.pitch, now.roll,
                   now.speedL, now.speedR, now.servo, now.camHeading];
    const moving = after.some((value, i) => Math.abs(value - before[i])
                              > SETTLE) || (Math.abs(now.speedL) > 1e-3)
      || (Math.abs(now.speedR) > 1e-3) || shaking;
    if (moving || state.b_dirty) {
      schedule();
    } else {
      state.lastFrame = null;             // Nothing is changing: stop.
    }
  }

  // The loop runs only while something changes and the tab is visible.
  function schedule() {
    if ((state.raf === null) && !document.hidden) {
      state.raf = requestAnimationFrame(frame);
    }
  }

  function wake() {
    state.b_dirty = true;
    schedule();
  }

  function describe() {
    const live = state.lastLive;
    const tel = state.lastTel;
    if (!live && !tel) { return "3D view of the car. No data yet."; }
    const parts = ["3D view of the car."];
    if (byId("c-view").classList.contains("stale")) {
      parts.push("No new data: this is the last known state.");
    }
    parts.push(byId("chip-ctrl-text").textContent + ", "
               + byId("chip-motion").textContent.toLowerCase() + ".");
    if (tel) {
      parts.push("Wheel speeds " + tel.speed_l_mms + " and "
                 + tel.speed_r_mms + " millimetres per second.");
      parts.push("Pitch " + tel.pitch_deg + ", roll " + tel.roll_deg
                 + " degrees.");
      parts.push(tel.closest_mm > 0 ? "Obstacle at " + tel.closest_mm
                 + " millimetres." : "No obstacle echo.");
      if (state.b_impact) {
        parts.push(tel.class_name === "OVERTURNED" ? "Overturned." : "Lifted.");
      }
    }
    if (live) {
      parts.push("Heading " + Math.abs(live.heading_deg) + " degrees "
                 + (live.heading_deg >= 0 ? "left" : "right")
                 + " of the start.");
    }
    parts.push("Estimated position " + poseText() + " It drifts.");
    return parts.join(" ");
  }

  function poseText() {
    return (-estimate.z).toFixed(2) + " m ahead and "
      + Math.abs(estimate.x).toFixed(2) + " m to the "
      + (estimate.x <= 0 ? "left" : "right") + " of the start, after "
      + estimate.travelled.toFixed(2) + " m.";
  }

  // -- Wiring.
  bus.addEventListener("live", (ev) => onLive(ev.detail));
  bus.addEventListener("telemetry", (ev) => onTelemetry(ev.detail));
  bus.addEventListener("heartbeat", (ev) => onHeartbeat(ev.detail));
  bus.addEventListener("theme", () => requestAnimationFrame(repaint));
  ["(prefers-color-scheme: dark)", "(prefers-contrast: more)"].forEach(
    (query) => matchMedia(query).addEventListener("change", repaint));
  document.addEventListener("visibilitychange", () => {
    state.lastFrame = null;
    wake();
  });
  new ResizeObserver(resize).observe(canvas);
  canvas.addEventListener("webglcontextlost", (ev) => {
    ev.preventDefault();
    bus.dispatchEvent(new CustomEvent("scene-failed",
                                      {detail: "the graphics context was "
                                               + "lost"}));
  });
  byId("trail-clear").addEventListener("click", clearEstimate);
  document.querySelectorAll("[data-cam]").forEach((button) => {
    button.addEventListener("click", () => {
      state.cam = button.dataset.cam;
      document.querySelectorAll("[data-cam]").forEach((other) => {
        other.setAttribute("aria-pressed", String(other === button));
      });
      wake();
    });
  });
  setInterval(() => {
    canvas.setAttribute("aria-label", describe());
    byId("scene-pose").textContent = poseText();
  }, LABEL_EVERY_MS);
  setInterval(() => {
    const text = document.hidden ? "paused (tab hidden)"
      : (state.frames === 0 ? "idle (nothing is changing)"
         : state.frames + " fps");
    state.frames = 0;
    if (text !== state.fpsText) {
      state.fpsText = text;
      byId("fps").textContent = text;
    }
  }, 1000);
  const gl = renderer.getContext();
  byId("scene-info").textContent = "three.js r" + THREE.REVISION
    + ", WebGL " + (gl.getParameter(gl.VERSION) || "")
    + ", pixel ratio " + renderer.getPixelRatio();

  showBeam(false);
  repaint();
  resize();
  if (window.s1Last.live) { onLive(window.s1Last.live); }
  if (window.s1Last.tel) { onTelemetry(window.s1Last.tel); }
  window.s1SceneReady = true;
}

try {
  start();
} catch (err) {
  bus.dispatchEvent(new CustomEvent("scene-failed",
                                    {detail: "this browser has no WebGL"}));
}

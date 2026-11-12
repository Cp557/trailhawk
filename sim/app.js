// The simulator page: draws the VTOL and its world with three.js, reads the keyboard, and steps
// physics.js and control.js at 400 Hz. Data from out/sim/ (written by sim.py).
import * as THREE from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";
import { createController, update } from "./control.js";
import { clamp, createModel, degrees, euler, motorPose, newState, radians, rotate, step, telemetry } from "./physics.js";

const DATA = "../out/sim/";
const DT = 1 / 400;
const STICK_RATE = 4; // keyboard sticks move full scale in 1/4 s
const HIKER = [-15, -6]; // where you stand, north and east of the pad (m): the ground camera is your eyes
const CHASE = [4.5, 1.5]; // chase camera distance behind and height above the aircraft (m)

const ac = await (await fetch(DATA + "aircraft.json")).json();
const parts = await (await fetch(DATA + "parts.json")).json();
const model = createModel(ac);

// north-east-down (m) to three.js (x east, y up, z south)
const world = (p) => new THREE.Vector3(p[1], -p[2], -p[0]);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xa9c9e8);
scene.fog = new THREE.Fog(0xa9c9e8, 200, 1100);
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.05, 3000);
scene.add(new THREE.HemisphereLight(0xe3f0ff, 0x4d5b3a, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.normalBias = 0.02;
Object.assign(sun.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 1, far: 300 });
scene.add(sun, sun.target);
addEventListener("resize", () => {
  renderer.setSize(innerWidth, innerHeight);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
});

// Grass with soft, low-contrast mottling: a mipmapped texture, so it stays steady into the distance
function grass() {
  const size = 256;
  const canvas = Object.assign(document.createElement("canvas"), { width: size, height: size });
  const paint = canvas.getContext("2d");
  paint.fillStyle = "#5f7c3d";
  paint.fillRect(0, 0, size, size);
  let seed = 7;
  const random = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 900; i++) {
    const [x, y, r] = [random() * size, random() * size, 4 + random() * 14];
    paint.fillStyle = random() < 0.5 ? "rgba(72, 98, 46, 0.35)" : "rgba(86, 114, 54, 0.35)";
    for (const dx of [-size, 0, size]) {
      for (const dy of [-size, 0, size]) {
        paint.beginPath(); // drawn on all sides so the tile repeats seamlessly
        paint.arc(x + dx, y + dy, r, 0, 2 * Math.PI);
        paint.fill();
      }
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(300, 300); // 20 m tiles
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}
// Ground, then pad, are painted first as flat layers that write no depth: nothing is below them,
// and the pad can't flicker against the grass it lies on
const flat = (geometry, material, order) => {
  const mesh = new THREE.Mesh(geometry, Object.assign(material, { depthWrite: false }));
  mesh.rotation.x = -Math.PI / 2;
  mesh.receiveShadow = true;
  mesh.renderOrder = order;
  scene.add(mesh);
};
flat(new THREE.PlaneGeometry(6000, 6000), new THREE.MeshLambertMaterial({ map: grass() }), -2);
flat(new THREE.CircleGeometry(2.5, 48), new THREE.MeshLambertMaterial({ color: 0x3b3b3e }), -1);

// You: a hiker facing the pad. Circling flies around you.
const hiker = new THREE.Group();
for (const [geometry, color, y, z] of [
  [new THREE.CapsuleGeometry(0.13, 0.65, 4, 12), 0x3a4250, 0.45, 0], // legs
  [new THREE.CapsuleGeometry(0.2, 0.4, 4, 12), 0xd2452e, 1.15, 0], // jacket
  [new THREE.SphereGeometry(0.12, 16, 12), 0xe0b48c, 1.65, 0], // head
  [new THREE.BoxGeometry(0.34, 0.5, 0.2), 0x2f5d3a, 1.2, -0.22], // backpack
]) {
  const mesh = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({ color }));
  mesh.position.set(0, y, z);
  mesh.castShadow = true;
  hiker.add(mesh);
}
hiker.position.copy(world([...HIKER, 0]));
hiker.lookAt(world([0, 0, 0]));
scene.add(hiker);

// The aircraft: CAD meshes (mm) under one transform from CAD to the world. Each motor's parts sit
// in a pivot at its hinge (front motors tilt) and its prop in a spinner on the motor axis.
const aircraft = new THREE.Group();
aircraft.matrixAutoUpdate = false;
scene.add(aircraft);
const cad = new THREE.Group();
cad.matrixAutoUpdate = false;
cad.matrix.set(-0.001, 0, 0, ac.cg[0] / 1000, 0, 0.001, 0, -ac.cg[1] / 1000, 0, 0, -0.001, ac.cg[2] / 1000, 0, 0, 0, 1); // CAD mm to body axes
aircraft.add(cad);
const toVector = (v) => new THREE.Vector3(...v);
const motors = ac.motors.map((motor) => {
  const pose = motorPose(ac, motor, motor.mesh_tilt);
  const hinge = toVector(motor.hinge);
  const axisPoint = toVector(motor.hinge).add(toVector(pose.tiltCad([0, 0, motor.hub[2]])));
  const hub = toVector(motor.hinge).add(toVector(pose.tiltCad(motor.hub)));
  const axis = toVector(pose.tiltCad([-1, 0, 0]));
  const pivot = new THREE.Group();
  pivot.position.copy(hinge);
  const spinner = new THREE.Group();
  spinner.position.copy(axisPoint).sub(hinge);
  const disc = new THREE.Mesh(
    new THREE.CircleGeometry((ac.prop.diameter / 2) * 1000, 48),
    new THREE.MeshBasicMaterial({ color: 0x222222, transparent: true, opacity: 0, side: THREE.DoubleSide, depthWrite: false }),
  );
  disc.position.copy(hub).sub(hinge);
  disc.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
  pivot.add(spinner, disc);
  cad.add(pivot);
  return { motor, pivot, spinner, disc, axis, axisPoint, hinge, angle: 0 };
});

const loader = new STLLoader();
await Promise.all(
  parts.map(async (part) => {
    let geometry = loader.parse(await (await fetch(DATA + part.file)).arrayBuffer());
    geometry.deleteAttribute("normal");
    geometry = mergeVertices(geometry, 1e-3);
    geometry.computeVertexNormals();
    const color = new THREE.Color().setRGB(...part.color, THREE.SRGBColorSpace);
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05 }));
    mesh.castShadow = true;
    const [, name, leaf] = part.path.split("/");
    const unit = motors.find((m) => m.motor.name === name);
    if (!unit) {
      cad.add(mesh);
    } else if (leaf === "propeller" || leaf === "prop_nut") {
      mesh.position.copy(unit.axisPoint).negate();
      unit.spinner.add(mesh);
    } else {
      mesh.position.copy(unit.hinge).negate();
      unit.pivot.add(mesh);
    }
  }),
);

// Keyboard
const keys = new Set();
let transitionPressed = false;
let orbitPressed = false;
let notice = { text: "", until: 0 };
const tell = (text) => (notice = { text, until: performance.now() + 2500 });
let paused = false;
let chase = true;
const help = document.getElementById("help");
addEventListener("keydown", (event) => {
  if (event.code.startsWith("Arrow") || event.code === "Space") event.preventDefault();
  keys.add(event.code);
  if (event.repeat) return;
  if (event.code === "KeyT") transitionPressed = true;
  if (event.code === "KeyO") orbitPressed = true;
  if ((event.code === "KeyT" || event.code === "KeyO") && controller.mode === "landed") tell("Take off first: hold W");
  if (event.code === "KeyR") reset();
  if (event.code === "KeyP") paused = !paused;
  if (event.code === "KeyC") chase = !chase;
  if (event.code === "KeyH") help.style.display = help.style.display === "none" ? "" : "none";
});
addEventListener("keyup", (event) => keys.delete(event.code));
addEventListener("blur", () => keys.clear());
const sticks = { pitch: 0, roll: 0, yaw: 0, throttle: 0 };
const axis = (plus, minus) => (keys.has(plus) ? 1 : 0) - (keys.has(minus) ? 1 : 0);
function readSticks(dt) {
  const target = { pitch: axis("ArrowDown", "ArrowUp"), roll: axis("ArrowRight", "ArrowLeft"), yaw: axis("KeyD", "KeyA"), throttle: axis("KeyW", "KeyS") };
  for (const key in sticks) sticks[key] += clamp(target[key] - sticks[key], -STICK_RATE * dt, STICK_RATE * dt);
  return { ...sticks };
}

let state, controller, last;
let cameraYaw = 0; // the chase camera swings round behind the heading, smoothly
function reset() {
  state = newState(model);
  controller = createController();
  last = telemetry(model, state);
  cameraYaw = 0;
}
reset();

function draw(dt) {
  const columns = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map((e) => world(rotate(state.q, e)));
  aircraft.matrix.makeBasis(...columns).setPosition(world(state.p));
  aircraft.matrixWorldNeedsUpdate = true;
  motors.forEach((unit, i) => {
    if (unit.motor.tilting) unit.pivot.rotation.y = radians(state.tilt[i] - unit.motor.mesh_tilt);
    unit.angle += unit.motor.spin * Math.min(state.rpm[i] / 60, 9) * 2 * Math.PI * dt;
    unit.spinner.quaternion.setFromAxisAngle(unit.axis, unit.angle);
    unit.disc.material.opacity = 0.22 * Math.min(1, state.rpm[i] / 3000);
  });
  const target = world(state.p);
  const [, , yaw] = euler(state.q);
  cameraYaw += Math.atan2(Math.sin(yaw - cameraYaw), Math.cos(yaw - cameraYaw)) * (1 - Math.exp(-3 * dt));
  hiker.visible = chase;
  if (chase) {
    camera.fov = 55;
    camera.position.copy(world([state.p[0] - CHASE[0] * Math.cos(cameraYaw), state.p[1] - CHASE[0] * Math.sin(cameraYaw), Math.min(state.p[2] - CHASE[1], -0.5)]));
  } else {
    camera.position.copy(world([...HIKER, -1.65]));
    camera.fov = clamp(degrees(2 * Math.atan(3 / camera.position.distanceTo(target))), 3, 55);
  }
  camera.updateProjectionMatrix();
  camera.lookAt(target);
  sun.position.copy(target).add(new THREE.Vector3(40, 90, 25));
  sun.target.position.copy(target);
  renderer.render(scene, camera);
}

const hud = document.getElementById("hud");
const message = document.getElementById("message");
const MODES = { landed: "LANDED", hover: "HOVER", transition: "TRANSITION", plane: "PLANE", back: "BACK TRANSITION", crashed: "CRASHED" };
function show(text, warning = false) {
  message.style.display = text ? "" : "none";
  message.textContent = text;
  message.className = warning ? "panel warning" : "panel";
}
function updateHud() {
  const [, , yaw] = euler(state.q);
  const act = controller.last ?? { throttle: [0, 0, 0] };
  const front = act.throttle.filter((_, i) => ac.motors[i].tilting);
  const rear = act.throttle.find((_, i) => !ac.motors[i].tilting);
  const percent = (x) => `${Math.round(100 * x)}%`;
  const left = 1 - state.energy / ac.battery_wh;
  const climb = Math.round(last.climb * 10) / 10 || 0; // no "-0.0"
  const rows = [
    ["Airspeed", `${last.airspeed.toFixed(1)} m/s` + (controller.mode === "plane" ? ` (target ${controller.speedTarget.toFixed(1)})` : "")],
    ["Altitude", `${last.altitude.toFixed(1)} m`],
    ["Climb", `${climb >= 0 ? "+" : ""}${climb.toFixed(1)} m/s`],
    ["Heading", `${String((Math.round(degrees(yaw)) + 360) % 360).padStart(3, "0")}°`],
    ["Throttle", `front ${percent(Math.max(...front))}, rear ${percent(rear)}`],
    ["Motor tilt", `${Math.round(90 - controller.baseTilt)}° forward`],
    ["Power", `${Math.round(last.watts)} W`],
    ["Battery", `${percent(Math.max(0, left))} of ${ac.battery_wh.toFixed(0)} Wh usable`],
    ["Air", ac.air],
  ];
  if (controller.orbit) rows.splice(1, 0, ["Circling you", `${ac.orbit_radius} m out (now ${Math.round(Math.hypot(state.p[0] - HIKER[0], state.p[1] - HIKER[1]))} m), ${Math.round(controller.orbitHeight)} m up`]);
  hud.innerHTML = `<div class="mode">${MODES[controller.mode]}${controller.orbit ? ", CIRCLING YOU" : ""}${paused ? " (PAUSED)" : ""}</div><table>${rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("")}</table>`;
  if (state.crashed) show(`Crashed: ${state.crashed}. Press R to restart.`, true);
  else if (performance.now() < notice.until) show(notice.text);
  else if (controller.mode === "landed" && state.time < 1) show("Hold W to take off");
  else if (last.stalled > 0.5 && ["plane", "transition"].includes(controller.mode)) show("STALL", true);
  else if (left < 0) show("Battery at its 20% reserve: land", true);
  else show("");
}

let previous = performance.now();
let accumulator = 0;
function frame(now) {
  const dt = Math.min(0.05, (now - previous) / 1000);
  previous = now;
  if (!paused) {
    const input = { ...readSticks(dt), transition: false };
    accumulator += dt;
    while (accumulator >= DT) {
      input.transition = transitionPressed;
      input.orbit = orbitPressed ? HIKER : null;
      transitionPressed = orbitPressed = false;
      last = step(model, state, update(controller, model, state, last, input, DT), DT);
      accumulator -= DT;
    }
  }
  draw(dt);
  updateHud();
  requestAnimationFrame(frame);
}
show("");
requestAnimationFrame(frame);

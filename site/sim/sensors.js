// What the aircraft's cameras make of the world, a look a second: which animals and rocks each one
// detects, where it puts them, and how much ground it has covered. A model, not image processing:
// the pixels across a target follow from the camera's resolution and range, and the odds of
// detecting or recognizing it from Johnson's criteria (line pairs across the target for even odds)
// through the ACQUIRE model's curve. Its thresholds and canopy figures are assumptions.
// Cameras (aircraft.json, from parts.py): a FLIR Lepton 3.5 thermal camera, 160 x 120 pixels across
// 57 deg, and a Raspberry Pi Camera Module 3, 66 deg across, whose frames a detector reads at DETECTOR.
// Points are [north, east, down] in metres from the pad.
import { cross, dot, euler, norm, radians, sub } from "./physics.js";
import { SIZE, WILDLIFE, gaussian } from "./world.js";

const DETECTOR = [1920, 1080]; // px: color frames as the detector reads them (the camera takes 4608 x 2592)
const TILT = radians(45); // the turret holds its cameras this far down; it only pans
const LOOK_AHEAD = 150; // m: while circling, the turret pans toward the trail this far past the hiker,
const SWEEP = radians(45); // sweeping this far either side of it
const SWEEP_SECONDS = 12; // and back in this long, so the looks cover a wide strip ahead
const CYCLES = { detect: 1, recognize: 4 }; // Johnson's criteria: line pairs across a target for even odds
const CANOPY = { thermal: 0.7, color: 0.9 }; // assumed: share of the odds that full forest takes away
const WARM_ROCK = 0.5; // assumed: a sun-warmed rock stands out half as well as an animal in thermal
const LAKE_RANGE = 600; // m: the color camera maps the lake with its center in view this close
const BEAR_RANGE = 150; // m: a bear named this close to the trail ahead of the hiker gets a warning

// ACQUIRE's curve for the odds of a task with `cycles` across the target, even odds at `half`
export function odds(cycles, half) {
  const ratio = cycles / half;
  const k = ratio ** (2.7 + 0.7 * ratio);
  return k / (1 + k);
}

// Each camera's view across and down (rad), and the angle a pixel spans
export function cameras(ac) {
  const camera = (hfov, [across, down]) => {
    const h = radians(hfov);
    return { h, v: 2 * Math.atan((Math.tan(h / 2) * down) / across), pixel: h / across };
  };
  return { thermal: camera(ac.cameras.thermal.hfov, [ac.cameras.thermal.pixels, (ac.cameras.thermal.pixels * 3) / 4]), color: camera(ac.cameras.color.hfov, DETECTOR) };
}

// Where the cameras look from the aircraft: panned toward `toward` ([north, east]), or the way it's
// heading, turned `pan` further, and TILT down. Forward, right and image-down unit vectors.
export function aim(state, toward, pan = 0) {
  const bearing = pan + (toward ? Math.atan2(toward[1] - state.p[1], toward[0] - state.p[0]) : euler(state.q)[2]);
  const forward = [Math.cos(bearing) * Math.cos(TILT), Math.sin(bearing) * Math.cos(TILT), Math.sin(TILT)];
  const right = [-Math.sin(bearing), Math.cos(bearing), 0];
  return { from: [...state.p], forward, right, down: cross(forward, right) };
}

// Whether the offset `v` from the cameras is inside one camera's view
function inView(camera, view, v) {
  const ahead = dot(v, view.forward);
  return ahead > 0 && Math.abs(dot(v, view.right)) < ahead * Math.tan(camera.h / 2) && Math.abs(dot(v, view.down)) < ahead * Math.tan(camera.v / 2);
}

// Whether the point `v` away is in sight, the ground not in the way: checked every 10 m, short of
// its last 3 m
function clear(land, from, v, range) {
  const steps = Math.ceil(range / 10);
  for (let k = 1; k < steps; k++) {
    const f = Math.min(k / steps, 1 - 3 / range);
    if (land.ground(from[0] + f * v[0], from[1] + f * v[1]) > -(from[2] + f * v[2])) return false;
  }
  return true;
}

// Where the middle of the view meets level ground at height `h` (the cameras look TILT down, so it
// always does)
function onGround(view, h) {
  const drop = -h - view.from[2];
  return [view.from[0] + (view.forward[0] * drop) / view.forward[2], view.from[1] + (view.forward[1] * drop) / view.forward[2]];
}

// Which cameras can see an animal (in their view, and the ground not in the way), and its range
export function sees(cams, land, view, animal) {
  const target = [...animal.position, -(land.terrain(...animal.position) + 0.6 * animal.size[1])];
  const v = sub(target, view.from);
  const range = norm(v);
  const names = ["thermal", "color"].filter((name) => inView(cams[name], view, v));
  return { names: names.length && clear(land, view.from, v, range) ? names : [], range };
}

// One look: what each camera detects of the wildlife it can see. The thermal camera finds warm
// shapes (detection); the color camera names them (recognition): elk, deer, bear or rock. Each
// report puts the target where the camera saw it, off by 1 m plus 1% of the range. Under trees the
// odds fall by CANOPY times the forest's density there. `next` is the random stream.
export function look(cams, land, view, wildlife, next) {
  const reports = [];
  wildlife.forEach((animal, index) => {
    const [length, height] = animal.size;
    const { names, range } = sees(cams, land, view, animal);
    const cover = land.forest(animal.position);
    for (const name of names) {
      const cycles = Math.sqrt(length * height) / (range * cams[name].pixel) / 2; // across the target's mean size
      const chance =
        name === "thermal"
          ? odds(cycles, CYCLES.detect) * (1 - CANOPY.thermal * cover) * (animal.kind === "rock" ? WARM_ROCK : 1)
          : odds(cycles, CYCLES.recognize) * (1 - CANOPY.color * cover);
      if (next() >= chance) continue;
      const error = 1 + 0.01 * range;
      reports.push({ index, camera: name, label: name === "thermal" ? "warm" : animal.kind, position: [animal.position[0] + error * gaussian(next), animal.position[1] + error * gaussian(next)] });
    }
  });
  const lake = [...land.lake.center, -land.water];
  const toLake = sub(lake, view.from);
  if (norm(toLake) < LAKE_RANGE && inView(cams.color, view, toLake)) reports.push({ index: "lake", camera: "color", label: "lake", position: land.lake.center });
  return reports;
}

// The ground the color camera has covered, in CELL m squares across the world, row by row from the
// north-west corner: marked once inside its view and closer than where it would recognize an elk
// even odds (the ground's folds ignored)
const CELL = 10;
const SPAN = (2 * SIZE) / CELL; // cells across
export const cellAt = ([north, east]) => Math.floor((SIZE - north) / CELL) * SPAN + Math.floor((east + SIZE) / CELL);
const cellCenter = (r, c) => [SIZE - (r + 0.5) * CELL, -SIZE + (c + 0.5) * CELL];
export function createCoverage() {
  return { cells: new Uint8Array(SPAN ** 2) };
}
function cover(coverage, cams, land, view) {
  const camera = cams.color;
  const [length, height] = WILDLIFE.elk.size;
  const reach = Math.sqrt(length * height) / (2 * CYCLES.recognize * camera.pixel); // m
  const center = onGround(view, land.ground(view.from[0], view.from[1]));
  const h = land.ground(...center);
  const [r0, c0] = [Math.floor((SIZE - center[0] - reach) / CELL), Math.floor((center[1] + SIZE - reach) / CELL)];
  for (let r = Math.max(0, r0); r < Math.min(SPAN, r0 + (2 * reach) / CELL); r++) {
    for (let c = Math.max(0, c0); c < Math.min(SPAN, c0 + (2 * reach) / CELL); c++) {
      const v = sub([...cellCenter(r, c), -h], view.from);
      if (norm(v) < reach && inView(camera, view, v)) coverage.cells[r * SPAN + c] = 1;
    }
  }
}

// The cells within 150 m of the trail (`points`, every fifth checked): the share of them covered is
// how much of the hike's surroundings the cameras have seen
export function nearTrail(points) {
  const near = [];
  for (let r = 0; r < SPAN; r++) {
    for (let c = 0; c < SPAN; c++) {
      const p = cellCenter(r, c);
      if (points.some((q, k) => k % 5 === 0 && Math.abs(p[0] - q[0]) < 150 && Math.hypot(p[0] - q[0], p[1] - q[1]) < 150)) near.push(r * SPAN + c);
    }
  }
  return near;
}

// The bears named within BEAR_RANGE of `ahead`, the trail's points from the hiker on. Each as
// [pin key, metres off the trail].
function bearsAhead(sightings, ahead) {
  const near = [];
  for (const [key, pin] of sightings) {
    if (pin.label !== "bear") continue;
    const off = Math.min(...ahead.map((p) => Math.hypot(p[0] - pin.position[0], p[1] - pin.position[1])));
    if (off < BEAR_RANGE) near.push([key, off]);
  }
  return near;
}

// What the cameras have reported, a pin per target: labelled warm until the color camera names it,
// at its latest reported place. Tracking is perfect: each animal keeps its pin (a real tracker would
// match reports by place).
function record(sightings, reports) {
  for (const report of reports) {
    const pin = sightings.get(report.index) ?? { label: "warm" };
    if (report.label !== "warm") pin.label = report.label;
    pin.position = report.position;
    sightings.set(report.index, pin);
  }
}

// The turret's view: while `circling`, sweeping either side of the trail LOOK_AHEAD m past the
// hiker; else ahead
export const turret = (land, state, circling, walked) =>
  circling ? aim(state, land.along(walked + LOOK_AHEAD), SWEEP * Math.sin((2 * Math.PI * state.time) / SWEEP_SECONDS)) : aim(state, null);

// The look a second that the page takes and the checks repeat: the turret's view; the reports
// pinned in `sightings` and the ground marked in `coverage`, with the dice from `rolls`. Returns the
// view, the wildlife as it stood, and the bears named near the trail ahead of the hiker.
export function survey(cams, land, state, circling, walked, { sightings, coverage, rolls }) {
  const view = turret(land, state, circling, walked);
  const wildlife = land.wildlifeAt(state.time);
  record(sightings, look(cams, land, view, wildlife, rolls));
  cover(coverage, cams, land, view);
  const ahead = land.trail.points.slice(land.trail.points.indexOf(land.along(walked)));
  return { view, wildlife, bears: bearsAhead(sightings, ahead) };
}

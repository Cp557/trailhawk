// Flies the simulator headless and prints the results as JSON, for test_sim.py:
//   node tests/check.mjs site/data/aircraft.json
// On flat ground, on made-up slopes and hills, and over generated worlds following the walking hiker.
import { readFileSync } from "node:fs";
import { createController, missionInput, missionStart, update } from "../site/sim/control.js";
import { DT, createModel, euler, newState, radians, step, telemetry, wrap } from "../site/sim/physics.js";
import { aim, cameras, createCoverage, look, nearTrail, odds, sees, survey } from "../site/sim/sensors.js";
import { SPACING, createBeacon, createWorld, distance, samplePoint, seeded } from "../site/sim/world.js";

const model = createModel(JSON.parse(readFileSync(process.argv[2], "utf8")));
const still = { pitch: 0, roll: 0, yaw: 0, throttle: 0, transition: false };

// Fly `seconds` with inputs from `input(time)`; returns the telemetry of every step.
function fly(state, controller, seconds, input = () => still, m = model) {
  const log = [];
  let t = telemetry(m, state);
  for (let time = 0; time < seconds; time += DT) {
    t = step(m, state, update(controller, m, state, t, input(time), DT), DT);
    log.push({ ...t, mode: controller.mode, time: state.time, north: state.p[0], east: state.p[1] });
  }
  return log;
}

const mean = (values) => values.reduce((a, b) => a + b, 0) / values.length;
const last = (log, seconds) => log.filter((t) => t.time > log[log.length - 1].time - seconds);

// Steady starts: hovering at 30 m, and flying level at 15.5 m/s in plane mode, both heading north
function hovering(m = model, altitude = 30) {
  const controller = createController();
  controller.mode = "hover";
  return [newState(m, { altitude }), controller];
}
function cruising(altitude = 100, m = model) {
  const controller = createController();
  Object.assign(controller, { mode: "plane", baseTilt: 0, speedTarget: 15.5, i: { speed: 0.6, trim: 0.07 } });
  return [newState(m, { altitude, speed: 15.5, pitch: 0.07, tilt: 0 }), controller];
}

const hoverLog = last(fly(...hovering(), 12), 4);
const cruiseLog = last(fly(...cruising(), 40), 15);

// Circling a point 150 m east, from level flight north at 40 m and the slowest cruise speed; then
// holding the pitch stick forward (down) for 5 s, which should lower the held height
const center = [0, 150];
const orbiting = cruising(40);
const orbitLog = last(fly(...orbiting, 150, (time) => ({ ...still, orbit: time < DT ? center : null })), 60);
fly(...orbiting, 5, () => ({ ...still, pitch: -1 }));
const afterDive = last(fly(...orbiting, 30), 5);
const distances = orbitLog.map((t) => Math.hypot(t.north - center[0], t.east - center[1]));
const bearings = orbitLog.map((t) => Math.atan2(t.east - center[1], t.north - center[0]));
const swept = bearings.slice(1).reduce((sum, b, i) => sum + wrap(b - bearings[i]), 0);

// What holding one key does from a steady start: change in north, east, heading, altitude, airspeed
function effect([state, controller], input, seconds) {
  const [north, east, heading, altitude, airspeed] = [state.p[0], state.p[1], euler(state.q)[2], -state.p[2], Math.hypot(...state.v)];
  fly(state, controller, seconds, () => ({ ...still, ...input }));
  const turn = euler(state.q)[2] - heading;
  return {
    north: state.p[0] - north,
    east: state.p[1] - east,
    heading: wrap(turn),
    altitude: -state.p[2] - altitude,
    airspeed: Math.hypot(...state.v) - airspeed,
  };
}

// Take off, transition at 40 m, fly on, back-transition, hover, land
const tripState = newState(model);
const trip = createController();
const climb = fly(tripState, trip, 16, () => ({ ...still, throttle: 1 }));
const outbound = fly(tripState, trip, 90, (time) => ({ ...still, transition: time < DT })); // long enough for the stopped rear motor to spin all the way down
const inbound = fly(tripState, trip, 25, (time) => ({ ...still, transition: time < DT }));
const landing = fly(tripState, trip, 60, () => ({ ...still, throttle: -1 }));
const plane = outbound.find((t) => t.mode === "plane");
const altitudes = [...outbound, ...inbound].map((t) => t.altitude);

// Made-up terrain. Sitting landed on a 5 deg slope (rising to the north) for a minute after settling.
const tilted = createModel(model.ac, (north) => north * Math.tan(radians(5)));
const sitting = [newState(tilted), createController()];
fly(...sitting, 5, undefined, tilted);
const settled = [...sitting[0].p];
fly(...sitting, 60, undefined, tilted);
// Landing on that slope from 10 m up, and on a valley floor 30 m below the pad
const slopeLanding = hovering(tilted, 10);
fly(...slopeLanding, 25, () => ({ ...still, throttle: -1 }), tilted);
const valley = createModel(model.ac, () => -30);
const valleyLanding = hovering(valley, -20);
fly(...valleyLanding, 25, () => ({ ...still, throttle: -1 }), valley);
// Circling over ground 20 m above the pad, and flying level into a 30 deg ridge 200 m ahead
const raised = createModel(model.ac, () => 20);
const raisedOrbit = cruising(40, raised);
const raisedLog = last(fly(...raisedOrbit, 150, (time) => ({ ...still, orbit: time < DT ? center : null }), raised), 30);
const ridge = createModel(model.ac, (north) => Math.max(0, north - 200) * Math.tan(radians(30)));
const ridgeRun = cruising(40, ridge);
fly(...ridgeRun, 40, undefined, ridge);

// Generated worlds (world.js): what the ground, trail, lake and trees promise
const least = (values) => values.reduce((a, b) => Math.min(a, b), Infinity);
const most = (values) => values.reduce((a, b) => Math.max(a, b), -Infinity);
function worldChecks(land) {
  const pad = [];
  for (let north = -8; north <= 8; north++) for (let east = -8; east <= 8; east++) if (Math.hypot(north, east) <= 8) pad.push(Math.abs(land.ground(north, east)));
  const trail = land.trail.points.map((p) => land.terrain(...p));
  const trees = Array.from({ length: land.trees.length / 3 }, (_, i) => [land.trees[3 * i], land.trees[3 * i + 1]]);
  const [lakeNorth, lakeEast] = land.lake.center;
  const rim = Array.from({ length: 180 }, (_, i) => {
    const bearing = (2 * Math.PI * i) / 180;
    return land.terrain(lakeNorth + land.lake.radius * Math.cos(bearing), lakeEast + land.lake.radius * Math.sin(bearing));
  });
  let wet = 0;
  land.heights.forEach((h, i) => {
    if (h < land.water && distance(samplePoint(i), land.lake.center) < land.lake.radius) wet++;
  });
  return {
    pad: most(pad),
    trail_length: land.trail.length,
    steepest_trail: most(trail.slice(10).map((h, i) => Math.abs(h - trail[i]) / 20)), // over 20 m
    trail_above_water: least(trail) - land.water,
    trees: trees.length,
    tree_to_trail: least(trees.map((p) => land.trailDistance(...p))),
    tree_to_pad: least(trees.map((p) => Math.hypot(...p))),
    tree_above_water: least(trees.map((p) => land.terrain(...p))) - land.water,
    lake_area: wet * SPACING ** 2,
    rim_above_water: least(rim) - land.water,
  };
}

// The cameras alone, on made-up ground: from 100 m up, panned toward a point 75 m north (as when
// circling), 60 looks at an elk there in the open, under full forest and behind a ridge; and at one
// 75 m behind the aircraft
const cams = cameras(model.ac);
const above = aim(newState(model, { altitude: 100 }), [75, 0]);
const meadow = (forest = 0, ground = () => 0) => ({ ground, terrain: ground, forest: () => forest, lake: { center: [9000, 9000] }, water: -100 });
const elk = (north) => ({ kind: "elk", position: [north, 0], size: [2.4, 1.5] });
function looks(land, animal) {
  const [rolls, tally] = [seeded(5), { thermal: 0, color: 0 }];
  for (let i = 0; i < 60; i++) for (const report of look(cams, land, above, [animal], rolls)) tally[report.camera]++;
  return tally;
}

// What the cameras found on a flight: animals found and named, by how far each one's spot is from
// the trail; rocks flagged warm and ruled out; the lake; the share of ground within 150 m of the
// trail covered; and any pin for an animal that was never in view
function findings(land, sightings, coverage, inView) {
  const points = land.trail.points;
  const off = (p) => least(points.map((q) => Math.hypot(p[0] - q[0], p[1] - q[1])));
  const wild = land.wildlifeAt(0).map((animal, i) => ({ ...animal, off: off(animal.position), pin: sightings.get(i) }));
  const animals = wild.filter((a) => a.kind !== "rock");
  const tally = (list) => ({ named: list.filter((a) => a.pin?.label === a.kind).length, of: list.length });
  const rocks = wild.filter((a) => a.kind === "rock");
  const near = nearTrail(points);
  return {
    within_100: tally(animals.filter((a) => a.off < 100)),
    from_100_to_200: tally(animals.filter((a) => a.off >= 100 && a.off < 200)),
    beyond_200: tally(animals.filter((a) => a.off >= 200)),
    bears: tally(animals.filter((a) => a.kind === "bear")),
    rocks: { flagged: rocks.filter((a) => a.pin).length, ruled_out: rocks.filter((a) => a.pin?.label === "rock").length },
    lake_mapped: sightings.has("lake"),
    seen_near_trail: near.filter((cell) => coverage.cells[cell]).length / near.length,
    pins_never_in_view: [...sightings.keys()].filter((key) => key !== "lake" && !inView.has(key)).length,
  };
}

// The page's own flight (missionStart, missionInput): circling the hiker from the start as he sets
// off, and following him until a minute after he reaches the lake. Distances are to where he is,
// not to his phone's fixes, from a minute and a half in. The cameras look once a second, as on the page.
function following(land) {
  const real = createModel(model.ac, land.ground);
  const beacon = createBeacon(land.seed);
  const [state, controller] = missionStart(real, beacon(land.hikerAt(0).position, 0));
  let walk = 0;
  while (!land.hikerAt(walk).done) walk++;
  let t = telemetry(real, state);
  const log = [];
  const [sightings, coverage, rolls, inView, warned] = [new Map(), createCoverage(), seeded(17), new Set(), new Set()];
  for (let n = 0; state.time < walk + 60 && !state.crashed; n++) {
    const hiker = land.hikerAt(state.time);
    const you = hiker.position;
    const fix = beacon(you, state.time);
    t = step(real, state, update(controller, real, state, t, missionInput(fix), DT), DT);
    if (n % Math.round(1 / DT) === 0 && controller.mode !== "landed") {
      const { view, wildlife, bears } = survey(cams, land, state, controller.orbit, hiker.walked, { sightings, coverage, rolls });
      wildlife.forEach((animal, i) => sees(cams, land, view, animal).names.length && inView.add(i));
      for (const [key] of bears) warned.add(key); // the page's warning
    }
    if (state.time > 90 && n % 40 === 0) log.push({ off: Math.hypot(state.p[0] - you[0], state.p[1] - you[1]), above: t.altitude - land.ground(...you), height: t.height });
  }
  return {
    crashed: state.crashed,
    mode: controller.mode,
    circling: controller.orbit !== null,
    walk_minutes: walk / 60,
    radius: mean(log.map((e) => e.off)),
    farthest_off: most(log.map((e) => Math.abs(e.off - model.ac.orbit_radius))),
    above_hiker: mean(log.map((e) => e.above)),
    lowest: least(log.map((e) => e.height)),
    highest: most(log.map((e) => e.height)),
    cameras: { ...findings(land, sightings, coverage, inView), bears_warned_of: warned.size },
  };
}
const FOLLOWED = [1, 2, 3]; // seeds flown to the lake; 0 and -7 only generated
const worlds = [...FOLLOWED, 0, -7].map((seed) => {
  const land = createWorld(seed);
  return { seed, ...worldChecks(land), flight: FOLLOWED.includes(seed) ? following(land) : null };
});
const [first, again] = [createWorld(1), createWorld(1)];
const repeatable = first.heights.every((h, i) => h === again.heights[i]) && first.trees.every((v, i) => v === again.trees[i]);

console.log(
  JSON.stringify({
    hover: {
      thrust_grams: model.ac.motors.map((_, i) => (mean(hoverLog.map((t) => t.thrusts[i])) / model.ac.gravity) * 1000),
      altitude_change: hoverLog[hoverLog.length - 1].altitude - 30,
      watts: mean(hoverLog.map((t) => t.watts)),
    },
    cruise: {
      watts: mean(cruiseLog.map((t) => t.watts)) - model.ac.electronics_watts,
      airspeed: mean(cruiseLog.map((t) => t.airspeed)),
      climb: mean(cruiseLog.map((t) => t.climb)),
    },
    orbit: {
      radius: mean(distances),
      farthest_off: Math.max(...distances.map((d) => Math.abs(d - model.ac.orbit_radius))),
      laps_per_minute: swept / (2 * Math.PI), // positive: clockwise seen from above
      altitude_change: orbitLog[orbitLog.length - 1].altitude - orbitLog[0].altitude,
      height: mean(orbitLog.map((t) => t.altitude)),
      watts: mean(orbitLog.map((t) => t.watts)),
      height_after_dive: mean(afterDive.map((t) => t.altitude)),
      climb_after_dive: mean(afterDive.map((t) => t.climb)),
      circling_after_dive: orbiting[1].orbit !== null,
    },
    sticks: {
      hover_forward: effect(hovering(), { pitch: -1 }, 3).north,
      hover_right: effect(hovering(), { roll: 1 }, 3).east,
      hover_yaw: effect(hovering(), { yaw: 1 }, 2).heading,
      hover_climb: effect(hovering(), { throttle: 1 }, 3).altitude,
      plane_dive: effect(cruising(), { pitch: -1 }, 5).altitude,
      plane_bank: effect(cruising(), { roll: 1 }, 5).heading,
      plane_rudder: effect(cruising(), { yaw: 1 }, 3).heading,
      plane_faster: effect(cruising(), { throttle: 1 }, 5).airspeed,
    },
    trip: {
      takeoff_altitude: climb[climb.length - 1].altitude,
      seconds_to_plane: plane ? plane.time - outbound[0].time : null,
      lowest: Math.min(...altitudes),
      highest: Math.max(...altitudes),
      final_mode: inbound[inbound.length - 1].mode,
      final_speed: inbound[inbound.length - 1].airspeed,
      landed: landing[landing.length - 1].mode,
      crashed: tripState.crashed,
    },
    slopes: {
      crept: Math.hypot(sitting[0].p[0] - settled[0], sitting[0].p[1] - settled[1]),
      slope_landing: slopeLanding[1].mode,
      slope_crashed: slopeLanding[0].crashed,
      valley_landing: valleyLanding[1].mode,
      valley_crashed: valleyLanding[0].crashed,
      raised_orbit_height: mean(raisedLog.map((t) => t.altitude)),
      ridge: ridgeRun[0].crashed,
    },
    worlds,
    repeatable,
    sensors: {
      even_odds: odds(1, 1),
      thermal_pixels: Math.sqrt(2.4 * 1.5) / (Math.hypot(75, 100 - 0.9) * cams.thermal.pixel),
      open: looks(meadow(), elk(75)),
      forest: looks(meadow(1), elk(75)),
      ridge: looks(meadow(0, (north) => (north > 30 && north < 50 ? 80 : 0)), elk(75)),
      behind: looks(meadow(), elk(-75)),
    },
  }),
);

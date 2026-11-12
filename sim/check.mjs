// Flies the simulator headless and prints the results as JSON, for test_sim.py:
//   node sim/check.mjs out/sim/aircraft.json
import { readFileSync } from "node:fs";
import { createController, update } from "./control.js";
import { createModel, euler, newState, step, telemetry } from "./physics.js";

const DT = 1 / 400;
const model = createModel(JSON.parse(readFileSync(process.argv[2], "utf8")));
const still = { pitch: 0, roll: 0, yaw: 0, throttle: 0, transition: false };

// Fly `seconds` with inputs from `input(time)`; returns the telemetry of every step.
function fly(state, controller, seconds, input = () => still) {
  const log = [];
  let t = telemetry(model, state);
  for (let time = 0; time < seconds; time += DT) {
    t = step(model, state, update(controller, model, state, t, input(time), DT), DT);
    log.push({ ...t, mode: controller.mode, time: state.time, pitch: euler(state.q)[1], north: state.p[0], east: state.p[1] });
  }
  return log;
}

const mean = (values) => values.reduce((a, b) => a + b, 0) / values.length;
const last = (log, seconds) => log.filter((t) => t.time > log[log.length - 1].time - seconds);

// Steady starts: hovering at 30 m, and flying level at 15.5 m/s in plane mode, both heading north
function hovering() {
  const controller = createController();
  controller.mode = "hover";
  return [newState(model, { altitude: 30 }), controller];
}
function cruising(altitude = 100) {
  const controller = createController();
  Object.assign(controller, { mode: "plane", baseTilt: 0, speedTarget: 15.5, i: { speed: 0.6, trim: 0.07 } });
  return [newState(model, { altitude, speed: 15.5, pitch: 0.07, tilt: 0 }), controller];
}

const hoverLog = last(fly(...hovering(), 12), 4);
const cruiseLog = last(fly(...cruising(), 40), 15);

// Circling a point 150 m east, from level flight north at 40 m and the slowest cruise speed; then
// holding the dive key (up) for 5 s, which should lower the held height
const center = [0, 150];
const orbiting = cruising(40);
const orbitLog = last(fly(...orbiting, 150, (time) => ({ ...still, orbit: time < DT ? center : null })), 60);
fly(...orbiting, 5, () => ({ ...still, pitch: -1 }));
const afterDive = last(fly(...orbiting, 30), 5);
const distances = orbitLog.map((t) => Math.hypot(t.north - center[0], t.east - center[1]));
const bearings = orbitLog.map((t) => Math.atan2(t.east - center[1], t.north - center[0]));
const swept = bearings.slice(1).reduce((sum, b, i) => sum + Math.atan2(Math.sin(b - bearings[i]), Math.cos(b - bearings[i])), 0);

// What holding one key does from a steady start: change in north, east, heading, altitude, airspeed
function effect([state, controller], input, seconds) {
  const [north, east, heading, altitude, airspeed] = [state.p[0], state.p[1], euler(state.q)[2], -state.p[2], Math.hypot(...state.v)];
  fly(state, controller, seconds, () => ({ ...still, ...input }));
  const turn = euler(state.q)[2] - heading;
  return {
    north: state.p[0] - north,
    east: state.p[1] - east,
    heading: Math.atan2(Math.sin(turn), Math.cos(turn)),
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
      pitch: mean(cruiseLog.map((t) => t.pitch)),
    },
    orbit: {
      radius: mean(distances),
      farthest_off: Math.max(...distances.map((d) => Math.abs(d - model.ac.orbit_radius))),
      laps_per_minute: swept / (2 * Math.PI), // positive: clockwise seen from above
      altitude_change: orbitLog[orbitLog.length - 1].altitude - orbitLog[0].altitude,
      height: mean(orbitLog.map((t) => t.altitude)),
      watts: mean(orbitLog.map((t) => t.watts)),
      airspeed: mean(orbitLog.map((t) => t.airspeed)),
      height_after_dive: mean(afterDive.map((t) => t.altitude)),
      climb_after_dive: mean(afterDive.map((t) => t.climb)),
      circling_after_dive: orbiting[1].orbit !== null,
    },
    keys: {
      hover_up: effect(hovering(), { pitch: -1 }, 3).north,
      hover_right: effect(hovering(), { roll: 1 }, 3).east,
      hover_d: effect(hovering(), { yaw: 1 }, 2).heading,
      hover_w: effect(hovering(), { throttle: 1 }, 3).altitude,
      plane_up: effect(cruising(), { pitch: -1 }, 5).altitude,
      plane_right: effect(cruising(), { roll: 1 }, 5).heading,
      plane_d: effect(cruising(), { yaw: 1 }, 3).heading,
      plane_w: effect(cruising(), { throttle: 1 }, 5).airspeed,
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
  }),
);

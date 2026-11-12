// Flight controller: a simplified ArduPilot tilt-tricopter QuadPlane, flown from the keyboard.
//   landed      on the ground, motors stopped. Climb (W) takes off into hover.
//   hover       like QLOITER: the arrows set ground speed, W/S the climb rate, A/D the yaw rate.
//               Holds altitude and brakes to a stop when the keys are released. Lands on touchdown.
//   transition  the front motors tilt forward to TILT_MAX, holding altitude, until min_speed
//   plane       like FBWB: the arrows set bank and climb rate, W/S the airspeed target, A/D rudder.
//               Front motors tilt fully forward; the rear motor stops.
//   back        back transition: the motors tilt up and hover control brakes to a stop
// Circling (like LOITER): in plane mode the controller banks to fly clockwise circles of
// orbit_radius around a point and climbs or descends to orbit_height, then holds it; the climb keys
// move the held height and the pilot keeps the airspeed. Turning it on in hover starts a transition first.
// Input: pitch, roll, yaw, throttle in -1..1 (pitch +1 is the stick pulled back); transition: true
// on the frame T is pressed; orbit: the [north, east] point to circle on the frame O is pressed.
import { clamp, cross, degrees, euler, motorPose, radians } from "./physics.js";

const TILT_MAX = 45; // deg forward of vertical while transitioning (Q_TILT_MAX)
const TILT_RATE_UP = 15; // deg/s toward forward flight (Q_TILT_RATE_UP)
const TILT_RATE_DOWN = 45; // deg/s back to vertical (Q_TILT_RATE_DN)
const TRANSITION_PITCH = radians(4); // about the wing's trim at min_speed
const MIX_TILT = 15; // deg: below this the front motors barely lift, so mix as if here
const HOVER_SPEED = 6; // m/s of ground speed at full stick
const HOVER_CLIMB = 2.5; // m/s
const LAND_SPEED = 0.7; // m/s of descent at touchdown, more higher up (Q_LAND_SPEED)
const LEAN = radians(25);
const YAW_RATE = radians(60);
const BANK = radians(45);
const PLANE_CLIMB = 3; // m/s
const BRAKE = 2.5; // m/s^2 of deceleration in the back transition
const BRAKE_PITCH = radians(6); // most nose-up while braking, so the wing doesn't balloon
const STALL_MARGIN = 1.4; // banks are limited so a level turn needs at most 1/1.4 of the wing's maximum lift
const GAIN = {
  velocity: radians(5), // lean per m/s of ground speed error
  angle: 3, // 1/s: attitude error to rate
  heading: 2, // 1/s: heading error to yaw rate
  rate: [6, 6, 3], // 1/s: rate error to angular acceleration (roll, pitch, yaw)
  rateI: [3, 3, 1.5],
  climb: 2, // 1/s: climb rate error to vertical acceleration
  climbI: 1,
  bank: 2.5,
  pitch: 3,
  planeRate: [5, 5, 2],
  planeRateI: [2, 2, 1],
  planeClimb: 0.05, // rad of pitch per m/s of climb error
  planeClimbI: 0.03,
  speed: 0.08, // throttle per m/s of airspeed error
  speedI: 0.04,
  course: 0.4, // 1/s: course error to turn rate while circling
  height: 0.2, // 1/s: height error to climb rate while circling
};

export function createController() {
  return { mode: "landed", i: {}, baseTilt: 90, speedTarget: 0, brakeSpeed: 0, landing: 0, heading: null, orbit: null, orbitHeight: 0, last: null };
}

// Proportional-integral term; the integral lives in c.i[key], clamped to +-limit
function pi(c, key, error, kp, ki, dt, limit) {
  c.i[key] = clamp((c.i[key] ?? 0) + error * ki * dt, -limit, limit);
  return kp * error + c.i[key];
}

function solve3(m, b) {
  const det = (a) => a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  const d = det(m);
  return [0, 1, 2].map((k) => det(m.map((row, i) => row.map((x, j) => (j === k ? b[i] : x)))) / d);
}

// Yaw rate: the pilot's, or holding the heading where the pilot left it
function yawRate(c, input, yaw) {
  if (input.yaw !== 0 || c.heading === null) {
    c.heading = yaw;
    return input.yaw * YAW_RATE;
  }
  return GAIN.heading * Math.atan2(Math.sin(c.heading - yaw), Math.cos(c.heading - yaw));
}

// Ground velocity along and across the heading
function groundVelocity(s, yaw) {
  const [north, east] = s.v;
  return [north * Math.cos(yaw) + east * Math.sin(yaw), -north * Math.sin(yaw) + east * Math.cos(yaw)];
}

const idle = (ac) => ({ throttle: [0, 0, 0], tilt: ac.motors.map(() => 90), aileron: 0, elevator: 0, rudder: 0 });

// Each motor's [lift, roll moment, pitch moment] per newton of thrust at its tilt
function thrustColumns(ac, tilts) {
  return ac.motors.map((m, i) => {
    const { hub, axis } = motorPose(ac, m, m.tilting ? Math.max(MIX_TILT, tilts[i]) : 90);
    const [roll, pitch] = cross(hub, axis);
    return [-axis[2], roll, pitch];
  });
}

// Surface deflections for an angular-acceleration demand, gains scaled by dynamic pressure
function surfaces(model, moment, airspeed) {
  const q = 0.5 * model.ac.density * Math.max(airspeed, 5) ** 2;
  const deflect = (control, k) => clamp(moment[k] / (q * model.controlPower[control][k]), -radians(25), radians(25));
  return { aileron: deflect("aileron", 0), elevator: deflect("elevator", 1), rudder: deflect("rudder", 2) };
}

// Multicopter control: attitude and climb rate targets to motor thrusts. Roll and pitch come from
// differential thrust, yaw from tilting the front motors opposite ways (Q_TILT_TYPE 2), up to
// yaw_tilt. Attitude comes first: lift is cut before a motor saturates. With airspeed the
// control surfaces take over the attitude, as ArduPilot mixes them in VTOL modes.
function vtol(c, model, s, t, { roll, pitch, yawRate, climb, tilt }, dt) {
  const { ac } = model;
  const [phi, theta] = euler(s.q);
  const rates = [GAIN.angle * (roll - phi), GAIN.angle * (pitch - theta), yawRate];
  const moment = rates.map((r, k) => ac.inertia[k][k] * pi(c, `rate${k}`, r - s.w[k], GAIN.rate[k], GAIN.rateI[k], dt, 10));
  const assist = clamp((t.airspeed - 6) / 6, 0, 1); // share of the attitude moments left to the surfaces
  const lift = (ac.mass * (ac.gravity + pi(c, "climb", climb + s.v[2], GAIN.climb, GAIN.climbI, dt, 5))) / Math.max(0.5, Math.cos(phi) * Math.cos(theta));
  const tilts = ac.motors.map((m) => (m.tilting ? tilt : 90));
  const columns = thrustColumns(ac, tilts);
  const matrix = [0, 1, 2].map((row) => columns.map((column) => column[row]));
  const perLift = solve3(matrix, [1, 0, 0]);
  const forMoment = solve3(matrix, [0, moment[0] * (1 - assist), moment[1] * (1 - assist)]);
  // A motor tilted toward forward flight pushes more than it lifts: cap it so it doesn't race ahead
  const cap = ac.motors.map((m, i) => model.maxThrust * (m.tilting ? clamp(1.5 * Math.sin(radians(tilts[i])), 0, 1) : 1));
  const room = perLift.map((l, i) => (l > 0 ? (cap[i] - forMoment[i]) / l : Infinity));
  const collective = clamp(Math.min(lift, ...room), 0, lift);
  const thrust = perLift.map((l, i) => clamp(collective * l + forMoment[i], 0, cap[i]));
  const front = ac.motors.flatMap((m, i) => (m.tilting ? [i] : []));
  const arm = Math.abs(motorPose(ac, ac.motors[front[0]], tilt).hub[1]);
  const frontThrust = Math.max(1, (thrust[front[0]] + thrust[front[1]]) / 2);
  const limit = Math.sin(radians(ac.yaw_tilt));
  const yawTilt = degrees(Math.asin(clamp((moment[2] * (1 - assist)) / (2 * arm * frontThrust), -limit, limit)));
  return {
    throttle: thrust.map((f) => Math.sqrt(f / model.maxThrust)),
    tilt: ac.motors.map((m) => (m.tilting ? tilt + Math.sign(m.hinge[1]) * yawTilt : 90)),
    ...surfaces(model, moment.map((m) => m * assist), t.airspeed),
  };
}

// Bank for flying clockwise around c.orbit: steer for the tangent, turned toward the circle (45 deg
// from one radius off), plus the steady bank of a turn of orbit_radius once on it
function orbitBank(c, ac, s, speed) {
  const [north, east] = [s.p[0] - c.orbit[0], s.p[1] - c.orbit[1]];
  const off = Math.hypot(north, east) - ac.orbit_radius;
  const course = Math.atan2(s.v[1], s.v[0]);
  const wanted = Math.atan2(east, north) + Math.PI / 2 + Math.atan(off / ac.orbit_radius);
  const on = clamp(1 - Math.abs(off) / ac.orbit_radius, 0, 1);
  const turn = GAIN.course * Math.atan2(Math.sin(wanted - course), Math.cos(wanted - course)) + (on * speed) / ac.orbit_radius;
  return Math.atan((speed * turn) / ac.gravity);
}

// Climb rate while circling: the pilot's, or holding the height where the pilot left it
function orbitClimb(c, input, altitude) {
  if (input.pitch !== 0) {
    c.orbitHeight = altitude;
    return input.pitch * PLANE_CLIMB;
  }
  return clamp(GAIN.height * (c.orbitHeight - altitude), -PLANE_CLIMB, PLANE_CLIMB);
}

// Fixed-wing control: bank and climb rate targets through the control surfaces (gains scaled by
// dynamic pressure), airspeed through the front motors' throttle.
function plane(c, model, s, t, input, dt) {
  const { ac } = model;
  const speed = Math.max(t.airspeed, 5);
  c.speedTarget = clamp(c.speedTarget + input.throttle * 3 * dt, ac.min_speed, ac.max_speed);
  const [phi, theta] = euler(s.q);
  const climb = c.orbit ? orbitClimb(c, input, t.altitude) : input.pitch * PLANE_CLIMB; // stick back climbs
  const path = Math.asin(clamp(climb / speed, -0.5, 0.5));
  const pitch = clamp(path + pi(c, "trim", climb - t.climb, GAIN.planeClimb, GAIN.planeClimbI, dt, radians(15)), -radians(20), radians(20));
  const turn = (ac.gravity * Math.tan(phi)) / speed; // heading rate of a coordinated turn
  const bankLimit = Math.min(BANK, Math.acos(clamp(STALL_MARGIN * (ac.stall_speed / speed) ** 2, 0, 1)));
  const bank = clamp(c.orbit ? orbitBank(c, ac, s, speed) : input.roll * BANK, -bankLimit, bankLimit);
  const rates = [GAIN.bank * (bank - phi), GAIN.pitch * (pitch - theta) + turn * Math.sin(phi), turn * Math.cos(phi) + input.yaw * radians(30)];
  const moment = rates.map((r, k) => ac.inertia[k][k] * pi(c, `plane${k}`, r - s.w[k], GAIN.planeRate[k], GAIN.planeRateI[k], dt, 10));
  const throttle = clamp(pi(c, "speed", c.speedTarget - t.airspeed, GAIN.speed, GAIN.speedI, dt, 1), 0, 1);
  // Until the front motors are fully forward, the rear motor cancels their thrust's pitch moment
  const columns = thrustColumns(ac, ac.motors.map(() => c.baseTilt));
  const frontPitch = columns.reduce((sum, column, i) => sum + (ac.motors[i].tilting ? column[2] * (t.thrusts[i] ?? 0) : 0), 0);
  const rear = ac.motors.findIndex((m) => !m.tilting);
  const rearThrust = c.baseTilt > 0 ? clamp(-frontPitch / columns[rear][2], 0, model.maxThrust) : 0;
  return {
    throttle: ac.motors.map((m, i) => (m.tilting ? throttle : Math.sqrt(rearThrust / model.maxThrust))),
    tilt: ac.motors.map((m) => (m.tilting ? c.baseTilt : 90)),
    ...surfaces(model, moment, t.airspeed),
  };
}

function toggleTransition(c, s) {
  if (c.mode === "hover" && !s.onGround) {
    c.mode = "transition";
  } else if (c.mode === "back") {
    c.mode = "transition";
  } else if (c.mode === "transition" || c.mode === "plane") {
    const [, , yaw] = euler(s.q);
    c.mode = "back";
    c.orbit = null;
    c.brakeSpeed = Math.max(0, groundVelocity(s, yaw)[0]);
    c.i = { rate0: c.i.plane0, rate1: c.i.plane1, rate2: c.i.plane2 }; // keep the surfaces' trim
  }
}

function toggleOrbit(c, s, center, height) {
  if (c.orbit) c.orbit = null;
  else if (c.mode === "plane" || c.mode === "transition") Object.assign(c, { orbit: center, orbitHeight: height });
  else if ((c.mode === "hover" || c.mode === "back") && !s.onGround) Object.assign(c, { orbit: center, orbitHeight: height, mode: "transition" });
}

// Actuator commands for this step; `t` is the last step's telemetry.
export function update(c, model, s, t, input, dt) {
  const { ac } = model;
  if (s.crashed) c.mode = "crashed";
  if (input.transition) toggleTransition(c, s);
  if (input.orbit) toggleOrbit(c, s, input.orbit, ac.orbit_height);
  const [, , yaw] = euler(s.q);
  const ground = groundVelocity(s, yaw);
  let act;
  if (c.mode === "landed" || c.mode === "crashed") {
    if (c.mode === "landed" && input.throttle > 0) {
      c.mode = "hover";
      c.i = {};
    }
    act = idle(ac);
  } else if (c.mode === "hover") {
    act = vtol(c, model, s, t, {
      roll: clamp(GAIN.velocity * (input.roll * HOVER_SPEED - ground[1]), -LEAN, LEAN),
      pitch: clamp(-GAIN.velocity * (-input.pitch * HOVER_SPEED - ground[0]), -LEAN, LEAN),
      yawRate: yawRate(c, input, yaw),
      climb: Math.max(input.throttle * HOVER_CLIMB, -(LAND_SPEED + 0.35 * t.altitude)),
      tilt: (c.baseTilt = 90),
    }, dt); // prettier-ignore
    c.landing = s.onGround && input.throttle <= 0 ? c.landing + dt : 0;
    if (c.landing > 0.5) c.mode = "landed";
  } else if (c.mode === "transition") {
    c.baseTilt = Math.max(90 - TILT_MAX, c.baseTilt - TILT_RATE_UP * dt);
    act = vtol(c, model, s, t, { roll: input.roll * radians(15), pitch: TRANSITION_PITCH, yawRate: yawRate(c, input, yaw), climb: 0, tilt: c.baseTilt }, dt);
    if (t.airspeed >= ac.min_speed) {
      const frontThrottle = act.throttle.filter((_, i) => ac.motors[i].tilting);
      c.mode = "plane";
      c.i = { speed: Math.max(...frontThrottle), trim: TRANSITION_PITCH, plane0: c.i.rate0, plane1: c.i.rate1, plane2: c.i.rate2 };
      c.speedTarget = Math.max(ac.min_speed + 2, t.airspeed);
    }
  } else if (c.mode === "plane") {
    c.baseTilt = Math.max(0, c.baseTilt - TILT_RATE_UP * dt);
    c.heading = null;
    act = plane(c, model, s, t, input, dt);
  } else if (c.mode === "back") {
    c.baseTilt = Math.min(90, c.baseTilt + TILT_RATE_DOWN * dt);
    c.brakeSpeed = Math.max(0, c.brakeSpeed - BRAKE * dt);
    act = vtol(c, model, s, t, {
      roll: clamp(-GAIN.velocity * ground[1], -LEAN, LEAN),
      pitch: clamp(clamp(-GAIN.velocity * (c.brakeSpeed - ground[0]), -LEAN, BRAKE_PITCH) - GAIN.planeClimb * (t.climb - input.throttle * HOVER_CLIMB), -LEAN, BRAKE_PITCH),
      yawRate: yawRate(c, input, yaw),
      climb: input.throttle * HOVER_CLIMB,
      tilt: c.baseTilt,
    }, dt); // prettier-ignore
    if (c.baseTilt >= 90 && Math.hypot(...ground) < 2) c.mode = "hover";
  }
  c.last = act;
  return act;
}

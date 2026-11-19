// Flight model of the VTOL: a rigid body with three motors, lifting panels (wing, tail, fin),
// parasite drag and skids. Data: site/data/aircraft.json, written by drone/sim.py.
// Units SI. Body axes: x forward, y right, z down, origin at the CG. World axes: north, east,
// down, origin at the pad; the ground is at z = -ground(north, east). CAD axes (mm): X aft, Y right, Z up.

const RPM_LAG = 0.08; // s, motor spin-up time constant (estimate)
const TILT_RATE = 400; // deg/s, tilt servo slew (KST X12-708: 0.07 s per 60 deg, unloaded)
const STIFFNESS = 3000; // N/m per skid contact point
const DAMPING = 60; // N s/m per skid contact point
const FRICTION = 0.6; // skids on grass
const CRASH_SPEED = 3; // m/s: a skid hitting the ground faster than this is a crash
const PLATE = 1.2; // normal-force coefficient of a stalled panel (flat plate)
const STALL_BLEND = (8 * Math.PI) / 180; // past stall, lift falls to the flat plate's over this angle

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const norm = (a) => Math.sqrt(dot(a, a));
export const radians = (deg) => (deg * Math.PI) / 180;
export const degrees = (rad) => (rad * 180) / Math.PI;
export const clamp = (x, low, high) => Math.min(high, Math.max(low, x));
export const wrap = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle)); // to -pi..pi
export const DT = 1 / 400; // s: the physics step

const matrixTimes = (m, v) => [dot(m[0], v), dot(m[1], v), dot(m[2], v)];

function invert(m) {
  const [[a, b, c], [d, e, f], [g, h, i]] = m;
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  return [
    [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
    [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
    [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
  ];
}

// Quaternions [w, x, y, z] rotate body vectors into the world.
function multiply(p, q) {
  return [
    p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
    p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
    p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
    p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0],
  ];
}

export function rotate(q, v) {
  const [, x, y, z] = multiply(multiply(q, [0, ...v]), [q[0], -q[1], -q[2], -q[3]]);
  return [x, y, z];
}

const unrotate = (q, v) => rotate([q[0], -q[1], -q[2], -q[3]], v);

function fromEuler(roll, pitch, yaw) {
  const [cr, sr, cp, sp, cy, sy] = [roll, roll, pitch, pitch, yaw, yaw].map((a, i) => (i % 2 ? Math.sin(a / 2) : Math.cos(a / 2)));
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}

export function euler(q) {
  const [w, x, y, z] = q;
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    Math.asin(clamp(2 * (w * y - z * x), -1, 1)),
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  ];
}

// Linear interpolation in a table, held at the ends (or `right` past the last x).
function interpolate(x, xs, ys, right = ys[ys.length - 1]) {
  if (x <= xs[0]) return ys[0];
  if (x >= xs[xs.length - 1]) return right;
  let i = 1;
  while (xs[i] < x) i++;
  return ys[i - 1] + ((ys[i] - ys[i - 1]) * (x - xs[i - 1])) / (xs[i] - xs[i - 1]);
}

// CAD geometry into body axes
const toBody = (v) => [-v[0], v[1], -v[2]];
const bodyPoint = (ac, p) => scale(toBody(sub(p, ac.cg)), 0.001);

// Rotation about CAD +Y: tilt 0 points a front motor forward, 90 straight up
function tiltCad(v, deg) {
  const [c, s] = [Math.cos(radians(deg)), Math.sin(radians(deg))];
  return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
}

// A motor at `tilt`: prop hub (body, m) and thrust axis (body unit vector)
export function motorPose(ac, motor, tilt) {
  return {
    hub: bodyPoint(ac, add(motor.hinge, tiltCad(motor.hub, tilt))),
    axis: toBody(tiltCad([-1, 0, 0], tilt)),
    tiltCad: (v) => tiltCad(v, tilt),
  };
}

function propThrust(model, rpm, inflow) {
  const { prop } = model.ac;
  const n = rpm / 60;
  if (n <= 0) return 0;
  const advance = Math.max(0, inflow) / (n * prop.diameter);
  return interpolate(advance, prop.advance, prop.thrust, 0) * model.ac.density * n * n * prop.diameter ** 4;
}

function propPower(model, rpm, inflow) {
  const { prop } = model.ac;
  const n = rpm / 60;
  if (n <= 0) return 0;
  const advance = Math.max(0, inflow) / (n * prop.diameter);
  return interpolate(advance, prop.advance, prop.power) * model.ac.density * n ** 3 * prop.diameter ** 5;
}

// Lift and separated-drag coefficients of a panel at effective angle of attack `alpha` (rad).
// Linear up to cl_max, then falling to a flat plate's.
function coefficients(panel, alpha) {
  const linear = panel.slope * (alpha - radians(panel.zero_lift));
  const plate = PLATE * Math.sin(alpha) * Math.cos(alpha);
  const stalled = clamp((Math.abs(linear) - panel.cl_max) / (panel.slope * STALL_BLEND), 0, 1);
  const attached = clamp(linear, -panel.cl_max, panel.cl_max);
  return { cl: attached * (1 - stalled) + plate * stalled, separated: stalled * PLATE * Math.sin(alpha) ** 2, stalled };
}

// Precomputed geometry. `ground(north, east)` is the terrain's height (m above the pad), flat by
// default. `controlPower` is each control's moment (N m) per radian per pascal of dynamic pressure,
// for the flight controller.
export function createModel(ac, ground = () => 0) {
  const surfaces = ac.surfaces.map((s) => ({ ...s, r: bodyPoint(ac, s.center), n: toBody(s.normal) }));
  const controlPower = { aileron: [0, 0, 0], elevator: [0, 0, 0], rudder: [0, 0, 0] };
  for (const s of surfaces) {
    if (s.control) controlPower[s.control] = add(controlPower[s.control], scale(cross(s.r, s.n), s.area * s.slope * s.tau));
  }
  const n = ac.prop.max_rpm / 60;
  return {
    ac,
    ground,
    surfaces,
    controlPower,
    inertiaInverse: invert(ac.inertia),
    skids: ac.skids.map((p) => bodyPoint(ac, p)),
    crashPoints: ac.crash_points.map((p) => bodyPoint(ac, p)),
    maxThrust: interpolate(0, ac.prop.advance, ac.prop.thrust) * ac.density * n * n * ac.prop.diameter ** 4,
  };
}

// Sitting at [north, east] (the pad by default), motors stopped and tilted up, as low as the skids
// allow; or flying level there toward `heading` (rad from north) at `speed` and `altitude` (m above the pad).
export function newState(model, { north = 0, east = 0, heading = 0, altitude = 0, speed = 0, pitch = 0, tilt = 90 } = {}) {
  const q = fromEuler(0, pitch, heading);
  const resting = () => Math.max(...model.skids.map((r) => rotate(q, r)).map(([n, e, down]) => model.ground(north + n, east + e) + down));
  return {
    p: [north, east, -(altitude || resting())],
    v: [speed * Math.cos(heading), speed * Math.sin(heading), 0],
    q,
    w: [0, 0, 0],
    rpm: [0, 0, 0],
    tilt: model.ac.motors.map((m) => (m.tilting ? tilt : 90)),
    time: 0,
    energy: 0, // Wh from the battery
    crashed: null,
    onGround: !altitude,
    wingCl: 0, // last step's, for the tail's downwash
  };
}

// Advance `state` by dt. `act`: throttle[3] (0..1), tilt[3] (deg, front motors only),
// aileron, elevator, rudder (rad, trailing edge down / left positive). Returns telemetry.
export function step(model, state, act, dt) {
  const { ac } = model;
  if (state.crashed) return telemetry(model, state);
  const vBody = unrotate(state.q, state.v);
  let force = [0, 0, 0];
  let moment = [0, 0, 0];
  const apply = (f, r) => {
    force = add(force, f);
    moment = add(moment, cross(r, f));
  };

  // Motors
  const thrusts = [];
  let watts = ac.electronics_watts;
  ac.motors.forEach((motor, i) => {
    if (motor.tilting) state.tilt[i] += clamp(act.tilt[i] - state.tilt[i], -TILT_RATE * dt, TILT_RATE * dt);
    state.rpm[i] += (clamp(act.throttle[i], 0, 1) * ac.prop.max_rpm - state.rpm[i]) * (1 - Math.exp(-dt / RPM_LAG));
    if (state.rpm[i] < 1) state.rpm[i] = 0; // stopped: spinning down for ever would underflow to 0/0 advance
    const { hub, axis } = motorPose(ac, motor, state.tilt[i]);
    const inflow = dot(add(vBody, cross(state.w, hub)), axis);
    const thrust = propThrust(model, state.rpm[i], inflow);
    const shaft = propPower(model, state.rpm[i], inflow);
    apply(scale(axis, thrust), hub);
    if (shaft > 0) {
      moment = add(moment, scale(axis, (-motor.spin * shaft) / (2 * Math.PI * (state.rpm[i] / 60)))); // reaction torque
      watts += shaft / interpolate(shaft, ac.efficiency.shaft, ac.efficiency.ratio);
    }
    thrusts.push(thrust);
  });

  // Lifting panels; the tail sees the wing's downwash
  const rho = ac.density;
  let wingLift = 0;
  let wingArea = 0;
  let stalled = 0;
  for (const s of model.surfaces) {
    const air = scale(add(vBody, cross(state.w, s.r)), -1); // air velocity over the panel
    const along = dot(air, [-1, 0, 0]);
    const across = dot(air, s.n);
    const speed2 = along * along + across * across;
    if (speed2 < 0.25) continue;
    const speed = Math.sqrt(speed2);
    const downwash = s.downwash ? (2 * state.wingCl) / (Math.PI * ac.surfaces[0].aspect) : 0;
    const deflection = s.control ? clamp(act[s.control], -radians(25), radians(25)) : 0;
    const alpha = Math.atan2(across, along) + radians(s.incidence) + s.tau * deflection - downwash;
    const { cl, separated, stalled: k } = coefficients(s, alpha);
    const cd = (Math.min(Math.abs(cl), s.cl_max) ** 2 / (Math.PI * ac.oswald * s.aspect)) * (1 - k) + separated;
    const flow = scale(add(scale([-1, 0, 0], along), scale(s.n, across)), 1 / speed);
    const liftDirection = scale(sub(scale(s.n, along), scale([-1, 0, 0], across)), 1 / speed);
    const q = 0.5 * rho * speed2;
    apply(scale(add(scale(liftDirection, cl), scale(flow, cd)), q * s.area), s.r);
    moment = add(moment, scale(cross(s.n, [-1, 0, 0]), s.moment * q * s.area * s.chord));
    if (s.name.startsWith("wing")) {
      wingLift += cl * s.area;
      wingArea += s.area;
      stalled = Math.max(stalled, k);
    }
  }
  state.wingCl = wingArea ? wingLift / wingArea : 0;

  // Parasite drag of the whole airframe
  apply(scale(vBody, -0.5 * rho * ac.drag_area * norm(vBody)), [0, 0, 0]);

  // Ground: spring-damper skids pushing straight up, so the aircraft sits still on a slope, with
  // friction across it; a crash point touching is a crash
  let ground = [0, 0, 0];
  state.onGround = false;
  const at = (r) => add(state.p, rotate(state.q, r));
  const depth = (p) => p[2] + model.ground(p[0], p[1]);
  for (const r of model.crashPoints) {
    if (depth(at(r)) > 0) state.crashed = "hit the ground";
  }
  for (const r of model.skids) {
    const p = at(r);
    const below = depth(p);
    if (below <= 0) continue;
    const velocity = add(state.v, rotate(state.q, cross(state.w, r)));
    const closing = dot(velocity, intoGround(model, p));
    if (closing > CRASH_SPEED) state.crashed = `landed at ${closing.toFixed(1)} m/s`;
    const normal = Math.max(0, STIFFNESS * below + DAMPING * velocity[2]);
    const slide = Math.hypot(velocity[0], velocity[1]) + 0.2;
    const f = [(-FRICTION * normal * velocity[0]) / slide, (-FRICTION * normal * velocity[1]) / slide, -normal];
    ground = add(ground, f);
    moment = add(moment, cross(r, unrotate(state.q, f)));
    state.onGround = true;
  }

  // Integrate (semi-implicit Euler)
  const acceleration = add(scale(add(rotate(state.q, force), ground), 1 / ac.mass), [0, 0, ac.gravity]);
  state.v = add(state.v, scale(acceleration, dt));
  state.p = add(state.p, scale(state.v, dt));
  const spin = sub(moment, cross(state.w, matrixTimes(ac.inertia, state.w)));
  state.w = add(state.w, scale(matrixTimes(model.inertiaInverse, spin), dt));
  const turn = multiply(state.q, [0, ...state.w]);
  const q = state.q.map((c, i) => c + 0.5 * turn[i] * dt);
  const size = Math.hypot(...q);
  state.q = q.map((c) => c / size);
  state.time += dt;
  state.energy += (watts * dt) / 3600;
  return telemetry(model, state, thrusts, watts, stalled);
}

// Unit vector into the terrain (north, east, down) under a point, from the slope there
function intoGround(model, [north, east]) {
  const d = 0.5;
  const dn = (model.ground(north + d, east) - model.ground(north - d, east)) / (2 * d);
  const de = (model.ground(north, east + d) - model.ground(north, east - d)) / (2 * d);
  return scale([dn, de, 1], 1 / Math.hypot(dn, de, 1));
}

// What the instruments read; without the step's thrusts and power, before the first step.
// Altitude is above the pad (like ArduPilot's, above home); height is above the ground below.
export function telemetry(model, state, thrusts = [0, 0, 0], watts = 0, stalled = 0) {
  return {
    airspeed: norm(state.v),
    altitude: -state.p[2],
    height: -state.p[2] - model.ground(state.p[0], state.p[1]),
    climb: -state.v[2],
    thrusts,
    watts,
    stalled,
  };
}

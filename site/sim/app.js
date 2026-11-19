// The simulator page: the VTOL flies its mission by itself (control.js's missionStart), stepping
// physics.js and control.js at 400 Hz, drawn with three.js: a chase view on the left, and on the
// right its camera's view, its readings and the map. Data from ../data/ (written by drone/sim.py);
// the aircraft's parts come through ../aircraft.js, shared with the model page.
import * as THREE from "three";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";
import { loadAircraft } from "../aircraft.js";
import { missionInput, missionStart, update } from "./control.js";
import { DT, clamp, createModel, degrees, euler, motorPose, radians, rotate, step, telemetry, wrap } from "./physics.js";
import { cameras, cellAt, createCoverage, nearTrail, survey, turret } from "./sensors.js";
import { CELLS, SIZE, SPACING, createBeacon, createWorld, distance, samplePoint, seeded, smoothstep } from "./world.js";

const DATA = "../data/";
const CHASE = [4.5, 1.5]; // chase camera distance behind and height above the aircraft (m)
const SEED = 1; // the world without ?seed= in the address

const { ac, meshes } = await loadAircraft(DATA);
const asked = Number.parseInt(new URLSearchParams(location.search).get("seed"), 10);
const land = createWorld(Number.isFinite(asked) ? asked : SEED);
const model = createModel(ac, land.ground);
const ground = (p) => model.ground(p[0], p[1]);

// north-east-down (m) to three.js (x east, y up, z south)
const world = (p) => new THREE.Vector3(p[1], -p[2], -p[0]);

// A logarithmic depth buffer keeps the distant ridges from flickering against each other. One canvas
// fills the window, in the panel's color; the two views are drawn into the boxes of #view and #camera.
const renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
renderer.domElement.id = "scene";
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene();
// The sky: blue overhead, fading to haze at the horizon, which is the fog's color, so the far
// mountains fade into it
function skyTexture() {
  const canvas = Object.assign(document.createElement("canvas"), { width: 2, height: 256 });
  const pen = canvas.getContext("2d");
  const fade = pen.createLinearGradient(0, 0, 0, 256);
  [[0, "#3c74bd"], [0.4, "#9dbfe0"], [0.5, "#cad8e4"], [1, "#cad8e4"]].forEach(([at, color]) => fade.addColorStop(at, color));
  pen.fillStyle = fade;
  pen.fillRect(0, 0, 2, 256);
  return Object.assign(new THREE.CanvasTexture(canvas), { mapping: THREE.EquirectangularReflectionMapping, colorSpace: THREE.SRGBColorSpace });
}
scene.background = skyTexture();
scene.fog = new THREE.Fog(0xcad8e4, 400, 4000); // haze: about a quarter on the mountains round the world's edge
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.05, 8000);
scene.add(new THREE.HemisphereLight(0xe3f0ff, 0x4d5b3a, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.normalBias = 0.02;
Object.assign(sun.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 1, far: 300 });
scene.add(sun, sun.target);
addEventListener("resize", () => renderer.setSize(innerWidth, innerHeight));

// Ground grain: tiling noise around a mean of 0.5, in blobs from a sixteenth of the tile down to single
// texels. Mipmapped, so it blends to its mean (no change) wherever it is finer than a screen pixel.
function grainTexture() {
  const size = 256;
  const random = seeded(11);
  const smooth = (t) => t * t * (3 - 2 * t);
  const value = new Float32Array(size * size);
  for (let cells = 16, weight = 1; cells <= size; cells *= 2, weight *= 0.85) {
    const lattice = Array.from({ length: cells * cells }, random);
    const at = (x, y) => lattice[(y % cells) * cells + (x % cells)]; // wraps, so the tile repeats seamlessly
    for (let y = 0; y < size; y++) {
      for (let x = 0; x < size; x++) {
        const [fx, fy] = [(x * cells) / size, (y * cells) / size];
        const [x0, y0, sx, sy] = [Math.floor(fx), Math.floor(fy), smooth(fx % 1), smooth(fy % 1)];
        const top = at(x0, y0) + (at(x0 + 1, y0) - at(x0, y0)) * sx;
        const bottom = at(x0, y0 + 1) + (at(x0 + 1, y0 + 1) - at(x0, y0 + 1)) * sx;
        value[y * size + x] += weight * (top + (bottom - top) * sy);
      }
    }
  }
  const mean = value.reduce((a, b) => a + b) / value.length;
  const spread = Math.sqrt(value.reduce((a, v) => a + (v - mean) ** 2, 0) / value.length);
  const data = Uint8Array.from(value, (v) => clamp(128 + (40 * (v - mean)) / spread, 0, 255));
  const texture = new THREE.DataTexture(data, size, size, THREE.RedFormat);
  Object.assign(texture, {
    wrapS: THREE.RepeatWrapping,
    wrapT: THREE.RepeatWrapping,
    magFilter: THREE.LinearFilter,
    minFilter: THREE.LinearMipmapLinearFilter,
    generateMipmaps: true,
    anisotropy: renderer.capabilities.getMaxAnisotropy(),
    needsUpdate: true,
  });
  return texture;
}
const grain = grainTexture();

// Multiplies the grain into the color at four tile sizes, 0.8, 3.5, 15 and 70 m, each turned (cosine,
// sine) so their repeats don't line up: texture from eye level up to circling height, in world position.
function addGrain(shader) {
  shader.uniforms.grain = { value: grain };
  shader.vertexShader = shader.vertexShader
    .replace("#include <common>", "#include <common>\nvarying vec2 vGround;")
    .replace("#include <begin_vertex>", "#include <begin_vertex>\nvGround = (modelMatrix * vec4(transformed, 1.0)).xz;");
  shader.fragmentShader = shader.fragmentShader
    .replace(
      "#include <common>",
      `#include <common>
      uniform sampler2D grain;
      varying vec2 vGround;
      float grainAt(float tile, float c, float s) { return texture2D(grain, mat2(c, s, -s, c) * vGround / tile).r; }`,
    )
    .replace(
      "#include <map_fragment>",
      `#include <map_fragment>
      float grit = grainAt(0.8, 0.88, 0.48) + grainAt(3.5, 0.36, 0.93) + grainAt(15.0, -0.42, 0.91) + grainAt(70.0, -0.9, 0.43);
      diffuseColor.rgb *= 1.0 + 0.5 * (grit - 2.0);`,
    );
}

// Ground colors (sRGB), mixed by rules below
const PAINT = Object.fromEntries(
  Object.entries({ lush: "#62803a", dry: "#9d9a5b", forest: "#4b5a2f", tundra: "#86805e", worn: "#8e7d5c", shore: "#7d6f52", bed: "#4f5745", rock: "#7d776e", snow: "#eef1f3" }).map(([name, hex]) => [name, new THREE.Color(hex)]),
);
// The color of the ground at `p` (height h, `up` the up part of its normal): meadow in lush and dry
// patches, darker under forest, tundra above the treeline, worn along the trail, muddy at the shore
// and dark under water, rock where steep, snow on the high tops
function paint(color, p, h, up) {
  const steep = 1 - up;
  const wet = distance(p, land.lake.center) < land.lake.radius;
  color.copy(PAINT.lush).lerp(PAINT.dry, land.meadow(p)).lerp(PAINT.forest, 0.8 * land.forest(p));
  color.lerp(PAINT.tundra, smoothstep(100, 150, h));
  color.lerp(PAINT.worn, 0.7 * (1 - smoothstep(2, 7, land.trailDistance(...p))));
  if (wet) color.lerp(PAINT.shore, 1 - smoothstep(land.water + 0.3, land.water + 1.5, h)).lerp(PAINT.bed, 1 - smoothstep(land.water - 1.5, land.water, h));
  color.lerp(PAINT.rock, smoothstep(0.2, 0.4, steep));
  return color.lerp(PAINT.snow, smoothstep(170, 220, h) * (1 - smoothstep(0.3, 0.5, steep)));
}

// The ground, grained: a vertex on every height sample and each cell split into the same two
// triangles as `land.ground()`, so what the aircraft lands on is what is drawn. Returns its geometry.
function groundMesh() {
  const count = (CELLS + 1) ** 2;
  const position = new Float32Array(3 * count);
  for (let i = 0; i < count; i++) {
    const [north, east] = samplePoint(i);
    position.set([east, land.heights[i], -north], 3 * i);
  }
  const index = new Uint32Array(6 * CELLS * CELLS);
  for (let r = 0, n = 0; r < CELLS; r++) {
    for (let c = 0; c < CELLS; c++, n += 6) {
      const [nw, sw] = [r * (CELLS + 1) + c, (r + 1) * (CELLS + 1) + c];
      index.set([nw, sw, nw + 1, nw + 1, sw, sw + 1], n);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(position, 3));
  geometry.setIndex(new THREE.BufferAttribute(index, 1));
  geometry.computeVertexNormals();
  const colors = new Float32Array(3 * count);
  const color = new THREE.Color();
  for (let i = 0; i < count; i++) {
    const [east, h, south] = position.subarray(3 * i, 3 * i + 3);
    colors.set(paint(color, [-south, east], h, geometry.attributes.normal.getY(i)).toArray(), 3 * i);
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  const material = new THREE.MeshLambertMaterial({ vertexColors: true });
  material.onBeforeCompile = addGrain;
  const mesh = new THREE.Mesh(geometry, material);
  mesh.receiveShadow = true;
  scene.add(mesh);
  return geometry;
}

// The lake: a disc at the water's level; the hills round its edge stand above it
function lake() {
  const material = new THREE.MeshPhongMaterial({ color: 0x2f5a70, specular: 0x9ab8c8, shininess: 80, transparent: true, opacity: 0.9 });
  const mesh = new THREE.Mesh(new THREE.CircleGeometry(land.lake.radius, 96).rotateX(-Math.PI / 2), material);
  mesh.position.copy(world([...land.lake.center, -land.water]));
  scene.add(mesh);
}

// The trail: a strip of dirt 1.4 m wide, a pair of points every metre laid 6 cm above the ground
// (lifted, not pushed forward in depth, which the logarithmic depth buffer would ignore)
function trail() {
  const half = land.trail.points.flatMap((p, i, all) => (i ? [[(all[i - 1][0] + p[0]) / 2, (all[i - 1][1] + p[1]) / 2], p] : [p]));
  const points = half.filter((p) => Math.hypot(...p) > 2.5); // from the pad's edge
  const position = new Float32Array(6 * points.length);
  points.forEach((p, i) => {
    const [a, b] = [points[Math.max(i - 1, 0)], points[Math.min(i + 1, points.length - 1)]];
    const along = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const side = [(-0.7 * (b[1] - a[1])) / along, (0.7 * (b[0] - a[0])) / along];
    for (const [k, edge] of [[0, 1], [1, -1]]) {
      const q = [p[0] + edge * side[0], p[1] + edge * side[1]];
      position.set(world([...q, -ground(q) - 0.06]).toArray(), 6 * i + 3 * k);
    }
  });
  const index = points.slice(1).flatMap((_, i) => [2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 2, 2 * i + 1, 2 * i + 3]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(position, 3));
  geometry.setIndex(index);
  geometry.computeVertexNormals();
  const material = new THREE.MeshLambertMaterial({ color: 0x9a8662, side: THREE.DoubleSide });
  material.onBeforeCompile = addGrain;
  const mesh = new THREE.Mesh(geometry, material);
  mesh.receiveShadow = true;
  scene.add(mesh);
}

// The forest: each tree a cone of needles on a trunk, sunk 30 cm so slopes don't show under it.
// Drawing only, like the rocks.
function trees() {
  const count = land.trees.length / 3;
  const crowns = new THREE.InstancedMesh(new THREE.ConeGeometry(1, 1, 7).translate(0, 0.5, 0), new THREE.MeshLambertMaterial({ flatShading: true }), count);
  const trunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(1, 1, 1, 5).translate(0, 0.5, 0), new THREE.MeshLambertMaterial({ color: 0x4a3a2b }), count);
  const random = seeded(3);
  const [matrix, turn, color] = [new THREE.Matrix4(), new THREE.Quaternion(), new THREE.Color()];
  for (let i = 0; i < count; i++) {
    const [north, east, height] = land.trees.subarray(3 * i, 3 * i + 3);
    const base = world([north, east, -land.terrain(north, east) + 0.3]);
    turn.setFromAxisAngle(new THREE.Vector3(0, 1, 0), 2 * Math.PI * random());
    crowns.setMatrixAt(i, matrix.compose(base.clone().setY(base.y + 0.2 * height), turn, new THREE.Vector3(0.22 * height, 0.85 * height, 0.22 * height)));
    trunks.setMatrixAt(i, matrix.compose(base, turn, new THREE.Vector3(0.025 * height, 0.3 * height, 0.025 * height)));
    crowns.setColorAt(i, color.setHSL(0.26 + 0.07 * random(), 0.3 + 0.2 * random(), 0.12 + 0.08 * random(), THREE.SRGBColorSpace));
  }
  scene.add(crowns, trunks);
}

// Rocks scattered within 40 m of the pad, clear of it and of the trail: something sharp to see up close.
// Drawing only; the physics doesn't know about them.
function rocks(count) {
  const random = seeded(5);
  let shape = new THREE.IcosahedronGeometry(1, 1);
  shape.deleteAttribute("normal");
  shape.deleteAttribute("uv");
  shape = mergeVertices(shape); // shared corners, so the lumps below don't open cracks
  const corners = shape.attributes.position;
  for (let i = 0; i < corners.count; i++) {
    const lump = 0.75 + 0.4 * random();
    corners.setXYZ(i, corners.getX(i) * lump, corners.getY(i) * lump, corners.getZ(i) * lump);
  }
  const mesh = new THREE.InstancedMesh(shape, new THREE.MeshLambertMaterial({ flatShading: true }), count);
  const matrix = new THREE.Matrix4();
  for (let i = 0; i < count; ) {
    const [r, a] = [40 * Math.sqrt(random()), 2 * Math.PI * random()];
    const spot = [r * Math.cos(a), r * Math.sin(a)];
    if (r < 4 || land.trailDistance(...spot) < 1.5) continue;
    const size = 0.05 + 0.45 * random() ** 3; // mostly pebbles, a few half-metre rocks
    const scale = new THREE.Vector3(size * (0.8 + 0.4 * random()), size * (0.45 + 0.3 * random()), size * (0.8 + 0.4 * random()));
    const turn = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.3 * (random() - 0.5), 2 * Math.PI * random(), 0.3 * (random() - 0.5)));
    mesh.setMatrixAt(i, matrix.compose(world([...spot, -ground(spot) + 0.35 * scale.y]), turn, scale)); // two-thirds buried
    mesh.setColorAt(i, new THREE.Color().setHSL(0.06 + 0.05 * random(), 0.08 + 0.12 * random(), 0.25 + 0.2 * random(), THREE.SRGBColorSpace));
    i++;
  }
  mesh.receiveShadow = true;
  scene.add(mesh);
}

// An animal from boxes and cylinders, facing +z with its feet at 0, about its WILDLIFE size: elk
// with antlers, deer at two thirds of an elk, a bulky bear; or a boulder for a sun-warmed rock
function beast(kind) {
  const group = new THREE.Group();
  const part = (geometry, color, [x, y, z], [tilt, splay] = [0, 0]) => {
    const mesh = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({ color, flatShading: kind === "rock" }));
    mesh.position.set(x, y, z);
    mesh.rotation.set(tilt, 0, splay);
    group.add(mesh);
  };
  const box = (w, h, l) => new THREE.BoxGeometry(w, h, l);
  const leg = (r, h) => new THREE.CylinderGeometry(r, 0.8 * r, h, 6);
  if (kind === "rock") {
    part(new THREE.IcosahedronGeometry(1, 0).scale(0.7, 0.45, 0.6), 0x7d756a, [0, 0.2, 0]);
  } else if (kind === "bear") {
    for (const [x, z] of [[-0.25, 0.45], [0.25, 0.45], [-0.25, -0.45], [0.25, -0.45]]) part(leg(0.11, 0.5), 0x2e2620, [x, 0.25, z]);
    part(box(0.75, 0.7, 1.3), 0x2e2620, [0, 0.82, 0]);
    part(box(0.42, 0.38, 0.45), 0x2e2620, [0, 0.95, 0.8]);
    part(box(0.2, 0.16, 0.18), 0x5a4a3a, [0, 0.88, 1.08]); // snout
  } else {
    const s = kind === "elk" ? 1 : 0.66;
    const [coat, dark] = kind === "elk" ? [0x8b6643, 0x4f3a28] : [0xa57b50, 0x6b4e33];
    for (const [x, z] of [[-0.22, 0.7], [0.22, 0.7], [-0.22, -0.7], [0.22, -0.7]]) part(leg(0.06 * s, 0.85 * s), dark, [x * s, 0.425 * s, z * s]);
    part(box(0.55 * s, 0.65 * s, 1.75 * s), coat, [0, 1.15 * s, 0]);
    part(box(0.28 * s, 0.7 * s, 0.32 * s), dark, [0, 1.5 * s, 0.85 * s], [-0.5, 0]); // neck
    part(box(0.24 * s, 0.28 * s, 0.55 * s), coat, [0, 1.85 * s, 1.1 * s]); // head
    if (kind === "elk") for (const side of [-1, 1]) part(leg(0.025, 0.9), 0xd8c8a0, [0.15 * side, 2.3, 0.95], [-0.35, -0.5 * side]); // antlers
  }
  return group;
}
const beasts = land.wildlifeAt(0).map((animal) => {
  const group = beast(animal.kind);
  scene.add(group);
  return group;
});

const groundGeometry = groundMesh();
lake();
trail();
trees();
rocks(1500);
// The pad, laid on the ground: rings every 25 cm, so it follows the ground's folds instead of cutting through them
const pad = new THREE.RingGeometry(0, 2.5, 48, 10).rotateX(-Math.PI / 2);
const corners = pad.attributes.position;
for (let i = 0; i < corners.count; i++) corners.setY(i, model.ground(-corners.getZ(i), corners.getX(i)) + 0.03);
const padMesh = new THREE.Mesh(pad, new THREE.MeshLambertMaterial({ color: 0x3b3b3e }));
padMesh.receiveShadow = true;
scene.add(padMesh);

// You: a hiker who sets off up the trail when the aircraft takes off. Circling follows your phone's fixes.
const hiker = new THREE.Group();
const hips = [-1, 1].map((side) => {
  const hip = new THREE.Group();
  hip.position.set(0.09 * side, 0.85, 0);
  const leg = new THREE.Mesh(new THREE.CapsuleGeometry(0.075, 0.7, 4, 12), new THREE.MeshLambertMaterial({ color: 0x3a4250 }));
  leg.position.y = -0.425;
  leg.castShadow = true;
  hip.add(leg);
  hiker.add(hip);
  return hip;
});
for (const [geometry, color, y, z] of [
  [new THREE.CapsuleGeometry(0.2, 0.4, 4, 12), 0xd2452e, 1.15, 0], // jacket
  [new THREE.SphereGeometry(0.12, 16, 12), 0xe0b48c, 1.65, 0], // head
  [new THREE.BoxGeometry(0.34, 0.5, 0.2), 0x2f5d3a, 1.2, -0.22], // backpack
]) {
  const mesh = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({ color }));
  mesh.position.set(0, y, z);
  mesh.castShadow = true;
  hiker.add(mesh);
}
scene.add(hiker);
// Stand him where he is, facing up the trail, legs swinging a stride (1.5 m) per cycle while he walks
function placeHiker() {
  hiker.position.copy(world([...you.position, -ground(you.position)]));
  hiker.rotation.y = Math.PI - you.heading; // the model faces +z, the scene's south
  const swing = you.speed > 0 ? 0.45 * Math.sin((2 * Math.PI * you.walked) / 1.5) : 0;
  hips.forEach((hip, i) => (hip.rotation.x = i ? swing : -swing));
}

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

for (const mesh of meshes) {
  mesh.castShadow = true;
  const [, name, leaf] = mesh.userData.path.split("/");
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
}

// A message over the chase view for a few seconds
let notice = { text: "", until: 0 };
const tell = (text, seconds) => (notice = { text, until: performance.now() + 1000 * seconds });

// The cameras (sensors.js): a look each second of sim time once airborne, the turret panned toward
// the trail LOOK_AHEAD m past you while circling, else ahead of the aircraft. The color camera's
// view is drawn too, from the aircraft, with the airframe hidden.
const cams = cameras(ac);
const droneCamera = new THREE.PerspectiveCamera(degrees(cams.color.v), 16 / 9, 0.05, 8000);
function aimDroneCamera() {
  const view = turret(land, state, controller.orbit, you.walked);
  droneCamera.position.copy(world(state.p));
  droneCamera.up.copy(world(view.down)).negate();
  droneCamera.lookAt(world(state.p.map((x, k) => x + view.forward[k])));
}
const [sightings, coverage, rolls, warned] = [new Map(), createCoverage(), seeded(17), new Set()]; // pins, ground covered, the dice, bears warned of
let looked = -1; // the last look (s)
function lookAround() {
  if (controller.mode === "landed" || Math.floor(state.time) <= looked) return;
  looked = Math.floor(state.time);
  // A bear named near the trail ahead of you: say so once; the map keeps it
  const { bears } = survey(cams, land, state, controller.orbit, you.walked, { sightings, coverage, rolls });
  for (const [key, off] of bears) {
    if (!warned.has(key)) tell(`Bear spotted ${Math.round(off)} m from the trail ahead`, 6);
    warned.add(key);
  }
}

// The map: the trail's country from above, darker where the cameras haven't covered, with what they
// have found
const mapPen = document.querySelector("#map canvas").getContext("2d");
const mapCaption = document.getElementById("found");
const MAP_PIXELS = 600; // the canvas's own: sharp at the panel's width on a 2x screen
const PX = MAP_PIXELS / 300; // line widths and marks below are sized for 300 pixels
const PINS = { warm: ["?", "#ff9a3d"], elk: ["E", "#c98a4b"], deer: ["D", "#e2bd7a"], bear: ["B", "#e5432f"], rock: ["×", "#a3a3a3"], lake: ["L", "#59a8e0"] };
// The area shown: the trail and 250 m round it, square
const [mapCenter, mapHalf] = (() => {
  const [north, east] = [0, 1].map((k) => land.trail.points.map((p) => p[k]));
  const [n0, n1, e0, e1] = [Math.min(...north), Math.max(...north), Math.min(...east), Math.max(...east)];
  return [[(n0 + n1) / 2, (e0 + e1) / 2], Math.max(n1 - n0, e1 - e0) / 2 + 250];
})();
const toMap = ([north, east]) => [((east - mapCenter[1] + mapHalf) / (2 * mapHalf)) * MAP_PIXELS, ((mapCenter[0] - north + mapHalf) / (2 * mapHalf)) * MAP_PIXELS];
const fromMap = (x, y) => [mapCenter[0] + mapHalf - ((y + 0.5) / MAP_PIXELS) * 2 * mapHalf, mapCenter[1] - mapHalf + ((x + 0.5) / MAP_PIXELS) * 2 * mapHalf];
// The ground's colors under a light from the north-west, and the lake blue
const mapBase = (() => {
  const image = new ImageData(MAP_PIXELS, MAP_PIXELS);
  const [colors, normals] = [groundGeometry.attributes.color, groundGeometry.attributes.normal];
  const color = new THREE.Color();
  for (let y = 0; y < MAP_PIXELS; y++) {
    for (let x = 0; x < MAP_PIXELS; x++) {
      const [north, east] = fromMap(x, y);
      const i = clamp(Math.round((SIZE - north) / SPACING), 0, CELLS) * (CELLS + 1) + clamp(Math.round((east + SIZE) / SPACING), 0, CELLS);
      const light = 0.6 + 0.5 * (-0.5 * normals.getX(i) + 0.7 * normals.getY(i) - 0.5 * normals.getZ(i));
      color.fromBufferAttribute(colors, i);
      if (land.ground(north, east) > land.terrain(north, east)) color.set(0x3f6f8a);
      color.multiplyScalar(light).convertLinearToSRGB();
      image.data.set([255 * color.r, 255 * color.g, 255 * color.b, 255], 4 * (y * MAP_PIXELS + x));
    }
  }
  return createImageBitmap(image);
})();
const mapCells = Int32Array.from({ length: MAP_PIXELS ** 2 }, (_, i) => cellAt(fromMap(i % MAP_PIXELS, Math.floor(i / MAP_PIXELS))));
const mapFog = new ImageData(MAP_PIXELS, MAP_PIXELS);
const nearCells = nearTrail(land.trail.points); // the share of these covered is the caption's "seen near the trail"
const base = await mapBase;
const fogPen = Object.assign(document.createElement("canvas"), { width: MAP_PIXELS, height: MAP_PIXELS }).getContext("2d");
function drawMap() {
  mapCells.forEach((cell, i) => (mapFog.data[4 * i + 3] = coverage.cells[cell] ? 0 : 150));
  fogPen.putImageData(mapFog, 0, 0);
  mapPen.drawImage(base, 0, 0);
  mapPen.drawImage(fogPen.canvas, 0, 0);
  mapPen.strokeStyle = "rgba(240, 220, 180, 0.85)";
  mapPen.lineWidth = 1.5 * PX;
  mapPen.beginPath();
  land.trail.points.forEach((p, i) => mapPen[i ? "lineTo" : "moveTo"](...toMap(p)));
  mapPen.stroke();
  const ring = (center, radius, fill, edge) => {
    mapPen.beginPath();
    mapPen.arc(...center, radius, 0, 2 * Math.PI);
    if (fill) {
      mapPen.fillStyle = fill;
      mapPen.fill();
    }
    mapPen.strokeStyle = edge;
    mapPen.stroke();
  };
  mapPen.lineWidth = PX;
  if (controller.orbit) ring(toMap(controller.orbit), (ac.orbit_radius * MAP_PIXELS) / (2 * mapHalf), null, "rgba(255, 154, 82, 0.8)");
  ring(toMap(you.position), 3.5 * PX, "#ffffff", "#1a1a1a");
  // The aircraft: a triangle along its heading
  mapPen.save();
  mapPen.translate(...toMap(state.p));
  mapPen.rotate(euler(state.q)[2]);
  mapPen.scale(PX, PX);
  mapPen.fillStyle = "#ff9a52";
  mapPen.beginPath();
  [[0, -7], [5, 5], [-5, 5]].forEach(([x, y], i) => mapPen[i ? "lineTo" : "moveTo"](x, y));
  mapPen.fill();
  mapPen.restore();
  // The pins, lettered
  Object.assign(mapPen, { font: `bold ${9 * PX}px system-ui, sans-serif`, textAlign: "center", textBaseline: "middle" });
  for (const pin of sightings.values()) {
    const [letter, color] = PINS[pin.label];
    const at = toMap(pin.position);
    ring(at, 6.5 * PX, color, "#111");
    mapPen.fillStyle = "#111";
    mapPen.fillText(letter, ...at);
  }
  const count = (label) => [...sightings.values()].filter((pin) => pin.label === label).length;
  const found = ["elk", "deer", "bear"].filter(count).map((kind) => `${count(kind)} ${kind}${kind === "bear" && count(kind) > 1 ? "s" : ""}`);
  if (count("warm")) found.push(`${count("warm")} unknown`);
  const notes = [found.length ? `Found ${found.join(", ")}` : "Nothing found yet"];
  if (count("rock")) notes.push(`${count("rock")} warm rock${count("rock") > 1 ? "s" : ""} ruled out`);
  if (sightings.has("lake")) notes.push("lake mapped");
  notes.push(`${Math.round((100 * nearCells.filter((cell) => coverage.cells[cell]).length) / nearCells.length)}% seen near the trail`);
  mapCaption.textContent = notes.join(" · ");
}

// The flight: already circling you as you set off from the trailhead
const beacon = createBeacon(land.seed); // your phone
let you = land.hikerAt(0);
let fix = beacon(you.position, 0); // its last fix
const [state, controller] = missionStart(model, fix);
let last = telemetry(model, state);
let cameraYaw = euler(state.q)[2]; // the chase camera swings round behind the heading, smoothly

// Draws `eye`'s view into the part of the window under `box`
const viewBox = document.getElementById("view");
const cameraBox = document.getElementById("camera");
function drawInto(box, eye) {
  const { left, bottom, width, height } = box.getBoundingClientRect();
  renderer.setViewport(left, innerHeight - bottom, width, height);
  renderer.setScissor(left, innerHeight - bottom, width, height);
  eye.aspect = width / height;
  eye.updateProjectionMatrix();
  renderer.render(scene, eye);
}

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
  cameraYaw += wrap(yaw - cameraYaw) * (1 - Math.exp(-3 * dt));
  placeHiker();
  land.wildlifeAt(state.time).forEach((animal, i) => {
    beasts[i].position.copy(world([...animal.position, -land.terrain(...animal.position)]));
    beasts[i].rotation.y = Math.PI - animal.heading; // the models face +z, the scene's south
  });
  const behind = [state.p[0] - CHASE[0] * Math.cos(cameraYaw), state.p[1] - CHASE[0] * Math.sin(cameraYaw)];
  camera.position.copy(world([...behind, Math.min(state.p[2] - CHASE[1], -ground(behind) - 0.5)]));
  camera.lookAt(target);
  sun.position.copy(target).add(new THREE.Vector3(40, 90, 25));
  sun.target.position.copy(target);
  renderer.setScissorTest(false);
  renderer.setClearColor(0x16191d); // the panel's background, set each time: the shadow pass leaves white
  renderer.clear();
  renderer.setScissorTest(true);
  drawInto(viewBox, camera);
  aimDroneCamera();
  aircraft.visible = false;
  drawInto(cameraBox, droneCamera);
  aircraft.visible = true;
}

const modeLine = document.getElementById("mode");
const readings = document.getElementById("readings");
const message = document.getElementById("message");
const MODES = { landed: "LANDED", hover: "HOVER", transition: "TRANSITION", plane: "PLANE", back: "BACK TRANSITION", crashed: "CRASHED" };
function show(text, warning = false) {
  message.hidden = !text;
  message.textContent = text;
  message.className = warning ? "warning" : "";
}
function updatePanel() {
  const left = 1 - state.energy / ac.battery_wh;
  const away = Math.hypot(state.p[0] - you.position[0], state.p[1] - you.position[1]);
  const rows = [
    ["Airspeed", `${last.airspeed.toFixed(1)} m/s`],
    ["Height", `${Math.round(last.height)} m above the ground`],
    ["From you", `${Math.round(away)} m` + (controller.orbit ? `, circling at ${ac.orbit_radius} m` : "")],
    ["Power", `${Math.round(last.watts)} W`],
    ["Battery", `${Math.round(100 * Math.max(0, left))}% of ${ac.battery_wh.toFixed(0)} Wh usable`],
    ["Air", ac.air],
    ["You", you.done ? "at the lake" : `walking at ${you.speed.toFixed(1)} m/s, ${Math.round(you.walked)} of ${land.trail.length} m`],
  ];
  modeLine.textContent = `${MODES[controller.mode]}${controller.orbit ? " · CIRCLING YOU" : ""}`;
  readings.innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
  if (state.crashed) show(`Crashed: ${state.crashed}`, true);
  else if (performance.now() < notice.until) show(notice.text);
  else if (last.stalled > 0.5 && ["plane", "transition"].includes(controller.mode)) show("STALL", true);
  else if (left < 0) show("Battery at its 20% reserve", true);
  else show("");
}

let previous = performance.now();
let accumulator = 0;
function frame(now) {
  const dt = Math.min(0.05, (now - previous) / 1000);
  previous = now;
  accumulator += dt;
  while (accumulator >= DT) {
    you = land.hikerAt(state.time);
    fix = beacon(you.position, state.time);
    last = step(model, state, update(controller, model, state, last, missionInput(fix), DT), DT);
    lookAround();
    accumulator -= DT;
  }
  draw(dt);
  updatePanel();
  if (now - mapDrawn > 250) {
    mapDrawn = now;
    drawMap();
  }
  requestAnimationFrame(frame);
}
let mapDrawn = 0; // when the map was last drawn (ms); four times a second is enough
show("");
requestAnimationFrame(frame);

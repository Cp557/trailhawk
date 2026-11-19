// The model page: the VTOL's CAD parts with their edges, to turn, zoom, see inside and pull apart.
// aircraft.js loads them from data/ (written by trailhawk/sim.py) in CAD axes (+X nose to tail,
// +Y right wing, +Z up, mm); the model group turns +Z to the scene's up.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { CREASE, loadAircraft } from "./aircraft.js";

const VIEW = [-1, 1.1, 0.75]; // first camera direction, CAD axes: ahead of the nose, right and above
const EXPLODE_MS = 700; // to explode or put back
const XRAY = 0.15; // opacity of the painted shell in X-ray
const EXPLODE = 0.5; // fully exploded, each part moves this fraction of its distance from the middle
const HOVER = new THREE.Color(0x2f8cff); // tint of the part under the pointer

const message = document.getElementById("message");
let loaded;
try {
  loaded = await loadAircraft("data/");
} catch (error) {
  message.textContent = "The model didn't load. Serve this folder over http (python -m trailhawk.sim).";
  throw error;
}
const { ac, meshes } = loaded;

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
const sun = new THREE.DirectionalLight(0xffffff, 1.2);
sun.position.set(-1.5, 2, -1); // above the nose and right wing, like the first view
scene.add(sun);

// Each part with a line along every fold sharper than CREASE, so the shading and the lines agree
const model = new THREE.Group();
model.rotation.x = -Math.PI / 2;
scene.add(model);
const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x8a939d }); // light gray: shows on the black airframe and the orange
const box = new THREE.Box3();
for (const mesh of meshes) {
  Object.assign(mesh.material, { polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }); // lines draw over the faces
  mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, CREASE), edgeMaterial));
  mesh.geometry.computeBoundingBox();
  box.union(mesh.geometry.boundingBox);
  model.add(mesh);
}
const middle = box.getCenter(new THREE.Vector3());
for (const mesh of meshes) mesh.userData.out = mesh.geometry.boundingBox.getCenter(new THREE.Vector3()).sub(middle);
model.updateMatrixWorld();
const target = model.localToWorld(middle.clone());
const radius = box.getSize(new THREE.Vector3()).length() / 2;

document.getElementById("specs").innerHTML = `
  <dt>Wingspan</dt><dd>${((box.max.y - box.min.y) / 1000).toFixed(1)} m</dd>
  <dt>Weight</dt><dd>${ac.mass.toFixed(1)} kg</dd>
  <dt>Flight time</dt><dd>${Math.round(ac.loiter_minutes)} min</dd>`;

// Camera: orbit about the middle, zooming toward the pointer; slow turntable until the first touch
const camera = new THREE.PerspectiveCamera(30, innerWidth / innerHeight, 10, 1e5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.copy(target);
controls.enableDamping = true;
controls.zoomToCursor = true;
controls.autoRotate = true;
controls.autoRotateSpeed = 0.6;
controls.addEventListener("start", () => (controls.autoRotate = false));
// Far enough back that the whole aircraft fits the narrower side of the window
const half = THREE.MathUtils.degToRad(camera.fov / 2);
const narrowest = Math.min(half, Math.atan(Math.tan(half) * camera.aspect));
const direction = new THREE.Vector3(...VIEW).applyEuler(model.rotation).normalize();
camera.position.copy(target).addScaledVector(direction, (0.7 * radius) / Math.sin(narrowest));

// The part under the pointer: tinted, with its name (taps work too). In X-ray, through the shell.
const label = document.getElementById("part");
const ray = new THREE.Raycaster();
let hovered = null;
function hover(mesh, event) {
  if (mesh !== hovered) {
    if (hovered) hovered.material.color.copy(hovered.userData.color);
    hovered = mesh;
    if (mesh) {
      mesh.userData.color = mesh.material.color.clone();
      mesh.material.color.lerp(HOVER, 0.45);
      label.textContent = mesh.userData.path.split("/").filter(Boolean).reverse().join(" · ").replaceAll("_", " ");
    }
  }
  label.hidden = !mesh;
  if (mesh) {
    const x = Math.min(event.clientX + 14, innerWidth - label.offsetWidth - 8);
    label.style.transform = `translate(${x}px, ${event.clientY + 14}px)`;
  }
}
function pick(event) {
  ray.setFromCamera(new THREE.Vector2((event.clientX / innerWidth) * 2 - 1, 1 - (event.clientY / innerHeight) * 2), camera);
  const solid = meshes.filter((mesh) => !mesh.material.transparent);
  hover(ray.intersectObjects(solid, false)[0]?.object ?? null, event);
}
renderer.domElement.addEventListener("pointermove", (event) => event.pointerType === "mouse" && event.buttons === 0 && pick(event));
renderer.domElement.addEventListener("pointerleave", (event) => hover(null, event));
renderer.domElement.addEventListener("click", pick);

// Tools
const tools = document.getElementById("tools");
function toggle(id, apply) {
  const button = document.getElementById(id);
  button.addEventListener("click", () => {
    const on = button.getAttribute("aria-pressed") !== "true";
    button.setAttribute("aria-pressed", on);
    apply(on);
  });
}
toggle("edges", (on) => meshes.forEach((mesh) => (mesh.children[0].visible = on)));
toggle("xray", (on) => {
  for (const mesh of meshes) {
    if (mesh.userData.role) Object.assign(mesh.material, { transparent: on, opacity: on ? XRAY : 1, depthWrite: !on, needsUpdate: true });
  }
});
// Explode slides each part out from the middle, or back, over EXPLODE_MS
let explosion = { from: 0, to: 0, start: 0 };
const exploded = () => THREE.MathUtils.lerp(explosion.from, explosion.to, THREE.MathUtils.smoothstep(performance.now() - explosion.start, 0, EXPLODE_MS));
toggle("explode", (on) => (explosion = { from: exploded(), to: on ? 1 : 0, start: performance.now() }));

addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
message.remove();
tools.hidden = false;
renderer.setAnimationLoop(() => {
  controls.update();
  const out = EXPLODE * exploded();
  for (const mesh of meshes) mesh.position.copy(mesh.userData.out).multiplyScalar(out);
  renderer.render(scene, camera);
});

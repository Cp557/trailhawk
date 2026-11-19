// The VTOL's parts as three.js meshes, for the model page (cad.js) and the simulator (sim/app.js).
// `data` is the folder trailhawk/sim.py writes: aircraft.json (flight data and the paint's colors by
// role) and one STL per part in CAD mm (+X nose to tail, +Y right wing, +Z up), listed in parts.json.
import * as THREE from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";

// Faces meeting at more than this (deg) are shaded as a sharp fold; a curved face's facets are finer
export const CREASE = 40;

const loadJson = async (url) => (await fetch(url)).json();

// { ac, meshes }: one mesh per part, with userData.path ("/motor_right/propeller") and, for a
// painted part, userData.role: the role in ac.paint whose color it has.
export async function loadAircraft(data) {
  const [ac, parts] = await Promise.all([loadJson(data + "aircraft.json"), loadJson(data + "parts.json")]);
  const loader = new STLLoader();
  const meshes = await Promise.all(
    parts.map(async (part) => {
      const stl = loader.parse(await (await fetch(data + part.file)).arrayBuffer());
      const geometry = toCreasedNormals(stl, THREE.MathUtils.degToRad(CREASE));
      const color = new THREE.Color().setRGB(...part.color, THREE.SRGBColorSpace);
      const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05 }));
      const role = Object.keys(ac.paint).find((key) => new THREE.Color(ac.paint[key]).getHex() === color.getHex());
      mesh.userData = { path: part.path, role };
      return mesh;
    }),
  );
  return { ac, meshes };
}

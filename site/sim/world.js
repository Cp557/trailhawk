// The world the simulator flies over, generated from a seed: rolling hills ringed by mountains, a
// trail from the pad to a lake, patches of forest, and a hiker who walks the trail carrying a GPS
// beacon. The physics, the page and check.mjs all use it, so what the aircraft touches is what is
// drawn. Points are [north, east] in metres from the pad; heights are metres above it.

export const SIZE = 1500; // m from the pad to the world's edge, north, south, east and west
export const SPACING = 5; // m between height samples
export const CELLS = (2 * SIZE) / SPACING; // a side; the pad is the middle sample
const HILLS = 40; // m: the height of the hills' noise
const PEAKS = [1000, 1450]; // m from the pad where the mountains start and reach full height
const LAKE_DISTANCE = [800, 950]; // m from the pad
const DIP = [40, 220]; // m: depth and radius of the basin the lake fills
const LAKE_DEPTH = 8; // m of water above the basin's bottom
const WATER_RADIUS = 400; // m: the water is a disc this big round the basin; the hills above it hide its edge
const TRAIL_STEP = 2; // m between the trail's centerline points
const TRAIL_WIDTH = [3, 22]; // m from the centerline: graded flat, then eased back into the hills
const GRADE = 0.15; // steepest rise or fall of the trail per metre
const HIKER_START = 10; // m along the trail from the pad
const TREELINE = 120; // m above the pad, give or take 25 m
// Wildlife (m): length, height at the shoulder, and how far each wanders from its spot. Sun-warmed
// rocks too, which a thermal camera can take for animals.
export const WILDLIFE = {
  elk: { size: [2.4, 1.5], wander: 30 },
  deer: { size: [1.6, 1.0], wander: 20 },
  bear: { size: [1.7, 1.0], wander: 40 },
  rock: { size: [1.4, 0.8], wander: 0 },
};
const BEACON_ERROR = 3; // m: spread of the GPS error, north and east
const BEACON_KEEP = Math.exp(-1 / 20); // the error wanders, with a 20 s time constant

// Repeatable pseudo-random numbers in 0..1 (Lehmer); seed 1 to 2^31 - 2
export const seeded = (seed) => () => (seed = (seed * 16807) % 2147483647) / 2147483647;

// 0..1 from three integers: the murmur3 finalizer
function hash(seed, x, y) {
  let h = Math.imul(x, 0x27d4eb2d) ^ Math.imul(y, 0x165667b1) ^ Math.imul(seed, 0x9e3779b1);
  h = Math.imul(h ^ (h >>> 15), 0x85ebca6b);
  h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}
// A standard normal draw from the stream `next` (Box-Muller)
export const gaussian = (next) => Math.sqrt(-2 * Math.log(1 - next())) * Math.cos(2 * Math.PI * next());
const random = (seed, stream) => seeded(1 + Math.floor(hash(seed, stream, 0) * 2147483645));

const smooth = (t) => t * t * (3 - 2 * t);
const clamp01 = (x) => Math.min(Math.max(x, 0), 1);
export const smoothstep = (a, b, x) => smooth(clamp01((x - a) / (b - a)));
export const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

// Value noise: random values on an integer lattice, smoothly interpolated; -1..1
function noise(seed, x, y) {
  const [x0, y0] = [Math.floor(x), Math.floor(y)];
  const [sx, sy] = [smooth(x - x0), smooth(y - y0)];
  const at = (i, j) => hash(seed, x0 + i, y0 + j);
  const top = at(0, 0) + (at(1, 0) - at(0, 0)) * sx;
  const bottom = at(0, 1) + (at(1, 1) - at(0, 1)) * sx;
  return 2 * (top + (bottom - top) * sy) - 1;
}

// Fractal noise with features from `size` m down: each layer half the size and weight of the last,
// turned so the lattices don't line up; about -1..1, mostly within ±0.5
function fractal(seed, [north, east], size, layers) {
  let [x, y, total, weight, sum] = [east / size, north / size, 0, 1, 0];
  for (let i = 0; i < layers; i++) {
    total += weight * noise(seed + 7919 * i, x, y);
    sum += weight;
    [x, y] = [1.6 * x - 1.2 * y, 1.2 * x + 1.6 * y];
    weight /= 2;
  }
  return total / sum;
}

// Height on the grid, flat across the two triangles of each cell (split along its south-west to
// north-east diagonal), as the page draws it. Past the edge, the edge's height.
function sample(grid, north, east) {
  const r = Math.min(Math.max((SIZE - north) / SPACING, 0), CELLS);
  const c = Math.min(Math.max((east + SIZE) / SPACING, 0), CELLS);
  const [r0, c0] = [Math.min(Math.floor(r), CELLS - 1), Math.min(Math.floor(c), CELLS - 1)];
  const [fr, fc] = [r - r0, c - c0];
  const i = r0 * (CELLS + 1) + c0;
  const [nw, ne, sw, se] = [grid[i], grid[i + 1], grid[i + CELLS + 1], grid[i + CELLS + 2]];
  return fr + fc <= 1 ? nw + fc * (ne - nw) + fr * (sw - nw) : se + (1 - fc) * (sw - se) + (1 - fr) * (ne - se);
}

// Each grid sample's [north, east], row by row from the north-west corner
export const samplePoint = (i) => [SIZE - Math.floor(i / (CELLS + 1)) * SPACING, -SIZE + (i % (CELLS + 1)) * SPACING];

// Tobler's hiking function: walking speed (m/s) on a grade (rise over run), 6 km/h at 5% downhill
const tobler = (grade) => (6 / 3.6) * Math.exp(-3.5 * Math.abs(grade + 0.05));

// A centerline from the pad toward `end` that bends: one wide arc, an S and small wiggles, each
// fading out at both ends; a point every TRAIL_STEP m
function meander(next, end) {
  const along = Math.hypot(...end);
  const side = [-end[1] / along, end[0] / along];
  const [arc, s, wiggle, phase] = [(2 * next() - 1) * 250, (2 * next() - 1) * 90, (2 * next() - 1) * 30, 2 * Math.PI * next()];
  const curve = Array.from({ length: 4001 }, (_, i) => {
    const t = i / 4000;
    const off = Math.sin(Math.PI * t) * (arc + s * Math.sin(2 * Math.PI * t + phase) + wiggle * Math.sin(7 * Math.PI * t + 2 * phase));
    return [t * end[0] + off * side[0], t * end[1] + off * side[1]];
  });
  const points = [curve[0]];
  let left = TRAIL_STEP;
  for (let i = 1; i < curve.length; i++) {
    let [from, gap] = [curve[i - 1], distance(curve[i - 1], curve[i])];
    while (gap >= left) {
      const f = left / gap;
      from = [from[0] + f * (curve[i][0] - from[0]), from[1] + f * (curve[i][1] - from[1])];
      points.push(from);
      gap -= left;
      left = TRAIL_STEP;
    }
    left -= gap;
  }
  return points;
}

export function createWorld(seed) {
  const next = random(seed, 1);
  const hills = (p) => HILLS * fractal(seed * 16 + 1, p, 600, 4);

  // The lake: in the lowest of 24 spots at about LAKE_DISTANCE, in a basin dug into the hills
  const spots = Array.from({ length: 24 }, (_, i) => {
    const [bearing, far] = [(2 * Math.PI * (i + next())) / 24, LAKE_DISTANCE[0] + (LAKE_DISTANCE[1] - LAKE_DISTANCE[0]) * next()];
    return [far * Math.cos(bearing), far * Math.sin(bearing)];
  });
  const lake = spots.reduce((low, p) => (hills(p) < hills(low) ? p : low));
  const raw = (p) => {
    const crests = 1 - Math.abs(fractal(seed * 16 + 2, p, 700, 5)); // ridged: sharp tops
    const mountains = smoothstep(...PEAKS, Math.hypot(...p)) ** 2 * (100 + 160 * crests); // 100 to 260 m: sky shows above them
    return hills(p) + mountains - DIP[0] * Math.exp(-((distance(p, lake) / DIP[1]) ** 2));
  };
  const pad = raw([0, 0]);
  const water = raw(lake) - pad + LAKE_DEPTH;

  // The trail: its heights follow the ground averaged over 80 m, start level with the pad for 25 m, are cut
  // and filled to at most GRADE (a pass each way), and end before the shore
  const centerline = meander(next, lake);
  const ground0 = centerline.map((p) => raw(p) - pad);
  const span = 40 / TRAIL_STEP;
  const sums = [0];
  for (const h of ground0) sums.push(sums[sums.length - 1] + h);
  const level = ground0.map((_, i) => {
    const [a, b] = [Math.max(0, i - span), Math.min(ground0.length, i + span + 1)];
    return i * TRAIL_STEP < 25 ? 0 : (sums[b] - sums[a]) / (b - a);
  });
  const rise = GRADE * TRAIL_STEP;
  for (let i = level.length - 2; i * TRAIL_STEP >= 25; i--) level[i] = Math.min(Math.max(level[i], level[i + 1] - rise), level[i + 1] + rise);
  for (let i = 1; i < level.length; i++) level[i] = Math.min(Math.max(level[i], level[i - 1] - rise), level[i - 1] + rise);
  const shore = level.findIndex((h, i) => h < water + 1.5 || ground0[i] < water + 1);
  const points = centerline.slice(0, shore < 0 ? undefined : shore);
  const z = level.slice(0, points.length);

  // Heights: the hills, a flat clearing round the pad, and the trail graded into them
  const heights = new Float32Array((CELLS + 1) ** 2);
  const trailDistance = new Float32Array(heights.length).fill(30);
  const trailHeight = new Float32Array(heights.length);
  for (let i = 0; i < heights.length; i++) {
    const p = samplePoint(i);
    heights[i] = (raw(p) - pad) * smoothstep(12, 35, Math.hypot(...p));
  }
  const reach = Math.ceil(TRAIL_WIDTH[1] / SPACING) + 1;
  points.forEach((p, k) => {
    const [r, c] = [Math.round((SIZE - p[0]) / SPACING), Math.round((p[1] + SIZE) / SPACING)];
    for (let i = Math.max(0, r - reach); i <= Math.min(CELLS, r + reach); i++) {
      for (let j = Math.max(0, c - reach); j <= Math.min(CELLS, c + reach); j++) {
        const cell = i * (CELLS + 1) + j;
        const d = distance(p, samplePoint(cell));
        if (d < trailDistance[cell]) [trailDistance[cell], trailHeight[cell]] = [d, z[k]];
      }
    }
  });
  for (let i = 0; i < heights.length; i++) {
    const graded = 1 - smoothstep(...TRAIL_WIDTH, trailDistance[i]);
    heights[i] += (trailHeight[i] - heights[i]) * graded;
  }
  const terrain = (north, east) => sample(heights, north, east);

  // Forest in patches, below the treeline, off the steepest slopes, clear of the trail, the pad
  // clearing and the water: a tree on most spots of a jittered 9 m grid
  const forest = (p) => smoothstep(0.05, 0.2, fractal(seed * 16 + 3, p, 350, 4));
  // Meadow: from lush (0) to dry (1), in patches about 250 m across with smaller ones in them
  const meadow = (p) => smoothstep(-0.3, 0.3, 0.7 * fractal(seed * 16 + 5, p, 250, 3) + 0.3 * fractal(seed * 16 + 6, p, 45, 2));
  const trees = [];
  const place = random(seed, 2);
  for (let north = -SIZE + 4.5; north < SIZE; north += 9) {
    for (let east = -SIZE + 4.5; east < SIZE; east += 9) {
      const p = [north + 8 * (place() - 0.5), east + 8 * (place() - 0.5)];
      const [height, chance] = [10 + 12 * place(), place()];
      if (chance > 0.85 * forest(p)) continue;
      const h = terrain(...p);
      const slope = Math.hypot(terrain(p[0] + 2, p[1]) - terrain(p[0] - 2, p[1]), terrain(p[0], p[1] + 2) - terrain(p[0], p[1] - 2)) / 4;
      const treeline = TREELINE + 25 * fractal(seed * 16 + 4, p, 200, 2);
      if (h > treeline || h < water + 1.5 || slope > 0.7 || sample(trailDistance, ...p) < 5 || Math.hypot(...p) < 35) continue;
      trees.push(p[0], p[1], height);
    }
  }

  // The hiker's walk: the time to reach each centerline point at Tobler's speed, from HIKER_START
  const start = HIKER_START / TRAIL_STEP;
  const times = z.map(() => 0);
  for (let i = start + 1; i < z.length; i++) times[i] = times[i - 1] + TRAIL_STEP / tobler((z[i] - z[i - 1]) / TRAIL_STEP);
  // The trail's point `metres` along his walk, clamped to its ends
  const along = (metres) => points[Math.min(Math.max(Math.round(start + metres / TRAIL_STEP), 0), points.length - 1)];
  // Where he is `seconds` after setting off: position, heading (rad from north), metres walked,
  // speed (m/s) and whether he has reached the lake
  const hikerAt = (seconds) => {
    let [i, last] = [start, z.length - 2]; // the last point reached: a binary search
    while (i < last) {
      const mid = (i + last + 1) >> 1;
      if (times[mid] <= seconds) i = mid;
      else last = mid - 1;
    }
    const f = clamp01((seconds - times[i]) / (times[i + 1] - times[i]));
    const [a, b] = [points[i], points[i + 1]];
    return {
      position: [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])],
      heading: Math.atan2(b[1] - a[1], b[0] - a[0]),
      walked: (i + f - start) * TRAIL_STEP,
      speed: seconds < times[z.length - 1] ? tobler((z[i + 1] - z[i]) / TRAIL_STEP) : 0,
      done: seconds >= times[z.length - 1],
    };
  };

  // Wildlife along the hike: herds of elk in the open 50-250 m from the trail, deer at the forest's
  // edges 30-200 m from it, a bear 40-120 m from it past its first 40%, sun-warmed rocks 30-200 m
  // from it, and a herd near the trailhead; each on a spot its wander keeps clear of water and the
  // mountains.
  // From their own random stream, so they leave the rest of the world as it was; a rule that finds
  // no spot in 400 tries places nothing.
  const scatter = random(seed, 4);
  const nearest = (p) => points.reduce((best, q, i) => (distance(p, q) < best[0] ? [distance(p, q), i] : best), [Infinity, 0]);
  const fits = (p, kind) => {
    const reach = WILDLIFE[kind].wander + 15;
    const slope = Math.hypot(terrain(p[0] + 5, p[1]) - terrain(p[0] - 5, p[1]), terrain(p[0], p[1] + 5) - terrain(p[0], p[1] - 5)) / 10;
    const dry = Array.from({ length: 8 }, (_, k) => [p[0] + reach * Math.cos((Math.PI * k) / 4), p[1] + reach * Math.sin((Math.PI * k) / 4)]).every(
      (q) => terrain(...q) > water + 2 || distance(q, lake) > WATER_RADIUS,
    );
    return Math.hypot(...p) + reach < PEAKS[0] && slope < 0.4 && dry;
  };
  const find = (kind, rule) => {
    for (let tries = 0; tries < 400; tries++) {
      const p = [(2 * scatter() - 1) * PEAKS[0], (2 * scatter() - 1) * PEAKS[0]];
      const [off, index] = nearest(p);
      if (fits(p, kind) && rule(p, off, index / points.length)) return p;
    }
    return null;
  };
  const wildlife = [];
  const add = (kind, home, herd = wildlife.length) => wildlife.push({ kind, home, herd, phase: [1000 * scatter(), 1000 * scatter()], facing: 2 * Math.PI * scatter() });
  for (let herd = 0; herd < 3; herd++) {
    const center = find("elk", (p, off) => forest(p) < 0.3 && off > 50 && off < 250);
    const count = 3 + Math.floor(4 * scatter());
    for (let i = 0; center && i < count; i++) add("elk", [center[0] + 20 * (2 * scatter() - 1), center[1] + 20 * (2 * scatter() - 1)], -1 - herd);
  }
  for (let i = 0; i < 5; i++) {
    const home = find("deer", (p, off) => forest(p) > 0.2 && forest(p) < 0.7 && off > 30 && off < 200);
    if (home) add("deer", home);
  }
  const den = find("bear", (p, off, along) => forest(p) < 0.6 && off > 40 && off < 120 && along > 0.4);
  if (den) add("bear", den);
  for (let i = 0; i < 4; i++) {
    const spot = find("rock", (p, off) => forest(p) < 0.3 && off > 30 && off < 200);
    if (spot) add("rock", spot);
  }
  // And a herd grazing near the trailhead, so the cameras find something in their first sweeps: 3-5
  // elk in the open, 120-220 m along the walk and 40-120 m to one side of the trail
  for (let tries = 0; tries < 400; tries++) {
    const k = Math.min(Math.round(start + (120 + 100 * scatter()) / TRAIL_STEP), points.length - 2);
    const [a, b] = [points[k], points[k + 1]];
    const side = (scatter() < 0.5 ? -1 : 1) * (40 + 80 * scatter()) / distance(a, b);
    const center = [a[0] - side * (b[1] - a[1]), a[1] + side * (b[0] - a[0])];
    if (!fits(center, "elk") || forest(center) >= 0.3 || nearest(center)[0] < 40) continue;
    const count = 3 + Math.floor(3 * scatter());
    for (let i = 0; i < count; i++) add("elk", [center[0] + 15 * (2 * scatter() - 1), center[1] + 15 * (2 * scatter() - 1)], -4);
    break;
  }
  // A smooth wander, -1..1, along time: a herd shares one, and each animal adds a little of its own
  const wave = (x) => noise(seed * 16 + 7, x, 0);
  const drift = (animal, t) => {
    const { wander } = WILDLIFE[animal.kind];
    const leader = animal.herd < 0 ? wildlife.find((other) => other.herd === animal.herd) : animal;
    const own = animal === leader ? 0 : 4;
    return [0, 1].map((k) => animal.home[k] + wander * wave(leader.phase[k] + t / 100) + own * wave(animal.phase[k] + t / 60));
  };
  // Every animal and rock `seconds` in: kind, position, heading (rad from north) and size
  const wildlifeAt = (seconds) =>
    wildlife.map((animal) => {
      const [before, after] = [drift(animal, seconds - 2), drift(animal, seconds + 2)];
      const moving = Math.hypot(after[0] - before[0], after[1] - before[1]) > 0.05;
      return {
        kind: animal.kind,
        position: drift(animal, seconds),
        heading: moving ? Math.atan2(after[1] - before[1], after[0] - before[0]) : animal.facing,
        size: WILDLIFE[animal.kind].size,
      };
    });

  return {
    seed,
    heights, // a sample every SPACING m, row by row from the north-west corner
    water, // m above the pad
    lake: { center: lake, radius: WATER_RADIUS }, // the water: a disc round the basin
    trail: { points, length: (z.length - 1 - start) * TRAIL_STEP }, // length: what he walks
    trees: Float32Array.from(trees), // north, east, height
    // What the aircraft touches: the terrain, or the water's surface on the disc
    ground: (north, east) => (distance([north, east], lake) < WATER_RADIUS ? Math.max(terrain(north, east), water) : terrain(north, east)),
    terrain,
    trailDistance: (north, east) => sample(trailDistance, north, east),
    forest,
    meadow,
    hikerAt,
    along,
    wildlifeAt,
  };
}

// The hiker's phone: a fix once a second, off by an error that wanders round his true position
export function createBeacon(seed) {
  const next = random(seed, 3);
  let error = [BEACON_ERROR * gaussian(next), BEACON_ERROR * gaussian(next)];
  let [fix, due] = [null, 0];
  return (position, seconds) => {
    if (seconds >= due) {
      error = error.map((e) => BEACON_KEEP * e + Math.sqrt(1 - BEACON_KEEP ** 2) * BEACON_ERROR * gaussian(next));
      fix = [position[0] + error[0], position[1] + error[1]];
      due = Math.floor(seconds) + 1;
    }
    return fix;
  };
}

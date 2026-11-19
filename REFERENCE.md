# Reference

Trailhawk: a parametric CAD model of a small tri-tiltrotor VTOL drone, written with [build123d](https://github.com/gumyr/build123d) (0.13) and viewed with [ocp_vscode](https://github.com/bernhard-42/vscode-ocp-cad-viewer). It is specified with real, buyable parts (`trailhawk/parts.py`) in ArduPilot's tilt-tricopter layout. Its mission is circling over a hiker on high trails while scanning for wildlife, so it is sized for 12,000 ft, comes apart to fit a backpack and carries thermal and color cameras with an onboard computer. It has weight-and-balance, power-and-endurance and strength models driven by those parts' data, a printable, bed-sized split of its airframe, photoreal renders, and a static website: a page to turn the model over in 3D, and a browser flight simulator where it flies itself over a generated mountain world, following a hiker walking a trail to a lake and mapping the wildlife its cameras find.

## Layout

| Path | Purpose |
| --- | --- |
| `LICENSE` | MIT. |
| `README.md` | The public overview: the problem, the requirements with their results, the design choices, what to test first, and the parts list with prices (checked 2026-11-19) and links. |
| `trailhawk/` | The Python package: the model, its analyses and exports. Run a module with `python -m drone.<module>` from the repo root. |
| `trailhawk/parts.py` | The real parts: name, link, mass, size and the specs the models use (motor test table, prop coefficients, servo torque, battery capacity). Single source for `vtol.py`, `balance.py` and `performance.py`. |
| `trailhawk/cad.py` | Shared build123d helpers: NACA `section`s lofted into `surface`s on `span_plane`/`fin_plane` stations, the `propeller`, `labeled`, `world_parts` (every leaf of an assembly in place, with its path), and the exports: `export_and_show` (STEP and STL to `out/`, then the viewer), `export_parts` (one STL per part and a `parts.json` manifest) and `show_in_viewer`. |
| `trailhawk/vtol.py` | Tri-tiltrotor VTOL (`build_vtol()`), built from the `cad.py` helpers with part sizes from `parts.py`: airframe, moving parts located at their pivots, motor hinges, carbon spars and joiners, internal components, payload turret, paint and surface details. Run it to export and display. |
| `trailhawk/fly.py` | Viewer animation of the VTOL: takeoff, transition, banked lap, landing. |
| `trailhawk/balance.py` | Weight and balance: mass of every part, CG in cruise and hover, MAC, wing loading, stall speed, hover thrust split. Run it for a report. |
| `trailhawk/performance.py` | Power and flight time from the parts' data, in any `Air`: a propulsion check against the motor maker's test, hover and full-throttle thrust and current, tilt-servo load, cruise drag buildup and power curve, and takeoff-cruise-landing and circling missions. Run it for a report at 12,000 ft and sea level. |
| `trailhawk/strength.py` | Strength of the carbon structure: spars, joiners, center fitting, tail, boom and skids under gust, thrust and landing loads. Run it for a report. |
| `trailhawk/pack.py` | Packing list: the sections the airframe separates into on the trail, and whether each fits a backpack. Run it for a report. |
| `trailhawk/print_parts.py` | Printable airframe: hollowed, trimmed, channeled and pocketed printed parts cut into pieces that fit a 256 mm bed. Run it to write `out/print/*.stl` and display the pieces. |
| `trailhawk/render.py`, `trailhawk/render_scene.py` | Exports every part with its paint color, then runs Blender headless to render `out/render/studio.png` and `aerial.png`. |
| `trailhawk/sim.py` | The website's data and local server: the simulator's flight data (`aircraft(vtol)`), `--export` to rewrite `site/data/`, and a server for `site/`. |
| `site/` | The website, all static files. `index.html` and `cad.js`: the model page. `favicon.svg`: the tab icon, a soaring hawk (Andy Wilson's *Buteo* silhouette from PhyloPic, public domain) in orange on the site's ink color. `trailhawk.jpg`: the studio render (`python -m trailhawk.render`), shown at the top of `README.md` and as the link preview (`og:image`). `aircraft.js`: the parts loader both pages share. `data/`: what `python -m trailhawk.sim --export` writes, committed. Vercel deploys it from `main` (project `trailhawk`, root directory `site`) to https://trailhawk.vercel.app; production is public, previews need a Vercel login. `sim/`: the flight simulator, where `physics.js` is the flight model, `control.js` the flight controller, `world.js` the generated world (ground, trail, lake, trees, wildlife, the walking hiker and his phone), `sensors.js` what the aircraft's cameras detect, and `app.js` and `index.html` the page: the chase view, the drone camera's view, the readings and the map. |
| `tests/` | `unittest` checks: solid validity, key dimensions, pivots, prop and tilt clearance, component and spar fit, the flight timeline, balance limits and tail volumes, the propulsion model against the motor test, thrust, current and servo margins at altitude, the energy budget, that print pieces fit the bed, conserve material and clear the hardware, that every field section fits the pack, that every strength check has a positive margin, that the simulator matches the reports, flies a whole trip, handles slopes, valleys and ridges, generates repeatable worlds with a walkable trail, follows the walking hiker to the lake under the 400 ft ceiling, that its cameras detect what the model says and map the hike, and that the committed site data matches the model. `check.mjs` flies the simulator headless for `test_sim.py` (needs Node.js; skipped without it). |
| `requirements.txt` | Python dependencies. |
| `.claude/launch.json` | Launch configs for the Claude desktop app's browser pane: `ocp-viewer` (the standalone viewer) and `site` (`python -m trailhawk.sim --serve` on port 8123), both run with `.venv/bin/python` (macOS/Linux path). |
| `out/` | Generated, not source: `vtol.step` (colored, labeled assembly) and `vtol.stl`, `print/<piece>.stl` and `render/`. |

## Usage

Setup needs Python 3 (tested on 3.14), Node.js (for `test_sim.py`) and Blender 5.2 (for renders). On macOS:

```bash
brew install python node && brew install --cask blender
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

With the venv active, `python` is the venv's interpreter on both systems. From the repo root:

```bash
python -m ocp_viewer          # standalone viewer (no VS Code): open http://localhost:3939/viewer
python -m trailhawk.vtol          # writes out/vtol.step and out/vtol.stl, then sends the model to the viewer
python -m trailhawk.fly           # shows the VTOL over a ground pad and starts the flight animation
python -m trailhawk.balance       # prints the weight and balance report
python -m trailhawk.performance   # prints the power and flight-time report
python -m trailhawk.print_parts   # writes out/print/<piece>.stl (48 pieces) and shows them
python -m trailhawk.pack          # prints the packing list
python -m trailhawk.strength      # prints the strength report
python -m trailhawk.render        # renders out/render/studio.png and aerial.png with Blender (set BLENDER if it is not in the default folder)
python -m trailhawk.sim           # serves the website and opens http://localhost:8123/: the model page, and the simulator at /sim/ (?seed=N picks the world)
python -m trailhawk.sim --export  # first rewrites site/data/ from the model: do this after changing it
python -m unittest            # runs every test in tests/ (about 4 minutes)
```

`export_and_show(shape, name)` writes both files and sends the shape to the viewer (`show_in_viewer`: `ocp_vscode.show` on port 3939), which is served by either the standalone viewer (`python -m ocp_viewer`) or the VS Code OCP CAD Viewer extension. If neither is running, `show()` prints a `CommsWarning` that it cannot reach the viewer; the exports still complete and the script exits 0. The viewer does not keep models across page reloads; re-run the script after reloading. If the animation loads paused, press ▶ in the viewer's animation bar (bottom right). The toolbar's transparency toggle shows the internal components.

## Conventions

- Units: millimetres.
- Axes: +X runs nose to tail, +Y is the right wing, +Z is up. The nose (motor face) is at `x = 0`; thrust points toward −X.
- `section(naca, chord, plane, pitch)` places a NACA 4-digit profile with its **quarter chord at the plane origin**, chord along the plane's local X, lift side along local Y, rotated nose-up by `pitch` degrees.
- `span_plane(x, y, z)` is used for horizontal surfaces (lofted along Y, local Y = +Z); `fin_plane(x, z)` for the fin (lofted along Z, local Y = +Y).
- Lifting surfaces are lofted between stations; the right half is built and mirrored about `Plane.XZ`, then fused.
- `span_plane(...).rotated((cant, 0, 0))` rolls a section about X by `cant` degrees (used for the VTOL winglets).
- `propeller(radius, pitch)` is built with its hub at the origin, spinning about X; callers place it with `Pos(...)`.
- Bought parts take their size, mass and specs from `parts.py` (checked 2026-11-05, links in each entry). Anything not from a datasheet is marked as an estimate where it is defined.
- Assembly children's `location` is relative to their parent `Compound`. The viewer turns every node into a group with that location, and animation rotations pivot about the group origin. Code that needs world coordinates composes the parent locations (`cad.world_parts`, or `balance.leaves` with the front motors tilted).
- `pivot_at(solid, loc)` keeps a solid in place but makes its location exactly `loc`, baking the inverse transform into the geometry. build123d 0.13 has no `relocate`, and its `transform_shape` only rewrites the location, so the helper calls `BRepBuilderAPI_Transform(..., copy=True)` directly.
- `hinged(solid, hinge0, hinge1, gap=2)` cuts a control surface off the +X side of a lifting surface along the hinge line, leaving a 2 mm slot, and returns it pivoted on the hinge (origin `hinge0`, local X along the hinge).
- `wing_stations()`, `tail_stations()` and `fin_stations()` return the (plane, chord, pitch) stations that the surfaces, spars, servo positions and MAC are all derived from. `chord_point(station, fraction)` and `surface_point(a, b, s, fraction, camber)` locate points on them.
- **Volumes:** build123d's default `.volume` undercounts the lofted airfoil solids by up to ~25% (the wing reads 4.82e6 mm³; the adaptive integral and a mesh both give ≈ 6.5e6). Areas and centroids are accurate to < 1%. Where volume matters, use `BRepGProp.VolumeProperties_s(shape.wrapped, props, 1e-4)` as `test_print.volume` does. Zero-overlap checks are unaffected.
- **Booleans:** a cutting tool that is a `Compound` of overlapping solids makes OCCT booleans silently wrong; fuse tools first (`print_parts.union`). A long thin boolean (a whole-length 1.2 mm fuselage liner) came out invalid, so joiner sleeves are cut from a 16 mm band at each joint instead.

## VTOL (`vtol.py`)

Tri-tiltrotor in ArduPilot's tilt-tricopter layout, built in cruise configuration: the front motors (`TILTING`) face forward, the rear motor is fixed upright. `build_vtol()` returns a `Compound` labeled `vtol` (78 solids, envelope 1326 × 2325 × 546 mm). Paint (`PAINT`, by role): carbon-black airframe and tail nacelle (#1e1e21), orange front pods and control surfaces (#ef6c1f), black skids and dark gray surface details. Dark reads as a silhouette against the sky from the ground; orange stands out against forest. Motors, props, lenses and fittings keep their own colors.

| Label | Geometry | Moves (in `fly.py`) |
| --- | --- | --- |
| `fuselage` | Half-ellipsoid nose (`x = 0 → 100`) fused to a loft of the 7 `FUSELAGE` ellipses out to `x = 1200`: deep pod (max 160 × 176 mm) tapering into a ~30 mm tail boom that rises to `z = 48`. Separates at `BOOM_JOINT_X` = 640. `fuselage(inset)` gives the cavity inside an `inset`-thick skin. The union is passed through `.fix()` to be BRep-valid. | — |
| `details` | Panel lines and vents standing 0.3 mm proud of the skin (`surface_details()`): seams at the hatch and boom joint, three cooling slots each side of the computer. | — |
| `wing` | NACA 2412, span 2200 mm to the winglet roots (2325 with winglets). Straight inner panel to `y = 340` (`WING_CRANK_Y`), chord 260 → 230, quarter chord at `x = 390` (`WING_X`); outer panel swept 12° to `y = 1100` (`WING_TIP_Y`), chord 230 → 110; then a 4-section winglet rolling up to ~88° and rising ~140 mm. 2° dihedral, root at `z = 60`, incidence 2° → −1°. Inboard of `y = 130` (`WING_BLEND_Y`) the root flares into the fuselage (`wing_blend`: chord to 315 mm, section to 16% thick). Separates at `WING_JOINTS` = ±140 and ±480. | — |
| `aileron_right`, `aileron_left` | Outer panel aft of 70% chord, from 35% to 92% of the outer panel span (`AILERON`). | `rx` (hinge); same angle on both = roll |
| `horizontal_tail` | NACA 0010, span 520, chord 160 → 100, quarter chord swept 25° from `x = 1080`, `z = 46`. Bolts on as one piece. | — |
| `elevator_right`, `elevator_left` | Aft of 65% chord, 6% → 98% span (`ELEVATOR`). | `rx`; left takes the negated angle |
| `fin` | NACA 0010, `z = 40 → 300` (`FIN_TIP_Z`), chord 200 → 110, quarter chord `x = 1050 → 1120`, with a dorsal fillet ahead of the hinge line blending its root into the boom. | — |
| `rudder` | Aft of 65% chord, 11% → 86% of fin height, clear of boom and tip pod (`RUDDER`). The three layouts (span fractions, hinge chord fraction) also place the simulator's control surfaces. | `rx` |
| `nacelles` | The 2 front pods: fixed circular-section pods running aft from each tilting motor's nose point (`MOTORS`), under the wing at `y = ±245` (`POD_Y`, far enough out to keep `PROP_GAP` at the prop tips), `x = 200 → 580`, axis `z = 48`. `nacelle(nose, length, inset)` gives the cavity. | — |
| `tail_nacelle` | The same pod on the fin tip under the rear motor, `x = 1050 → 1230`, `z = 300`; holds `esc_tail`. A separate leaf so it takes the airframe's paint. | — |
| `motor_right`, `motor_left`, `motor_tail` | Assemblies located at their hinge `hinge(nose)` = nose + `HINGE_UP` (31) in Z, directly above the nacelle front face; the rear one is fixed at 90° (upright) and unbolts for packing. Children: `mount` (back plate, tab and knuckle bored for the pin); `motor`, the AT2820 (Ø35.2 can, `CAN_LENGTH` 43.5, plus a Ø5 shaft `SHAFT_OUT` 20 — the datasheet gives only the 63.5 mm total, so the split is an estimate), `MOTOR_GAP` (12) ahead of the hinge on the nacelle axis; `prop_nut`; `propeller`, 12×6 in, hub at local `(HUB_X, 0, −HINGE_UP)`. A front motor stays ahead of its nacelle face through the whole swing. | Front: `ry` tilt 0 = cruise, 90 = hover, up to 90 + `YAW_TILT` (10) for yaw; `propeller` spins `rx` |
| `hinges` | Fixed half of each hinge (`<motor>_hinge`, located at the hinge), set into the top of the nacelle front. Front: `tilt_servo` (KST X12-708, 12 × 27.5 × 23, direct drive: its output spline is the hinge axis and drives the knuckle; spline position assumed 6 mm from the top), `bearing` lug on the other side, Ø3 `hinge_pin`. Rear: a lug each side and the pin bolting the mount upright. | — |
| `spars` | Carbon tubes, 1 mm wall (`spar_lines()`). Each wing spar is one straight line (`spar_point(y)`, in plan swept 8°, along the airfoil's mean line) so straight joiners fit it: `wing_root_tube_*` (`y = 5 → 140`, Ø16), `wing_spar_inner_*` (`140 → 480`, Ø16), `wing_spar_outer_*` (`480 → 850`, Ø16, stops at `SPAR_END_Y`). Loose joiners: `wing_joiner_inner_*` Ø14 × 240 at `y = 140`, `wing_joiner_outer_*` Ø14 × 200 at `y = 480` (`JOINER_EMBED`). `boom_spar` Ø16 from `x = 550` (`BOOM_SOCKET_START`) to 1030, in a `boom_socket` Ø18 in the pod. `tail_spar_*` Ø6 and `fin_spar` Ø8 unchanged. | — |
| `center_fitting` | Printed block inside the fuselage, 44 × 100 × 30 (`FITTING_SIZE`), bonding the two root tubes where the straight spars meet in a shallow V (`center_fitting()`). | — |
| `avionics` | Boxes of the real parts' sizes (`components()`: label → host, center, size): `battery` (Flywoo 4S2P Li-ion, 145 × 45 × 45 estimated, `BATTERY_CENTER` = (227, 0, 0), the CG trim), `computer` (Pi 5 + AI HAT+, `COMPUTER_CENTER`, nose), `airspeed_sensor` (Matek ASPD-4525 board, `AIRSPEED_SENSOR_CENTER` = (40, 0, 0), ahead of the computer), `flight_controller` (`FLIGHT_CONTROLLER_CENTER`, where the wiring's mass is booked too), `receiver`, `gps` (boom, `x = 700`, above the boom tube), `esc_*`, `aileron_servo_*`, `elevator_servo_*`, `rudder_servo`. Each sits inside its host with at least `SKIN` to spare, and none touches another. | — |
| `landing_gear` | Two faired skids at `y = ±POD_Y`: a rail (ruled loft, Ø20, `RAIL_X` = 280 → 620, `z = SKID_Z = −125`) on one NACA 0025 strut, 60 mm chord, that runs up through the pod to just under the wing spar (`strut_top()`). Lowest point `GROUND_Z = −135`. | — |
| `gimbal_mount` | Short stem inside the nose above the ball. | — |
| `pitot` | The airspeed sensor's pitot tube (`pitot()`), Ø5 (`PITOT_RADIUS`, an estimate), on the nose's axis from 50 mm ahead of the tip, in clean air, back into the nose (`PITOT_X`). Its drag, about 0.1% of `CD0`, is left out. | — |
| `gimbal` | Turret at `GIMBAL_CENTER = (170, 0, −72)`, ball `CAMERA_RADIUS` 32 half buried in the belly (hangs `CAMERA_EXPOSED` 29 below it): `camera` (ball), `thermal_lens`, `lens` (color), both looking 45° down and forward. | `rz` pan |

Tested invariants: every prop disc, grown by `PROP_GAP` (25 mm: print tolerance, hinge slop, since 1° of tilt moves the tip 2.7 mm, and blade flex), clears the fuselage, wing, tail, fin, nacelles and skids, the front ones at 0°, 30°, 60°, 90° and 100° tilt, the rear one upright; each motor's mount, motor and prop nut clear the nacelles and hinge hardware through the swing; control surfaces don't overlap their parent surface; components sit inside their hosts, clear the rest of the airframe and don't touch each other; spars are fully enclosed; the skids are the lowest point.

## Flight animation (`fly.py`)

`fly()` shows the VTOL plus a `ground` disc (top at `GROUND_Z`, centered on the lap) and adds viewer tracks under `/Group/vtol`, sampled at `FPS = 10` over `DURATION = 27` s, loop-seamless:

| Time (s) | Phase |
| --- | --- |
| 0–2 | On the pad, front motors up, all props spinning |
| 2–5 | Vertical climb to `ALTITUDE = 1500` |
| 5–8 | Transition: front motors tilt 90 → 0 while accelerating into the lap, elevator up; the rear prop spins down |
| 8–20 | Left lap (radius `RADIUS = 2000`), banked 25° (aileron pulse on roll-in/out, rudder in, gimbal pans 90° toward the pad) |
| 20–23 | Transition back to hover over the pad, front motors 0 → 90; the rear prop spins up |
| 23–26 | Descent and landing |

The aircraft moves with a `t` track (nose position) and a `q` track: yaw about Z applied after body roll about X. The viewer interpolates rotation keyframes along the shortest path, so consecutive keyframes must differ by less than 180°. Props spin at `PROP_RPS = 4` (144° per keyframe), and the left prop counter-rotates. Each front-motor track rotates its whole unit (mount included) about the hinge. The rear prop's angle is `rear_spin(t)`, the integral of `tilt(t) / 90` (full speed hovering, stopped in cruise).

## Weight and balance (`balance.py`)

Masses: printed parts (fuselage, wing, tail, fin, nacelles, tail nacelle, control surfaces) weigh `area × SKIN × PRINT_DENSITY` (0.8 mm of foamed LW-PLA at 0.6 g/cm³); spars weigh their tube volume × 1.55 g/cm³; bought parts weigh their published mass from `parts.py`; the rest (prop nut, mount, lugs, pin, landing gear, gimbal mount, center fitting, turret, pitot tube) are estimates in `GRAMS`; a 130 g `wiring` allowance (wires, the connectors at the dry joints, fasteners) sits at the flight controller. `mass_items(vtol, tilt)` walks the assembly composing parent locations, with the front motors tilted.

Current report:

| Quantity | Value |
| --- | --- |
| All-up mass | 2896 g (battery 574, wing 421, motors 417, fuselage 153, wiring 130, …) |
| CG, cruise | x 436.6, y 0.2, z 63.7 mm = 26.6% MAC (`CG_RANGE` 25–30%) |
| CG, hover | x 437.6, z 72.3 mm |
| MAC | 203.8 mm, leading edge at x 382.4 (trapezoid panels, winglets excluded) |
| Wing area / loading | 0.425 m², 68.1 g/dm² |
| Stall speed (CL 1.2) | 9.5 m/s at sea level, 11.9 m/s at 12,000 ft |
| Tail volumes | horizontal 0.55 (`TAIL_VOLUME_RANGE` 0.4–0.7), vertical 0.028 (`FIN_VOLUME_RANGE` 0.02–0.05) |
| Hover thrust split | right 34.2%, left 34.2%, rear 31.6% (≈ 990 / 915 g) |

`hover_shares` solves the three thrusts that sum to the weight and balance pitch and roll about the hover CG. `Air(density, viscosity)` holds the conditions the models use: `SEA_LEVEL` and `DESIGN_AIR` (12,000 ft on a 10 °C day, 0.793 kg/m³). Slide `BATTERY_CENTER` along X to trim: 1 mm of battery moves the CG ≈ 0.2 mm. `inertia(vtol)` is the inertia tensor about the cruise CG in CAD axes: every mass as a point at its center, plus the printed skins' and tubes' own spread from their geometry. Roll 0.29, pitch 0.30, yaw 0.52 kg·m² (the heavy tail motor makes pitch match roll).

Not counted: slicer infill (≈ 250 g at 5% in the wing, tail and fin, which would move the CG aft by ≈ 2% MAC), servo linkages, wiring runs and hatch retention. The printed-part, wiring and turret masses are estimates.

## Power and flight time (`performance.py`)

Model (every function takes the `Air` it flies in):
- **Prop:** APC's coefficients for the 12x6E (`PROP["data"]`, the simulated 6000 rpm block): thrust `Ct ρ n² D⁴`, shaft power `Cp ρ n³ D⁵`, both functions of the advance ratio J = V / (nD). `rpm_for_thrust` solves for the rpm by bisection.
- **Motor and ESC:** battery power = shaft power / efficiency, with efficiency interpolated against shaft power from T-Motor's measured 4S test of the AT2820 on an APC 12×6 (`MOTOR["test"]`). Below the lowest test point (40% throttle, 94 W shaft) it holds that point's 0.70; cruise runs well below it, so cruise power is an extrapolation.
- **Full throttle:** the test's 100% row scaled by (V / 14.59 V)², at a sagged 3.2 V per cell. In thin air the motor spins up along a straight torque-speed line until it matches the prop's torque (`max_static`), which costs 24% of the thrust at 12,000 ft.
- **Tilt servo:** the worst steady load is (1.6 × hover thrust + motor-unit weight) × `HINGE_UP`, against the KST X12-708's stall torque interpolated at the flight controller's servo rail, 7.2 V.
- **Cruise drag:** `CD0 = LEAKAGE (1.1) × (Σ Cf·FF·S_wet + Σ CD·A_frontal) / S`. Turbulent flat-plate `Cf` at each component's Reynolds number, which is conservative at these sizes; Raymer form factors; CAD wetted areas. Bluff parts: the part of the camera ball below the belly (CD 0.47), front tilt-servo tops (1.0), the upright rear motor (0.8), the stopped rear prop (1.0 on 6 mm of projected blade, an estimate). The faired skids count as wetted area with an airfoil form factor.
- **Induced drag:** `CL² / (π e AR)` with e = 0.8 (winglets not credited) and AR = 11.4.
- **Electronics:** 14.5 W all the time (`ELECTRONICS_WATTS`): computer 10, cameras 1.5, avionics and servos 3. Estimates.
- **Battery:** 4S 9 Ah at 3.6 V per cell, 130 Wh, of which 80% is usable.
- **Hover limit:** `thinnest_air` is the air density at which full throttle on the sagged pack just meets `THRUST_MARGIN` over the busiest motor's hover share (`hover_need`); `hottest_day` turns it into a temperature limit at a pressure altitude (standard-atmosphere pressure), and `heaviest_takeoff` gives the weight limit in given air. The margin is thin and no CAD change fixes it (taking off ≈ 200 g, or a bigger motor), so it is reported as an operating limit.
- **Missions:** 90 s at hover power, then cruising at the best-range speed, or circling at `ORBIT_RADIUS` = 75 m (the turn's load factor raises induced drag).

Current report:

| Quantity | 12,000 ft, 10 °C (design) | Sea level |
| --- | --- | --- |
| Hover, with electronics | 532 W, 37 A at 14.4 V | 434 W, 30 A |
| Hover only | 11.7 min | 14.4 min |
| Full throttle at 12.8 V | 1619 g per motor (needs 1587 g for `THRUST_MARGIN` 1.6), 24.7 A | 2120 g, 32.3 A |
| CD0 | 0.049 | 0.047 |
| Slowest cruise (1.3 × stall) | 15.5 m/s, L/D 10.6, 96 + 14.5 W | 12.5 m/s, L/D 10.9, 77 + 14.5 W |
| Circling 75 m radius | 113 W, **48 min** | 92 W, **61 min** |
| Cruising | 49 min, 45 km | 61 min, 46 km |

Hover limit (1.6× on a sagged pack): air down to 0.769 kg/m³, that is 12,000 ft up to 19 °C, 13,000 ft up to 7 °C, 14,000 ft up to −4 °C (10,000 ft up to 42 °C). Heaviest takeoff at 12,000 ft, 10 °C: 2955 g, 59 g over the current 2896 g.

- CD0 breakdown: wing 43%, rear prop 10%, fuselage 9%, tail 7%, landing gear 7%, rear motor 7%, fin 5%, nacelles 5%, camera 4%, tilt servos 3%.
- Propulsion check against T-Motor's measurements is unchanged: within 7% up to 60% throttle (tested).
- Tilt servo: 8.3 kgf·cm at 7.2 V vs 5.5 kgf·cm load, 1.5× (`SERVO_MARGIN` 1.5). No spare.
- Circling at the design point must last at least `LOITER_MINUTES` (45, a requirement in `README.md`); it lasts 48 (tested).
- The best-endurance speed is pinned at the 1.3 × stall floor: the wing is working well below its best L/D (12 at CL ≈ 1.2). Treat drag-side numbers as ±20–30%; the skin-friction estimate is on the high side at these Reynolds numbers, so real flight times are more likely to be above these than below them.

## Parts list

Checked 2026-11-05 (battery and payload 2026-11-09); links are in `parts.py`. Prices, with links to sellers in stock, are in `README.md` (checked 2026-11-19); they and stock change.

| Qty | Part | Mass | Notes |
| --- | --- | --- | --- |
| 3 | T-Motor AT2820 Long Shaft KV880 | 139 g | 3–4S, 45 A / 700 W for 180 s |
| 2 + 1 | APC 12x6E (LP12060E) + 12x6EP (LP12060EP, reverse) | 27 g | 6.35 mm bore with adapter rings; a 5 mm ring for the motor shaft is unconfirmed. Buy spares |
| 3 | Hobbywing XRotor Pro 50A | 56 g | 4–6S, 50 A continuous; PWM (OneShot unsupported) |
| 1 | Flywoo Explorer 4S2P 9000 mAh Li-ion (Molicel P45B), XT60 | 573.5 g | Dimensions not published; 145 × 45 × 45 mm assumed from the cells. Listing says 45 A continuous, which reads as the per-cell figure (2 × 45 A = 90 A for the pack): confirm before relying on it. Since sold with BAK P45D cells, same SKU and mass (2026-11-19): re-check the current rating |
| 1 | Raspberry Pi 5 with AI HAT+ (13 TOPS) and active cooler | ≈ 82 g | Onboard detection. Board mass and loaded power unpublished, so estimated |
| 1 | Teledyne FLIR Lepton 3.5 (160 × 120, 57° HFOV) on a GroupGets PureThermal 3 USB board | ≈ 6 g | Thermal camera. Resolves ≈ 0.75 m per pixel at 120 m: a warm animal is a 1–2 pixel hot spot that points the color camera at it. Mass and power estimated |
| 1 | Raspberry Pi Camera Module 3 | 4 g | Color camera, 66° HFOV |
| 1 | Holybro Kakute H743-Wing | 28 g | ArduPilot `KakuteH7-Wing`, 14 PWM outputs (10 used), 90 A current sensor |
| 1 | Matek ASPD-4525 digital airspeed sensor (I2C), with pitot tube and tubing | 3.5 g | Plugs into the Kakute's I2C port (JST-GH). For the transition and slow circling in thin air, where GPS ground speed is a poor stand-in. Board size and the pitot tube (≈ 2 g) unpublished, so estimated. Out of stock at two US stores (2026-11-19) |
| 1 | Matek M10Q-5883 GPS and compass | 8 g | Discontinued (2026-11). `README.md` lists the Foxeer M10Q-180 V2 instead (18 × 18 × 8 mm, 7.5 g, fits the space); its plug is unchecked against the Kakute's GPS port |
| 1 | RadioMaster RP3 V2 ExpressLRS | 4.6 g | |
| 2 | KST X12-708 tilt servo | 20 g | 6.3 / 8.6 / 9.3 kgf·cm at 6 / 7.4 / 8.4 V; needs ≥ 6 V; often out of stock |
| 2 | KST X08 Plus aileron servo | 9 g | 8 mm thick |
| 3 | KST X06 V6.0 tail servo | 6 g | 7 mm thick |
| — | Roll-wrapped carbon tubes, 1 mm wall: Ø16 × 14 (≈ 2.2 m), Ø18 × 16 (≈ 0.1 m), Ø14 × 12 (≈ 0.9 m joiners), Ø8 × 6 (≈ 0.3 m), Ø6 × 4 (≈ 0.4 m), Ø10 × 8 (skid struts), Ø12 × 10 (skid rails); Ø3 pins | — | Generic. The joiners need a close sliding fit in the Ø14 bores |

Not chosen: wiring and connectors (130 g allowance), the turret's pan servo and bearing (60 g estimate), the skid tubes and fairings (90 g estimate). The Kakute's 5 V regulator is rated 2 A, too little to also run the Pi 5, so `README.md` adds a 5 A one (Matek BEC12S-Pro); its mass isn't in the model.

## ArduPilot setup (checked against ardupilot.org's tilt-rotor and QuadPlane frame-setup pages, 2026-11-05)

- `Q_ENABLE` 1, `Q_FRAME_CLASS` 7 (tri), `Q_TILT_ENABLE` 1, `Q_TILT_MASK` 3 (front motors tilt), `Q_TILT_TYPE` 2 (vectored yaw, no rear tilt servo), `Q_TILT_YAW_ANGLE` 10 (`YAW_TILT`: how far the front motors tilt past 90°).
- Outputs: 5 = motor 1 front right, 6 = motor 2 front left, 8 = motor 4 rear; tilt servos `SERVOn_FUNCTION` 75 (left) and 76 (right). Ailerons, elevators and rudder on the remaining outputs.
- In forward flight only the tilt-mask motors run, so the rear motor stops. This is from ArduPilot's source behavior and a forum post; the docs don't state it.
- Docs suggest a `Q_TILT_MAX` around 45° and `Q_TILT_RATE_UP`/`DN` around 15°/s.
- The strength check assumes the airspeed limit is set to `MAX_SPEED` = 20 m/s (`ARSPD_FBW_MAX`).

## Build risks

- **Prop tip to fuselage:** about 27 mm in cruise with the pods at ±245 mm, measured by growing the 20 mm thick test disc until it touches; 38 mm at 30° tilt and 89 mm from 60°. `PROP_GAP` (25 mm) is tested.
- The tilt servo's 100° travel exactly covers 0–100° (cruise to yaw limit). The 100° figure is from servodatabase.com and the torque figures from a reseller; KST's datasheet PDF couldn't be read. Check the real servo's endpoints, or lower `Q_TILT_YAW_ANGLE`.
- The servo rail voltage is listed differently by Holybro (6 V, or 8 V by jumper) and ArduPilot (6 / 7.2 V); the model assumes 7.2 V. At 6 V the tilt servo margin drops to 1.1×.
- Cruise runs below the motor test's lowest point, so cruise power is extrapolated at that point's 0.70 efficiency.
- APC's data is simulated, and T-Motor's table names "APC 12*6" without the "E".
- The motor's can/shaft split and the tilt servo's spline position are estimates; check against the parts before printing the mounts.
- **Hover thrust at altitude:** 1619 g per motor at 12,000 ft, 10 °C on a sagged pack against 1587 g needed: the 1.6× margin passes with 2% to spare. Treat the hover limit (`performance.py`) as an operating limit: at 12,000 ft no hotter than 19 °C, at 13,000 ft no hotter than 7 °C, and no more than 59 g of extra payload. More margin needs a bigger motor.
- **Battery current:** all three motors at full throttle draw 97 A at sea level, over the pack's 90 A and the flight controller's 90 A continuous sensor (220 A peak), so treat it as a short burst. Hover is 30–37 A. If the listing's 45 A is the real pack limit, hover at altitude is close to it. Check the XT60 and main leads too.
- **Thermal resolution:** the Lepton puts about one pixel on a bear at 120 m (two at 60 m), so it finds warm spots rather than identifying animals; the color camera (≈ 3 cm per pixel at 120 m) does the identifying in daylight. The upgrade is a FLIR Boson 320 with the 6.3 mm lens ($2,131, ≈ 0.23 m per pixel), which fits the same turret.
- **Heat on dark parts:** foamed LW-PLA softens at about 55 °C, which a black skin can reach sitting in high-altitude sun. Print the carbon-black parts in colorFabb LW-ASA (Vicat 96 °C, sold in black; about 0.43 g/cm³ foamed at 40% flow, so the 0.6 g/cm³ `PRINT_DENSITY` stays conservative). `PRINTED_SKIN` was estimated for LW-PLA; check it holds for LW-ASA. The tail nacelle (holding `esc_tail`) is black too; its print piece is one of the `nacelles` pieces, so print that one in LW-ASA. Park in shade, since the computer and battery sit inside the black pod.
- **The joints are untested designs:** a dry fit of carbon joiners and a printed center fitting. The strength margins depend on the wall-bearing allowable, which is an estimate.

## Printable airframe (`print_parts.py`)

`printed_parts(vtol)` prepares each printed part, then `print_pieces(vtol)` cuts each separate solid into the fewest equal slabs across one axis that each fit `PRINT_BED` = 256 × 256 × 256 mm, giving 48 pieces. The wing and fuselage are first cut at the dry joints (`WING_CUTS`, every wing joint's y, and `BOOM_JOINT_X`), which are not glued. The hatch is cut where `vtol.hatch()` puts its seam lines.

| Part | Prepared as | Cut across | Pieces |
| --- | --- | --- | --- |
| `fuselage` | `SKIN` shell, spar holes, turret opening, hatch removed, cut at the boom joint | X | 5 + 3 sleeves |
| `hatch` | Top of the nose over the computer and battery, from 2 mm above the battery | — | 1 |
| `wing` | Trimmed to the fuselage skin, cut at the wing joints, spar channels, aileron-servo pockets, strut holes | Y | 12 |
| `horizontal_tail` | Trimmed to the fuselage, spar channels, servo pockets | Y | 2 |
| `fin` | Trimmed to the fuselage and tail nacelle, spar channel, servo pocket | Z | 1 |
| `nacelles` | `SKIN` shells, trimmed to the wing and fin, holes for the skid struts, pockets for the hinge hardware | X | 5 + 2 sleeves |
| ailerons, elevators, rudder | As modeled | Y / Z | 2 + 2 + 1 + 1 + 1 |
| `mount`, `bearing` | Hinge parts (the rear motor has a lug each side) | — | 3 + 4 |
| `center_fitting` | The spar block; print it in nylon | — | 1 |
| `landing_gear` | The faired skids, around their carbon tubes | X | 2 |

- Spar channels have `FIT` = 0.15 mm radial clearance. A 16 mm wide, 1.2 mm thick joiner sleeve (`SLEEVE`) glued inside the hollow fuselage and nacelles keys each glued joint.
- The fuselage and nacelles are hollowed in CAD. The solid pieces print as shells with 2 perimeters (0.8 mm) and 0–5% infill.
- `fits_bed` searches any axis up and any yaw in 5° steps. STLs are exported in assembly pose, so orient them in the slicer.
- Tested: every piece is one valid solid that fits the bed; cutting conserves volume (adaptive integral, 0.1%); no piece intersects a spar or component; sleeves stay clear of the shells; the battery and computer lift straight out through the hatch.

## Packing (`pack.py`)

`field_sections(vtol)` groups every part into the sections that stay assembled on the trail; each must fit `PACK` = 700 × 350 × 260 mm, the usable inside of a 65 L pack (`fits_bed`, any orientation). The wing separates at `WING_JOINTS`, the boom at `BOOM_JOINT_X`, the skids and props pull off, and the rear motor unbolts.

| Section | Size (mm) | Holds |
| --- | --- | --- |
| `pod` | 691 × 282 × 206 | Fuselage forward of the boom joint, wing stubs, avionics, computer, turret, pitot tube (9 mm to spare in length) |
| `boom` | 680 × 322 × 62 | Boom with fin, rudder, tail nacelle, GPS |
| `tail` | 520 × 236 × 16 | Horizontal tail and elevators |
| `wing_inner_*` | 460 × 342 × 76 | Inner wing, motor pod, front motor (prop off) |
| `wing_outer_*` | 684 × 315 × 171 | Outer wing, winglet, aileron |
| `skid_*` | 340 × 198 × 20 | Strut and rail |
| `rear_motor`, `prop_*`, `joiner_*` | up to 305 long | Loose small parts |

Every section fits on its own, but together they take about 22 L of solid volume and pack badly: the pod, both inner wings and the winglets are bulky shapes. Plan on a dedicated pack or most of a 65 L pack, leaving little room for camping gear.

## Strength (`strength.py`)

Limit loads × `SAFETY_FACTOR` (1.5). Spars take all the bending (the printed skin is ignored) and no weight relief is credited, so wing moments are conservative.

- **Flight:** Limit load factor 5.1 g: a 10 m/s gust at the 20 m/s speed limit would give 6.7 g, but the wing stalls first (`STRUCTURE_CL_MAX` 1.4). Spanwise lift from the Schrenk approximation.
- **Landing:** 6.7 g (1.5 m/s stopped in 20 mm of skid flex), with 50% ground drag.
- **Allowables (MPa):** carbon bending 420 (0.7 × a supplier's 600 axial compression), tube wall under a joiner end 120 (estimate), printed skin 5, nylon fitting 40 (estimates).

| Check | Stress (MPa) of allowable | Margin |
| --- | --- | --- |
| Wing root tube at the center fitting | 310 / 420 | +36% |
| Wing joiner, inner joint | 302 / 420 | +39% |
| Spar wall at the inner joiner end | 84 / 120 | +42% |
| Spar just past the inner joiner | 165 / 420 | +155% |
| Wing joiner, outer joint | 109 / 420 | +286% |
| Spar wall at the outer joiner end | 44 / 120 | +174% |
| Printed skin past the spar end | 1.6 / 5 | +219% |
| Root tube, full hover thrust | 58 / 420 | +622% |
| Root tube, hard landing | 211 / 420 | +99% |
| Skid strut, ground drag | 233 / 420 | +80% |
| Skid rail, landing on its end | 279 / 420 | +51% |
| Spar torsion from the strut | 41 / 210 | +418% |
| Tail spar root | 85 / 420 | +392% |
| Fin spar root | 50 / 420 | +733% |
| Boom tube at the pod joint | 112 / 420 | +274% |
| Boom socket wall | 85 / 120 | +42% |

Not checked: flutter, wing twist, printed parts around the hinges and strut holes, retaining pins or screws at the joints. Test a joint to failure before flying.

## Renders (`render.py`)

`python -m trailhawk.render` exports each part with its paint color to `out/render/` (`cad.export_parts` at 0.2 mm and 0.1 rad, about 54 MB), then runs Blender (`BLENDER_DEFAULT`, the standard install path for Windows or macOS, else `blender` on the PATH; or the `BLENDER` environment variable) on `render_scene.py`: `studio.png` on a light grey backdrop with soft area lights (the camera sees the backdrop, but reflections see a dark room, so black parts stay black; the rim light sits high so the wings don't mirror it), and `aerial.png` banked over a procedural pine forest. Dark parts are satin with a light clear coat. Cycles, 96 samples with denoising, 1600 × 900.

## Model page (`site/index.html`, `site/cad.js`)

The website's front page: the VTOL as its CAD parts, to turn over in the browser, with a one-line description, its wingspan (from the meshes), weight (`mass`) and circling flight time (`loiter_minutes`, at 12,000 ft) from `site/data/aircraft.json`, a link to the simulator, and a GitHub icon (Hugeicons, MIT) linking to the repo. It loads `site/data/` through `aircraft.js`, which the simulator uses too: one mesh per part, smooth-shaded except across folds sharper than 40° (`CREASE`), each painted part tagged with the role (airframe, accents, details, skids) whose color it has (`paint` in `aircraft.json`).
- Drag turns it, right-drag pans, and the wheel or a pinch zooms toward the pointer. It turns slowly on its own until the first touch.
- Edges draws a light gray line along every fold sharper than `CREASE`, as a CAD viewer does (light so it shows on the black airframe): panel joins, seams, vents, trailing edges and part outlines.
- X-ray makes the painted shell see-through (15%), showing the spars, battery, computer, servos and the rest inside; the part under the pointer is then picked through the shell.
- Explode slides each part out from the middle to half its distance, or back, over 0.7 s.
- The part under the pointer (or a tapped one) is tinted blue and named from its path, e.g. "tilt servo · motor right hinge · hinges".
- The meshes are the simulator's (4.8 MB): every facet within 0.5 mm of the true surface, so the skins are smooth and the seam rings, built 0.3 mm proud of the surface, sit on them.
- Opened as a file rather than from a server, it says to serve the folder.

## Flight simulator (`trailhawk/sim.py`, `site/sim/`)

`python -m trailhawk.sim` serves `site/` on `http://localhost:8123` (with a `.js` MIME fix for Windows, and `Cache-Control: no-cache` so a reload picks up edits and new exports) and opens it; `--serve` only serves. `--export` first rewrites `site/data/`: `aircraft.json` and a mesh of every part (`cad.export_parts`: facets within 0.5 mm of the surface and turning at most 0.5 rad between neighbors; 68 parts, 4.8 MB), clearing the old meshes. `export_parts` meshes with an absolute deflection through OCP directly: build123d's `export_stl` takes it as a fraction of each edge's length, which left the big skins visibly faceted. The internal parts are drawn too, hidden inside the skin. `site/data/` is committed so `site/` can be hosted as static files. Exports aren't byte-identical from run to run (the CAD kernel's volume integrals and the boolean-cut panel lines vary in their last bits), so the server doesn't export by default; `test_sim.py` checks the committed `aircraft.json` against the model to 9 significant figures. The pages load three.js 0.160.0 from jsDelivr, so they need internet. It flies the design's mission by itself as a virtual flight test of the model's mass, inertia and aerodynamics; it is not a test of ArduPilot or its settings.

**Flight data (`aircraft(vtol)`), all in `DESIGN_AIR`:** mass, cruise CG and `balance.inertia`; the drag area `CD0·S` at the best-range speed; APC's `Ct`/`Cp` tables, the full-throttle rpm on a nominal 14.4 V pack and the motor test's efficiency curve; the usable battery energy; each motor's hinge, hub offset and spin; `min_speed` (1.3 × stall) and `max_speed` (`strength.MAX_SPEED`); skid and crash points; the paint (`PAINT`, by role); and for `sensors.js`, each camera's field of view across and the thermal camera's pixels (`parts.py`); and for the model page, the circling flight time (`loiter_minutes`). The wing (inner and outer panel each side), horizontal tail and fin are lifting panels at their mean-chord quarter points (`balance.trapezoid`): Helmbold lift slope from aspect ratio, NACA 2412 zero-lift −2.1° and Cm −0.05, `CL_MAX` 1.2 (tail and fin 1.0), dihedral in the panel normals, and control effectiveness from thin-airfoil flap theory times the span the surface covers. Winglets are left out.

**Flight model (`physics.js`):** a rigid body stepped at 400 Hz. Body axes are x forward, y right, z down; the world is north-east-down from the pad, with the ground at z = −`ground(north, east)` (`createModel(ac, ground)`; flat, z = 0, by default). Each motor follows its throttle with an 80 ms lag (below 1 rpm it has stopped) and makes `Ct(J)` thrust along its tilted axis, with the prop's reaction torque. Each panel sees its local flow (rotation included); the tail also sees the wing's downwash `2CL/(πAR)`. Past `cl_max` lift falls to a flat plate's over 8°, with separated drag `1.2 sin²α`. Induced drag uses e = 0.8, and parasite drag acts at the CG. Skids are spring-dampers with friction. Their spring pushes straight up against the depth below the terrain and friction acts horizontally, so the aircraft sits still on a slope: a friction law with no static range would creep downhill. A skid closing on the terrain faster than 3 m/s, measured along the slope's normal, is a crash. So is any crash point touching the ground (nose, camera ball, wing tips, tail tips, tail pod). On flat ground this is exactly the original flat-ground model: the flat-ground checks give bit-identical results. `newState` sits the aircraft as low as its skids allow, or starts it flying level at a given place, heading, speed and altitude. Telemetry gives `altitude` above the pad (like ArduPilot's, above home) and `height` above the ground below.

**Flight controller (`control.js`)**, a simplified ArduPilot tilt-tricopter flown by sticks (pitch, roll, yaw, throttle, each −1..1):

| Mode | What it does |
| --- | --- |
| `landed` | Motors stopped. Throttle up takes off. |
| `hover` (like QLOITER) | Pitch and roll set ground speed (6 m/s), throttle the climb rate (2.5 m/s, slowing to 0.7 m/s at touchdown by height above the ground), yaw the yaw rate. Holds altitude and heading. Lands after 0.5 s on the ground. |
| `transition` | Front motors tilt to 45° forward (`Q_TILT_MAX`) at 15°/s (`Q_TILT_RATE_UP`), holding altitude at 4° pitch, until `min_speed`. |
| `plane` (like FBWB) | Roll sets bank (45°, less near stall: a level turn may use at most 1/1.4 of the wing's maximum lift), pitch the climb rate (3 m/s), throttle the airspeed target, yaw the rudder. The front motors finish tilting forward while the rear motor balances their pitch moment, then it stops. |
| `back` | Motors tilt up at 45°/s; brakes at 2.5 m/s² with at most 6° nose-up, so it doesn't balloon; then `hover`. |

**Circling (like LOITER):** in `plane` the controller flies clockwise circles of `ORBIT_RADIUS` (75 m, from `performance.py`) around a point and climbs or descends at up to 3 m/s to `ORBIT_HEIGHT` above the ground there (100 m, from `sim.py`: under the 120 m / 400 ft drone ceiling), then holds that altitude. **Following:** given a moving `target` (the page passes the hiker's latest phone fix every step), the circle's center moves to it and the held altitude moves by the change in the ground's height under it, so the circle stays over him as he walks. With no target it circles a fixed point, as before. Over sloping ground the height above the ground below varies round the circle. The pitch stick still climbs and dives, and it holds the height where the stick centers, then follows the ground from there; the throttle keeps the airspeed. It steers for the circle's tangent, turned toward the circle (45° from one radius off), plus the steady bank of a 75 m turn once on it (18° at 15.5 m/s). On a fixed circle the bank holds steady; following the walking hiker it varies smoothly once a lap (about 18–28°), since it flies faster past him on one side than the other. Turning circling on in hover starts a transition first; turning it on again stops it.

**The page's flight (`missionStart`, `missionInput`):** it starts already circling: on the circle round the hiker's first phone fix, 100 m above the ground there, flying clockwise at `min_speed` with the controller trimmed for it. It follows his fixes to the lake and circles him there. Takeoff and transition aren't on the page; the trip test still flies them. `check.mjs` flies the same start. Starting in the air, it circles for 56 minutes in world 1 before the battery reaches its 20% reserve (111 W on average).

The multicopter mixer solves for the thrusts from each motor's full lift, roll and pitch per newton at its tilt, and cuts lift before attitude when a motor saturates. A tilting motor's thrust is capped at 1.5 sin(tilt) of full, so it doesn't push forward while still near horizontal. Yaw comes from tilting the front motors opposite ways (up to `YAW_TILT`). With airspeed (6 → 12 m/s) the control surfaces take over the attitude moments. Surface gains scale with dynamic pressure through each control's moment per radian per pascal (`controlPower`). The surfaces' roll-rate loop is stiff (`GAIN.planeRate`), because the wing's own roll damping is large: with a weak loop the rate integrator does the work, so banks lag and then overshoot, and circling rocked side to side every 10 s. A 45° bank settles in about 2 s, 2–3° over.

**The page (`app.js`, `index.html`):** no controls; it flies `missionInput` over a generated world (below), and `?seed=N` picks the world.
- The chase view fills the left three quarters, centered on the aircraft from 4.5 m behind its heading and 1.5 m up (and at least 0.5 m above the ground).
- The right quarter shows:
  - the mode;
  - the color camera's view: the turret's aim (`sensors.turret`) at the camera's 66° across, with the airframe hidden;
  - the readings: airspeed, height above the ground, distance from the hiker, power, battery, the air, and the hiker's progress;
  - the map (below), with what the cameras found under it, and a link back to the model page.
- In portrait the chase view sits on top, and below it the camera and readings sit beside the map.
- Both views are drawn on one canvas that fills the window, cleared to the panel's color, into the boxes of `#view` and `#camera` (viewport and scissor), at about 6 ms of script a frame.
- A message over the chase view shows "Loading the model…" until the page is ready, and while they apply: a crash, STALL, the battery's 20% reserve and the bear warning (6 s).

**Tested (`test_sim.py` through `node tests/check.mjs`):**
- The committed `site/data/aircraft.json` matches the model.
- Hover thrusts are within 0.4% of `hover_shares` (2% allowed); hover power is 532 W, matching `performance.hover` (3% allowed).
- The sticks move it the right way: in hover, pitch forward moves forward, roll right moves right, yaw right turns right and throttle up climbs; in plane, pitch forward dives, roll right turns right, yaw right turns right and throttle up speeds up.
- Level flight at 15.5 m/s in design air draws 96.7 W against `performance.cruise`'s 96.6 W (10% allowed).
- Circling a point 150 m away from level flight at 40 m climbs to and holds 100 m, settles on a 74.6 m circle (never more than 0.5 m off), clockwise, and draws 113 W against `mission`'s 113 W loiter power (10% allowed). Holding the pitch stick forward for 5 s while circling lowers the held height to 86 m, and it stays circling there.
- A whole trip works: take off to 39 m, transition in 4.5 s, fly on for 90 s (long enough for the stopped rear motor to spin all the way down), back-transition (peaking 7 m higher), hover and land, with no crash.
- On made-up terrain:
  - Landed on a 5° slope, it moves 1e-17 m in a minute (1 cm allowed).
  - It lands on that slope from 10 m up, and on a valley floor 30 m below the pad. Easing the descent by altitude above the pad instead of height above the ground would climb there.
  - Circling over ground 20 m up, it holds 120 m.
  - Flying level into a 30° ridge, it crashes ("hit the ground").
- Generated worlds (seeds 1, 2, 3, 0 and −7):
  - The same seed gives the same heights and trees.
  - The pad's clearing is flat: 0 m within 8 m.
  - Trails are 708–888 m long, at most 0.16 steep over 20 m, and end 1.7–2.0 m above the water.
  - Trees stand at least 5 m from the trail, 35 m from the pad and 1.5 m above the water.
  - Lakes cover 21,000–30,000 m², and the hills round the water disc stand 22–34 m above it.
- The page's own flight, in seeds 1–3: circling the hiker from the start, following him until a minute after he reaches the lake (a 9.5–11.6 minute walk).
  - It doesn't crash.
  - From a minute and a half in, it averages 75.6–75.9 m from where he really is, never more than 15 m off the circle.
  - It holds 100.4–100.6 m above the ground he stands on.
  - It stays 72–113 m above the ground below, under the 120 m ceiling.
  - A one-off run of seeds 1–20 gave the same: no crash, 65–113 m above the ground and at most 16 m off.
- The cameras (`sensors.js`), in this model: 60 looks at an elk 75 m out from 100 m up, as when circling.
  - It spans 2.46 thermal pixels; the thermal camera detects it in 40 looks and the color camera names it in all 60.
  - Under full forest: 16 and 5. Behind a ridge, or behind the aircraft: none.
  - ACQUIRE's curve gives even odds at the 50% point.
- What the cameras find on those follow flights (seeds 1–3). A one-off run with ten dice streams per seed gave the ranges.
  - The bear was named in all 30 runs, and the page's warning (`bearsAhead`) fired for it. Every warm rock flagged (120) was ruled out.
  - The lake is mapped, and 92–94% of the ground within 150 m of the trail is covered.
  - They name 10 of 12, 8 of 8 and 9–12 of 14 animals within 200 m of the trail, and 7–8 of 15 beyond.
  - No pin is for an animal that was never in a camera's view.

**Not modeled:** wind, prop wash and ground effect, pack voltage sag (thrust stays at the nominal pack's), air density with height (the air is 12,000 ft's everywhere), and servo and airframe flex. The gains are hand-tuned for this model, not ArduPilot's.

## World (`site/sim/world.js`)

`createWorld(seed)` generates the world the simulator flies over from a whole-number seed, the same every time, in about 0.3 s. The page reads `?seed=` from its address (1 without one). Points are metres north and east of the pad, heights metres above it. The physics, the page and `check.mjs` all use it, so what the aircraft touches is what is drawn.

- **Heights:** a 3 × 3 km square (±`SIZE` = 1,500 m) sampled every 5 m (601 × 601), with the pad the middle sample.
  - Rolling hills: value noise in 4 layers from 600 m, up to about ±40 m.
  - Mountains: ridged noise rising from 1,000 m out to full height (100–260 m) at 1,450 m. They close the view and hide the world's edge, and are low enough that sky shows above them from the circle.
  - A Gaussian basin 40 m deep with a 220 m radius holds the lake.
  - The pad sits in a flat clearing: flat within 12 m, eased back into the hills by 35 m.
- **Lake:** in the lowest of 24 spots 800–950 m from the pad, with water 8 m above the basin's bottom.
  - The water is a disc 400 m round the basin's center. The hills along its edge stand well above it (tested), so only the lake shows.
  - On the disc the water's surface is the ground, so the aircraft can sit on it. Elsewhere a hollow below the water's level stays dry.
- **Trail:** from the pad toward the lake, with a point every 2 m.
  - Its line is straight but bent by one wide arc (up to 250 m), an S (90 m) and wiggles (30 m), each fading out at both ends.
  - Its heights follow the ground averaged over 80 m and start level with the pad for 25 m. They are cut and filled to at most 15% per 2 m, one pass each way.
  - It ends where it would come within 1.5 m of the water.
  - The grid is graded to it: flat within 3 m of the centerline, easing back into the hills by 22 m. `trailDistance(north, east)` gives the distance to it, capped at 30 m.
- **Forest:** patches of noise (`forest(p)`, 0–1), giving 7,700–23,300 trees (seeds 1–20).
  - Where it is dense, a tree stands on up to 85% of the spots of a jittered 9 m grid, 10–22 m tall.
  - Trees grow below a treeline at 120 ± 25 m and on slopes under 0.7.
  - They stand at least 5 m from the trail, 35 m from the pad and 1.5 m above the water.
- **Ground (`ground(north, east)`):** flat across the two triangles of each cell, split along its south-west to north-east diagonal; past the edge, the edge's height. The page draws the grid with the same split.
- **The hiker (`hikerAt(seconds)`):** walks the trail from 10 m along it and stops at the end, 9–13 minutes later.
  - His speed is Tobler's hiking function for the grade: 6 exp(−3.5 |grade + 0.05|) km/h, 5.0 km/h on the flat.
  - It gives his position, heading, metres walked, speed and whether he has arrived.
  - The page starts his clock when the aircraft leaves the ground, and R resets it.
- **His phone (`createBeacon(seed)`):** a fix once a second, off by an error that wanders round him.
  - Each second the error keeps exp(−1/20) of itself (a 20 s time constant) and gains fresh Gaussian noise. Its spread is 3 m north and east, about a phone's GPS in the open.
  - Circling follows the fixes, not him.
- **Wildlife (`wildlifeAt(seconds)`):** from their own random stream, so they leave the rest of the world as it was.
  - Kinds (`WILDLIFE`, m): elk 2.4 long and 1.5 tall, deer 1.6 and 1.0, bear 1.7 and 1.0, and sun-warmed rocks 1.4 and 0.8, which a thermal camera can take for animals.
  - Placed along the hike:
    - three herds of 3–6 elk in the open 50–250 m from the trail;
    - up to five deer at the forest's edges 30–200 m from it;
    - a bear 40–120 m from it, past the first 40% of the trail;
    - four warm rocks in the open 30–200 m from it.
  - Each spot keeps the animal's wander clear of water and inside the mountains, on a slope under 0.4. A rule that finds no spot in 400 tries places nothing: over 100 random seeds there were 9–18 elk, 1–5 deer, 0–1 bear and 4 rocks.
  - Each animal wanders round its spot smoothly: elk up to 30 m (a herd together, each 4 m of its own), deer 20 m and the bear 40 m, at about 0.4 m/s, pausing now and then. Rocks stay put.
  - `along(metres)` gives the trail's point that far along his walk.

**Drawing (`app.js`):**
- **Ground mesh:** a vertex on every sample, colored by rules:
  - meadow in lush and dry patches (`meadow(p)`, 0–1: patches about 250 m across with 45 m ones in them), darker under forest;
  - tundra above 100–150 m;
  - worn brown within about 7 m of the trail;
  - muddy at the shore and dark under the water;
  - rock blending in on slopes from 37° to 53°;
  - snow above 170–220 m where it isn't too steep.
- **Grain:** keeps the ground sharp up close. `addGrain` multiplies the color by a generated, tiling noise texture (mean 0.5, ±16% typical) sampled in world position at four tile sizes: 0.8, 3.5, 15 and 70 m, each turned so the repeats don't line up. Mipmapping blends each size to its mean, no change, where it is finer than a screen pixel.
- **Trail:** a strip of dirt 1.4 m wide, with its points every metre laid 6 cm above `ground()`. It is lifted rather than depth-offset: the logarithmic depth buffer writes depth from the shader, so `polygonOffset` would do nothing.
- **Lake:** a slightly see-through Phong disc at the water's level.
- **Trees:** one instanced mesh of 7-sided cones (crowns from 20% of the height, 22% as wide as the tree is tall) and one of trunks, sunk 30 cm so slopes don't show under them.
- **Rocks:** 1,500 seeded rocks (one instanced mesh of lumpy icosahedra, mostly pebbles, a few up to about 1.4 m across, two-thirds buried) lie within 40 m of the pad. They stay at least 4 m from its center and 1.5 m from the trail.
- **Animals:** boxes and cylinders: elk with antlers, deer at two thirds of an elk, a bulky dark bear, and a boulder for each warm rock. Each faces the way it's wandering.
- Trees, rocks and animals are drawing only: the physics doesn't see them, and the aircraft flies through them.
- **Pad:** a disc of rings every 25 cm, each point laid 3 cm above `ground()`.
- **Hiker:** his legs swing a stride (1.5 m) per cycle as he walks.
- **Sky:** the scene's background is a gradient from blue overhead to haze at the horizon. The fog, from 0.4 to 4 km, is that haze, so the far mountains fade into it.
- The page uses a logarithmic depth buffer and an 8 km far plane.

## Cameras (`site/sim/sensors.js`)

A model of what the aircraft's cameras make of the world, not image processing: from each camera's resolution and the range, the pixels across a target; from those, the odds of detecting or recognizing it. The thresholds and canopy figures are assumptions, so its results hold "in this model".

**The cameras (`aircraft.json`, from `parts.py`):**
- FLIR Lepton 3.5 thermal camera: 160 × 120 pixels across 57°.
- Raspberry Pi Camera Module 3: 66° across. A detector reads its frames at 1920 × 1080 (`DETECTOR`; the camera takes 4608 × 2592).
- The turret only pans, so the cameras look 45° down (`TILT`). While circling, it pans toward the trail 150 m past the hiker (`LOOK_AHEAD`), sweeping ±45° either side of it (`SWEEP`) and back every 12 s (`SWEEP_SECONDS`), so its looks cover a wide strip ahead of him (held still on the trail, the cameras named under half as many animals); otherwise, the way the aircraft is heading (`turret`, which the page also draws the color camera's view from).

**A look (`look`), once a second of sim time once airborne,** through `survey`, which the page and `check.mjs` both run:
- **In view:** each animal or rock is checked against each camera's view, then for a line of sight. The ground is sampled every 10 m between them, short of the last 3 m.
- **Size:** pixels are counted across the target's mean size, √(length × height), as newer NVESD work uses in place of the smallest dimension. Line pairs (cycles) are half the pixels.
- **Odds:** each camera's per-look odds come from ACQUIRE's curve, P = (N/N50)^E / (1 + (N/N50)^E) with E = 2.7 + 0.7·N/N50, at Johnson's 50% points.
  - The thermal camera detects at N50 = 1 cycle, finding a warm shape it can't name. Rocks count half (`WARM_ROCK`).
  - The color camera recognizes at N50 = 4 cycles, naming it: elk, deer, bear or rock.
  - Under trees the odds fall by 70% (thermal) or 90% (color) times the forest's density there (`CANOPY`).
  - From 100 m up and 75 m out, an elk spans 2.5 thermal pixels (thermal detection about 0.67 a look) and 25 color ones. Thermal detection falls to even odds at about 150 m; color recognition at about 400 m.
- **Report:** each report puts the target where the camera saw it, off by 1 m plus 1% of the range. The lake counts as mapped once its center is in the color camera's view within 600 m.

**Sightings (`record`):** a pin per target, labelled warm until the color camera names it, at its latest reported place. Tracking is perfect: each animal keeps its own pin, where a real tracker would match reports by place. A rock the thermal camera flagged and the color camera named is a false alarm ruled out.

**Coverage (`cover`):** the ground in 10 m cells (`CELL`, `cellAt`), each marked once inside the color camera's view and closer than where it would name an elk at even odds (about 400 m). The ground's folds are ignored.

**The map (`app.js`):** in the panel, as wide as it is (600 canvas pixels, sharp on a 2× screen), drawn four times a second.
- It shows the trail and 250 m round it, in the ground's colors under a light from the north-west, with the lake blue.
- Ground the cameras haven't covered is darker.
- Over it: the trail, the circle being flown, you (a white dot), the aircraft (an orange triangle) and the pins. Pins are lettered: E elk, D deer, B bear (red), ? unnamed, × a rock ruled out, L the lake.
- A caption lists what was found, the false alarms ruled out, whether the lake is mapped, and the share of ground within 150 m of the trail covered (`nearTrail`, the same cells `check.mjs` scores).
- A bear named within 150 m of the trail ahead of you (`BEAR_RANGE`, `bearsAhead`) shows a warning once, for 6 s; after that its red pin stays on the map.

Sources for the criteria and curve: [Johnson's criteria](https://ipfs.ssi.eecc.de/ipns/en.wikipedia-on-ipfs.org/wiki/Johnson's_criteria), and ACQUIRE's curve as quoted in [US patent 7,149,652](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7149652) (checked 2026-11-18).

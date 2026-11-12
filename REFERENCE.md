# Reference

Parametric CAD models of two small fixed-wing drones (a conventional trainer and a tri-tiltrotor VTOL), written with [build123d](https://github.com/gumyr/build123d) (0.13) and viewed with [ocp_vscode](https://github.com/bernhard-42/vscode-ocp-cad-viewer). The VTOL is specified with real, buyable parts (`parts.py`) in ArduPilot's tilt-tricopter layout. Its mission is circling over a hiker on high trails while scanning for wildlife, so it is sized for 12,000 ft, comes apart to fit a backpack and carries thermal and color cameras with an onboard computer. It has weight-and-balance, power-and-endurance and strength models driven by those parts' data, a printable, bed-sized split of its airframe, photoreal renders, and a browser flight simulator flown from the keyboard.

## Files

| File | Purpose |
| --- | --- |
| `parts.py` | The real parts: name, link, mass, size and the specs the models use (motor test table, prop coefficients, servo torque, battery capacity). Single source for `vtol.py`, `balance.py` and `performance.py`. |
| `drone.py` | Conventional trainer (`build_drone()`) plus the shared helpers (`section`, `surface`, `span_plane`, `fin_plane`, `propeller`, `labeled`, `export_and_show`). Run it to export and display. |
| `vtol.py` | Tri-tiltrotor VTOL (`build_vtol()`), built from the `drone.py` helpers with part sizes from `parts.py`: airframe, moving parts located at their pivots, motor hinges, carbon spars and joiners, internal components, payload turret, paint and surface details. Run it to export and display. |
| `fly.py` | Viewer animation of the VTOL: takeoff, transition, banked lap, landing. |
| `balance.py` | Weight and balance: mass of every part, CG in cruise and hover, MAC, wing loading, stall speed, hover thrust split. Run it for a report. |
| `performance.py` | Power and flight time from the parts' data, in any `Air`: a propulsion check against the motor maker's test, hover and full-throttle thrust and current, tilt-servo load, cruise drag buildup and power curve, and takeoff-cruise-landing and circling missions. Run it for a report at 12,000 ft and sea level. |
| `strength.py` | Strength of the carbon structure: spars, joiners, center fitting, tail, boom and skids under gust, thrust and landing loads. Run it for a report. |
| `pack.py` | Packing list: the sections the airframe separates into on the trail, and whether each fits a backpack. Run it for a report. |
| `render.py`, `render_scene.py` | Exports every part with its paint color, then runs Blender headless to render `out/render/studio.png` and `aerial.png`. |
| `sim.py`, `sim/` | Browser flight simulator. `sim.py` exports the flight data and a light mesh to `out/sim/` and serves the project; `sim/physics.js` is the flight model, `sim/control.js` the flight controller, `sim/app.js` and `sim/index.html` the three.js view, keyboard and instruments, and `sim/check.mjs` flies it headless for the tests. |
| `print_parts.py` | Printable airframe: hollowed, trimmed, channeled and pocketed printed parts cut into pieces that fit a 256 mm bed. Run it to write `out/print/*.stl` and display the pieces. |
| `test_drone.py`, `test_vtol.py`, `test_balance.py`, `test_performance.py`, `test_print.py`, `test_pack.py`, `test_strength.py`, `test_sim.py` | `unittest` checks: solid validity, key dimensions, pivots, prop and tilt clearance, component and spar fit, the flight timeline, balance limits and tail volumes, the propulsion model against the motor test, thrust, current and servo margins at altitude, the energy budget, that print pieces fit the bed, conserve material and clear the hardware, that every field section fits the pack, that every strength check has a positive margin, and that the simulator matches the reports and flies a whole trip (needs Node.js, skipped without it). |
| `requirements.txt` | Python dependencies. |
| `.claude/launch.json` | Launch configs for the Claude desktop app's browser pane: `ocp-viewer` (the standalone viewer) and `sim` (`python sim.py --serve` on port 8123). |
| `out/` | Generated `<name>.step` (colored, labeled assembly) and `<name>.stl` for `drone` and `vtol`, plus `print/<piece>.stl`, `render/` and `sim/`. Not source. |

## Usage

```bash
pip install -r requirements.txt
python -m ocp_viewer        # standalone viewer (no VS Code): open http://localhost:3939/viewer
python drone.py             # writes out/drone.step, out/drone.stl, then sends the model to the viewer
python vtol.py              # same for out/vtol.step, out/vtol.stl
python fly.py               # shows the VTOL over a ground pad and starts the flight animation
python balance.py           # prints the weight and balance report
python performance.py       # prints the power and flight-time report
python print_parts.py       # writes out/print/<piece>.stl (48 pieces) and shows them
python pack.py              # prints the packing list
python strength.py          # prints the strength report
python render.py            # renders out/render/studio.png and aerial.png with Blender (set BLENDER if it is not in the default folder)
python sim.py               # flight simulator: exports out/sim/, serves and opens http://localhost:8123/sim/ (--serve skips the export)
python -m unittest -v       # runs all test_*.py (about 6 minutes)
```

`export_and_show(shape, name)` writes both files and calls `ocp_vscode.show` on port 3939, which is served by either the standalone viewer (`python -m ocp_viewer`) or the VS Code OCP CAD Viewer extension. If neither is running, `show()` prints a `CommsWarning` that it cannot reach the viewer; the exports still complete and the script exits 0. The viewer does not keep models across page reloads; re-run the script after reloading. If the animation loads paused, press ▶ in the viewer's animation bar (bottom right). The toolbar's transparency toggle shows the internal components.

## Conventions

- Units: millimetres.
- Axes: +X runs nose to tail, +Y is the right wing, +Z is up. The nose (motor face) is at `x = 0`; thrust points toward −X.
- `section(naca, chord, plane, pitch)` places a NACA 4-digit profile with its **quarter chord at the plane origin**, chord along the plane's local X, lift side along local Y, rotated nose-up by `pitch` degrees.
- `span_plane(x, y, z)` is used for horizontal surfaces (lofted along Y, local Y = +Z); `fin_plane(x, z)` for the fin (lofted along Z, local Y = +Y).
- Lifting surfaces are lofted between stations; the right half is built and mirrored about `Plane.XZ`, then fused.
- `span_plane(...).rotated((cant, 0, 0))` rolls a section about X by `cant` degrees (used for the VTOL winglets).
- `propeller(radius, pitch)` is built with its hub at the origin, spinning about X; callers place it with `Pos(...)`. Defaults are the trainer's 10×5 in.
- Bought parts take their size, mass and specs from `parts.py` (checked 2026-11-05, links in each entry). Anything not from a datasheet is marked as an estimate where it is defined.
- Assembly children's `location` is relative to their parent `Compound`. The viewer turns every node into a group with that location, and animation rotations pivot about the group origin. Code that needs world coordinates composes the parent locations (see `balance.leaves`).
- `pivot_at(solid, loc)` keeps a solid in place but makes its location exactly `loc`, baking the inverse transform into the geometry. build123d 0.13 has no `relocate`, and its `transform_shape` only rewrites the location, so the helper calls `BRepBuilderAPI_Transform(..., copy=True)` directly.
- `hinged(solid, hinge0, hinge1, gap=2)` cuts a control surface off the +X side of a lifting surface along the hinge line, leaving a 2 mm slot, and returns it pivoted on the hinge (origin `hinge0`, local X along the hinge).
- `wing_stations()`, `tail_stations()` and `fin_stations()` return the (plane, chord, pitch) stations that the surfaces, spars, servo positions and MAC are all derived from. `chord_point(station, fraction)` and `surface_point(a, b, s, fraction, camber)` locate points on them.
- **Volumes:** build123d's default `.volume` undercounts the lofted airfoil solids by up to ~25% (the wing reads 4.82e6 mm³; the adaptive integral and a mesh both give ≈ 6.5e6). Areas and centroids are accurate to < 1%. Where volume matters, use `BRepGProp.VolumeProperties_s(shape.wrapped, props, 1e-4)` as `test_print.volume` does. Zero-overlap checks are unaffected.
- **Booleans:** a cutting tool that is a `Compound` of overlapping solids makes OCCT booleans silently wrong; fuse tools first (`print_parts.union`). A long thin boolean (a whole-length 1.2 mm fuselage liner) came out invalid, so joiner sleeves are cut from a 16 mm band at each joint instead.

## Trainer (`drone.py`)

`build_drone()` returns a `Compound` labeled `drone` whose children are one valid solid each:

| Label | Geometry |
| --- | --- |
| `fuselage` | Smooth loft of 6 ellipses from `x = 0` to `x = 920`; max 112 × 128 mm; tail boom rises to centerline `z = 32`. |
| `wing` | NACA 2412, half-span 700 (span 1400), chord 240 → 150, quarter chord unswept at `x = 300`, root at `z = 35`, 3° dihedral, 2° root incidence, 2° washout (tip 0°). Area 0.273 m², AR 7.2, MAC 198 mm. |
| `horizontal_stabilizer` | NACA 0010, span 500, chord 140 → 100, quarter chord `x = 810 → 820`, `z = 30`, 0° incidence. |
| `vertical_stabilizer` | NACA 0010, `z = 30 → 210`, chord 160 → 80, quarter chord `x = 800 → 850`. |
| `motor` | Ø28 × 28 cylinder, `x = −28 → 0`. |
| `spinner` | Cone Ø32 base at `x = −30`, 36 long. |
| `propeller` | 2-blade 10×5 in (radius 127, pitch 127), NACA 4410 blades at `x = −38`; blade angle `atan(P / 2πr)`; Ø16 × 10 hub. |

Overall envelope: 986 × 1400 × 337 mm (X × Y × Z).

The wing and tail roots overlap the fuselage; parts are separate solids in the assembly, not fused to each other.

## VTOL (`vtol.py`)

Tri-tiltrotor in ArduPilot's tilt-tricopter layout, built in cruise configuration: the front motors (`TILTING`) face forward, the rear motor is fixed upright. `build_vtol()` returns a `Compound` labeled `vtol` (76 solids, envelope 1276 × 2325 × 546 mm). Paint: carbon-black airframe and tail nacelle (`CARBON`), orange front pods and control surfaces (`ORANGE`), black skids (`BLACK`). Dark reads as a silhouette against the sky from the ground; orange stands out against forest.

| Label | Geometry | Moves (in `fly.py`) |
| --- | --- | --- |
| `fuselage` | Half-ellipsoid nose (`x = 0 → 100`) fused to a loft of the 7 `FUSELAGE` ellipses out to `x = 1200`: deep pod (max 160 × 176 mm) tapering into a ~30 mm tail boom that rises to `z = 48`. Separates at `BOOM_JOINT_X` = 640. `fuselage(inset)` gives the cavity inside an `inset`-thick skin. The union is passed through `.fix()` to be BRep-valid. | — |
| `details` | Panel lines and vents standing 0.3 mm proud of the skin (`surface_details()`): seams at the hatch and boom joint, three cooling slots each side of the computer. | — |
| `wing` | NACA 2412, span 2200 mm to the winglet roots (2325 with winglets). Straight inner panel to `y = 340` (`WING_CRANK_Y`), chord 260 → 230, quarter chord at `x = 390` (`WING_X`); outer panel swept 12° to `y = 1100` (`WING_TIP_Y`), chord 230 → 110; then a 4-section winglet rolling up to ~88° and rising ~140 mm. 2° dihedral, root at `z = 60`, incidence 2° → −1°. Inboard of `y = 130` (`WING_BLEND_Y`) the root flares into the fuselage (`wing_blend`: chord to 315 mm, section to 16% thick). Separates at `WING_JOINTS` = ±140 and ±480. | — |
| `aileron_right`, `aileron_left` | Outer panel aft of 70% chord, from 35% to 92% of the outer panel span. | `rx` (hinge); same angle on both = roll |
| `horizontal_tail` | NACA 0010, span 520, chord 160 → 100, quarter chord swept 25° from `x = 1080`, `z = 46`. Bolts on as one piece. | — |
| `elevator_right`, `elevator_left` | Aft of 65% chord, 6% → 98% span. | `rx`; left takes the negated angle |
| `fin` | NACA 0010, `z = 40 → 300` (`FIN_TIP_Z`), chord 200 → 110, quarter chord `x = 1050 → 1120`, with a dorsal fillet ahead of the hinge line blending its root into the boom. | — |
| `rudder` | Aft of 65% chord, 11% → 86% of fin height (clear of boom and tip pod). | `rx` |
| `nacelles` | The 2 front pods: fixed circular-section pods running aft from each tilting motor's nose point (`MOTORS`), under the wing at `y = ±245` (`POD_Y`, far enough out to keep `PROP_GAP` at the prop tips), `x = 200 → 580`, axis `z = 48`. `nacelle(nose, length, inset)` gives the cavity. | — |
| `tail_nacelle` | The same pod on the fin tip under the rear motor, `x = 1050 → 1230`, `z = 300`; holds `esc_tail`. A separate leaf so it can be painted black. | — |
| `motor_right`, `motor_left`, `motor_tail` | Assemblies located at their hinge `hinge(nose)` = nose + `HINGE_UP` (31) in Z, directly above the nacelle front face; the rear one is fixed at 90° (upright) and unbolts for packing. Children: `mount` (back plate, tab and knuckle bored for the pin); `motor`, the AT2820 (Ø35.2 can, `CAN_LENGTH` 43.5, plus a Ø5 shaft `SHAFT_OUT` 20 — the datasheet gives only the 63.5 mm total, so the split is an estimate), `MOTOR_GAP` (12) ahead of the hinge on the nacelle axis; `prop_nut`; `propeller`, 12×6 in, hub at local `(HUB_X, 0, −HINGE_UP)`. A front motor stays ahead of its nacelle face through the whole swing. | Front: `ry` tilt 0 = cruise, 90 = hover, up to 90 + `YAW_TILT` (10) for yaw; `propeller` spins `rx` |
| `hinges` | Fixed half of each hinge (`<motor>_hinge`, located at the hinge), set into the top of the nacelle front. Front: `tilt_servo` (KST X12-708, 12 × 27.5 × 23, direct drive: its output spline is the hinge axis and drives the knuckle; spline position assumed 6 mm from the top), `bearing` lug on the other side, Ø3 `hinge_pin`. Rear: a lug each side and the pin bolting the mount upright. | — |
| `spars` | Carbon tubes, 1 mm wall (`spar_lines()`). Each wing spar is one straight line (`spar_point(y)`, in plan swept 8°, along the airfoil's mean line) so straight joiners fit it: `wing_root_tube_*` (`y = 5 → 140`, Ø16), `wing_spar_inner_*` (`140 → 480`, Ø16), `wing_spar_outer_*` (`480 → 850`, Ø16, stops at `SPAR_END_Y`). Loose joiners: `wing_joiner_inner_*` Ø14 × 240 at `y = 140`, `wing_joiner_outer_*` Ø14 × 200 at `y = 480` (`JOINER_EMBED`). `boom_spar` Ø16 from `x = 550` (`BOOM_SOCKET_START`) to 1030, in a `boom_socket` Ø18 in the pod. `tail_spar_*` Ø6 and `fin_spar` Ø8 unchanged. | — |
| `center_fitting` | Printed block inside the fuselage, 44 × 100 × 30, bonding the two root tubes where the straight spars meet in a shallow V (`center_fitting()`). | — |
| `avionics` | Boxes of the real parts' sizes (`components()`: label → host, center, size): `battery` (Flywoo 4S2P Li-ion, 145 × 45 × 45 estimated, `BATTERY_CENTER` = (227, 0, 0), the CG trim), `computer` (Pi 5 + AI HAT+, `COMPUTER_CENTER`, nose), `flight_controller`, `receiver`, `gps` (boom, `x = 700`, above the boom tube), `esc_*`, `aileron_servo_*`, `elevator_servo_*`, `rudder_servo`. Each sits inside its host with at least `SKIN` to spare, and none touches another. | — |
| `landing_gear` | Two faired skids at `y = ±POD_Y`: a rail (ruled loft, Ø20, `RAIL_X` = 280 → 620, `z = SKID_Z = −125`) on one NACA 0025 strut, 60 mm chord, that runs up through the pod to just under the wing spar. Lowest point `GROUND_Z = −135`. | — |
| `gimbal_mount` | Short stem inside the nose above the ball. | — |
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

Masses: printed parts (fuselage, wing, tail, fin, nacelles, tail nacelle, control surfaces) weigh `area × SKIN × PRINT_DENSITY` (0.8 mm of foamed LW-PLA at 0.6 g/cm³); spars weigh their tube volume × 1.55 g/cm³; bought parts weigh their published mass from `parts.py`; the rest (prop nut, mount, lugs, pin, landing gear, gimbal mount, center fitting, turret) are estimates in `GRAMS`; a 130 g `wiring` allowance (wires, the connectors at the dry joints, fasteners) sits at the flight controller. `mass_items(vtol, tilt)` walks the assembly composing parent locations, with the front motors tilted.

Current report:

| Quantity | Value |
| --- | --- |
| All-up mass | 2890 g (battery 574, wing 421, motors 417, fuselage 153, wiring 130, …) |
| CG, cruise | x 437.4, y 0.2, z 63.8 mm = 27.0% MAC (`CG_RANGE` 25–30%) |
| CG, hover | x 438.4, z 72.4 mm |
| MAC | 203.8 mm, leading edge at x 382.4 (trapezoid panels, winglets excluded) |
| Wing area / loading | 0.425 m², 68.0 g/dm² |
| Stall speed (CL 1.2) | 9.5 m/s at sea level, 11.8 m/s at 12,000 ft |
| Tail volumes | horizontal 0.55 (`TAIL_VOLUME_RANGE` 0.4–0.7), vertical 0.028 (`FIN_VOLUME_RANGE` 0.02–0.05) |
| Hover thrust split | right 34.2%, left 34.1%, rear 31.7% (≈ 989 / 916 g) |

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
| Hover, with electronics | 531 W, 37 A at 14.4 V | 433 W, 30 A |
| Hover only | 11.7 min | 14.4 min |
| Full throttle at 12.8 V | 1619 g per motor (needs 1582 g for `THRUST_MARGIN` 1.6), 24.7 A | 2120 g, 32.3 A |
| CD0 | 0.049 | 0.047 |
| Slowest cruise (1.3 × stall) | 15.5 m/s, L/D 10.6, 96 + 14.5 W | 12.5 m/s, L/D 10.9, 77 + 14.5 W |
| Circling 75 m radius | 113 W, **48 min** | 92 W, **61 min** |
| Cruising | 49 min, 46 km | 61 min, 46 km |

Hover limit (1.6× on a sagged pack): air down to 0.766 kg/m³, that is 12,000 ft up to 20 °C, 13,000 ft up to 9 °C, 14,000 ft up to −2 °C (10,000 ft up to 44 °C). Heaviest takeoff at 12,000 ft, 10 °C: 2959 g, 69 g over the current 2890 g.

- CD0 breakdown: wing 43%, rear prop 10%, fuselage 9%, tail 7%, landing gear 7%, rear motor 7%, fin 5%, nacelles 5%, camera 4%, tilt servos 3%.
- Propulsion check against T-Motor's measurements is unchanged: within 7% up to 60% throttle (tested).
- Tilt servo: 8.3 kgf·cm at 7.2 V vs 5.5 kgf·cm load, 1.5× (`SERVO_MARGIN` 1.5). No spare.
- The best-endurance speed is pinned at the 1.3 × stall floor: the wing is working well below its best L/D (12 at CL ≈ 1.2). Treat drag-side numbers as ±20–30%; the skin-friction estimate is on the high side at these Reynolds numbers, so real flight times are more likely to be above these than below them.

## Parts list

Checked 2026-11-05 (battery and payload 2026-11-09); links are in `parts.py`. Prices and stock change.

| Qty | Part | Mass | Notes |
| --- | --- | --- | --- |
| 3 | T-Motor AT2820 Long Shaft KV880 | 139 g | 3–4S, 45 A / 700 W for 180 s |
| 2 + 1 | APC 12x6E (LP12060E) + 12x6EP (LP12060EP, reverse) | 27 g | 6.35 mm bore with adapter rings; a 5 mm ring for the motor shaft is unconfirmed. Buy spares |
| 3 | Hobbywing XRotor Pro 50A | 56 g | 4–6S, 50 A continuous; PWM (OneShot unsupported) |
| 1 | Flywoo Explorer 4S2P 9000 mAh Li-ion (Molicel P45B), XT60 | 573.5 g | Dimensions not published; 145 × 45 × 45 mm assumed from the cells. Listing says 45 A continuous, which reads as the per-cell figure (2 × 45 A = 90 A for the pack): confirm before relying on it |
| 1 | Raspberry Pi 5 with AI HAT+ (13 TOPS) and active cooler | ≈ 82 g | Onboard detection. Board mass and loaded power unpublished, so estimated |
| 1 | Teledyne FLIR Lepton 3.5 (160 × 120, 57° HFOV) on a GroupGets PureThermal 3 USB board | ≈ 6 g | Thermal camera, $164 + $120 for the board. Resolves ≈ 0.75 m per pixel at 120 m: a warm animal is a 1–2 pixel hot spot that points the color camera at it. Mass and power estimated |
| 1 | Raspberry Pi Camera Module 3 | 4 g | Color camera, 66° HFOV |
| 1 | Holybro Kakute H743-Wing | 28 g | ArduPilot `KakuteH7-Wing`, 14 PWM outputs (10 used), 90 A current sensor |
| 1 | Matek M10Q-5883 GPS and compass | 8 g | Beware counterfeits |
| 1 | RadioMaster RP3 V2 ExpressLRS | 4.6 g | |
| 2 | KST X12-708 tilt servo | 20 g | 6.3 / 8.6 / 9.3 kgf·cm at 6 / 7.4 / 8.4 V; needs ≥ 6 V; often out of stock |
| 2 | KST X08 Plus aileron servo | 9 g | 8 mm thick |
| 3 | KST X06 V6.0 tail servo | 6 g | 7 mm thick |
| — | Roll-wrapped carbon tubes, 1 mm wall: Ø16 × 14 (≈ 2.2 m), Ø18 × 16 (≈ 0.1 m), Ø14 × 12 (≈ 0.9 m joiners), Ø8 × 6 (≈ 0.3 m), Ø6 × 4 (≈ 0.4 m), Ø10 × 8 (skid struts), Ø12 × 10 (skid rails); Ø3 pins | — | Generic. The joiners need a close sliding fit in the Ø14 bores |

Not chosen: wiring and connectors (130 g allowance), the turret's pan servo and bearing (60 g estimate), the skid tubes and fairings (90 g estimate).

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
- **Hover thrust at altitude:** 1619 g per motor at 12,000 ft, 10 °C on a sagged pack against 1582 g needed: the 1.6× margin passes with 2% to spare. Treat the hover limit (`performance.py`) as an operating limit: at 12,000 ft no hotter than 20 °C, at 13,000 ft no hotter than 9 °C, and no more than 69 g of extra payload. More margin needs a bigger motor.
- **Battery current:** all three motors at full throttle draw 97 A at sea level, over the pack's 90 A and the flight controller's 90 A continuous sensor (220 A peak), so treat it as a short burst. Hover is 30–37 A. If the listing's 45 A is the real pack limit, hover at altitude is close to it. Check the XT60 and main leads too.
- **Thermal resolution:** the Lepton puts about one pixel on a bear at 120 m (two at 60 m), so it finds warm spots rather than identifying animals; the color camera (≈ 3 cm per pixel at 120 m) does the identifying in daylight. The upgrade is a FLIR Boson 320 with the 6.3 mm lens ($2,131, ≈ 0.23 m per pixel), which fits the same turret.
- **Heat on dark parts:** foamed LW-PLA softens at about 55 °C, which a black skin can reach sitting in high-altitude sun. Print the carbon-black parts in colorFabb LW-ASA (Vicat 96 °C, sold in black; about 0.43 g/cm³ foamed at 40% flow, so the 0.6 g/cm³ `PRINT_DENSITY` stays conservative). `PRINTED_SKIN` was estimated for LW-PLA; check it holds for LW-ASA. The tail nacelle (holding `esc_tail`) is black too; its print piece is one of the `nacelles` pieces, so print that one in LW-ASA. Park in shade, since the computer and battery sit inside the black pod.
- **The joints are untested designs:** a dry fit of carbon joiners and a printed center fitting. The strength margins depend on the wall-bearing allowable, which is an estimate.

## Printable airframe (`print_parts.py`)

`printed_parts(vtol)` prepares each printed part, then `print_pieces(vtol)` cuts each separate solid into the fewest equal slabs across one axis that each fit `PRINT_BED` = 256 × 256 × 256 mm, giving 48 pieces. The wing and fuselage are first cut at the dry joints (`WING_JOINTS`, `BOOM_JOINT_X`), which are not glued.

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
| `pod` | 641 × 282 × 206 | Fuselage forward of the boom joint, wing stubs, avionics, computer, turret |
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

`python render.py` exports each part with its paint color to `out/render/`, then runs Blender (`BLENDER_DEFAULT`, or the `BLENDER` environment variable) on `render_scene.py`: `studio.png` on a light grey backdrop with soft area lights (the camera sees the backdrop, but reflections see a dark room, so black parts stay black; the rim light sits high so the wings don't mirror it), and `aerial.png` banked over a procedural pine forest. Dark parts are satin with a light clear coat. Cycles, 96 samples with denoising, 1600 × 900.

## Flight simulator (`sim.py`, `sim/`)

`python sim.py` writes `out/sim/aircraft.json` and a light mesh of the model (`render.export_parts` at 0.3 mm tolerance, internal parts left out), serves the project on `http://localhost:8123` (with a `.js` MIME fix for Windows, and `Cache-Control: no-cache` so a reload picks up edits and new exports) and opens `/sim/`. `python sim.py --serve` only serves. The page loads three.js 0.160.0 from jsDelivr, so it needs internet. It is a handling toy for getting a feel for the aircraft, not a test of ArduPilot or its settings.

**Flight data (`aircraft(vtol)`), all in `DESIGN_AIR`:** mass, cruise CG and `balance.inertia`; the drag area `CD0·S` at the best-range speed; APC's `Ct`/`Cp` tables, the full-throttle rpm on a nominal 14.4 V pack and the motor test's efficiency curve; the usable battery energy; each motor's hinge, hub offset and spin; `min_speed` (1.3 × stall) and `max_speed` (`strength.MAX_SPEED`); skid and crash points. The wing (inner and outer panel each side), horizontal tail and fin are lifting panels at their mean-chord quarter points: Helmbold lift slope from aspect ratio, NACA 2412 zero-lift −2.1° and Cm −0.05, `CL_MAX` 1.2 (tail and fin 1.0), dihedral in the panel normals, and control effectiveness from thin-airfoil flap theory times the span the surface covers. Winglets are left out.

**Flight model (`physics.js`):** a rigid body stepped at 400 Hz. Body axes are x forward, y right, z down; the world is north-east-down with the ground at z = 0. Each motor follows its throttle with an 80 ms lag (below 1 rpm it has stopped) and makes `Ct(J)` thrust along its tilted axis, with the prop's reaction torque. Each panel sees its local flow (rotation included); the tail also sees the wing's downwash `2CL/(πAR)`. Past `cl_max` lift falls to a flat plate's over 8°, with separated drag `1.2 sin²α`. Induced drag uses e = 0.8, and parasite drag acts at the CG. Skids are spring-dampers with friction. A skid arriving faster than 3 m/s, or any crash point touching the ground (nose, camera ball, wing tips, tail tips, tail pod), is a crash.

**Flight controller (`control.js`)**, a simplified ArduPilot tilt-tricopter:

| Mode | What it does |
| --- | --- |
| `landed` | Motors stopped. W takes off. |
| `hover` (like QLOITER) | Arrows set ground speed (6 m/s), W/S the climb rate (2.5 m/s, slowing to 0.7 m/s at touchdown), A/D the yaw rate. Holds altitude and heading. Lands after 0.5 s on the ground. |
| `transition` | Front motors tilt to 45° forward (`Q_TILT_MAX`) at 15°/s (`Q_TILT_RATE_UP`), holding altitude at 4° pitch, until `min_speed`. |
| `plane` (like FBWB) | Arrows set bank (45°, less near stall: a level turn may use at most 1/1.4 of the wing's maximum lift) and climb rate (3 m/s), W/S the airspeed target, A/D the rudder. The front motors finish tilting forward while the rear motor balances their pitch moment, then it stops. |
| `back` | Motors tilt up at 45°/s; brakes at 2.5 m/s² with at most 6° nose-up, so it doesn't balloon; then `hover`. |

**Circling (O, like LOITER):** in `plane` the controller flies clockwise circles of `ORBIT_RADIUS` (75 m, from `performance.py`) around the hiker and climbs or descends at up to 3 m/s to `ORBIT_HEIGHT` (100 m above the hiker, from `sim.py`: under the 120 m / 400 ft drone ceiling), then holds it. ↑/↓ still dive and climb, and it holds the height where you let go; W/S keep the airspeed. It steers for the circle's tangent, turned toward the circle (45° from one radius off), plus the steady bank of a 75 m turn once on it (18° at 15.5 m/s). Pressing O in hover starts a transition first; T or O again stops it.

The multicopter mixer solves for the thrusts from each motor's full lift, roll and pitch per newton at its tilt, and cuts lift before attitude when a motor saturates. A tilting motor's thrust is capped at 1.5 sin(tilt) of full, so it doesn't push forward while still near horizontal. Yaw comes from tilting the front motors opposite ways (up to `YAW_TILT`). With airspeed (6 → 12 m/s) the control surfaces take over the attitude moments. Surface gains scale with dynamic pressure through each control's moment per radian per pascal (`controlPower`). T toggles the transition. C switches between the chase camera and the hiker's eyes (you stand 15 m south and 6 m west of the pad), R restarts, P pauses and H hides the key help. The world is flat grass (a mipmapped, low-contrast texture, steady into the distance), the pad and the hiker. The grass and then the pad are drawn first and write no depth, so the pad, lying on the grass, can't flicker against it. The instruments show mode, airspeed, altitude, climb, heading, throttle, tilt, battery power and charge left.

**Tested (`test_sim.py` through `node sim/check.mjs`):**
- Hover thrusts are within 0.4% of `hover_shares` (2% allowed); hover power is 531 W, matching `performance.hover` (3% allowed).
- Every key does what the help says: in hover ↑ moves forward, → moves right, D yaws right and W climbs; in plane ↑ dives, → turns right, D yaws right and W speeds up.
- Level flight at 15.5 m/s in design air draws 96.6 W against `performance.cruise`'s 96.5 W (10% allowed).
- Circling a point 150 m away from level flight at 40 m climbs to and holds 100 m, settles on a 74.6 m circle (never more than 1.3 m off), clockwise, and draws 113 W against `mission`'s 113 W loiter power (10% allowed). Holding ↑ for 5 s while circling lowers the held height to 86 m, and it stays circling there.
- A whole trip works: take off to 39 m, transition in 4.5 s, fly on for 90 s (long enough for the stopped rear motor to spin all the way down), back-transition (peaking 7 m higher), hover and land, with no crash.

**Not modeled:** wind, prop wash and ground effect, pack voltage sag (thrust stays at the nominal pack's), terrain, and servo and airframe flex. The gains are hand-tuned for this model, not ArduPilot's.


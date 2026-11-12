"""Browser flight simulator for the VTOL.

`python sim.py` writes the flight data (out/sim/aircraft.json) and a light mesh of the model
(out/sim/*.stl) and serves the project at http://localhost:8123/sim/, opening it in the browser.
`python sim.py --serve` only serves, reusing the last export.

The flight data comes from the same models as the reports, all in DESIGN_AIR: mass, CG and inertia
(balance.py), propulsion and drag (performance.py), and wing, tail and fin panels (vtol.py).
sim/physics.js flies it and sim/control.js is a simplified ArduPilot-style flight controller. It is
a handling toy for getting a feel for the aircraft, not a test of ArduPilot or its settings.
"""

import json
import sys
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from math import acos, pi, sin, sqrt
from pathlib import Path

from balance import CL_MAX, DESIGN_AIR, G, center_of_gravity, inertia, mass_items, stall_speed
from drone import OUT_DIR
from performance import (
    ADVANCE, DIAMETER, ELECTRONICS_WATTS, ORBIT_RADIUS, OSWALD, POWER_COEFFICIENT, SPEED_MARGIN, THRUST_COEFFICIENT, max_static,
    mission, motor_test, parasite_drag_area, rpm_for_thrust, wing_reference,
)  # fmt: skip
from parts import BATTERY
from render import export_parts
from strength import MAX_SPEED
from vtol import (
    CAMERA_RADIUS, FIN_TIP_Z, GIMBAL_CENTER, GROUND_Z, HINGE_UP, HUB_X, MOTORS, POD_Y, RAIL_X, TILTING, YAW_TILT,
    build_vtol, chord_point, fin_stations, hinge, lerp, tail_stations, wing_stations,
)  # fmt: skip

ROOT = Path(__file__).parent
FOLDER = OUT_DIR / "sim"
PORT = 8123
HIDDEN = ("/spars/", "/avionics/", "/center_fitting")  # inside the skin: not drawn
AIRFOILS = {"2412": (-2.1, -0.05), "0010": (0.0, 0.0)}  # zero-lift angle (deg), quarter-chord moment coefficient
TAIL_CL_MAX = 1.0  # thin symmetric tail sections (estimate)
ORBIT_HEIGHT = 100  # m above the hiker that circling climbs to: under the 120 m (400 ft) drone ceiling


def lift_slope(aspect):
    """Lift slope (per rad) of a finite wing, Helmbold's formula."""
    return 2 * pi * aspect / (2 + sqrt(aspect**2 + 4))


def flap_effectiveness(chord_fraction):
    """Angle-of-attack change per unit deflection of a plain flap (thin-airfoil theory)."""
    theta = acos(2 * chord_fraction - 1)
    return 1 - (theta - sin(theta)) / pi


def panel(name, a, b, airfoil, aspect, cl_max, mirror=False, control=None, span=(0, 0), hinge_at=1.0, downwash=False):
    """Trapezoid panel between stations a and b: quarter-chord point and incidence at its mean
    aerodynamic chord, lift normal (tilted by dihedral), area and lift data. `mirror` gives the
    left side. A `control` surface runs over the `span` fraction of the panel, aft of `hinge_at`."""
    taper = b[1] / a[1]
    along = (1 + 2 * taper) / (3 * (1 + taper))  # spanwise position of the mean chord
    center = lerp(chord_point(a, 0.25), chord_point(b, 0.25), along)
    spanwise = (chord_point(b, 0.25) - chord_point(a, 0.25)).normalized()
    normal = a[0].x_dir.cross(spanwise).normalized()
    normal = normal if normal.dot(a[0].y_dir) > 0 else -normal
    height = abs((b[0].origin - a[0].origin).dot(a[0].z_dir))
    flip = -1 if mirror else 1
    zero_lift, moment = AIRFOILS[airfoil]
    tau = flap_effectiveness(1 - hinge_at) * (span[1] - span[0]) if control else 0
    return {
        "name": name,
        "center": [center.X, flip * center.Y, center.Z],  # CAD mm
        "normal": [normal.X, flip * normal.Y, normal.Z],
        "area": (a[1] + b[1]) / 2 * height / 1e6,  # m^2
        "chord": 2 / 3 * a[1] * (1 + taper + taper**2) / (1 + taper) / 1000,  # m
        "incidence": lerp(a[2], b[2], along),  # deg
        "zero_lift": zero_lift,
        "moment": moment,
        "slope": lift_slope(aspect),
        "cl_max": cl_max,
        "aspect": aspect,
        "control": control,
        "tau": tau * (-1 if mirror and control == "aileron" else 1),  # ailerons deflect opposite ways
        "downwash": downwash,
    }


def surfaces():
    """Wing, horizontal tail and fin as lifting panels. Winglets are left out."""
    _, wing_aspect = wing_reference()
    root, crank, tip = wing_stations()
    tail_root, tail_tip = tail_stations()
    tail_span = 2 * (tail_tip[0].origin.Y - tail_root[0].origin.Y)
    tail_aspect = tail_span**2 / ((tail_root[1] + tail_tip[1]) * tail_span / 2)
    bottom, top = fin_stations()
    fin_height = top[0].origin.Z - bottom[0].origin.Z
    fin_aspect = fin_height**2 / ((bottom[1] + top[1]) / 2 * fin_height)
    result = []
    for mirror, side in ((False, "right"), (True, "left")):
        result += [
            panel(f"wing_inner_{side}", root, crank, "2412", wing_aspect, CL_MAX, mirror),
            panel(f"wing_outer_{side}", crank, tip, "2412", wing_aspect, CL_MAX, mirror, "aileron", (0.35, 0.92), 0.7),
            panel(f"tail_{side}", tail_root, tail_tip, "0010", tail_aspect, TAIL_CL_MAX, mirror, "elevator", (0.06, 0.98), 0.65, downwash=True),
        ]  # fmt: skip
    return result + [panel("fin", bottom, top, "0010", fin_aspect, TAIL_CL_MAX, control="rudder", span=(0.11, 0.86), hinge_at=0.65)]


def contact_points(vtol):
    """(skids, crash points) in CAD mm: the skid rail ends, and points that must never touch the
    ground: nose, camera ball, wing tips, tail tips and the tail pod."""
    skids = [[x, y, GROUND_Z] for x in RAIL_X for y in (POD_Y, -POD_Y)]
    tip = chord_point(wing_stations()[-1], 0.25)
    tail_tip = chord_point(tail_stations()[-1], 0.25)
    box = vtol.bounding_box()
    crash = [
        [box.min.X, 0, 0],
        [GIMBAL_CENTER[0], 0, GIMBAL_CENTER[2] - CAMERA_RADIUS],
        *([tip.X, y, tip.Z] for y in (tip.Y, -tip.Y)),
        *([tail_tip.X, y, tail_tip.Z] for y in (tail_tip.Y, -tail_tip.Y)),
        [box.max.X, 0, FIN_TIP_Z],
    ]
    return skids, crash


def aircraft(vtol):
    """Everything sim/physics.js needs, as plain data (SI units unless noted)."""
    items = mass_items(vtol)
    grams = sum(g for _, g, _ in items)
    stall = stall_speed(grams, DESIGN_AIR)
    results = mission(vtol, DESIGN_AIR)
    max_thrust, _ = max_static(BATTERY["cells"] * BATTERY["cell_volts"], DESIGN_AIR)
    _, _, _, battery_watts, _, shaft_watts, _ = motor_test()
    skids, crash = contact_points(vtol)
    flip = [-1, 1, -1]  # CAD axes (X aft, Y right, Z up) -> body axes (x forward, y right, z down)
    tensor = inertia(vtol)
    return {
        "air": "12,000 ft, 10 C",
        "density": DESIGN_AIR.density,
        "gravity": G,
        "mass": grams / 1000,
        "cg": list(center_of_gravity(items)),  # CAD mm
        "inertia": [[flip[i] * flip[j] * tensor[i][j] for j in range(3)] for i in range(3)],  # body axes
        "drag_area": parasite_drag_area(vtol, results["range_speed"], DESIGN_AIR)[0],  # m^2
        "oswald": OSWALD,
        "stall_speed": stall,
        "min_speed": SPEED_MARGIN * stall,
        "max_speed": MAX_SPEED,
        "orbit_radius": ORBIT_RADIUS,  # m, circling the hiker
        "orbit_height": ORBIT_HEIGHT,  # m
        "prop": {
            "diameter": DIAMETER,
            "advance": ADVANCE.tolist(),
            "thrust": THRUST_COEFFICIENT.tolist(),
            "power": POWER_COEFFICIENT.tolist(),
            "max_rpm": rpm_for_thrust(max_thrust, 0, DESIGN_AIR),  # full throttle on a nominal battery
        },
        "efficiency": {"shaft": shaft_watts.tolist(), "ratio": (shaft_watts / battery_watts).tolist()},
        "electronics_watts": ELECTRONICS_WATTS,
        "battery_wh": results["energy"] / 3600,  # usable
        "motors": [
            {
                "name": name,
                "hinge": list(hinge(nose)),  # CAD mm
                "hub": [HUB_X, 0, -HINGE_UP],  # from the hinge at tilt 0
                "tilting": name in TILTING,
                "mesh_tilt": 0 if name in TILTING else 90,  # pose of the exported mesh
                "spin": -1 if name == "motor_left" else 1,  # the left prop turns the other way
            }
            for name, (nose, _) in MOTORS.items()
        ],
        "yaw_tilt": YAW_TILT,
        "surfaces": surfaces(),
        "skids": skids,
        "crash_points": crash,
    }


def export():
    vtol = build_vtol()
    export_parts(vtol, FOLDER, tolerance=0.3, hidden=HIDDEN)
    (FOLDER / "aircraft.json").write_text(json.dumps(aircraft(vtol), indent=1))


class Handler(SimpleHTTPRequestHandler):
    # Windows' registry can map .js to text/plain, which browsers refuse for modules
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript", ".json": "application/json"}

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")  # recheck on every load, so edits and new exports show
        super().end_headers()


def serve(open_browser):
    url = f"http://localhost:{PORT}/sim/"
    server = ThreadingHTTPServer(("127.0.0.1", PORT), partial(Handler, directory=str(ROOT)))
    print(f"Flight simulator at {url} (Ctrl+C to stop)")
    if open_browser:
        webbrowser.open(url)
    server.serve_forever()


if __name__ == "__main__":
    serve_only = "--serve" in sys.argv
    if not serve_only:
        export()
    serve(open_browser=not serve_only)

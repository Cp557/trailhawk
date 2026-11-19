"""The website's data and local server: the model page and the browser flight simulator.

`python -m trailhawk.sim` serves site/ at http://localhost:8123/ and opens it: the model page, with
the flight simulator at /sim/. `--export` first rewrites site/data/ from the model: the flight data
(aircraft.json) and a light mesh of every part (one STL each, listed in parts.json), which both pages
load. site/data/ is committed, so the site can be hosted as static files; re-export after changing
the model. `--serve` only serves, without opening a browser. The simulator generates the world it
flies over from a seed (site/sim/world.js): ?seed= in the address, or N for a new one.

The flight data comes from the same models as the reports, all in DESIGN_AIR: mass, CG and inertia
(balance.py), propulsion and drag (performance.py), and wing, tail and fin panels (vtol.py).
site/sim/physics.js flies it and site/sim/control.js is a simplified ArduPilot-style flight
controller. It is a handling toy for getting a feel for the aircraft, not a test of ArduPilot or its
settings.
"""

import json
import sys
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from math import acos, pi, sin, sqrt
from pathlib import Path

from trailhawk.balance import CL_MAX, DESIGN_AIR, G, center_of_gravity, inertia, mass_items, stall_speed, trapezoid
from trailhawk.cad import export_parts
from trailhawk.performance import (
    ADVANCE, DIAMETER, ELECTRONICS_WATTS, NOMINAL_VOLTS, ORBIT_RADIUS, OSWALD, POWER_COEFFICIENT, SPEED_MARGIN, THRUST_COEFFICIENT, max_static,
    mission, motor_test, parasite_drag_area, rpm_for_thrust, wing_reference,
)  # fmt: skip
from trailhawk.parts import COLOR_CAMERA, THERMAL_CAMERA
from trailhawk.strength import MAX_SPEED, TAIL_CL_MAX
from trailhawk.vtol import (
    AILERON, CAMERA_RADIUS, ELEVATOR, FIN_TIP_Z, GIMBAL_CENTER, GROUND_Z, HINGE_UP, HUB_X, MOTORS, PAINT, POD_Y, RAIL_X, RUDDER, TILTING, YAW_TILT,
    build_vtol, chord_point, fin_stations, hinge, lerp, tail_stations, wing_stations,
)  # fmt: skip

SITE = Path(__file__).parents[1] / "site"
FOLDER = SITE / "data"
PORT = 8123
AIRFOILS = {"2412": (-2.1, -0.05), "0010": (0.0, 0.0)}  # zero-lift angle (deg), quarter-chord moment coefficient
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
    area, chord, along = trapezoid(a, b)
    center = lerp(chord_point(a, 0.25), chord_point(b, 0.25), along)
    spanwise = (chord_point(b, 0.25) - chord_point(a, 0.25)).normalized()
    normal = a[0].x_dir.cross(spanwise).normalized()
    normal = normal if normal.dot(a[0].y_dir) > 0 else -normal
    flip = -1 if mirror else 1
    zero_lift, moment = AIRFOILS[airfoil]
    tau = flap_effectiveness(1 - hinge_at) * (span[1] - span[0]) if control else 0
    return {
        "name": name,
        "center": [center.X, flip * center.Y, center.Z],  # CAD mm
        "normal": [normal.X, flip * normal.Y, normal.Z],
        "area": area / 1e6,  # m^2
        "chord": chord / 1000,  # m, the mean aerodynamic chord
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
    tail_aspect = tail_span**2 / (2 * trapezoid(tail_root, tail_tip)[0])
    bottom, top = fin_stations()
    fin_height = top[0].origin.Z - bottom[0].origin.Z
    fin_aspect = fin_height**2 / trapezoid(bottom, top)[0]
    result = []
    for mirror, side in ((False, "right"), (True, "left")):
        result += [
            panel(f"wing_inner_{side}", root, crank, "2412", wing_aspect, CL_MAX, mirror),
            panel(f"wing_outer_{side}", crank, tip, "2412", wing_aspect, CL_MAX, mirror, "aileron", *AILERON),
            panel(f"tail_{side}", tail_root, tail_tip, "0010", tail_aspect, TAIL_CL_MAX, mirror, "elevator", *ELEVATOR, downwash=True),
        ]  # fmt: skip
    return result + [panel("fin", bottom, top, "0010", fin_aspect, TAIL_CL_MAX, False, "rudder", *RUDDER)]


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
    max_thrust, _ = max_static(NOMINAL_VOLTS, DESIGN_AIR)
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
        "loiter_minutes": results["loiter_minutes"],  # circling flight time, for the model page
        "paint": PAINT,  # vtol.py's colors, by role: the pages find each painted part's role by its color
        "cameras": {  # for sim/sensors.js: field of view across (deg) and, for the thermal camera, its pixels across
            "thermal": {"hfov": THERMAL_CAMERA["hfov"], "pixels": THERMAL_CAMERA["pixels"]},
            "color": {"hfov": COLOR_CAMERA["hfov"]},
        },
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
    """Rewrite site/data/ from the model."""
    vtol = build_vtol()
    for old in FOLDER.glob("*.stl"):
        old.unlink()  # a part that is gone takes its mesh with it
    export_parts(vtol, FOLDER, 0.5, 0.5)  # mm, rad: smooth skins at about 5 MB; the seams sit on them
    (FOLDER / "aircraft.json").write_text(json.dumps(aircraft(vtol), indent=1))


class Handler(SimpleHTTPRequestHandler):
    # Windows' registry can map .js to text/plain, which browsers refuse for modules
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript", ".json": "application/json"}

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")  # recheck on every load, so edits and new exports show
        super().end_headers()


def serve(open_browser):
    url = f"http://localhost:{PORT}/"
    server = ThreadingHTTPServer(("127.0.0.1", PORT), partial(Handler, directory=str(SITE)))
    print(f"The model at {url} and the flight simulator at {url}sim/ (Ctrl+C to stop)")
    if open_browser:
        webbrowser.open(url)
    server.serve_forever()


if __name__ == "__main__":
    if "--export" in sys.argv:
        export()
    serve(open_browser="--serve" not in sys.argv)

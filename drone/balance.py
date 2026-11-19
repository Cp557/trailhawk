"""Weight and balance of the VTOL: total mass, center of gravity (CG) in cruise and hover, wing
loading, stall speed and how the three motors share the weight in hover.

Printed parts weigh their skin (area * SKIN * PRINT_DENSITY), carbon spars their tube volume
* CARBON_DENSITY, bought parts their datasheet masses (parts.py), and the rest the estimates
in GRAMS.
Run this file for a report.
"""

from collections import namedtuple
from math import sqrt

import numpy as np
from build123d import Compound, Location, Pos, Rot, Vector
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from drone.parts import AILERON_SERVO, BATTERY, COLOR_CAMERA, COMPUTER, ESC, FLIGHT_CONTROLLER, GPS, MOTOR, PROP, RECEIVER, TAIL_SERVO, THERMAL_CAMERA, TILT_SERVO
from drone.vtol import (
    FLIGHT_CONTROLLER_CENTER, HINGE_UP, HUB_X, MOTORS, SKIN, TILTING, WING_TIP_Y, build_vtol, chord_point, fin_stations,
    hinge, lerp, spar_lines, tail_stations, wing_stations,
)  # fmt: skip

PRINT_DENSITY = 0.6e-3  # g/mm^3, foamed LW-PLA
CARBON_DENSITY = 1.55e-3  # g/mm^3
PRINTED = {
    "fuselage", "wing", "horizontal_tail", "fin", "nacelles", "tail_nacelle",
    "aileron_right", "aileron_left", "elevator_right", "elevator_left", "rudder",
}  # fmt: skip
GRAMS = {
    # bought parts (parts.py)
    "motor": MOTOR["grams"], "propeller": PROP["grams"], "tilt_servo": TILT_SERVO["grams"],
    "battery": BATTERY["grams"], "flight_controller": FLIGHT_CONTROLLER["grams"], "computer": COMPUTER["grams"],
    "receiver": RECEIVER["grams"], "gps": GPS["grams"],
    "esc_right": ESC["grams"], "esc_left": ESC["grams"], "esc_tail": ESC["grams"],
    "aileron_servo_right": AILERON_SERVO["grams"], "aileron_servo_left": AILERON_SERVO["grams"],
    "elevator_servo_right": TAIL_SERVO["grams"], "elevator_servo_left": TAIL_SERVO["grams"],
    "rudder_servo": TAIL_SERVO["grams"],
    # estimates
    "prop_nut": 5, "mount": 8, "bearing": 2, "hinge_pin": 1, "landing_gear": 90, "gimbal_mount": 10,
    "center_fitting": 45,  # printed nylon block, partly hollow
    "details": 2,  # paint on the panel lines
    "camera": 60,  # turret: ball shell, pan servo and bearing
    "thermal_lens": THERMAL_CAMERA["grams"], "lens": COLOR_CAMERA["grams"],
}  # fmt: skip
WIRING = ("wiring", 130, FLIGHT_CONTROLLER_CENTER)  # wires, connectors at the dry joints and fasteners; not modeled, booked at the flight controller

CG_RANGE = (0.25, 0.30)  # of the mean aerodynamic chord, the usual range for a conventional tail
CL_MAX = 1.2
G = 9.81
TAIL_VOLUME_RANGE = (0.4, 0.7)  # horizontal tail volume, the usual range
FIN_VOLUME_RANGE = (0.02, 0.05)

Air = namedtuple("Air", "density viscosity")  # kg/m^3, kinematic m^2/s
SEA_LEVEL = Air(1.225, 1.46e-5)
DESIGN_AIR = Air(0.793, 2.22e-5)  # 12,000 ft on a 10 C day: the high trail this drone is sized for


def leaves(node, loc, tilt):
    """(label, shape, location of its parent) for every leaf, with the front motors tilted `tilt` deg."""
    for child in node.children:
        if not child.children:
            yield child.label, child, loc
            continue
        child_loc = loc * child.location
        if child.label in TILTING:
            child_loc = child_loc * Rot(0, tilt, 0)
        yield from leaves(child, child_loc, tilt)


def mass_items(vtol, tilt=0):
    """(label, grams, center) for every leaf of the assembly plus the wiring allowance."""
    carbon = set(spar_lines())
    items = [(WIRING[0], WIRING[1], Vector(WIRING[2]))]
    for label, shape, loc in leaves(vtol, Location(), tilt):
        if label in PRINTED:
            grams, center = shape.area * SKIN * PRINT_DENSITY, Compound(shape.faces()).center()
        elif label in carbon:
            grams, center = shape.volume * CARBON_DENSITY, shape.center()
        else:
            grams, center = GRAMS[label], shape.center()
        items.append((label, grams, (loc * Pos(center)).position))
    return items


def center_of_gravity(items):
    moment = Vector()
    for _, grams, center in items:
        moment += center * grams
    return moment / sum(grams for _, grams, _ in items)


def inertia(vtol):
    """Inertia tensor (kg m^2) about the cruise CG in CAD axes: every mass_items mass as a point at
    its center, plus the printed skins' and carbon tubes' own spread about theirs (parallel axes)."""
    items = mass_items(vtol)
    cg = np.array(tuple(center_of_gravity(items)))
    tensor = np.zeros((3, 3))
    for _, grams, center in items:
        r = np.array(tuple(center)) - cg
        tensor += grams * (r @ r * np.eye(3) - np.outer(r, r))
    carbon = set(spar_lines())
    for label, shape, loc in leaves(vtol, Location(), 0):
        if label in PRINTED or label in carbon:
            properties = GProp_GProps()
            if label in PRINTED:
                BRepGProp.SurfaceProperties_s((loc * shape).wrapped, properties)
                density = SKIN * PRINT_DENSITY
            else:
                BRepGProp.VolumeProperties_s((loc * shape).wrapped, properties)
                density = CARBON_DENSITY
            own = properties.MatrixOfInertia()  # about the part's own center
            tensor += density * np.array([[own.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return tensor * 1e-9  # g mm^2 -> kg m^2


def trapezoid(a, b):
    """(area mm^2, mean aerodynamic chord mm, its spanwise position as a fraction) of the
    trapezoidal panel between stations a and b."""
    taper = b[1] / a[1]
    height = abs((b[0].origin - a[0].origin).dot(a[0].z_dir))
    return (a[1] + b[1]) / 2 * height, 2 / 3 * a[1] * (1 + taper + taper**2) / (1 + taper), (1 + 2 * taper) / (3 * (1 + taper))


def mean_aerodynamic_chord():
    """(MAC, x of its leading edge, reference area) of both wing halves' trapezoidal panels,
    winglets excluded."""
    area = mac_moment = le_moment = 0
    root, crank, tip = wing_stations()
    for a, b in ((root, crank), (crank, tip)):
        panel, mac, along = trapezoid(a, b)
        area += panel
        mac_moment += panel * mac
        le_moment += panel * lerp(chord_point(a, 0).X, chord_point(b, 0).X, along)
    return mac_moment / area, le_moment / area, 2 * area


def hover_shares(vtol):
    """Fraction of the weight each motor lifts in a steady hover: the thrusts sum to the weight
    and balance in pitch and roll about the hover CG."""
    cg = center_of_gravity(mass_items(vtol, tilt=90))
    hubs = {name: (Location(hinge(nose)) * Rot(0, 90, 0) * Pos(HUB_X, 0, -HINGE_UP)).position for name, (nose, _) in MOTORS.items()}
    lift = [[1] * len(hubs), [hub.X - cg.X for hub in hubs.values()], [hub.Y - cg.Y for hub in hubs.values()]]
    return dict(zip(hubs, np.linalg.solve(lift, [1, 0, 0])))


def stall_speed(total_grams, air):
    """Stall speed (m/s) at CL_MAX in `air`."""
    _, _, area = mean_aerodynamic_chord()
    return sqrt(2 * total_grams / 1000 * G / (air.density * area / 1e6 * CL_MAX))


def tail_volumes(cg):
    """(horizontal, vertical) tail volume coefficients about the CG."""
    mac, _, area = mean_aerodynamic_chord()
    span = 2 * WING_TIP_Y

    def panel(a, b):
        return trapezoid(a, b)[0], chord_point(a, 0.25).X, chord_point(b, 0.25).X

    tail_area, tail_x0, tail_x1 = panel(*tail_stations())
    fin_area, fin_x0, fin_x1 = panel(*fin_stations())
    horizontal = 2 * tail_area * ((tail_x0 + tail_x1) / 2 - cg.X) / (area * mac)
    vertical = fin_area * ((fin_x0 + fin_x1) / 2 - cg.X) / (area * span)
    return horizontal, vertical


def cg_fraction(cg):
    """CG position as a fraction of the mean aerodynamic chord from its leading edge."""
    mac, mac_le, _ = mean_aerodynamic_chord()
    return (cg.X - mac_le) / mac


def report(vtol):
    items = mass_items(vtol)
    total = sum(grams for _, grams, _ in items)
    cg, hover_cg = center_of_gravity(items), center_of_gravity(mass_items(vtol, tilt=90))
    mac, mac_le, area = mean_aerodynamic_chord()
    by_label = {}
    for label, grams, _ in items:
        by_label[label] = by_label.get(label, 0) + grams
    for label, grams in sorted(by_label.items(), key=lambda item: -item[1]):
        print(f"{label:22s} {grams:7.1f} g")
    area_m2 = area / 1e6
    print(f"\ntotal                  {total:7.1f} g")
    print(f"CG cruise              x {cg.X:.1f}  y {cg.Y:.1f}  z {cg.Z:.1f} mm  ({cg_fraction(cg):.1%} MAC)")
    print(f"CG hover               x {hover_cg.X:.1f}  z {hover_cg.Z:.1f} mm")
    print(f"MAC                    {mac:.1f} mm, leading edge at x {mac_le:.1f}")
    print(f"wing area              {area_m2:.3f} m^2, loading {total / (area_m2 * 100):.1f} g/dm^2")
    print(f"stall speed            {stall_speed(total, SEA_LEVEL):.1f} m/s sea level, {stall_speed(total, DESIGN_AIR):.1f} m/s at 12,000 ft (CL {CL_MAX})")
    horizontal, vertical = tail_volumes(cg)
    print(f"tail volumes           horizontal {horizontal:.2f}, vertical {vertical:.3f}")
    for name, share in hover_shares(vtol).items():
        print(f"hover {name:16s} {share:6.1%} of weight ({share * total:.0f} g thrust)")


if __name__ == "__main__":
    report(build_vtol())

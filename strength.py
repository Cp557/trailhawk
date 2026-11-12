"""Strength check of the carbon structure: wing spars and joiners, tail, boom and skids, under
flight, hover-thrust and landing loads. Spars take all the bending (printed skins are ignored,
which is conservative) and the wing's weight and pod weights are not credited against lift.

Load cases are limit loads times SAFETY_FACTOR. Stresses are compared with allowable values;
the carbon ones are a supplier's figures with a knockdown, so treat the margins as estimates
until a joint and a tube have been tested. Run this file for a report.
"""

from math import pi

import numpy as np

from balance import G, SEA_LEVEL, mass_items, mean_aerodynamic_chord
from parts import BATTERY
from performance import max_static, weight, wing_reference
from vtol import (
    BOOM_JOINT_X, BOOM_RADIUS, BOOM_SOCKET_RADIUS, BOOM_SOCKET_START, JOINER_EMBED, JOINER_RADIUS, MOTORS, POD_Y,
    RAIL_X, SKID_Z, SPAR_END_Y, SPAR_RADIUS, WING_JOINTS, WING_TIP_Y, build_vtol, fin_stations,
    hinge, spar_point, tail_stations, wing_stations,
)  # fmt: skip

SAFETY_FACTOR = 1.5  # ultimate load over limit load
MAX_SPEED = 20  # m/s equivalent airspeed: set as ArduPilot's airspeed limit (ARSPD_FBW_MAX)
STRUCTURE_CL_MAX = 1.4  # the highest lift the wing could reach, higher than the stall-speed figure on purpose
GUST = 10  # m/s, a strong mountain gust
LANDING_SPEED = 1.5  # m/s, a hard touchdown (ArduPilot lands at under 1 m/s)
LANDING_STOP = 0.02  # m, skid and strut flex that stops the fall
FRICTION = 0.5  # ground drag on the skid during touchdown
TAIL_CL_MAX = 1.0

# Allowables in MPa
CARBON_BENDING = 0.7 * 600  # roll-wrapped T700 tube: 600 MPa axial compression (supplier), knocked down
CARBON_RING = 120  # across the wall of a tube loaded by a joiner end (estimate)
FITTING = 40  # printed nylon block (estimate)
PRINTED_SKIN = 5  # foamed LW-PLA: about half its ~10 MPa tensile strength, for the printed layer lines (estimate)


def section_modulus(radius):
    """Section modulus (mm^3) of a 1 mm wall round carbon tube of outer `radius`."""
    outer, inner = 2 * radius, 2 * (radius - 1)
    return pi / 32 * (outer**4 - inner**4) / outer


def ring_stress(moment, embed, radius):
    """Bending stress (MPa) across the wall of a 1 mm tube with a spigot of length `embed` (mm)
    inside it, under `moment` (N*mm): the spigot's end loads the wall over about embed/3, and the
    wall ring bends like a loaded hoop."""
    line_load = 4.5 * moment / embed**2  # N/mm of tube length
    return 6 * 0.16 * line_load * (radius - 0.5)


def chord_at(y):
    """Wing chord (mm) at span station y, from the three stations."""
    root, crank, tip = wing_stations()
    return float(np.interp(y, [root[0].origin.Y, crank[0].origin.Y, tip[0].origin.Y], [root[1], crank[1], tip[1]]))


def lift_per_mm(y, total):
    """Schrenk lift distribution (N/mm) across the half span: the average of the planform and an
    ellipse, normalized so the half span carries `total` / 2."""
    _, _, area = mean_aerodynamic_chord()
    half_span = WING_TIP_Y
    elliptic = 4 * area / (pi * 2 * half_span) * np.sqrt(np.maximum(1 - (y / half_span) ** 2, 0))
    return total * (np.array([chord_at(v) for v in y]) + elliptic) / 2 / area


def wing_moment(y, total):
    """Spanwise bending moment (N*mm) at station y under total lift `total` N."""
    span = np.linspace(y, WING_TIP_Y, 400)
    return np.trapezoid(lift_per_mm(span, total) * (span - y), span)


def gust_load_factor(vtol):
    """Load factor from a sharp gust of GUST m/s at MAX_SPEED, sea level (the worst air)."""
    area, aspect = wing_reference()
    w = weight(vtol)
    slope = 2 * pi / (1 + 2 / aspect)  # per radian
    mac = mean_aerodynamic_chord()[0] / 1000
    mass_ratio = 2 * (w / area) / (SEA_LEVEL.density * mac * slope * G)
    alleviation = 0.88 * mass_ratio / (5.3 + mass_ratio)
    return 1 + SEA_LEVEL.density * MAX_SPEED * GUST * slope * area * alleviation / (2 * w)


def limit_load_factor(vtol):
    """Limit load factor: a gust cannot pull more than the stall allows at MAX_SPEED."""
    area, _ = wing_reference()
    stall_limited = 0.5 * SEA_LEVEL.density * MAX_SPEED**2 * area * STRUCTURE_CL_MAX / weight(vtol)
    return min(stall_limited, gust_load_factor(vtol))


def landing_load_factor():
    return 1 + LANDING_SPEED**2 / (2 * G * LANDING_STOP)


def checks(vtol):
    """(name, demand, allowable, unit) for every check; demands are ultimate loads."""
    ultimate = limit_load_factor(vtol) * SAFETY_FACTOR
    lift = weight(vtol) * ultimate
    q = 0.5 * SEA_LEVEL.density * MAX_SPEED**2
    full_thrust = max_static(BATTERY["cells"] * BATTERY["cell_volts"], SEA_LEVEL)[0]
    landing = landing_load_factor() * SAFETY_FACTOR
    reaction = landing * weight(vtol) / 2  # each skid, N

    root_moment = wing_moment(0, lift)
    (inner_joint, outer_joint), (inner_embed, outer_embed) = WING_JOINTS, JOINER_EMBED
    results = [
        ("Wing root tube at the center fitting", root_moment / section_modulus(SPAR_RADIUS), CARBON_BENDING, "MPa"),
        ("Center fitting block, bending", root_moment / (44 * 30**2 / 6), FITTING, "MPa"),
    ]
    for joint, embed, name in ((inner_joint, inner_embed, "inner"), (outer_joint, outer_embed, "outer")):
        moment = wing_moment(joint, lift)
        results += [
            (f"Wing joiner tube at the {name} joint", moment / section_modulus(JOINER_RADIUS), CARBON_BENDING, "MPa"),
            (f"Spar wall at the {name} joiner's end", ring_stress(moment, embed, SPAR_RADIUS), CARBON_RING, "MPa"),
            (f"Spar just past the {name} joiner", wing_moment(joint + embed, lift) / section_modulus(SPAR_RADIUS), CARBON_BENDING, "MPa"),
        ]
    results.append(("Printed wing skin past the spar end", wing_skin_stress(lift), PRINTED_SKIN, "MPa"))

    # Hover: thrust at the pods bends the wing the other way with no lift to share it
    results.append(("Wing root tube, full thrust in hover", full_thrust * SAFETY_FACTOR * POD_Y / section_modulus(SPAR_RADIUS), CARBON_BENDING, "MPa"))

    # Landing: the skids push up at the pods and the wing outboard of them keeps falling
    results.append(("Wing root tube, hard landing", reaction * POD_Y / section_modulus(SPAR_RADIUS), CARBON_BENDING, "MPa"))
    results += skid_checks(vtol, reaction)
    results += tail_checks(vtol, q, full_thrust, landing)
    return results


def wing_skin_stress(lift):
    """Bending stress (MPa) in the printed skin alone, outboard of where the spar stops."""
    moment = wing_moment(SPAR_END_Y, lift)
    chord = chord_at(SPAR_END_Y)
    depth = 0.12 * chord
    # Top and bottom skin 0.8 mm thick, over the middle half of the chord
    return moment / (0.5 * chord * 0.8 * depth)


def skid_checks(vtol, reaction):
    """Strut and rail under a landing on one skid: one 10 mm tube in each faired part."""
    strut_top = spar_point(POD_Y).Z - SPAR_RADIUS - 1
    height = strut_top - SKID_Z
    tube_modulus = section_modulus(5)
    rail_modulus = section_modulus(6)
    overhang = (RAIL_X[1] - RAIL_X[0]) / 2
    return [
        ("Skid strut, ground drag at touchdown", FRICTION * reaction * height / tube_modulus, CARBON_BENDING, "MPa"),
        ("Skid rail, landing on its end", reaction * overhang / rail_modulus, CARBON_BENDING, "MPa"),
        ("Wing spar torsion from the skid strut", FRICTION * reaction * height / (2 * section_modulus(SPAR_RADIUS)), CARBON_BENDING / 2, "MPa"),
    ]


def tail_checks(vtol, q, full_thrust, landing):
    """Tail and fin spars and the boom."""
    tail_root, tail_tip = tail_stations()
    bottom, top = fin_stations()
    tail_half_area = (tail_root[1] + tail_tip[1]) / 2 * (tail_tip[0].origin.Y - tail_root[0].origin.Y) / 1e6
    fin_area = (bottom[1] + top[1]) / 2 * (top[0].origin.Z - bottom[0].origin.Z) / 1e6
    tail_lift = q * tail_half_area * TAIL_CL_MAX * SAFETY_FACTOR  # N, per side
    fin_lift = q * fin_area * TAIL_CL_MAX * SAFETY_FACTOR
    tail_arm = (tail_tip[0].origin.Y - tail_root[0].origin.Y) * 0.45
    fin_arm = (top[0].origin.Z - bottom[0].origin.Z) * 0.45

    rear = hinge(MOTORS["motor_tail"][0])
    boom_arm = rear[0] - BOOM_JOINT_X  # mm from the pod joint to the rear motor
    tail_x = tail_root[0].origin.X - BOOM_JOINT_X
    # Weight aft of the joint keeps falling at touchdown
    aft_weight = sum(grams * (center.X - BOOM_JOINT_X) for _, grams, center in mass_items(vtol) if center.X > BOOM_JOINT_X) / 1000 * G * landing
    boom_moment = max(
        full_thrust * SAFETY_FACTOR * boom_arm,  # rear motor at full thrust in hover
        2 * tail_lift * tail_x,  # tail pulling at the speed limit
        fin_lift * tail_x,  # fin side load
        aft_weight,  # hard landing
    )
    embed = BOOM_JOINT_X - BOOM_SOCKET_START
    return [
        ("Tail spar at the root", tail_lift * tail_arm / section_modulus(3), CARBON_BENDING, "MPa"),
        ("Fin spar at the root", fin_lift * fin_arm / section_modulus(4), CARBON_BENDING, "MPa"),
        ("Boom tube at the pod joint", boom_moment / section_modulus(BOOM_RADIUS), CARBON_BENDING, "MPa"),
        ("Boom socket wall", ring_stress(boom_moment, embed, BOOM_SOCKET_RADIUS), CARBON_RING, "MPa"),
    ]


def report(vtol):
    print(f"Limit load factor {limit_load_factor(vtol):.1f} g (gust alone {gust_load_factor(vtol):.1f} g, capped by stall at {MAX_SPEED} m/s)")
    print(f"Landing load factor {landing_load_factor():.1f} g ({LANDING_SPEED} m/s stopped in {LANDING_STOP * 1000:.0f} mm)")
    print(f"All stresses at ultimate load (limit x {SAFETY_FACTOR}):\n")
    for name, demand, allowable, unit in checks(vtol):
        print(f"  {name:42s} {demand:6.1f} of {allowable:5.0f} {unit}   margin {allowable / demand - 1:+6.0%}")


if __name__ == "__main__":
    report(build_vtol())

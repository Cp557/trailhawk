"""Tri-tiltrotor VTOL drone for loitering over a hiker: pod-and-boom fuselage, long cranked wing
with upturned tips that comes apart to pack, two under-wing motor pods and a motor pod on top
of the fin, hinged control surfaces, servo-driven tilt hinges, carbon spars, internal components,
faired landing skids and a pan turret. Modeled in cruise: the front motors face forward and the
rear motor is fixed upright, the ArduPilot tilt-tricopter layout. Bought parts take their real
sizes from parts.py.

Moving parts are located at their pivots so a viewer animation can rotate them in place
(see fly.py): front motors tilt about local Y (their hinge), propellers spin about local X,
control surfaces deflect about local X (their hinge line) and the gimbal pans about local Z.

Same conventions as drone.py: +X nose to tail, +Y right wing, +Z up, mm.
Run this file to export STEP/STL into out/ and send the model to the OCP CAD Viewer.
"""

from functools import reduce
from math import radians, tan
from operator import add

from build123d import (
    Align,
    Box,
    Circle,
    Compound,
    Cylinder,
    Ellipse,
    Keep,
    Location,
    Plane,
    Polyline,
    Pos,
    Rot,
    Solid,
    Sphere,
    Vector,
    extrude,
    loft,
    make_face,
    mirror,
    scale,
    split,
)
from build123d.topology.shape_core import downcast
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.TopLoc import TopLoc_Location

from drone import export_and_show, fin_plane, labeled, propeller, section, span_plane, surface
from parts import AILERON_SERVO, BATTERY, COMPUTER, ESC, FLIGHT_CONTROLLER, GPS, MOTOR, PROP, RECEIVER, TAIL_SERVO, TILT_SERVO

WING_X = 390  # quarter chord of the straight inner panel
WING_ROOT_Z = 60
DIHEDRAL = 2
POD_Y = 245  # wing motor pods sit at +/- POD_Y, far enough out for PROP_GAP
FIN_TIP_Z = 300
POD_RADIUS = 26
WING_CRANK_Y, WING_TIP_Y = 340, 1100  # to the winglet root; the winglet adds 60
WING_JOINTS = (140, 480)  # |y| of the dry wing joints: the panels between them pack separately
JOINER_EMBED = (120, 100)  # how far each joiner tube reaches into the spar on each side of its joint
BOOM_JOINT_X = 640  # dry joint between the pod and the boom
BOOM_SOCKET_START = 550  # the boom tube runs forward to here, inside a socket in the pod
SPAR_END_Y = 850  # the spar stops where the wing gets too thin for it
SPAR_RADIUS, JOINER_RADIUS, BOOM_RADIUS, BOOM_SOCKET_RADIUS = 8, 7, 8, 9  # 1 mm wall carbon tubes
SKIN = 0.8  # printed wall thickness: cavities are inset by it (print_parts.py, balance.py)

# Motor (parts.MOTOR): a can and its long shaft. The datasheet gives only the overall length, so
# the split is an estimate. The prop sits 6 mm behind the shaft tip, leaving room for the nut.
SHAFT_OUT = 20
CAN_LENGTH = MOTOR["length"] - SHAFT_OUT
PROP_RADIUS = PROP["diameter"] / 2
PROP_GAP = 25  # least prop-tip gap to the airframe: print tolerance, hinge slop (1 deg of tilt moves the tip 2.7 mm) and blade flex

# Motor hinge: directly above each nacelle's front face, clear of its skin. The motor hangs
# MOTOR_GAP ahead of the hinge, so swinging it up and forward never brings it into the nacelle.
HINGE_UP = POD_RADIUS + 5
MOTOR_GAP = 12
HUB_X = -MOTOR_GAP - MOTOR["length"] + 6 + PROP["hub_thickness"] / 2  # prop hub relative to the hinge

# Motors: name -> (front center of its fixed nacelle, nacelle length). A motor faces -X at 0 deg;
# rotating it +90 deg about its hinge's local Y points it straight up for hover.
MOTORS = {
    "motor_right": ((200, POD_Y, 48), 380),
    "motor_left": ((200, -POD_Y, 48), 380),
    "motor_tail": ((1050, 0, FIN_TIP_Z), 180),
}
# ArduPilot tilt-tricopter (Q_FRAME_CLASS 7, Q_TILT_MASK 3): only the front motors tilt; the rear
# motor is fixed upright and stops in forward flight. Hover yaw comes from tilting the front
# motors in opposite directions by up to YAW_TILT past vertical (Q_TILT_TYPE 2, Q_TILT_YAW_ANGLE).
TILTING = ("motor_right", "motor_left")
YAW_TILT = 10

SKID_Z = -125  # skid rail centerline
RAIL_RADIUS = 10
RAIL_X = (280, 620)
RAIL = [(0, 0.35), (0.06, 0.7), (0.18, 0.95), (0.35, 1), (0.65, 1), (0.85, 0.7), (1, 0.3)]  # (fraction of length, fraction of RAIL_RADIUS)
STRUT_CHORD = 60  # one faired strut per skid, NACA 0025, up to the wing spar
GROUND_Z = SKID_Z - RAIL_RADIUS
GIMBAL_CENTER = (170, 0, -72)  # the ball sits half inside the belly
CAMERA_RADIUS = 32
CAMERA_EXPOSED = 29  # how far the ball hangs below the belly
BATTERY_CENTER = (227, 0, 0)  # slide it along X to trim the center of gravity (balance.py)
COMPUTER_CENTER = (108, 0, 4)

# Fuselage cross-sections: (x, half width, half height, centerline z). A rounded pod tapering
# into a thin tail boom; a half-ellipsoid nose closes the first section.
FUSELAGE = [
    (100, 70, 72, 0),
    (260, 80, 88, 8),
    (430, 66, 74, 20),
    (600, 30, 34, 34),
    (780, 17, 19, 42),
    (1000, 14, 16, 46),
    (1200, 9, 11, 48),
]
NACELLE = [(0, POD_RADIUS), (0.2, 30), (0.65, 27), (1, 8)]  # (fraction of length, radius)

# Paint: carbon-black airframe and tail nacelle (a dark silhouette against the sky), orange front
# pods and control surfaces (easy to spot against forest), black skids
CARBON, BLACK, ORANGE, DARK = "#1e1e21","#1c1c1e", "#ef6c1f", "#3a3a3c"
WING_BLEND_Y = 130  # the wing root flares into the fuselage inboard of here


def pivot_at(solid, loc):
    """`solid` in the same place, re-expressed so its location is exactly `loc`.

    Viewer rotations of the result then pivot about `loc`. The inverse transform is baked into
    the geometry (copy=True); build123d's transform_shape only changes the location.
    """
    to_local = (loc.inverse() * solid.location).wrapped.Transformation()
    raw = solid.wrapped.Located(TopLoc_Location())
    return loc * Solid(downcast(BRepBuilderAPI_Transform(raw, to_local, True).Shape()))


def lerp(a, b, t):
    return a + (b - a) * t


def mirror_y(point):
    return Vector(point.X, -point.Y, point.Z)


def chord_point(station, fraction):
    """Point on a (plane, chord, pitch) station's chord line, `fraction` of chord from the LE."""
    plane, chord, _ = station
    return plane.origin + plane.x_dir * ((fraction - 0.25) * chord)


def surface_point(a, b, s, fraction, camber=0):
    """Point `s` of the way from station a to b, `fraction` of chord back from the LE and
    `camber` chords above the chord line."""
    chord = lerp(a[1], b[1], s)
    return lerp(chord_point(a, fraction), chord_point(b, fraction), s) + a[0].y_dir * (camber * chord)


def hinged(solid, hinge0, hinge1, gap=2):
    """Cut a control surface off the trailing (+X) side of `solid` along hinge0 -> hinge1.

    Returns (fixed part, control surface), separated by a `gap` mm slot. The control surface is
    located on the hinge: origin at hinge0, local X along the hinge.
    """
    axis = (hinge1 - hinge0).normalized()
    aft = Vector(300, 0, 0)

    def cutter(a, b):
        return extrude(make_face(Polyline(a, b, b + aft, a + aft, close=True)), amount=100, both=True)

    fixed = solid - cutter(hinge0 - Vector(gap, 0, 0), hinge1 - Vector(gap, 0, 0))
    flap = (solid & cutter(hinge0 + axis * gap, hinge1 - axis * gap)).solid()
    frame = Plane(hinge0, x_dir=axis, z_dir=Vector(1, 0, 0).cross(axis))
    return fixed, pivot_at(flap, frame.location)


def nose_cap(length, width, height):
    """Front half of an ellipsoid pointing -X, its base ellipse in the YZ plane at the origin."""
    ellipsoid = scale(Sphere(1), by=(length, width, height))
    return split(ellipsoid, bisect_by=Plane.YZ, keep=Keep.BOTTOM)


def yz_plane(x, y=0, z=0):
    return Plane((x, y, z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))


def tube(start, end, radius, bore=0):
    """Cylinder from `start` to `end`, hollow when `bore` (inner radius) is given."""
    axis = end - start
    plane = Plane(start, z_dir=axis)
    base = (Align.CENTER, Align.CENTER, Align.MIN)
    rod = plane * Cylinder(radius, axis.length, align=base)
    return rod - plane * Cylinder(bore, axis.length, align=base) if bore else rod


def fuselage(inset=0):
    """Outer skin, or with `inset` the cavity inside a skin that thick."""
    body = loft([yz_plane(x, 0, z) * Ellipse(w - inset, h - inset) for x, w, h, z in FUSELAGE])
    x, w, h, _ = FUSELAGE[0]
    nose = Pos(x, 0, 0) * nose_cap(x - inset, w - inset, h - inset)
    # The union of the spline nose and loft needs a shape fix to pass BRep validity
    return (nose + body).fix()


def dihedral_z(y):
    return WING_ROOT_Z + y * tan(radians(DIHEDRAL))


def wing_stations():
    """Right wing (root, crank, tip) as (plane, chord, pitch): a straight inner panel and an
    outer panel swept 12 deg, 2 deg dihedral, incidence 2 -> -1 deg."""
    sweep = tan(radians(12))
    return (
        (span_plane(WING_X, 0, WING_ROOT_Z), 260, 2),
        (span_plane(WING_X, WING_CRANK_Y, dihedral_z(WING_CRANK_Y)), 230, 1),
        (span_plane(WING_X + (WING_TIP_Y - WING_CRANK_Y) * sweep, WING_TIP_Y, dihedral_z(WING_TIP_Y)), 110, -1),
    )


def spar_point(y):
    """Point on the right wing's spar at span station y: one straight line from the centerline to
    SPAR_END_Y, so that straight joiner tubes fit it at every joint. It runs along the airfoil's
    mean line, from 30% chord near the root through 44% at the crank to 32% at its end."""
    start = Vector(WING_X - 5, 0, WING_ROOT_Z + 4.3)
    end = Vector(WING_X + 118, SPAR_END_Y, dihedral_z(SPAR_END_Y) + 2.9)
    return lerp(start, end, y / SPAR_END_Y)


def tail_stations():
    """Right horizontal tail (root, tip), quarter chord swept 25 deg."""
    return (span_plane(1080, 0, 46), 160, 0), (span_plane(1080 + 260 * tan(radians(25)), 260, 46), 100, 0)


def fin_stations():
    """Fin (bottom, top)."""
    return (fin_plane(1050, 40), 200, 0), (fin_plane(1120, FIN_TIP_Z), 110, 0)


def wing_blend(start):
    """Root fairing from the plain wing section `start` (a station) inboard into the fuselage: the
    chord and thickness grow so that the wing flows into the body with no hard corner."""
    x, _, z = start[0].origin
    sections = [
        section("2412", start[1], start[0], start[2]),
        section("2414", 275, span_plane(x, 95, z - 1), start[2]),
        section("2416", 315, span_plane(x, 45, z - 2), start[2]),
        section("2416", 315, span_plane(x, 0, z - 3), start[2]),
    ]
    return loft(sections)


def wing():
    """Returns (wing, right aileron, left aileron)."""
    root, crank, tip = wing_stations()
    tip_x, tip_y, tip_z = tip[0].origin
    s = WING_BLEND_Y / WING_CRANK_Y
    blend_start = (span_plane(WING_X, WING_BLEND_Y, dihedral_z(WING_BLEND_Y)), lerp(root[1], crank[1], s), lerp(root[2], crank[2], s))
    # Winglet: sections roll upward about X while sweeping back
    winglet = [
        tip,
        (span_plane(tip_x + 20, tip_y + 40, tip_z + 25).rotated((45, 0, 0)), 95, -1),
        (span_plane(tip_x + 45, tip_y + 55, tip_z + 70).rotated((80, 0, 0)), 80, 0),
        (span_plane(tip_x + 90, tip_y + 60, tip_z + 140).rotated((88, 0, 0)), 55, 0),
    ]
    half = wing_blend(blend_start) + surface("2412", [blend_start, crank]) + surface("2412", [crank, tip]) + surface("2412", winglet)
    # Ailerons on the outer packed panel, hinged at 70% chord
    p0, p1 = chord_point(crank, 0.7), chord_point(tip, 0.7)
    hinge = (lerp(p0, p1, 0.35), lerp(p0, p1, 0.92))
    right, aileron_right = hinged(half, *hinge)
    left, aileron_left = hinged(mirror(half, Plane.XZ), *map(mirror_y, hinge))
    return right + left, aileron_right, aileron_left


def horizontal_tail():
    """Returns (tail, right elevator, left elevator)."""
    root, tip = tail_stations()
    half = surface("0010", [root, tip])
    p0, p1 = chord_point(root, 0.65), chord_point(tip, 0.65)
    hinge = (lerp(p0, p1, 0.06), lerp(p0, p1, 0.98))
    right, elevator_right = hinged(half, *hinge)
    left, elevator_left = hinged(mirror(half, Plane.XZ), *map(mirror_y, hinge))
    return right + left, elevator_right, elevator_left


def fin():
    """Returns (fin, rudder). The rudder stops short of the boom and the fin-tip motor pod; a
    dorsal fillet ahead of the hinge line blends the fin's root into the boom."""
    bottom, top = fin_stations()
    p0, p1 = chord_point(bottom, 0.65), chord_point(top, 0.65)
    fixed, rudder = hinged(surface("0010", [bottom, top]), lerp(p0, p1, 0.11), lerp(p0, p1, 0.86))

    def dorsal(z, leading_edge):
        hinge_x = lerp(p0.X, p1.X, (z - bottom[0].origin.Z) / (top[0].origin.Z - bottom[0].origin.Z)) - 2
        chord = hinge_x - leading_edge
        return fin_plane(leading_edge + chord / 4, z), chord, 0

    return fixed + surface("0010", [dorsal(38, 880), dorsal(70, 975), dorsal(115, 1025)]), rudder


def nacelle(nose, length, inset=0):
    """Fixed pod running aft from a motor's nose point; with `inset`, the cavity inside it."""
    x, y, z = nose
    run = length - 2 * inset
    return loft([yz_plane(x + inset + f * run, y, z) * Circle(r - inset) for f, r in NACELLE])


def hinge(nose):
    """Tilt hinge point of the motor in front of a nacelle."""
    x, y, z = nose
    return (x, y, z + HINGE_UP)


def mount():
    """Tilting motor mount, in hinge coordinates: a back plate for the motor, a tab up to the
    hinge and a knuckle that the servo's output spline drives, bored for the hinge pin."""
    plate = Pos(-MOTOR_GAP + 1.5, 0, -HINGE_UP) * Rot(0, 90, 0) * Cylinder(POD_RADIUS, 3)
    tab = Pos(-MOTOR_GAP / 2, 0, -4) * Box(MOTOR_GAP, 8, 8)
    knuckle = Rot(90, 0, 0) * Cylinder(4.5, 8)
    bore = Rot(90, 0, 0) * Cylinder(1.6, 10)
    return plate + tab + knuckle - bore


def motor_unit(name, nose):
    """Motor located at its hinge: mount, motor (can and long shaft), prop nut and propeller.
    The motor axis runs HINGE_UP below the hinge, on the nacelle axis. Tilting motors start in
    cruise; the rear motor is fixed upright."""
    axis_z = -HINGE_UP
    can = Pos(-MOTOR_GAP - CAN_LENGTH / 2, 0, axis_z) * Rot(0, 90, 0) * Cylinder(MOTOR["diameter"] / 2, CAN_LENGTH)
    shaft = Pos(-MOTOR_GAP - CAN_LENGTH - SHAFT_OUT / 2, 0, axis_z) * Rot(0, 90, 0) * Cylinder(MOTOR["shaft"] / 2, SHAFT_OUT)
    nut = Pos(HUB_X - PROP["hub_thickness"] / 2, 0, axis_z) * nose_cap(10, 7, 7)
    unit = Compound(
        label=name,
        children=[
            labeled(mount(), "mount", "darkslategray"),
            labeled(can + shaft, "motor", "dimgray"),
            labeled(nut, "prop_nut", "silver"),
            labeled(Pos(HUB_X, 0, axis_z) * propeller(PROP_RADIUS, PROP["pitch"]), "propeller", "black"),
        ],
    )
    unit.location = Location(hinge(nose)) * (Location() if name in TILTING else Rot(0, 90, 0))
    return unit


def hinge_hardware(name, nose):
    """Fixed half of a motor's hinge, set into the top of the nacelle front, located at the hinge.
    A tilting motor gets a direct-drive tilt servo whose output spline is the hinge axis (it
    drives the mount's knuckle), a bearing lug on the other side and the hinge pin. The fixed
    rear motor gets a lug on each side, with the pin bolting its mount upright."""
    length, width, height = TILT_SERVO["size"]
    lug = Box(12, 6, 16)
    children = []
    if name in TILTING:
        # Spline 6 mm below the servo's top, which stands 6 mm above the hinge
        servo = Pos(0, 4.5 + height / 2, 6 - length / 2) * Box(width, height, length)
        children.append(labeled(servo, "tilt_servo", "royalblue"))
        bearing, pin_end = Pos(0, -7.5, -2) * lug, 4.5
    else:
        bearing, pin_end = Pos(0, -7.5, -2) * lug + Pos(0, 7.5, -2) * lug, 10.5
    pin = Pos(0, (pin_end - 10.5) / 2, 0) * Rot(90, 0, 0) * Cylinder(1.5, pin_end + 10.5)
    children += [labeled(bearing, "bearing", "darkslategray"), labeled(pin, "hinge_pin", "silver")]
    unit = Compound(label=f"{name}_hinge", children=children)
    unit.location = Location(hinge(nose))
    return unit


def spar_lines():
    """Carbon tubes, all 1 mm wall: label -> (start, end, outer radius). Each wing spar is one
    straight line in three lengths (root tube, inner, outer), and a loose joiner tube bridges each
    dry joint, slid inside the spars. The boom tube plugs into a socket in the pod. Tail spars run
    along the 30% chord line, and the fin tube from inside the boom up into the tail nacelle."""
    tail_root, tail_tip = tail_stations()
    bottom, top = fin_stations()
    tail = (chord_point(tail_root, 0.3), surface_point(tail_root, tail_tip, 0.7, 0.3))
    boom_start, boom_end = Vector(BOOM_SOCKET_START, 0, 22), Vector(1030, 0, 43)
    socket_end = lerp(boom_start, boom_end, (BOOM_JOINT_X - boom_start.X) / (boom_end.X - boom_start.X))
    lines = {
        "fin_spar": (chord_point(bottom, 0.3), chord_point(top, 0.3), 4),
        "boom_spar": (boom_start, boom_end, BOOM_RADIUS),
        "boom_socket": (boom_start, socket_end, BOOM_SOCKET_RADIUS),
    }
    (inner_joint, outer_joint), (inner_embed, outer_embed) = WING_JOINTS, JOINER_EMBED
    wing_lines = {
        "wing_root_tube": (5, inner_joint, SPAR_RADIUS),
        "wing_spar_inner": (inner_joint, outer_joint, SPAR_RADIUS),
        "wing_spar_outer": (outer_joint, SPAR_END_Y, SPAR_RADIUS),
        "wing_joiner_inner": (inner_joint - inner_embed, inner_joint + inner_embed, JOINER_RADIUS),
        "wing_joiner_outer": (outer_joint - outer_embed, outer_joint + outer_embed, JOINER_RADIUS),
    }
    for side, flip in (("right", Vector), ("left", mirror_y)):
        for label, (y0, y1, radius) in wing_lines.items():
            lines[f"{label}_{side}"] = (flip(spar_point(y0)), flip(spar_point(y1)), radius)
        lines[f"tail_spar_{side}"] = (flip(tail[0]), flip(tail[1]), 3)
    return lines


def lying(size):
    """A servo's (length, width, height) laid flat in a thin surface: (x, y, z) = (L, H, W)."""
    length, width, height = size
    return length, height, width


def components():
    """Internal components as boxes of their datasheet sizes (parts.py): label -> (host part,
    center, size (x, y, z)). Each sits inside its host with at least SKIN to spare."""
    root, crank, tip = wing_stations()
    tail_root, tail_tip = tail_stations()
    bottom, top = fin_stations()
    aileron_servo = surface_point(crank, tip, 0.4, 0.5, camber=0.019)  # NACA 2412 camber at 50%
    elevator_servo = surface_point(tail_root, tail_tip, 0.35, 0.5)
    parts = {
        "battery": ("fuselage", Vector(BATTERY_CENTER), BATTERY["size"]),
        "computer": ("fuselage", Vector(COMPUTER_CENTER), COMPUTER["size"]),
        "flight_controller": ("fuselage", Vector(420, 0, 20), FLIGHT_CONTROLLER["size"]),
        "receiver": ("fuselage", Vector(490, 0, 0), RECEIVER["size"]),
        "gps": ("fuselage", Vector(700, 0, 45), GPS["size"]),
        "esc_tail": ("tail_nacelle", Vector(1140, 0, FIN_TIP_Z + 10), ESC["size"]),
        "rudder_servo": ("fin", surface_point(bottom, top, 0.3, 0.47), TAIL_SERVO["size"]),
    }
    for side, flip in (("right", Vector), ("left", mirror_y)):
        parts[f"esc_{side}"] = ("nacelles", flip(Vector(330, POD_Y, 47)), ESC["size"])
        parts[f"aileron_servo_{side}"] = ("wing", flip(aileron_servo), lying(AILERON_SERVO["size"]))
        parts[f"elevator_servo_{side}"] = ("horizontal_tail", flip(elevator_servo), lying(TAIL_SERVO["size"]))
    return parts


def surface_details():
    """Panel lines and vents, standing 0.3 mm proud of the skin: the hatch and boom seams, and
    three cooling slots each side of the computer."""
    _, battery_center, (battery_length, _, height) = components()["battery"]
    hatch_end = battery_center.X + battery_length / 2 + 5
    hatch_z = battery_center.Z + height / 2 + 2
    hatch_start = COMPUTER_CENTER[0] - COMPUTER["size"][0] / 2 - 5
    seam = 1.2
    tools = [
        Pos(hatch_start, 0, 0) * Box(seam, 400, 400),
        Pos(hatch_end, 0, 0) * Box(seam, 400, 400),
        Pos(BOOM_JOINT_X, 0, 0) * Box(seam, 400, 400),
        Pos((hatch_start + hatch_end) / 2, 0, hatch_z) * Box(hatch_end - hatch_start, 400, seam),
        *[Pos(COMPUTER_CENTER[0], 0, z) * Box(40, 400, 3) for z in (-22, -14, -6)],
    ]
    return (fuselage(-0.3) & reduce(add, tools)) - fuselage()


def center_fitting():
    """Printed block inside the fuselage that bonds the two root tubes where the spars meet in a
    shallow V (the sweep and dihedral of the straight spars)."""
    center = spar_point(25)
    sockets = [tube(flip(spar_point(5)), flip(spar_point(60)), SPAR_RADIUS + 0.15) for flip in (Vector, mirror_y)]
    return Pos(center.X, 0, center.Z) * Box(44, 100, 30) - sockets[0] - sockets[1]


def landing_gear():
    """Two skids under the wing pods: a faired rail on one faired strut that runs up through the
    pod to the wing spar, so landing loads go straight into the spar."""
    strut_x = spar_point(POD_Y).X
    strut_top = spar_point(POD_Y).Z - SPAR_RADIUS - 1

    def skid(y):
        start, end = RAIL_X
        rail = loft([yz_plane(lerp(start, end, f), y, SKID_Z) * Circle(RAIL_RADIUS * k) for f, k in RAIL], ruled=True)  # ruled: never bulges past RAIL_RADIUS
        strut = Pos(0, y, 0) * surface("0025", [(fin_plane(strut_x, SKID_Z), STRUT_CHORD, 0), (fin_plane(strut_x, strut_top), STRUT_CHORD, 0)])
        return rail + strut

    return skid(POD_Y) + skid(-POD_Y)


def gimbal():
    """Pan turret under the nose: a ball carrying the thermal and color cameras, both looking 45 deg
    down and forward so they see past the belly it is half buried in. Located at the ball center."""

    def lens(y, radius):
        return Rot(0, -45, 0) * Pos(-30, y, 0) * Rot(0, 90, 0) * Cylinder(radius, 12)

    unit = Compound(
        label="gimbal",
        children=[
            labeled(Sphere(CAMERA_RADIUS), "camera", "darkslategray"),
            labeled(lens(-12, 11), "thermal_lens", "black"),
            labeled(lens(13, 6), "lens", "black"),
        ],
    )
    unit.location = Location(GIMBAL_CENTER)
    return unit


def build_vtol():
    wing_part, aileron_right, aileron_left = wing()
    tail, elevator_right, elevator_left = horizontal_tail()
    fin_part, rudder = fin()
    nacelles = Compound([nacelle(*MOTORS[name]) for name in TILTING])
    spars = [labeled(tube(start, end, r, bore=r - 1), label, "black") for label, (start, end, r) in spar_lines().items()]
    avionics = [labeled(Pos(center) * Box(*size), label, "orange") for label, (_, center, size) in components().items()]
    return Compound(
        label="vtol",
        children=[
            labeled(fuselage(), "fuselage", CARBON),
            labeled(surface_details(), "details", DARK),
            labeled(wing_part, "wing", CARBON),
            labeled(tail, "horizontal_tail", CARBON),
            labeled(fin_part, "fin", CARBON),
            labeled(aileron_right, "aileron_right", ORANGE),
            labeled(aileron_left, "aileron_left", ORANGE),
            labeled(elevator_right, "elevator_right", ORANGE),
            labeled(elevator_left, "elevator_left", ORANGE),
            labeled(rudder, "rudder", ORANGE),
            labeled(nacelles, "nacelles", ORANGE),
            labeled(nacelle(*MOTORS["motor_tail"]), "tail_nacelle", CARBON),
            *[motor_unit(name, nose) for name, (nose, _) in MOTORS.items()],
            Compound(label="hinges", children=[hinge_hardware(name, nose) for name, (nose, _) in MOTORS.items()]),
            Compound(label="spars", children=spars),
            labeled(center_fitting(), "center_fitting", "dimgray"),
            Compound(label="avionics", children=avionics),
            labeled(landing_gear(), "landing_gear", BLACK),
            labeled(Pos(GIMBAL_CENTER[0], 0, -32.5) * Cylinder(12, 15), "gimbal_mount", "dimgray"),
            gimbal(),
        ],
    )


if __name__ == "__main__":
    export_and_show(build_vtol(), "vtol")

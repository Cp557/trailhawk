"""Printable airframe: the printed parts of build_vtol(), cut into pieces that fit a print bed.

Every printed part is a SKIN-thick shell. The fuselage and nacelles are hollowed here; the
solid pieces (wing, tail, fin, control surfaces) print as shells with 2 perimeters (0.8 mm) and
0-5% infill. Spar tubes run through channels and key the wing and tail joints, joiner sleeves
glued inside the hollow fuselage and nacelles key theirs, components sit in pockets, the
fuselage has a hatch over the battery and computer, and the wing and tail roots are trimmed to
the fuselage skin so they glue onto it. The wing and fuselage are first cut at the dry joints
(WING_JOINTS, BOOM_JOINT_X) that come apart for packing; those are not glued.

Run this file to write out/print/<piece>.stl and show the pieces in the viewer.
"""

from functools import reduce
from operator import add

import numpy as np
from build123d import Box, Compound, Keep, Plane, Pos, Sphere, export_stl, split

from drone.cad import OUT_DIR, labeled, show_in_viewer
from drone.vtol import BOOM_JOINT_X, CAMERA_RADIUS, GIMBAL_CENTER, MOTORS, SKIN, WING_CUTS, build_vtol, components, fuselage, hatch, nacelle, spar_lines, tube

PRINT_BED = (256, 256, 256)  # mm, e.g. a Bambu Lab X1, P1 or A1
FIT = 0.15  # radial clearance around the spar tubes
SLEEVE = (16, 1.2)  # joiner sleeve: width across the cut, wall; glued inside hollow parts at every cut
X, Y, Z = range(3)


def fits_bed(piece, bed=PRINT_BED):
    """True if the piece fits the bed in some orientation: any axis up, any yaw in 5 deg steps."""
    points = np.array([tuple(v) for v in piece.tessellate(0.5)[0]])
    for up in (X, Y, Z):
        if np.ptp(points[:, up]) > bed[2]:
            continue
        flat = np.delete(points, up, axis=1)
        for angle in np.radians(range(0, 180, 5)):
            turned = flat @ [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
            if np.all(np.ptp(turned, axis=0) <= bed[:2]):
                return True
    return False


def axis_plane(axis, position):
    origin, normal = [0, 0, 0], [0, 0, 0]
    origin[axis], normal[axis] = position, 1
    return Plane(origin, z_dir=normal)


def slices(solid, axis, cuts):
    """Cut a solid across a world axis (X, Y or Z) at each position in `cuts`."""
    pieces, rest = [], solid
    for cut in cuts:
        plane = axis_plane(axis, cut)
        pieces += split(rest, plane, keep=Keep.BOTTOM).solids()
        rest = split(rest, plane, keep=Keep.TOP)
    return pieces + rest.solids()


def cut_to_fit(solid, axis):
    """(slabs, cut positions): the fewest equal slabs across `axis` that each fit the print bed."""
    box = solid.bounding_box()
    lo, hi = tuple(box.min)[axis], tuple(box.max)[axis]
    for count in range(1, 9):
        cuts = [lo + (hi - lo) * i / count for i in range(1, count)]
        pieces = slices(solid, axis, cuts)
        if all(fits_bed(piece) for piece in pieces):
            return pieces, cuts
    raise ValueError("no slab count fits the print bed")


def band(axis, position):
    """Slab SLEEVE[0] wide across `axis`, centered on a cut."""
    center, size = [0, 0, 0], [4000, 4000, 4000]
    center[axis], size[axis] = position, SLEEVE[0]
    return Pos(*center) * Box(*size)


def sleeves(cavity, core, trim):
    """Joiner sleeve maker for a hollow part: (axis, cut) -> the band between the part's cavity
    and a core SLEEVE[1] smaller, SLEEVE[0] wide across the cut, trimmed like the part."""

    def at(axis, position):
        ring = band(axis, position)
        return trim((cavity & ring) - (core & ring)).solids()

    return at


def union(shapes):
    """Fuse shapes into one cutting tool: OCCT booleans misbehave with overlapping tools."""
    return reduce(add, shapes)


def printed_parts(vtol):
    """Printed part -> (solid ready to print, axis it is cut across, sleeve maker or None),
    before cutting. Only the hollow parts get joiner sleeves."""
    parts = {child.label: child for child in vtol.children}
    outer = parts["fuselage"]
    channels = union([tube(start, end, r + FIT) for start, end, r in spar_lines().values()])
    pockets = union([Pos(center) * Box(*size) for _, center, size in components().values()])
    hinges = [hinge.location * part for hinge in parts["hinges"].children for part in hinge.children]
    mounts = [unit.location * part for unit in parts.values() if unit.label in MOTORS for part in unit.children]

    hatch_start, hatch_end, hatch_z = hatch()
    hatch_box = Pos((hatch_start + hatch_end) / 2, 0, hatch_z + 100) * Box(hatch_end - hatch_start, 400, 200)
    turret_opening = Pos(*GIMBAL_CENTER) * Sphere(CAMERA_RADIUS + 1.5)
    shell = outer - fuselage(SKIN) - channels - turret_opening
    fuselage_sleeves = sleeves(fuselage(SKIN), fuselage(SKIN + SLEEVE[1]), lambda shape: shape - hatch_box - channels)

    def nacelle_trim(shape):
        return shape - parts["wing"] - parts["fin"] - parts["landing_gear"] - union(hinges) - channels

    nacelles = Compound([nacelle(nose, run) - nacelle(nose, run, SKIN) for nose, run in MOTORS.values()])
    nacelle_sleeves = sleeves(
        Compound([nacelle(nose, run, SKIN) for nose, run in MOTORS.values()]),
        Compound([nacelle(nose, run, SKIN + SLEEVE[1]) for nose, run in MOTORS.values()]),
        nacelle_trim,
    )

    trimmed = {label: parts[label] - outer - channels - pockets for label in ("wing", "horizontal_tail")}
    wing_panels = Compound([panel for solid in (trimmed["wing"] - parts["landing_gear"]).solids() for panel in slices(solid, Y, WING_CUTS)])
    return {
        "fuselage": (Compound(slices(shell - hatch_box, X, [BOOM_JOINT_X])), X, fuselage_sleeves),
        "hatch": (shell & hatch_box, X, None),
        "wing": (wing_panels, Y, None),
        "horizontal_tail": (trimmed["horizontal_tail"], Y, None),
        "fin": (parts["fin"] - outer - parts["tail_nacelle"] - channels - pockets, Z, None),
        "nacelles": (nacelle_trim(nacelles), X, nacelle_sleeves),
        **{label: (parts[label], Y, None) for label in ("aileron_right", "aileron_left", "elevator_right", "elevator_left")},
        "rudder": (parts["rudder"], Z, None),
        "mount": (Compound([part for part in mounts if part.label == "mount"]), X, None),
        "bearing": (Compound([part for part in hinges if part.label == "bearing"]), X, None),
        "center_fitting": (parts["center_fitting"], X, None),
        "landing_gear": (parts["landing_gear"], X, None),
    }


def print_pieces(vtol):
    """Piece name -> solid: each printed part split into its separate solids, each of those cut
    into the fewest bed-sized slabs, plus a joiner sleeve across every cut in a hollow part."""
    pieces = {}
    for label, (solid, axis, sleeves_at) in printed_parts(vtol).items():
        slabs, joiners = [], []
        for part in solid.solids():
            cut, cuts = cut_to_fit(part, axis)
            slabs += cut
            if sleeves_at:
                near = part.bounding_box()
                joiners += [s for at in cuts for s in sleeves_at(axis, at) if s.bounding_box().intersects(near)]
        pieces.update({f"{label}_{i}": piece for i, piece in enumerate(slabs, 1)})
        pieces.update({f"{label}_sleeve_{i}": piece for i, piece in enumerate(joiners, 1)})
    return pieces


def export_pieces(pieces):
    """Write out/print/<piece>.stl and show the pieces in the viewer, in alternating colors."""
    folder = OUT_DIR / "print"
    folder.mkdir(parents=True, exist_ok=True)
    for name, piece in pieces.items():
        export_stl(piece, folder / f"{name}.stl", tolerance=0.05)
    colors = ("gainsboro", "lightsteelblue")
    shown = [labeled(piece, name, colors[i % 2]) for i, (name, piece) in enumerate(pieces.items())]
    show_in_viewer(Compound(label="print", children=shown))


if __name__ == "__main__":
    pieces = print_pieces(build_vtol())
    print(f"{len(pieces)} pieces written to {OUT_DIR / 'print'}")
    export_pieces(pieces)

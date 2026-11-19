"""Packing list: the sections the VTOL comes apart into on the trail, and whether each fits a pack.

A section stays assembled in the field. The wing separates at WING_JOINTS (loose joiner tubes),
the boom at BOOM_JOINT_X, the skids and props pull off, and the rear motor unbolts from the fin.
The joiner tubes and batteries are carried loose. Run this file for a report.
"""

from build123d import Compound
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from trailhawk.cad import world_parts
from trailhawk.print_parts import X, Y, fits_bed, slices
from trailhawk.vtol import BOOM_JOINT_X, WING_CUTS, WING_JOINTS, build_vtol

PACK = (700, 350, 260)  # mm, the usable inside of a 65 L backpacking pack


def wing_section(y):
    inner, outer = WING_JOINTS
    side = "right" if y > 0 else "left"
    if abs(y) < inner:
        return "pod"
    return f"wing_inner_{side}" if abs(y) < outer else f"wing_outer_{side}"


def section_of(path):
    """Which packed section the leaf at `path` belongs to."""
    label = path.rsplit("/", 1)[1]
    side = "right" if "right" in path else "left"
    if path.endswith("/propeller"):
        return f"prop_{path.split('/')[1].removeprefix('motor_')}"
    if "motor_tail" in path:
        return "rear_motor"
    if "motor_right" in path or "motor_left" in path or label.startswith("esc_") and label != "esc_tail":
        return f"wing_inner_{side}"
    if "joiner" in label:
        return label.removeprefix("wing_")
    if label.startswith("wing_spar"):
        return f"wing_{label.split('_')[2]}_{side}"
    if label.startswith("aileron"):
        return f"wing_outer_{side}"
    if label.startswith(("elevator", "horizontal_tail", "tail_spar")):
        return "tail"
    if label in ("fin", "tail_nacelle", "rudder", "rudder_servo", "fin_spar", "boom_spar", "gps", "esc_tail"):
        return "boom"
    return "pod"


def field_sections(vtol):
    """Section name -> Compound of its parts in assembly position."""
    sections = {}

    def add(name, shape):
        sections.setdefault(name, []).append(shape)

    for path, shape in world_parts(vtol):
        label = path.rsplit("/", 1)[1]
        if label == "wing":
            for piece in slices(shape, Y, WING_CUTS):
                add(wing_section(piece.bounding_box().center().Y), piece)
        elif label == "fuselage":
            for piece in slices(shape, X, [BOOM_JOINT_X]):
                add("pod" if piece.bounding_box().center().X < BOOM_JOINT_X else "boom", piece)
        elif label == "nacelles":
            for piece in shape.solids():
                add(wing_section(piece.bounding_box().center().Y), piece)
        elif label == "landing_gear":
            for piece in shape.solids():
                add("skid_right" if piece.bounding_box().center().Y > 0 else "skid_left", piece)
        else:
            add(section_of(path), shape)
    return {name: Compound(parts) for name, parts in sections.items()}


def litres(section):
    """Space the section's solids take up: build123d's default volume undercounts lofts."""
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(section.wrapped, properties, 1e-3)
    return properties.Mass() / 1e6


def report(vtol):
    sections = field_sections(vtol)
    print(f"Pack envelope {PACK[0]} x {PACK[1]} x {PACK[2]} mm")
    for name, section in sections.items():
        size = sorted(section.bounding_box().size, reverse=True)
        print(f"  {name:20s} {size[0]:5.0f} x {size[1]:4.0f} x {size[2]:4.0f} mm  {'fits' if fits_bed(section, PACK) else 'DOES NOT FIT'}")
    print(f"Sections take up {sum(map(litres, sections.values())):.0f} L of solid volume, before the gaps between them")


if __name__ == "__main__":
    report(build_vtol())

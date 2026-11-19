"""Shared build123d helpers: NACA sections and lofted surfaces, the propeller, labels, an
assembly's parts in place, and exports (STEP and STL, one STL per part, the OCP CAD Viewer).

Axes: +X runs nose to tail, +Y is the right wing, +Z is up. Units are mm.
"""

import json
from math import atan, degrees, pi
from pathlib import Path

from build123d import Airfoil, Axis, Color, Cylinder, Location, Plane, Rot, export_step, export_stl, loft, make_face
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

OUT_DIR = Path(__file__).parents[1] / "out"


def span_plane(x, y, z):
    """Section plane facing along Y: local X is chordwise (+X), local Y is up (+Z)."""
    return Plane((x, y, z), x_dir=(1, 0, 0), z_dir=(0, -1, 0))


def fin_plane(x, z):
    """Section plane facing along Z: local X is chordwise (+X), local Y is +Y."""
    return Plane((x, 0, z), x_dir=(1, 0, 0), z_dir=(0, 0, 1))


def section(naca, chord, plane, pitch=0.0):
    """NACA section with its quarter chord at the plane origin, pitched nose-up by `pitch` degrees."""
    profile = make_face(Airfoil(naca, finite_te=True)).face()
    profile = profile.scale(chord).translate((-chord / 4, 0, 0)).rotate(Axis.Z, -pitch)
    return plane * profile


def surface(naca, stations):
    """Loft NACA sections given as (plane, chord, pitch) stations."""
    return loft([section(naca, chord, plane, pitch) for plane, chord, pitch in stations])


def propeller(radius, pitch):
    """Two-blade propeller with its hub at the origin, spinning about X and pulling toward -X."""
    # (radius, chord); blade angle follows the geometric pitch atan(P / 2*pi*r)
    stations = [(6, 18), (40, 24), (80, 20), (radius, 8)]  # root sits inside the hub
    blade = surface(
        "4410",
        [
            (
                Plane((0, 0, r), x_dir=(0, 1, 0), z_dir=(0, 0, 1)),
                chord,
                degrees(atan(pitch / (2 * pi * r))),
            )
            for r, chord in stations
        ],
    )
    hub = Rot(0, 90, 0) * Cylinder(8, 10)
    return hub + blade + blade.rotate(Axis.X, 180)


def labeled(shape, label, color):
    shape.label, shape.color = label, Color(color)
    return shape


def world_parts(node, location=Location(), path=""):
    """(path, shape in world coordinates) for every leaf of an assembly, as built."""
    for child in node.children:
        child_path = f"{path}/{child.label}"
        if child.children:
            yield from world_parts(child, location * child.location, child_path)
        else:
            yield child_path, location * child


def show_in_viewer(*shapes):
    """Send shapes to the OCP CAD Viewer on port 3939, where both the VS Code extension and
    `python -m ocp_viewer` listen."""
    from ocp_vscode import set_port, show

    set_port(3939)
    show(*shapes, reset_camera="reset")


def export_and_show(shape, name):
    """Write out/<name>.step and out/<name>.stl, then send the shape to the viewer."""
    OUT_DIR.mkdir(exist_ok=True)
    export_step(shape, OUT_DIR / f"{name}.step")
    export_stl(shape, OUT_DIR / f"{name}.stl", tolerance=0.05)
    show_in_viewer(shape)


def export_parts(assembly, folder, deflection, angle):
    """Write one STL per leaf of `assembly` (in place, its color from its node) to `folder`, and a
    manifest of (file, path, color) as parts.json. Facets stray at most `deflection` mm from the
    surface and turn at most `angle` rad between neighbors (build123d's export_stl takes the
    deflection as a fraction of each edge's length instead, so big curved skins come out coarse)."""
    folder.mkdir(parents=True, exist_ok=True)
    manifest = []
    for i, (path, shape) in enumerate(world_parts(assembly)):
        name = f"{i:03d}.stl"
        BRepMesh_IncrementalMesh(shape.wrapped, deflection, False, angle, True)
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        writer.Write(shape.wrapped, str(folder / name))
        manifest.append({"file": name, "path": path, "color": list(shape.color)[:3]})
    (folder / "parts.json").write_text(json.dumps(manifest, indent=1))

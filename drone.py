"""Fixed-wing drone modeled with build123d.

Axes: +X runs nose to tail, +Y is the right wing, +Z is up. Units are mm.
Run this file to export STEP/STL into out/ and send the model to the OCP CAD Viewer.
"""

from math import atan, degrees, pi, radians, tan
from pathlib import Path

from build123d import (
    Airfoil,
    Align,
    Axis,
    Color,
    Compound,
    Cone,
    Cylinder,
    Ellipse,
    Plane,
    Pos,
    Rot,
    export_step,
    export_stl,
    loft,
    make_face,
    mirror,
)

OUT_DIR = Path(__file__).parent / "out"

WING_HALF_SPAN = 700
PROP_X = -38
PROP_RADIUS = 127  # 10x5 in propeller
PROP_PITCH = 127


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


def fuselage():
    # (x, half width, half height, centerline z)
    stations = [
        (0, 30, 34, 0),
        (80, 52, 58, 4),
        (250, 56, 64, 10),
        (480, 50, 56, 14),
        (700, 22, 26, 26),
        (920, 8, 10, 32),
    ]
    return loft(
        [
            Plane((x, 0, z), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * Ellipse(w, h)
            for x, w, h, z in stations
        ]
    )


def wing():
    tip_z = 35 + WING_HALF_SPAN * tan(radians(3))  # 3 deg dihedral
    right = surface(
        "2412",
        [
            (span_plane(300, 0, 35), 240, 2),  # 2 deg root incidence
            (span_plane(300, WING_HALF_SPAN, tip_z), 150, 0),  # 2 deg washout
        ],
    )
    return right + mirror(right, Plane.XZ)


def horizontal_stabilizer():
    right = surface("0010", [(span_plane(810, 0, 30), 140, 0), (span_plane(820, 250, 30), 100, 0)])
    return right + mirror(right, Plane.XZ)


def vertical_stabilizer():
    return surface("0010", [(fin_plane(800, 30), 160, 0), (fin_plane(850, 210), 80, 0)])


def motor():
    return Pos(-14, 0, 0) * Rot(0, 90, 0) * Cylinder(14, 28)


def spinner():
    tip_up = (Align.CENTER, Align.CENTER, Align.MIN)
    return Pos(-30, 0, 0) * Rot(0, -90, 0) * Cone(16, 0, 36, align=tip_up)


def propeller(radius=PROP_RADIUS, pitch=PROP_PITCH):
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


def build_drone():
    return Compound(
        label="drone",
        children=[
            labeled(fuselage(), "fuselage", "whitesmoke"),
            labeled(wing(), "wing", "steelblue"),
            labeled(horizontal_stabilizer(), "horizontal_stabilizer", "steelblue"),
            labeled(vertical_stabilizer(), "vertical_stabilizer", "steelblue"),
            labeled(motor(), "motor", "dimgray"),
            labeled(spinner(), "spinner", "orangered"),
            labeled(Pos(PROP_X, 0, 0) * propeller(), "propeller", "black"),
        ],
    )


def export_and_show(shape, name):
    """Write out/<name>.step and out/<name>.stl, then send the shape to the OCP CAD Viewer."""
    from ocp_vscode import set_port, show

    OUT_DIR.mkdir(exist_ok=True)
    export_step(shape, OUT_DIR / f"{name}.step")
    export_stl(shape, OUT_DIR / f"{name}.stl", tolerance=0.05)
    set_port(3939)  # default port of both the VS Code extension and `python -m ocp_viewer`
    show(shape, reset_camera="reset")


if __name__ == "__main__":
    export_and_show(build_drone(), "drone")

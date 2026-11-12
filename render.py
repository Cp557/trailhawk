"""Photoreal renders of the VTOL in Blender: studio on light grey, and flying over a pine forest.

Exports every part to out/render/ with its paint color, then runs Blender headless on
render_scene.py. Writes out/render/studio.png and out/render/aerial.png.
Set BLENDER if blender.exe is somewhere other than BLENDER_DEFAULT.
"""

import json
import os
import subprocess
from pathlib import Path

from build123d import export_stl

from drone import OUT_DIR
from pack import world_parts
from vtol import build_vtol

BLENDER_DEFAULT = r"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"
FOLDER = OUT_DIR / "render"


def export_parts(vtol, folder=FOLDER, tolerance=0.05, hidden=()):
    """Write one STL per leaf (the color comes from its node) and a manifest of (file, path, color)
    to `folder`, skipping leaves whose path starts with one of `hidden`."""
    folder.mkdir(parents=True, exist_ok=True)
    manifest = []
    for i, (path, shape) in enumerate(world_parts(vtol)):
        if path.startswith(hidden):
            continue
        name = f"{i:03d}.stl"
        export_stl(shape, folder / name, tolerance=tolerance, angular_tolerance=2 * tolerance)
        manifest.append({"file": name, "path": path, "color": list(shape.color)[:3]})
    (folder / "parts.json").write_text(json.dumps(manifest, indent=1))


def main():
    # Front motors in cruise; world_parts composes the assembly's locations
    export_parts(build_vtol())
    blender = os.environ.get("BLENDER", BLENDER_DEFAULT)
    scene = Path(__file__).parent / "render_scene.py"
    subprocess.run([blender, "--background", "--factory-startup", "--python-exit-code", "1", "--python", str(scene), "--", str(FOLDER)], check=True)
    print(f"Renders written to {FOLDER}")


if __name__ == "__main__":
    main()

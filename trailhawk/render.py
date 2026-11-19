"""Photoreal renders of the VTOL in Blender: studio on light grey, and flying over a pine forest.

Exports every part to out/render/ with its paint color, then runs Blender headless on
render_scene.py. Writes out/render/studio.png and out/render/aerial.png.
Set BLENDER if Blender is somewhere other than BLENDER_DEFAULT (per operating system).
"""

import os
import subprocess
import sys
from pathlib import Path

from trailhawk.cad import OUT_DIR, export_parts
from trailhawk.vtol import build_vtol

BLENDER_DEFAULT = {
    "win32": r"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe",
    "darwin": "/Applications/Blender.app/Contents/MacOS/Blender",
}.get(sys.platform, "blender")
FOLDER = OUT_DIR / "render"


def main():
    # Front motors in cruise; world_parts composes the assembly's locations. Fine meshes (0.2 mm,
    # 0.1 rad) keep close-up highlights smooth.
    export_parts(build_vtol(), FOLDER, 0.2, 0.1)
    blender = os.environ.get("BLENDER", BLENDER_DEFAULT)
    scene = Path(__file__).parent / "render_scene.py"
    subprocess.run([blender, "--background", "--factory-startup", "--python-exit-code", "1", "--python", str(scene), "--", str(FOLDER)], check=True)
    print(f"Renders written to {FOLDER}")


if __name__ == "__main__":
    main()

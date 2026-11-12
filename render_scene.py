"""Blender script (run by render.py): builds the studio and aerial scenes and renders them."""

import json
import math
import random
import sys
from pathlib import Path

import bpy
from mathutils import Euler, Vector

FOLDER = Path(sys.argv[sys.argv.index("--") + 1])
SIZE = (1600, 900)
SAMPLES = 96
METRES = 0.001  # the parts are in mm


def linear(channel):
    """sRGB channel to Blender's linear color space."""
    return channel / 12.92 if channel <= 0.04045 else ((channel + 0.055) / 1.055) ** 2.4


def material(name, color, roughness=0.42, coat=0.0):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    shader = result.node_tree.nodes["Principled BSDF"]
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Coat Weight"].default_value = coat
    shader.inputs["Coat Roughness"].default_value = 0.06
    return result


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def import_aircraft():
    """Import every part under one parent empty, so the aircraft can be posed as a whole."""
    root = bpy.data.objects.new("aircraft", None)
    bpy.context.collection.objects.link(root)
    for part in json.loads((FOLDER / "parts.json").read_text()):
        bpy.ops.wm.stl_import(filepath=str(FOLDER / part["file"]))
        obj = bpy.context.selected_objects[0]
        obj.parent = root
        obj.scale = (METRES, METRES, METRES)
        bpy.ops.object.shade_auto_smooth(angle=math.radians(20))
        color = [linear(c) for c in part["color"]]
        dark = sum(color) < 0.15  # carbon and black parts: satin, with a light clear coat
        obj.data.materials.append(material(part["path"], color, roughness=0.6 if dark else 0.38, coat=0.3 if dark else 0.0))
    return root


def add_light(kind, location, energy, size=1.0, rotation=(0, 0, 0)):
    data = bpy.data.lights.new(kind, kind)
    data.energy = energy
    if kind == "AREA":
        data.size = size
    obj = bpy.data.objects.new(kind, data)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    obj.rotation_euler = rotation
    return obj


def aim(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()


def add_camera(location, target, lens=70):
    data = bpy.data.cameras.new("camera")
    data.lens = lens
    obj = bpy.data.objects.new("camera", data)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    aim(obj, target)
    bpy.context.scene.camera = obj


def configure(output, world_color, strength, lighting=None):
    """Render settings and world. With `lighting`, the camera sees `world_color` but reflections
    and fill light see `lighting`: a light backdrop that doesn't wash out dark gloss."""
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = SAMPLES
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = SIZE
    scene.render.filepath = str(FOLDER / output)
    scene.render.image_settings.file_format = "PNG"
    world = bpy.data.worlds.new("world")
    world.use_nodes = True
    nodes, links = world.node_tree.nodes, world.node_tree.links
    background = nodes["Background"]
    background.inputs["Color"].default_value = (*world_color, 1)
    background.inputs["Strength"].default_value = strength
    if lighting is not None:
        unseen = nodes.new("ShaderNodeBackground")
        unseen.inputs["Color"].default_value = (*lighting, 1)
        mix = nodes.new("ShaderNodeMixShader")
        links.new(nodes.new("ShaderNodeLightPath").outputs["Is Camera Ray"], mix.inputs["Fac"])
        links.new(unseen.outputs["Background"], mix.inputs[1])
        links.new(background.outputs["Background"], mix.inputs[2])
        links.new(mix.outputs["Shader"], nodes["World Output"].inputs["Surface"])
    scene.world = world


def studio():
    """On a light grey backdrop: big soft lights above and behind, nose toward the camera."""
    clear_scene()
    root = import_aircraft()
    root.rotation_euler = Euler((0, 0, math.radians(180)))
    root.location = (0.45, 0, 0)
    add_light("AREA", (0, 0, 2.6), 170, size=4.5)
    for side in (-1, 1):
        area = add_light("AREA", (-1.8, 2.6 * side, 0.9), 70, size=2.0)
        aim(area, (0, 0, 0))
    rim = add_light("AREA", (2.2, 0, 2.4), 90, size=1.5)  # high, so the wings don't mirror it at this camera angle
    aim(rim, (0, 0, 0.1))
    add_camera((-2.9, -2.2, 1.0), (0.0, 0.0, 0.03), lens=62)
    configure("studio.png", (0.5, 0.5, 0.52), 1.0, lighting=(0.02, 0.02, 0.02))
    bpy.ops.render.render(write_still=True)


def forest(count=9000, radius=95):
    """A dark green ground scattered with cones of varied height and color: pines from above."""
    bpy.ops.mesh.primitive_plane_add(size=2 * radius)
    bpy.context.object.data.materials.append(material("ground", (0.018, 0.028, 0.012), roughness=0.95))
    random.seed(4)
    greens = [material(f"pine{i}", color, roughness=0.8) for i, color in enumerate([(0.012, 0.045, 0.018), (0.016, 0.06, 0.02), (0.022, 0.07, 0.025), (0.01, 0.035, 0.016)])]
    bpy.ops.mesh.primitive_cone_add(vertices=9, radius1=1, radius2=0, depth=1)
    templates = []
    for material_i in greens:
        template = bpy.context.object.copy()
        template.data = bpy.context.object.data.copy()
        template.data.materials.append(material_i)
        templates.append(template)
    for _ in range(count):
        tree = random.choice(templates).copy()
        bpy.context.collection.objects.link(tree)
        height = random.uniform(9, 20)
        width = height * random.uniform(0.13, 0.2)
        tree.scale = (width, width, height)
        tree.location = (random.uniform(-radius, radius), random.uniform(-radius, radius), height / 2)


def aerial():
    """Banked in the orbit, seen from above over the trees."""
    clear_scene()
    forest()
    root = import_aircraft()
    root.location = (0, 0, 95)
    root.rotation_euler = Euler((math.radians(-16), math.radians(3), math.radians(205)))
    sun = add_light("SUN", (0, 0, 150), 4.2, rotation=(math.radians(40), math.radians(16), math.radians(35)))
    sun.data.angle = math.radians(2)
    add_camera((-1.4, -2.6, 103.5), (0, 0, 95), lens=55)
    configure("aerial.png", (0.55, 0.68, 0.85), 0.9)
    bpy.ops.render.render(write_still=True)


studio()
aerial()

#!/usr/bin/env python3
"""Build the storage zones of Our_Structured_Warehouse.

Generates pallet-rack models, uniquely labelled box textures, per-zone floor markings and signs,
the shipping dock (conveyor, truck, drop-off points), cleans unused parts out of the workcell
mesh, and writes the storage section of the world.
The conveyor needs its plugin: run Sim/plugins/build_plugins.sh once.

Run from the repository root:  python3 Sim/tools/build_our_warehouse_storage.py
Re-running replaces everything it generated before.
"""
import math
import random
import re
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SIM = Path(__file__).resolve().parent.parent
MODELS = SIM / 'models'
WORLD = SIM / 'worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world'
WORKCELL_MESH = MODELS / 'our_workcell/meshes/mesh.dae'
WORKCELL_TEXTURES = MODELS / 'our_workcell/materials/textures'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

RACK_DEPTH = 0.84
RACK_HEIGHT = 2.0
POST = 0.07
POST_X = RACK_DEPTH / 2 - 0.04
SIDE_OFFSET = 0.205                  # centre of a half-depth box from the rack centre line
LEVEL_SETS = {
    5: (0.1, 0.55, 1.0, 1.45, 1.9),  # small / long boxes
    4: (0.1, 0.68, 1.26, 1.84),      # large boxes need more headroom
}

# name: (depth across rack, width along rack, height, mass)
BOXES = {
    'small': (0.4, 0.5, 0.3, 2.0),
    'long': (0.4, 0.85, 0.2, 4.0),
    'large': (0.78, 0.6, 0.45, 8.0),
}

# Layout in the workcell frame (every module uses the same layout, rotated with the module).
# Rows run along y; aisles between rows are 1.56 m wide, the main aisles along the
# module's inner edges join the neighbouring zones.
ROWS = [  # (row number, x, [(y centre, length, bays), ...])
    (1, -1.1, [(-3.1, 5.8, 6)]),
    (2, 1.3, [(-5.6, 6.8, 7), (3.2, 6.8, 7)]),
    (3, 3.7, [(-5.6, 6.8, 7), (3.2, 6.8, 7)]),
    (4, 6.1, [(-5.6, 6.8, 7), (3.2, 6.8, 7)]),
    (5, 8.5, [(-5.6, 6.8, 7), (3.2, 6.8, 7)]),
]
PALLETS = [(-1.09806, 6.31883), (-5.29667, 1.91644)]   # pallets in the workcell mesh
PALLET_TOP = 0.3272
REMOVED_POLES = ('pole1', 'pole2')                      # pillars at x = 5.02 and x = 0.47
SOUTH_WALL_Y = -9.769

TILE = 20.1026
C0 = (-0.040107, 0.052292)
ZONES = [  # (letter, module, i, j, yaw, contents, colour, fill, levels)
    ('A', 'SW', 0, 0, 0.0, 'small', (30, 110, 220), 0.25, 5),
    ('B', 'SE', 1, 0, math.pi / 2, 'long', (40, 160, 70), 0.25, 5),
    ('C', 'NE', 1, 1, math.pi, 'large', (210, 50, 40), 0.45, 4),
    ('D', 'NW', 0, 1, -math.pi / 2, 'mixed', (140, 60, 180), 0.3, 4),
]
ZONE_TITLES = {'small': 'SMALL BOXES', 'long': 'LONG BOXES', 'large': 'HEAVY BOXES', 'mixed': 'MIXED STORAGE'}
MIXED_LARGE_SHARE = 0.3


def fmt(v):
    return '0' if abs(v) < 1e-9 else f'{v:.5g}'


def font(size):
    return ImageFont.truetype(FONT, size)


# ------------------------------------------------------------- DAE writer

def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class Mesh:
    def __init__(self):
        self.pos, self.nrm, self.uv, self.tris = [], [], [], {}

    def face(self, mat, corners, n, uvs):
        base, ni = len(self.pos), len(self.nrm)
        self.pos.extend(corners)
        self.uv.extend(uvs)
        self.nrm.append(n)
        t = self.tris.setdefault(mat, [])
        for a, c in ((1, 2), (2, 3)):
            t.extend([(base, ni), (base + a, ni), (base + c, ni)])

    def box(self, mat, c, size, uvfun=None, u=(1, 0, 0)):
        """Box centred at c whose first axis points along u (u in the x-z plane)."""
        axes = (u, (0, 1, 0), cross(u, (0, 1, 0)))
        h = [s / 2 for s in size]
        for k in range(3):
            i, j = (k + 1) % 3, (k + 2) % 3
            for sign in (-1, 1):
                params = []
                for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    p = [0, 0, 0]
                    p[k], p[i], p[j] = sign, a, b
                    params.append(p)
                if sign < 0:
                    params.reverse()
                corners = [tuple(c[m] + sum(p[q] * h[q] * axes[q][m] for q in range(3)) for m in range(3))
                           for p in params]
                uvs = [uvfun(k, p, w) if uvfun else (0.5, 0.5) for p, w in zip(params, corners)]
                self.face(mat, corners, tuple(sign * x for x in axes[k]), uvs)

    def poly(self, mat, corners, n):
        """Flat quad (or degenerate triangle) with a single colour; winding follows n."""
        e1 = [corners[1][m] - corners[0][m] for m in range(3)]
        e2 = [corners[2][m] - corners[0][m] for m in range(3)]
        if sum(a * b for a, b in zip(cross(e1, e2), n)) < 0:
            corners = corners[::-1]
        self.face(mat, corners, n, [(0.5, 0.5)] * 4)

    def cylinder_y(self, mat, cx, cz, y0, y1, r, sides=16):
        pts = [(cx + r * math.cos(2 * math.pi * k / sides), cz + r * math.sin(2 * math.pi * k / sides))
               for k in range(sides)]
        for k in range(sides):
            (xa, za), (xb, zb) = pts[k], pts[(k + 1) % sides]
            a = 2 * math.pi * (k + 0.5) / sides
            self.poly(mat, [(xa, y0, za), (xb, y0, zb), (xb, y1, zb), (xa, y1, za)], (math.cos(a), 0, math.sin(a)))
            for y, s in ((y0, -1), (y1, 1)):
                self.poly(mat, [(cx, y, cz), (xa, y, za), (xb, y, zb), (cx, y, cz)], (0, s, 0))

    def plate(self, mat, c, right, up, w, h):
        """Single textured quad; the texture reads correctly when viewed from right x up."""
        corners = [tuple(c[m] + a * w / 2 * right[m] + b * h / 2 * up[m] for m in range(3))
                   for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        self.face(mat, corners, cross(right, up), [(0, 0), (1, 0), (1, 1), (0, 1)])

    def write(self, path, materials):
        """materials: name -> (r, g, b) colour or texture file name (looked up in ../materials/textures)."""
        used = [k for k in materials if self.tris.get(k)]
        fa = ' '.join(f'{v:.5g}' for p in self.pos for v in p)
        na = ' '.join(f'{v:.5g}' for n in self.nrm for v in n)
        ua = ' '.join(f'{v:.5g}' for t in self.uv for v in t)
        images, effects = [], []
        for k in used:
            m = materials[k]
            if isinstance(m, str):
                images.append(f'<image id="{k}-image" name="{k}-image"><init_from>{m}</init_from></image>')
                effects.append(
                    f'<effect id="{k}-effect"><profile_COMMON>'
                    f'<newparam sid="{k}-surface"><surface type="2D"><init_from>{k}-image</init_from></surface></newparam>'
                    f'<newparam sid="{k}-sampler"><sampler2D><source>{k}-surface</source></sampler2D></newparam>'
                    f'<technique sid="common"><lambert><diffuse><texture texture="{k}-sampler" texcoord="UVSET0"/>'
                    f'</diffuse></lambert></technique></profile_COMMON></effect>')
            else:
                effects.append(
                    f'<effect id="{k}-effect"><profile_COMMON><technique sid="common"><lambert>'
                    f'<diffuse><color sid="diffuse">{m[0]} {m[1]} {m[2]} 1</color></diffuse>'
                    f'</lambert></technique></profile_COMMON></effect>')
        mats = ''.join(f'<material id="{k}-material" name="{k}"><instance_effect url="#{k}-effect"/></material>'
                       for k in used)
        prims = ''.join(
            f'<triangles material="{k}-material" count="{len(self.tris[k]) // 3}">'
            f'<input semantic="VERTEX" source="#m-vertices" offset="0"/>'
            f'<input semantic="NORMAL" source="#m-normals" offset="1"/>'
            f'<input semantic="TEXCOORD" source="#m-uv" offset="2" set="0"/>'
            f'<p>{" ".join(f"{a} {b} {a}" for a, b in self.tris[k])}</p></triangles>' for k in used)
        binds = ''.join(
            f'<instance_material symbol="{k}-material" target="#{k}-material">'
            f'<bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/></instance_material>'
            for k in used)

        def source(name, data, count, params):
            p = ''.join(f'<param name="{x}" type="float"/>' for x in params)
            return (f'<source id="m-{name}"><float_array id="m-{name}-array" count="{count * len(params)}">{data}'
                    f'</float_array><technique_common><accessor source="#m-{name}-array" count="{count}" '
                    f'stride="{len(params)}">{p}</accessor></technique_common></source>')
        path.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images>{''.join(images)}</library_images>
  <library_effects>{''.join(effects)}</library_effects>
  <library_materials>{mats}</library_materials>
  <library_geometries><geometry id="m-mesh" name="mesh"><mesh>
    {source('positions', fa, len(self.pos), 'XYZ')}
    {source('normals', na, len(self.nrm), 'XYZ')}
    {source('uv', ua, len(self.uv), 'ST')}
    <vertices id="m-vertices"><input semantic="POSITION" source="#m-positions"/></vertices>
    {prims}
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="Scene" name="Scene">
    <node id="mesh" name="mesh"><instance_geometry url="#m-mesh"><bind_material><technique_common>
      {binds}
    </technique_common></bind_material></instance_geometry></node>
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
''')


def write_static_model(d, name, description, collisions=(), extra=''):
    col_xml = ''.join(f'''
      <collision name="{n}">
        <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0</pose>
        <geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry>
      </collision>''' for n, x, y, z, sx, sy, sz in collisions)
    (d / 'model.sdf').write_text(f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">{col_xml}
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://{name}/meshes/mesh.dae</uri></mesh></geometry>
      </visual>
    </link>{extra}
  </model>
</sdf>
''')
    (d / 'model.config').write_text(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>NU-CE27</name></author>
  <description>{description}</description>
</model>
''')


def new_model_dir(name, keep=()):
    d = MODELS / name
    if d.exists():
        for child in d.iterdir():
            if child.name not in keep:
                shutil.rmtree(child) if child.is_dir() else child.unlink()
    (d / 'meshes').mkdir(parents=True, exist_ok=True)
    (d / 'materials/textures').mkdir(parents=True, exist_ok=True)
    return d


# -------------------------------------------------------------- racks

# Shelves.png (from the workcell): black band on top, orange band in the middle,
# blue band with slots at the bottom. v = 0 is the bottom of the image.
def post_uv(k, p, w):
    if k == 2:
        return 0.5, 0.07
    other = 1 if k == 0 else 0
    return w[2] / 0.1, 0.03 + (p[other] + 1) / 2 * 0.43


def beam_uv(k, p, w):
    if k == 1:
        return 0.5, 0.63
    other = 2 if k == 0 else 0
    return w[1] / 0.5, 0.54 + (p[other] + 1) / 2 * 0.18


def solid_blue_uv(k, p, w):
    return 0.5, 0.07


def plywood_uv(k, p, w):
    if k == 2:
        return w[0] / 0.6, w[1] / 0.6
    return (w[1] if k == 0 else w[0]) / 0.6, w[2] / 0.6


def frame_positions(length, bays):
    bay = length / bays
    ys = [-length / 2 + i * bay for i in range(bays + 1)]
    return [min(max(y, -length / 2 + POST / 2), length / 2 - POST / 2) for y in ys]


def rack_name(levels, bays):
    return f'warehouse_rack_{levels}lvl_{bays}bay'


def build_rack_model(levels, length, bays):
    name = rack_name(levels, bays)
    d = new_model_dir(name)
    for tex in ('Shelves.png', 'Plywood.png'):
        shutil.copy(WORKCELL_TEXTURES / tex, d / 'materials/textures' / tex)
    lv = LEVEL_SETS[levels]
    frames = frame_positions(length, bays)
    m = Mesh()
    xi = POST_X - POST / 2
    for y in frames:
        for x in (-POST_X, POST_X):
            m.box('steel', (x, y, RACK_HEIGHT / 2), (POST, POST, RACK_HEIGHT), post_uv)
        nodes = (0.08, 0.7, 1.33, RACK_HEIGHT - 0.05)
        for z0, z1 in zip(nodes, nodes[1:]):
            dx, dz = 2 * xi, z1 - z0
            n = math.hypot(dx, dz)
            m.box('steel', (0, y, (z0 + z1) / 2), (n, 0.025, 0.025), solid_blue_uv, (dx / n, 0, dz / n))
        for z in (nodes[0], nodes[-1]):
            m.box('steel', (0, y, z), (2 * xi, 0.025, 0.025), solid_blue_uv)
    for z in lv:
        for x in (-POST_X, POST_X):
            m.box('steel', (x, 0, z - 0.04), (0.045, length - POST, 0.08), beam_uv)
        for y0, y1 in zip(frames, frames[1:]):
            m.box('plywood', (0, (y0 + y1) / 2, z - 0.01), (2 * POST_X - 0.045, y1 - y0 - POST, 0.02), plywood_uv)
    m.write(d / 'meshes/mesh.dae', {'steel': 'Shelves.png', 'plywood': 'Plywood.png'})
    collisions = [(f'shelf_{i}', 0, 0, z - 0.01, 2 * POST_X + POST, length, 0.02) for i, z in enumerate(lv)]
    for i, y in enumerate(frames):
        for j, x in enumerate((-POST_X, POST_X)):
            collisions.append((f'post_{i}_{j}', x, y, RACK_HEIGHT / 2, POST, POST, RACK_HEIGHT))
    write_static_model(d, name, f'Pallet rack, {length:.1f} m, {bays} bays, {levels} plywood shelves, '
                                f'open from both sides.', collisions)


# ---------------------------------------------------------- box textures

def cardboard(w, h, color, rng):
    img = Image.new('RGB', (w, h), color)
    px = img.load()
    for _ in range(w * h // 16):
        x, y = rng.randrange(w), rng.randrange(h)
        s = rng.randint(-14, 14)
        r, g, b = px[x, y]
        px[x, y] = (r + s, g + s, b + s)
    return img, ImageDraw.Draw(img)


def barcode(dr, x0, x1, y0, y1, rng):
    x = x0
    while x < x1:
        bw = rng.choice((2, 3, 5))
        dr.rectangle([x, y0, min(x + bw, x1), y1], fill=(20, 20, 20))
        x += bw + rng.choice((2, 3, 4))


def up_arrows(dr, cx, top, size, color=(30, 30, 30)):
    for ax in (cx - size, cx + size):
        dr.polygon([(ax, top), (ax - size * 0.8, top + size * 1.4), (ax + size * 0.8, top + size * 1.4)], fill=color)
        dr.rectangle([ax - size * 0.3, top + size * 1.4, ax + size * 0.3, top + size * 3.2], fill=color)


def draw_small(label, zc, rng):
    # drawn at the aisle-facing aspect (0.5 x 0.3 m)
    img, dr = cardboard(500, 300, (176, 132, 82), rng)
    dr.rectangle([0, 0, 500, 26], fill=(198, 170, 120))
    dr.rectangle([112, 60, 388, 255], fill=(245, 245, 240), outline=(40, 40, 40), width=3)
    dr.rectangle([112, 60, 388, 92], fill=zc)
    dr.text((250, 150), label, font=font(74), fill=(20, 20, 20), anchor='mm')
    barcode(dr, 130, 368, 205, 243, rng)
    up_arrows(dr, 58, 60, 17)
    dr.text((58, 135), 'UP', font=font(34), fill=(30, 30, 30), anchor='mm')
    dr.polygon([(425, 70), (475, 70), (462, 120), (438, 120)], outline=(150, 20, 20), width=5)
    dr.line([(450, 120), (450, 150)], fill=(150, 20, 20), width=5)
    dr.line([(432, 150), (468, 150)], fill=(150, 20, 20), width=5)
    dr.text((450, 185), 'FRAGILE', font=font(20), fill=(150, 20, 20), anchor='mm')
    return img


def draw_large(label, zc, rng):
    # drawn at the aisle-facing aspect (0.6 x 0.45 m)
    img, dr = cardboard(600, 450, (150, 106, 62), rng)
    dr.rectangle([270, 0, 330, 450], fill=(172, 140, 95))
    dr.rectangle([30, 70, 340, 300], fill=(250, 250, 245), outline=(30, 30, 30), width=4)
    dr.rectangle([30, 70, 340, 115], fill=zc)
    dr.text((185, 93), 'HEAVY GOODS', font=font(28), fill=(255, 255, 255), anchor='mm')
    dr.text((185, 185), label, font=font(78), fill=(20, 20, 20), anchor='mm')
    barcode(dr, 50, 320, 240, 285, rng)
    dr.rectangle([380, 70, 560, 180], fill=(210, 60, 20))
    dr.text((470, 110), 'HEAVY', font=font(40), fill=(255, 255, 255), anchor='mm')
    dr.text((470, 155), '8 kg', font=font(30), fill=(255, 255, 255), anchor='mm')
    for fx in (430, 510):
        dr.ellipse([fx - 14, 205, fx + 14, 233], fill=(30, 30, 30))
        dr.rectangle([fx - 10, 236, fx + 10, 290], fill=(30, 30, 30))
    dr.rectangle([430, 250, 510, 262], fill=(30, 30, 30))
    dr.text((470, 315), 'TEAM LIFT', font=font(26), fill=(30, 30, 30), anchor='mm')
    up_arrows(dr, 120, 340, 14)
    dr.text((320, 395), 'HANDLE WITH CARE', font=font(30), fill=(30, 30, 30), anchor='mm')
    return img


def draw_long(label, zc, rng):
    # drawn at the aisle-facing aspect (0.85 x 0.2 m)
    img, dr = cardboard(850, 200, (222, 214, 196), rng)
    dr.rectangle([0, 0, 110, 200], fill=zc)
    dr.text((55, 100), 'LONG', font=font(30), fill=(255, 255, 255), anchor='mm')
    dr.rectangle([290, 20, 590, 180], fill=(255, 255, 255), outline=(30, 30, 30), width=3)
    dr.text((440, 75), label, font=font(68), fill=(20, 20, 20), anchor='mm')
    barcode(dr, 310, 570, 130, 168, rng)
    up_arrows(dr, 190, 40, 16, zc)
    dr.text((190, 150), 'THIS SIDE UP', font=font(22), fill=zc, anchor='mm')
    dr.text((710, 70), 'KEEP', font=font(34), fill=(160, 25, 25), anchor='mm')
    dr.text((710, 115), 'DRY', font=font(34), fill=(160, 25, 25), anchor='mm')
    dr.rectangle([620, 40, 800, 150], outline=(160, 25, 25), width=4)
    return img


DRAW = {'small': draw_small, 'large': draw_large, 'long': draw_long}


class BoxFactory:
    """Hands out unique box IDs per zone and writes one texture + material per box."""

    def __init__(self):
        self.dir = MODELS / 'labeled_boxes'
        if self.dir.exists():
            shutil.rmtree(self.dir)
        (self.dir / 'materials/scripts').mkdir(parents=True)
        (self.dir / 'materials/textures').mkdir(parents=True)
        self.rng = random.Random(5)
        self.next_id = {}
        self.script = []
        self.count = {k: 0 for k in BOXES}

    def box(self, zone, zc, kind, x, y, z, yaw):
        n = self.next_id[zone] = self.next_id.get(zone, 0) + 1
        label = f'{zone}-{n:03d}'
        img = DRAW[kind](label, zc, self.rng)
        img.resize((256, 256), Image.LANCZOS).save(self.dir / f'materials/textures/{label}.jpg', quality=88)
        self.script.append(f'material LabeledBox/{label}\n{{\n  technique\n  {{\n    pass\n    {{\n'
                           f'      texture_unit\n      {{\n        texture {label}.jpg\n      }}\n    }}\n  }}\n}}\n')
        self.count[kind] += 1
        return box_xml(f'box_{zone}_{n:03d}', kind, label, x, y, z + BOXES[kind][2] / 2 + 0.002, yaw)

    def finish(self):
        (self.dir / 'materials/scripts/labeled_boxes.material').write_text('\n'.join(self.script))
        (self.dir / 'model.config').write_text('''<?xml version="1.0"?>
<model>
  <name>labeled_boxes</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>NU-CE27</name></author>
  <description>Textures and materials for the uniquely labelled boxes (A-### small, B-### long, C-### heavy, D-### mixed).</description>
</model>
''')
        (self.dir / 'model.sdf').write_text(
            f'<?xml version="1.0"?>\n<sdf version="1.6">\n'
            f'{box_xml("labeled_box", "small", "A-001", 0, 0, BOXES["small"][2] / 2, 0)}</sdf>\n')


def box_xml(name, kind, label, x, y, z, yaw):
    sx, sy, sz, m = BOXES[kind]
    return f'''    <model name='{name}'>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <link name='link'>
        <inertial>
          <mass>{m}</mass>
          <inertia><ixx>{m / 12 * (sy * sy + sz * sz):.5f}</ixx><iyy>{m / 12 * (sx * sx + sz * sz):.5f}</iyy><izz>{m / 12 * (sx * sx + sy * sy):.5f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name='collision'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><script>
            <uri>model://labeled_boxes/materials/scripts</uri>
            <uri>model://labeled_boxes/materials/textures</uri>
            <name>LabeledBox/{label}</name>
          </script></material>
        </visual>
      </link>
    </model>
'''


# ------------------------------------------------ zone markings and signs

def sign_texture(path, w, h, bg, lines):
    img = Image.new('RGB', (w, h), bg)
    dr = ImageDraw.Draw(img)
    dr.rectangle([6, 6, w - 7, h - 7], outline=(255, 255, 255), width=8)
    total = sum(s for _, s in lines) * 1.15
    y = (h - total) / 2
    for text, size in lines:
        dr.text((w / 2, y + size * 0.575), text, font=font(size), fill=(255, 255, 255), anchor='mm')
        y += size * 1.15
    img.save(path, quality=92)


def build_zone_layout(zone, contents, zc):
    """Floor lines, a floor decal, a wall banner and row plates, in the workcell frame."""
    name = f'zone_{zone.lower()}_layout'
    d = new_model_dir(name)
    tex = d / 'materials/textures'
    title = ZONE_TITLES[contents]
    sign_texture(tex / 'banner.jpg', 1024, 256, zc, [(f'ZONE {zone}', 110), (title, 60)])
    sign_texture(tex / 'floor.jpg', 1024, 320, zc, [(f'ZONE {zone}  -  {title}', 70)])
    materials = {'yellow': (0.95, 0.75, 0.05), 'zone': tuple(c / 255 for c in zc),
                 'banner': 'banner.jpg', 'floor': 'floor.jpg'}
    m = Mesh()
    z = 0.003
    lw = 0.06

    def line(x0, y0, x1, y1, mat):
        m.plate(mat, ((x0 + x1) / 2, (y0 + y1) / 2, z), (1, 0, 0), (0, 1, 0),
                abs(x1 - x0) + lw, abs(y1 - y0) + lw)

    for row, x, segs in ROWS:
        rn = f'{zone}{row}'
        sign_texture(tex / f'row_{rn}.jpg', 256, 128, zc, [(rn, 80)])
        materials[f'row_{rn}'] = f'row_{rn}.jpg'
        for yc, length, _ in segs:
            x0, x1 = x - RACK_DEPTH / 2 - 0.12, x + RACK_DEPTH / 2 + 0.12
            y0, y1 = yc - length / 2 - 0.12, yc + length / 2 + 0.12
            line(x0, y0, x1, y0, 'yellow')
            line(x0, y1, x1, y1, 'yellow')
            line(x0, y0, x0, y1, 'yellow')
            line(x1, y0, x1, y1, 'yellow')
            for end, s in ((yc - length / 2, -1), (yc + length / 2, 1)):
                c = (x, end + s * 0.005, RACK_HEIGHT + 0.16)
                m.plate(f'row_{rn}', c, (-s, 0, 0), (0, 0, 1), RACK_DEPTH, 0.32)
                m.plate(f'row_{rn}', (x, end - s * 0.015, c[2]), (s, 0, 0), (0, 0, 1), RACK_DEPTH, 0.32)
    # zone border along the main aisles shared with the neighbouring zones
    line(9.35, -9.5, 9.35, 7.25, 'zone')
    line(-2.0, 7.25, 9.35, 7.25, 'zone')
    m.plate('floor', (4.3, 8.6, 0.004), (1, 0, 0), (0, 1, 0), 5.0, 1.56)
    m.plate('banner', (4.6, SOUTH_WALL_Y + 0.03, 4.6), (-1, 0, 0), (0, 0, 1), 6.0, 1.5)
    m.write(d / 'meshes/mesh.dae', materials)
    write_static_model(d, name, f'Zone {zone} ({title.lower()}): floor markings and signs, workcell frame.')
    return name


# ------------------------------------------------------------ workcell mesh

def filter_triangles(mesh, material, keep):
    """Drop triangles of one material from the workcell mesh; keep(centroid) decides per triangle."""
    pos = list(map(float, re.search(r'<float_array id="WorkCell-mesh-positions-array" count="\d+">([^<]*)<',
                                    mesh).group(1).split()))
    block = re.search(rf'\s*<triangles material="{material}-material" count="\d+">.*?</triangles>', mesh, re.S)
    if not block:
        return mesh
    body = block.group(0)
    stride = max(map(int, re.findall(r'offset="(\d+)"', body))) + 1
    voff = int(re.search(r'semantic="VERTEX"[^>]*offset="(\d+)"', body).group(1))
    idx = list(map(int, re.search(r'<p>([^<]*)</p>', body).group(1).split()))
    kept = []
    for t in range(0, len(idx), 3 * stride):
        tri = idx[t:t + 3 * stride]
        vs = tri[voff::stride]
        centroid = tuple(sum(pos[3 * v + a] for v in vs) / 3000 for a in range(3))
        if keep(centroid):
            kept.extend(tri)
    if not kept:
        return mesh.replace(body, '')
    new = re.sub(r'count="\d+"', f'count="{len(kept) // (3 * stride)}"', body, count=1)
    new = re.sub(r'<p>[^<]*</p>', f'<p>{" ".join(map(str, kept))}</p>', new)
    return mesh.replace(body, new)


def clean_workcell_mesh():
    mesh = WORKCELL_MESH.read_text()
    for mat in ('ControlPanel', 'Yellow', '_4_-_Default'):   # control panel, old floor lines, bin mat
        mesh = filter_triangles(mesh, mat, lambda c: False)
    mesh = filter_triangles(mesh, 'Pillar', lambda c: c[0] < -2.0)
    WORKCELL_MESH.write_text(mesh)


# ------------------------------------------------------------ shipping dock
#
# One dock, in the quarter of DOCK_ZONE, at the roll-up door of the outer wall. The door is
# opened, a conveyor runs from inside the hall through the door into a truck parked outside.
# All coordinates are in the workcell frame of that quarter.

DOCK_ZONE = 'D'
DOCK_MESH = MODELS / 'our_workcell/meshes/mesh_dock.dae'
DOOR_Y = -1.3                        # centre of the wall opening (y -2.87 .. 0.27)
WALL3_SPLIT = (-2.87, 0.27)
TRUCK_REAR_X = -9.95                 # just outside the outer wall face (x = -9.656)
TRUCK_FLOOR = 0.6
BELT_TOP = 0.63                      # just above the truck floor so boxes slide off upright
BELT_START_X = -5.0
BELT_END_X = TRUCK_REAR_X - 1.5
BELT_WIDTH = 0.9
BELT_SPEED = 0.3
# outline of the hazard-striped dock floor: the whole cleared area between the outer wall,
# the stairs, row 1 and the north segment of row 2
DOCK_AREA = [(-8.8, -4.9), (-1.8, -4.9), (-1.8, 0.1), (0.6, 0.1), (0.6, 7.1), (-8.8, 7.1)]
DOCK_POINTS = [  # name, caption, x, y
    ('S1', 'LOAD', -4.2, DOOR_Y),
    ('S2', 'STAGING', -4.2, -3.7),
    ('S3', 'STAGING', -6.4, -3.7),
    ('S4', 'STAGING', -4.2, 0.2),
]
DOCK_REMOVED_COLLISIONS = ('large_pallet1', 'large_pallet2', 'large_pallet3', 'empty_pallet1', 'box_pallet1', 'pole3')
LOGO = Path(__file__).resolve().parent / 'assets/nile_university_logo.png'
PROJECT_TITLE = 'GRADUATION PROJECT'
PROJECT_TOPICS = 'Multi-Robot Planning  |  LLM Planning'
PROJECT_TEAM = 'Peter - Sotir - Bakr - Mahmoud - Tasneem'
PROJECT_BANNER = (4.3, 4.65, 7.0, 1.75)   # y centre, z centre, width, height on the dock quarter's west wall


def build_dock_mesh():
    """Workcell mesh for the dock quarter: door opened, pillars and all pallets removed for robot space."""
    mesh = WORKCELL_MESH.read_text()
    for mat in ('RollupDoor', 'Pillar', 'Pallet', 'PalletWrap'):
        mesh = filter_triangles(mesh, mat, lambda c: False)
    DOCK_MESH.write_text(mesh)


def project_texture(path, w, h, bg=(255, 255, 255), border=(0, 94, 184)):
    """Project banner: Nile University logo on the left, title, topics and team on the right."""
    img = Image.new('RGB', (w, h), bg)
    dr = ImageDraw.Draw(img)
    pad = h // 14
    dr.rectangle([0, 0, w - 1, h - 1], outline=border, width=pad // 2)
    dr.rectangle([0, h - pad * 2, w, h], fill=border)
    logo = Image.open(LOGO).convert('RGBA')
    lh = h - pad * 4
    logo = logo.resize((round(logo.width * lh / logo.height), lh), Image.LANCZOS)
    img.paste(logo, (pad * 2, pad), logo)
    x0 = pad * 4 + logo.width
    cx = (x0 + w - pad * 2) / 2
    dr.line([(x0 - pad, pad * 2), (x0 - pad, h - pad * 3)], fill=border, width=max(2, pad // 4))

    def fit(text, size, width):
        while size > 10 and dr.textlength(text, font=font(size)) > width:
            size -= 2
        return font(size)
    tw = w - x0 - pad * 3
    dr.text((cx, h * 0.24), PROJECT_TITLE, font=fit(PROJECT_TITLE, h // 5, tw), fill=border, anchor='mm')
    dr.text((cx, h * 0.47), PROJECT_TOPICS, font=fit(PROJECT_TOPICS, h // 8, tw), fill=(25, 45, 90), anchor='mm')
    dr.text((cx, h * 0.68), PROJECT_TEAM, font=fit(PROJECT_TEAM, h // 10, tw), fill=(60, 60, 60), anchor='mm')
    dr.text((w / 2, h - pad), 'Nile University  -  Computer Engineering  -  CE27', font=font(max(12, pad)),
            fill=(255, 255, 255), anchor='mm')
    img.save(path, quality=93)


# Conveyor.png (from the workcell): dark belt strip at the top, green side rail below it.
def belt_uv(k, p, w):
    return w[0] / 1.0, 0.9 + (p[1] + 1) / 2 * 0.09


def rail_uv(k, p, w):
    return w[0] / 1.0, 0.68 + (p[2] + 1) / 2 * 0.14


def build_conveyor():
    name = 'dock_conveyor'
    d = new_model_dir(name, keep=('plugins',))
    shutil.copy(WORKCELL_TEXTURES / 'Conveyor.png', d / 'materials/textures/Conveyor.png')
    length = BELT_START_X - BELT_END_X
    centre = (BELT_START_X + BELT_END_X) / 2
    hw = BELT_WIDTH / 2
    m = Mesh()
    m.box('belt', (0, 0, BELT_TOP - 0.03), (length, BELT_WIDTH, 0.06), belt_uv)
    for s in (-1, 1):
        m.box('belt', (0, s * (hw + 0.04), BELT_TOP - 0.05), (length, 0.08, 0.3), rail_uv)
    collisions = [('belt', 0, 0, BELT_TOP - 0.03, length, BELT_WIDTH, 0.06)]
    collisions += [(f'rail_{s}', 0, s * (hw + 0.04), BELT_TOP - 0.05, length, 0.08, 0.3) for s in (-1, 1)]
    # model +x points towards the truck; legs inside the truck stand on its floor, none in the wall
    for i, hx in enumerate((BELT_START_X - 0.3, -6.6, -8.1, -9.8)):
        lx = centre - hx
        bottom, top = 0.0, BELT_TOP - 0.2
        for s in (-1, 1):
            m.box('frame', (lx, s * (hw - 0.02), (bottom + top) / 2), (0.06, 0.06, top - bottom))
            collisions.append((f'leg_{i}_{s}', lx, s * (hw - 0.02), (bottom + top) / 2, 0.06, 0.06, top - bottom))
        m.box('frame', (lx, 0, bottom + 0.15), (0.05, BELT_WIDTH, 0.05))
    m.box('motor', (-length / 2 + 0.35, 0, BELT_TOP - 0.3), (0.35, 0.5, 0.25))
    m.write(d / 'meshes/mesh.dae', {'belt': 'Conveyor.png', 'frame': (0.35, 0.36, 0.38), 'motor': (0.9, 0.7, 0.05)})
    plugin = f'''
    <plugin name="conveyor_belt" filename="/workspace/models/{name}/plugins/libconveyor_belt.so">
      <belt_length>{fmt(length)}</belt_length>
      <belt_width>{fmt(BELT_WIDTH)}</belt_width>
      <belt_top>{fmt(BELT_TOP)}</belt_top>
      <reach>0.6</reach>
      <speed>{fmt(BELT_SPEED)}</speed>
      <overrun>0.3</overrun>
    </plugin>'''
    write_static_model(d, name, f'Powered belt conveyor, {length:.2f} m, moves boxes along +x '
                                f'(plugin built by Sim/plugins/build_plugins.sh).', collisions, plugin)
    return centre


def build_truck():
    """Box truck, origin at the rear of the cargo floor on the ground, driving direction +x."""
    name = 'delivery_truck'
    d = new_model_dir(name)
    tex = d / 'materials/textures'
    shutil.copy(WORKCELL_TEXTURES / 'Plywood.png', tex / 'Plywood.png')
    project_texture(tex / 'logo.jpg', 1680, 490)
    L, W, H, t = 5.2, 2.4, 2.3, 0.06
    f, top = TRUCK_FLOOR, TRUCK_FLOOR + H
    m = Mesh()
    parts = [  # material, name, centre, size, collide
        ('body', 'floor', (L / 2, 0, f - 0.05), (L, W, 0.1), True),
        ('body', 'wall_left', (L / 2, W / 2 - t / 2, (f + top) / 2), (L, t, H), True),
        ('body', 'wall_right', (L / 2, -W / 2 + t / 2, (f + top) / 2), (L, t, H), True),
        ('body', 'wall_front', (L - t / 2, 0, (f + top) / 2), (t, W, H), True),
        ('body', 'roof', (L / 2, 0, top + t / 2), (L, W, t), True),
        ('trim', 'chassis', (L / 2 + 0.8, 0, 0.42), (L + 1.9, 1.0, 0.16), True),
        ('trim', 'bumper', (-0.12, 0, 0.38), (0.12, W - 0.2, 0.16), True),
        ('cab', 'cab', (L + 1.05, 0, 1.5), (1.9, W - 0.1, 2.0), True),
        ('trim', 'cab_bumper', (L + 2.05, 0, 0.55), (0.14, W - 0.1, 0.3), True),
    ]
    for side in (-1, 1):
        parts.append(('trim', f'rear_post_{side}', (0.04, side * (W / 2 - 0.04), (f + top) / 2), (0.1, 0.1, H + t), False))
    parts.append(('trim', 'rear_header', (0.04, 0, top - 0.05), (0.1, W, 0.12), False))
    collisions = []
    for mat, n, c, size, collide in parts:
        m.box(mat, c, size)
        if collide:
            collisions.append((n, *c, *size))
    m.box('floor', (L / 2, 0, f + 0.001), (L - 2 * t, W - 2 * t, 0.002), plywood_uv)
    for side in (-1, 1):
        y = side * (W / 2 + 0.002)
        m.plate('logo', (L / 2, y, (f + top) / 2 + 0.1), (-side, 0, 0), (0, 0, 1), L - 0.4, 1.4)
        m.plate('glass', (L + 1.35, side * (W / 2 - 0.05 + 0.002), 2.0), (-side, 0, 0), (0, 0, 1), 0.9, 0.7)
    m.plate('glass', (L + 2.0 + 0.002, 0, 2.05), (0, 1, 0), (0, 0, 1), W - 0.4, 0.8)
    r = 0.28                                   # low-floor truck: wheels stay under the cargo floor
    for x in (0.8, 1.45, L + 1.3):
        for side in (-1, 1):
            y0 = side * (W / 2 - 0.3)
            m.cylinder_y('tyre', x, r, y0 - 0.14, y0 + 0.14, r)
            collisions.append((f'wheel_{x:.2f}_{side}', x, y0, r, 2 * r, 0.28, 2 * r))
    m.write(d / 'meshes/mesh.dae', {'body': (0.93, 0.93, 0.92), 'cab': (0.1, 0.3, 0.62), 'trim': (0.3, 0.3, 0.32),
                                    'tyre': (0.06, 0.06, 0.06), 'glass': (0.12, 0.18, 0.25),
                                    'floor': 'Plywood.png', 'logo': 'logo.jpg'})
    write_static_model(d, name, f'Box truck with an open rear, cargo floor at {TRUCK_FLOOR} m.', collisions)


def point_texture(path, name, caption):
    img = Image.new('RGB', (256, 256), (40, 40, 40))
    dr = ImageDraw.Draw(img)
    for k in range(0, 256, 32):
        dr.rectangle([k, 0, k + 15, 14], fill=(240, 190, 20))
        dr.rectangle([k, 241, k + 15, 255], fill=(240, 190, 20))
        dr.rectangle([0, k, 14, k + 15], fill=(240, 190, 20))
        dr.rectangle([241, k, 255, k + 15], fill=(240, 190, 20))
    dr.text((128, 110), name, font=font(96), fill=(255, 255, 255), anchor='mm')
    dr.text((128, 190), caption, font=font(34), fill=(240, 190, 20), anchor='mm')
    img.save(path, quality=92)


def build_dock_layout():
    name = 'shipping_dock_layout'
    d = new_model_dir(name)
    tex = d / 'materials/textures'
    sign_texture(tex / 'sign.jpg', 1024, 256, (230, 120, 20), [('SHIPPING DOCK 1', 100), ('OUTBOUND', 56)])
    sign_texture(tex / 'floor.jpg', 1024, 256, (230, 120, 20), [('SHIPPING DOCK', 110)])
    project_texture(tex / 'project.jpg', 1600, 400)
    materials = {'yellow': (0.95, 0.75, 0.05), 'black': (0.05, 0.05, 0.05), 'sign': 'sign.jpg', 'floor': 'floor.jpg',
                 'project': 'project.jpg'}
    m = Mesh()
    w, dash = 0.12, 0.3
    for (ax, ay), (bx, by) in zip(DOCK_AREA, DOCK_AREA[1:] + DOCK_AREA[:1]):
        length = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / length, (by - ay) / length
        for k in range(int(length / dash)):
            cx, cy = ax + ux * (k + 0.5) * dash, ay + uy * (k + 0.5) * dash
            sx, sy = (dash if ux else w), (dash if uy else w)
            m.plate('yellow' if k % 2 == 0 else 'black', (cx, cy, 0.003), (1, 0, 0), (0, 1, 0), sx, sy)
    for pname, caption, px, py in DOCK_POINTS:
        point_texture(tex / f'{pname}.jpg', pname, caption)
        materials[pname] = f'{pname}.jpg'
        m.plate(pname, (px, py, 0.004), (0, 1, 0), (-1, 0, 0), 1.2, 1.2)
    m.plate('floor', (-6.6, 0.25, 0.004), (1, 0, 0), (0, 1, 0), 3.0, 0.75)
    m.plate('sign', (-8.88, DOOR_Y, 4.2), (0, 1, 0), (0, 0, 1), 3.6, 0.9)
    by, bz, bw, bh = PROJECT_BANNER
    m.plate('project', (-8.88, by, bz), (0, 1, 0), (0, 0, 1), bw, bh)
    m.write(d / 'meshes/mesh.dae', materials)
    write_static_model(d, name, 'Shipping dock floor markings, drop-off points and door sign, workcell frame.')


def dock_world_edits(text):
    """Swap the dock quarter's workcell mesh and open its collisions at the door."""
    module = next(z[1] for z in ZONES if z[0] == DOCK_ZONE)
    s = text.index(f"<model name='workcell_{module}'>")
    e = text.index('\n    </model>', s)
    block = text[s:e]
    block = block.replace('model://our_workcell/meshes/mesh.dae', 'model://our_workcell/meshes/mesh_dock.dae')
    for name in DOCK_REMOVED_COLLISIONS:
        block = re.sub(rf"\s*<collision name='{name}'>.*?</collision>", '', block, flags=re.S)
    wall = re.search(r"(\s*)<collision name='wall3'>.*?</collision>", block, re.S)
    if wall:
        w = wall.group(0)
        pose = re.search(r"<pose frame=''>([^<]*)</pose>", w).group(1).split()
        size = re.search(r'<size>([^<]*)</size>', w).group(1).split()
        yc, ly = float(pose[1]), float(size[1])
        ya, yb = yc - ly / 2, yc + ly / 2
        parts = []
        for suffix, lo, hi in (('a', ya, WALL3_SPLIT[0]), ('b', WALL3_SPLIT[1], yb)):
            p = w.replace("name='wall3'", f"name='wall3_{suffix}'")
            p = p.replace(f"<pose frame=''>{' '.join(pose)}</pose>",
                          f"<pose frame=''>{pose[0]} {fmt((lo + hi) / 2)} {' '.join(pose[2:])}</pose>")
            p = p.replace(f"<size>{' '.join(size)}</size>", f"<size>{size[0]} {fmt(hi - lo)} {size[2]}</size>")
            parts.append(p)
        block = block.replace(w, ''.join(parts))
    return text[:s] + block + text[e:]


def dock_xml(conveyor_x):
    _, module, i, j, th, *_ = next(z for z in ZONES if z[0] == DOCK_ZONE)
    out = [f'    <!-- Shipping dock: module {module} -->\n']

    def include(name, uri, lx, ly, yaw):
        x, y = to_world(i, j, th, lx, ly)
        out.append(f"    <include>\n      <name>{name}</name>\n      <uri>model://{uri}</uri>\n"
                   f"      <pose>{fmt(x)} {fmt(y)} 0 0 0 {fmt(th + yaw)}</pose>\n    </include>\n")

    include('shipping_dock_layout', 'shipping_dock_layout', 0, 0, 0)
    include('dock_conveyor', 'dock_conveyor', conveyor_x, DOOR_Y, math.pi)
    include('delivery_truck', 'delivery_truck', TRUCK_REAR_X, DOOR_Y, math.pi)
    points = [(n, c, *to_world(i, j, th, x, y)) for n, c, x, y in DOCK_POINTS]
    return ''.join(out), points


def write_dock_points(points):
    _, _, i, j, th, *_ = next(z for z in ZONES if z[0] == DOCK_ZONE)
    lines = ['# Shipping dock locations in world coordinates (generated by build_our_warehouse_storage.py)',
             'drop_off_points:']
    for n, caption, x, y in points:
        lines.append(f'  {n}: {{caption: {caption}, x: {x:.3f}, y: {y:.3f}}}')
    bx, by = to_world(i, j, th, BELT_START_X, DOOR_Y)
    tx, ty = to_world(i, j, th, TRUCK_REAR_X - 2.6, DOOR_Y)
    lines += [f'conveyor_start: {{x: {bx:.3f}, y: {by:.3f}, belt_top: {BELT_TOP}}}',
              f'truck_cargo_centre: {{x: {tx:.3f}, y: {ty:.3f}, floor: {TRUCK_FLOOR}}}',
              f'conveyor_speed_topic: /gazebo/default/dock_conveyor/speed']
    (WORLD.parent / 'shipping_dock.yaml').write_text('\n'.join(lines) + '\n')


# ------------------------------------------------------------------- world

def to_world(i, j, th, lx, ly):
    c, s = math.cos(th), math.sin(th)
    return C0[0] + i * TILE + c * lx - s * ly, C0[1] + j * TILE + s * lx + c * ly


PALLET_LOADS = {
    'small': [('small', dx, dy, math.pi / 2, 0) for dx in (-0.55, 0, 0.55) for dy in (-0.45, 0, 0.45)],
    'long': [('long', dx, dy, 0, 0) for dx in (-0.75, -0.25, 0.25, 0.75) for dy in (-0.44, 0.44)],
    'large': [('large', dx, dy, 0, 0) for dx in (-0.5, 0.5) for dy in (-0.4, 0.4)],
    'mixed': [('large', -0.55, dy, 0, 0) for dy in (-0.35, 0.35)]
             + [('small', dx, dy, math.pi / 2, 0) for dx in (0.28, 0.8) for dy in (-0.45, 0, 0.45)]
             + [('long', -0.55, 0, math.pi / 2, BOXES['large'][2])],
}


def zone_xml(zone, module, i, j, th, contents, zc, fill, levels, boxes, rng):
    out = [f'    <!-- Zone {zone} ({ZONE_TITLES[contents].lower()}): module {module} -->\n']
    x, y = to_world(i, j, th, 0, 0)
    out.append(f"    <include>\n      <name>zone_{zone}_layout</name>\n      <uri>model://zone_{zone.lower()}_layout</uri>\n"
               f"      <pose>{fmt(x)} {fmt(y)} 0 0 0 {fmt(th)}</pose>\n    </include>\n")
    lv = LEVEL_SETS[levels]

    def place(kind, lx, ly, z, yaw):
        bx, by = to_world(i, j, th, lx, ly)
        out.append(boxes.box(zone, zc, kind, bx, by, z, th + yaw))

    for row, rx, segs in ROWS:
        for s, (yc, length, bays) in enumerate(segs):
            x, y = to_world(i, j, th, rx, yc)
            out.append(f"    <include>\n      <name>rack_{zone}{row}_{s}</name>\n"
                       f"      <uri>model://{rack_name(levels, bays)}</uri>\n"
                       f"      <pose>{fmt(x)} {fmt(y)} 0 0 0 {fmt(th)}</pose>\n    </include>\n")
            frames = frame_positions(length, bays)
            for y0, y1 in zip(frames, frames[1:]):
                ly = yc + (y0 + y1) / 2
                for z in lv:
                    if contents == 'large' or (contents == 'mixed' and rng.random() < MIXED_LARGE_SHARE):
                        if rng.random() < fill:
                            place('large', rx, ly, z, 0)
                        continue
                    for side in (-1, 1):
                        if rng.random() < fill:
                            kind = contents if contents != 'mixed' else rng.choice(('small', 'long'))
                            place(kind, rx + side * SIDE_OFFSET, ly, z, 0)
    for px, py in PALLETS if zone != DOCK_ZONE else ():   # the dock quarter's pallets are removed
        for kind, dx, dy, yaw, dz in PALLET_LOADS[contents]:
            place(kind, px + dx, py + dy, PALLET_TOP + dz, yaw)
    return ''.join(out)


def clean_world(text):
    text = re.sub(r"    <model name='workcell_bin[^']*'> ?\n.*?\n    </model>\n", '', text, flags=re.S)
    for name in ('control_panel',) + REMOVED_POLES:
        text = re.sub(rf"\s*<collision name='{name}'>.*?</collision>", '', text, flags=re.S)
    return text.replace('<shadows>1</shadows>', '<shadows>0</shadows>')


def update_world(conveyor_x):
    text = dock_world_edits(clean_world(WORLD.read_text()))
    start = re.search(r'    <!-- (Shelves and boxes|Racks and boxes|Zone A)[^\n]*-->', text)
    end = text.index('    <!-- Module SW -->')
    boxes = BoxFactory()
    rng = random.Random(27)
    xml = ''.join(zone_xml(*z, boxes, rng) for z in ZONES)
    boxes.finish()
    dock, points = dock_xml(conveyor_x)
    text = (text[:start.start()] if start else text[:end]) + xml + dock + text[end:]
    WORLD.write_text(text)
    write_dock_points(points)
    return boxes.count


if __name__ == '__main__':
    for old in ('warehouse_rack_short', 'warehouse_rack_long'):
        shutil.rmtree(MODELS / old, ignore_errors=True)
    for levels in LEVEL_SETS:
        for _, _, segs in ROWS:
            for _, length, bays in segs:
                build_rack_model(levels, length, bays)
    for zone, _, _, _, _, contents, zc, *_ in ZONES:
        build_zone_layout(zone, contents, zc)
    clean_workcell_mesh()
    build_dock_mesh()
    conveyor_x = build_conveyor()
    build_truck()
    build_dock_layout()
    count = update_world(conveyor_x)
    racks = sum(len(s) for _, _, s in ROWS) * len(ZONES)
    print(f'racks: {racks}, boxes: {count}, total boxes: {sum(count.values())}')

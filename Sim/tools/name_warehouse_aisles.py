#!/usr/bin/env python3
"""Name the aisles of Our_Structured_Warehouse and clear the wrapped pallets out of the storage zones.

Applied on top of the world written by build_our_warehouse_storage.py:
  * floor labels for every aisle between two rack rows (A1 .. A4) and the cross aisle through the
    gap in rows 2-5 (CROSS A)
  * aisles.yaml next to the world: entry / end waypoints of every aisle in world coordinates
  * the storage quarters use mesh_open.dae, a copy of the workcell mesh without REMOVED_PALLETS,
    and lose their collisions (the shared mesh.dae is left alone for the other worlds)

Run from the repository root:  python3 Sim/tools/name_warehouse_aisles.py
Re-running replaces everything it added before. Regenerate the maps afterwards:
  python3 scripts/generate_occupancy_grid.py
"""
import math
import re
import sys

from PIL import Image, ImageDraw

sys.dont_write_bytecode = True       # Sim/tools/__pycache__ is tracked
import build_our_warehouse_storage as w  # noqa: E402

OPEN_MESH = w.MODELS / 'our_workcell/meshes/mesh_open.dae'
# (collision name, x, y, size x, size y) of the wrapped pallets removed in every storage quarter
REMOVED_PALLETS = [
    ('large_pallet1', -3.33609, 6.31883, 2.14726, 1.79349),
    ('large_pallet2', -5.74492, 6.31883, 2.14726, 1.79349),
]

Z = 0.005                            # above the zone lines (0.003) and floor decals (0.004)
ROW_XS = [x for _, x, _ in w.ROWS]
SEGMENTS = [(yc - length / 2, yc + length / 2) for _, _, segs in w.ROWS for yc, length, _ in segs]
ROWS_SOUTH = min(a for a, _ in SEGMENTS)
ROWS_NORTH = max(b for _, b in SEGMENTS)
_, _, SPLIT = next(r for r in w.ROWS if len(r[2]) == 2)
CROSS_Y = (SPLIT[0][0] + SPLIT[0][1] / 2 + SPLIT[1][0] - SPLIT[1][1] / 2) / 2
ENTRY_Y = ROWS_NORTH + 0.8           # in the main aisle, just outside the aisle mouth
END_Y = ROWS_SOUTH + 0.4
CROSS_ENTRY_X = ROW_XS[-1] + w.RACK_DEPTH / 2 + 0.6
BOX_PALLET_EAST = w.PALLETS[0][0] + 2.14726 / 2   # the loaded pallet narrows the mouth of aisle 1


def mouth_x(zone, k):
    """Centre of the free floor at the north end of aisle k."""
    ax = (ROW_XS[k - 1] + ROW_XS[k]) / 2
    if k == 1 and zone != w.DOCK_ZONE:
        return (BOX_PALLET_EAST + ROW_XS[1] - w.RACK_DEPTH / 2) / 2
    return ax


def label_texture(path, text, width, bg):
    img = Image.new('RGB', (width, 256), bg)
    dr = ImageDraw.Draw(img)
    dr.rectangle([6, 6, width - 7, 249], outline=(255, 255, 255), width=10)
    size = 160
    while size > 20 and dr.textlength(text, font=w.font(size)) > width - 60:
        size -= 4
    dr.text((width / 2, 128), text, font=w.font(size), fill=(255, 255, 255), anchor='mm')
    img.save(path, quality=92)


def label(m, materials, tex, key, text, c, right, up, w_, h_, bg):
    if key not in materials:
        label_texture(tex / f'{key}.jpg', text, round(256 * w_ / h_), bg)
        materials[key] = f'{key}.jpg'
    m.plate(key, (c[0], c[1], Z), right, up, w_, h_)


def build_aisle_model(zone, zc):
    name = f'zone_{zone.lower()}_aisles'
    d = w.new_model_dir(name)
    tex = d / 'materials/textures'
    m, materials = w.Mesh(), {}
    toward_south = ((-1, 0, 0), (0, -1, 0))   # (right, up) of text read while driving towards -y
    toward_north = ((1, 0, 0), (0, 1, 0))
    for k in range(1, len(ROW_XS)):
        ax = (ROW_XS[k - 1] + ROW_XS[k]) / 2
        key, text = f'aisle_{zone}{k}', f'{zone}{k}'
        label(m, materials, tex, key, text, (mouth_x(zone, k), ROWS_NORTH - 0.5), *toward_south, 0.8, 0.4, zc)
        label(m, materials, tex, key, text, (ax, ROWS_SOUTH + 0.6), *toward_north, 0.8, 0.4, zc)
        label(m, materials, tex, key, text, (ax, CROSS_Y + 1.3), *toward_south, 0.8, 0.4, zc)
        label(m, materials, tex, key, text, (ax, CROSS_Y - 1.3), *toward_north, 0.8, 0.4, zc)
    cross = f'CROSS {zone}'
    label(m, materials, tex, 'cross', cross, (ROW_XS[-1], CROSS_Y), (0, 1, 0), (-1, 0, 0), 1.2, 0.42, zc)
    label(m, materials, tex, 'cross', cross, (ROW_XS[1], CROSS_Y), (0, -1, 0), (1, 0, 0), 1.2, 0.42, zc)
    m.write(d / 'meshes/mesh.dae', materials)
    w.write_static_model(d, name, f'Zone {zone}: floor labels of the aisles, workcell frame.')
    return name


def aisle_entries(zone, i, j, th):
    def point(lx, ly, yaw=None):
        x, y = w.to_world(i, j, th, lx, ly)
        p = f'x: {x:.3f}, y: {y:.3f}'
        return f'{{{p}, yaw: {math.remainder(th + yaw, 2 * math.pi):.4f}}}' if yaw is not None else f'{{{p}}}'

    out = []
    for k in range(1, len(ROW_XS)):
        ax = (ROW_XS[k - 1] + ROW_XS[k]) / 2
        out += [f'  {zone}{k}:',
                f'    zone: {zone}',
                f'    between_rows: [{zone}{k}, {zone}{k + 1}]',
                f'    entry: {point(mouth_x(zone, k), ENTRY_Y, -math.pi / 2)}',
                f'    end: {point(ax, END_Y)}']
    out += [f'  CROSS_{zone}:',
            f'    zone: {zone}',
            f'    crosses: [{", ".join(f"{zone}{k}" for k in range(1, len(ROW_XS)))}]',
            f'    entry: {point(CROSS_ENTRY_X, CROSS_Y, math.pi)}',
            f'    end: {point((ROW_XS[0] + ROW_XS[1]) / 2, CROSS_Y)}']
    return out


def write_aisles_yaml(entries):
    lines = ['# Named aisles of Our_Structured_Warehouse in world coordinates (generated by name_warehouse_aisles.py)',
             '# <zone><k> runs between rack rows <zone><k> and <zone><k+1>; CROSS_<zone> crosses them',
             '# through the gap in rows 2-5.',
             '# entry: where a robot enters (yaw faces into the aisle); end: the far end.',
             'aisles:'] + entries
    (w.WORLD.parent / 'aisles.yaml').write_text('\n'.join(lines) + '\n')


def build_open_mesh():
    boxes = [(px - sx / 2 - 0.04, px + sx / 2 + 0.04, py - sy / 2 - 0.04, py + sy / 2 + 0.04)
             for _, px, py, sx, sy in REMOVED_PALLETS]
    mesh = w.WORKCELL_MESH.read_text()
    for mat in ('Pallet', 'PalletWrap'):
        mesh = w.filter_triangles(mesh, mat, lambda c: not any(
            x0 < c[0] < x1 and y0 < c[1] < y1 for x0, x1, y0, y1 in boxes))
    OPEN_MESH.write_text(mesh)


def update_world(models):
    text = w.WORLD.read_text()
    text = re.sub(r"    <include>\n      <name>zone_[a-z]_aisles</name>.*?</include>\n", '', text, flags=re.S)
    for zone, module, *_ in w.ZONES:
        layout = re.search(rf"    <include>\n      <name>zone_{zone}_layout</name>.*?</include>\n", text, re.S)
        pose = re.search(r'<pose>[^<]*</pose>', layout.group(0)).group(0)
        inc = f"    <include>\n      <name>{models[zone]}</name>\n      <uri>model://{models[zone]}</uri>\n      {pose}\n    </include>\n"
        text = text[:layout.end()] + inc + text[layout.end():]
        if zone == w.DOCK_ZONE:
            continue
        s = text.index(f"<model name='workcell_{module}'>")
        e = text.index('\n    </model>', s)
        block = text[s:e].replace('model://our_workcell/meshes/mesh.dae', 'model://our_workcell/meshes/mesh_open.dae')
        for name, *_ in REMOVED_PALLETS:
            block = re.sub(rf"\s*<collision name='{name}'>.*?</collision>", '', block, flags=re.S)
        text = text[:s] + block + text[e:]
    w.WORLD.write_text(text)


if __name__ == '__main__':
    models, entries = {}, []
    for zone, _, i, j, th, _, zc, *_ in w.ZONES:
        models[zone] = build_aisle_model(zone, zc)
        entries += aisle_entries(zone, i, j, th)
    build_open_mesh()
    update_world(models)
    write_aisles_yaml(entries)
    print(f'aisles: {sum(1 for e in entries if not e.startswith("    "))}, removed '
          f'{", ".join(p[0] for p in REMOVED_PALLETS)} in {len(w.ZONES) - 1} storage quarters')

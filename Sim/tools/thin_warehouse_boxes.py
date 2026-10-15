#!/usr/bin/env python3
"""Thin out the boxes of Our_Structured_Warehouse and fix the ones the robots don't work with.

Every box is a full physics body, so ~575 of them halve the simulation speed. This keeps TARGET
boxes (the same share in every zone, always keeping the boxes the demos pick) and makes the
boxes outside INTERACTIVE_AREAS static: they stay in place as scenery and can't be pushed or
picked. Boxes in INTERACTIVE_AREAS stay fully simulated.

Run from the repository root after build_our_warehouse_storage.py:
    python3 Sim/tools/thin_warehouse_boxes.py
then regenerate the semantic database:
    python3 scripts/generate_semantic_db.py --world Our_Structured_Warehouse
Re-running keeps the remaining boxes and only reapplies the static flags.
"""
import random
import re
from collections import defaultdict
from pathlib import Path

WORLD = Path(__file__).resolve().parent.parent / 'worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world'
TARGET = 275
# (zone, y min, y max) in world coordinates. Rows 1 and 2 of zone D, next to the dock, where the
# demos pick; every other box is static.
INTERACTIVE_AREAS = [('D', 18.4, 21.8)]
KEEP = ('box_D_006', 'box_D_015')          # picked by demo_mir_ur_pick_to_conveyor.py / demo_pick_to_conveyor.py

BOX = re.compile(r"    <model name='(box_([A-Z])_\d+)'>\n(?:      <static>1</static>\n)?"
                 r"      <pose>\S+ (\S+).*?\n    </model>\n", re.S)


def interactive(zone, y):
    return any(zone == z and y0 <= y <= y1 for z, y0, y1 in INTERACTIVE_AREAS)


def main():
    text = WORLD.read_text()
    by_zone = defaultdict(list)
    for m in BOX.finditer(text):
        by_zone[m.group(2)].append(m.group(1))
    total = sum(len(v) for v in by_zone.values())

    rng = random.Random(275)
    keep = set()
    for zone, names in sorted(by_zone.items()):
        n = min(len(names), round(TARGET * len(names) / total))
        must = [b for b in names if b in KEEP]
        rest = [b for b in names if b not in KEEP]
        keep.update(must + rng.sample(rest, max(0, n - len(must))))

    dynamic = 0

    def edit(m):
        nonlocal dynamic
        name, zone, y = m.group(1), m.group(2), float(m.group(3))
        if name not in keep:
            return ''
        block = m.group(0).replace('      <static>1</static>\n', '')
        if interactive(zone, y):
            dynamic += 1
        else:
            block = block.replace('\n', '\n      <static>1</static>\n', 1)
        return block

    WORLD.write_text(BOX.sub(edit, text))
    counts = {z: sum(b in keep for b in names) for z, names in sorted(by_zone.items())}
    print(f'boxes: {total} -> {len(keep)} {counts}, interactive: {dynamic}, static: {len(keep) - dynamic}')


if __name__ == '__main__':
    main()

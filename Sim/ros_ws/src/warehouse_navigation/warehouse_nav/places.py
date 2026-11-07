"""Named places of the warehouse (config/waypoints.yaml, made by scripts/generate_waypoints.py)."""
import os

import yaml

PREFIXES = ('', 'box_', 'rack_', 'zone_')


def norm(name):
    return str(name).strip().lower().replace(' ', '_').replace('-', '_')


def default_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('warehouse_navigation'), 'config', 'waypoints.yaml')
    except Exception:   # not built yet: use the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'waypoints.yaml')


class Places:
    def __init__(self, path=None):
        self.places = yaml.safe_load(open(path or default_path()))['places']

    def find(self, name):
        """The place called `name` (case, spaces and dashes do not matter; 'A-001' finds box A-001), or None."""
        for prefix in PREFIXES:
            key = norm(prefix + norm(name))
            if key in self.places:
                return key, self.places[key]
        return None

    def pose(self, name):
        found = self.find(name)
        if found is None:
            raise KeyError(f'unknown place {name!r}')
        p = found[1]
        return p['x'], p['y'], p['yaw']

    def names(self, kind=None):
        return [n for n, p in self.places.items() if kind is None or p['kind'] == kind]

    def summary(self):
        """A short text for an LLM prompt: what can be asked for."""
        kinds = {}
        for n, p in self.places.items():
            kinds.setdefault(p['kind'], []).append(n)
        lines = []
        for kind, names in kinds.items():
            shown = ', '.join(names[:6]) + (f', ... ({len(names)} in all)' if len(names) > 6 else '')
            lines.append(f'{kind}: {shown}')
        return '\n'.join(lines)

import json
import math
import threading
import time
from collections import Counter

from .coordinate_utils import world_to_target


class ThreadSafeMemoryCore:
    def __init__(self, target_frame="map", world_to_target_transform=(0.0, 0.0, 0.0)):
        self._lock = threading.RLock()
        self._state = {}
        self._target_frame = target_frame
        self._transform = tuple(world_to_target_transform)
        self._last_update_time = None

    def seed_from_file(self, path):
        with open(path, "r", encoding="utf-8") as semantic_file:
            semantic_data = json.load(semantic_file)

        if not isinstance(semantic_data, dict):
            raise ValueError("Semantic database must be a JSON object")

        for entity_id, record in semantic_data.items():
            position = record["position"]
            loc, yaw_deg = world_to_target(
                float(position["x"]),
                float(position["y"]),
                float(position["z"]),
                float(record.get("rotation_yaw", 0.0)),
                self._transform,
            )
            dimensions = record["physical_dimensions"]
            size = [
                float(dimensions["width_x"]),
                float(dimensions["length_y"]),
                float(dimensions["height_z"]),
            ]
            self.seed_entity(
                entity_id,
                record.get("model_type", "unknown"),
                loc,
                yaw_deg,
                size,
                status="static_seeded",
            )

    def seed_entity(self, entity_id, entity_type, loc, yaw_deg, size, status="static_seeded"):
        if len(loc) != 3 or len(size) != 3:
            raise ValueError("Entity loc and size must each contain three values")
        with self._lock:
            self._state[entity_id] = {
                "id": entity_id,
                "type": entity_type,
                "loc": [float(value) for value in loc],
                "yaw_deg": float(yaw_deg) % 360.0,
                "size": [float(value) for value in size],
                "frame_id": self._target_frame,
                "status": status,
            }

    def mark_telemetry_received(self):
        with self._lock:
            self._last_update_time = time.monotonic()

    def has_entity(self, entity_id):
        with self._lock:
            return entity_id in self._state

    def update_entity_pose(self, entity_id, loc, yaw_deg):
        with self._lock:
            entity = self._state.get(entity_id)
            if entity is None:
                return False
            entity["loc"] = [float(value) for value in loc]
            entity["yaw_deg"] = float(yaw_deg) % 360.0
            entity["status"] = "live_tracking"
            return True

    def _copy_entity(self, entity):
        return {
            **entity,
            "loc": list(entity["loc"]),
            "size": list(entity["size"]),
        }

    def get_entity(self, entity_id):
        with self._lock:
            entity = self._state.get(entity_id)
            return self._copy_entity(entity) if entity else None

    def get_all_entities(self):
        with self._lock:
            return {key: self._copy_entity(value) for key, value in self._state.items()}

    def summary(self):
        with self._lock:
            categories = Counter(entity["type"] for entity in self._state.values())
            return {
                "total_objects": len(self._state),
                "categories": dict(categories),
                "frame_id": self._target_frame,
                "status": "live_tracking" if self._last_update_time is not None else "static_seeded",
            }

    def spatial(self, target_id, radius):
        with self._lock:
            target = self._state.get(target_id)
            if target is None:
                return None
            target_x, target_y = target["loc"][:2]
            matches = []
            for entity_id, entity in self._state.items():
                if entity_id == target_id:
                    continue
                x, y = entity["loc"][:2]
                distance = math.hypot(x - target_x, y - target_y)
                if distance <= radius:
                    matches.append({"id": entity_id, "dist": round(distance, 2)})
            return sorted(matches, key=lambda item: (item["dist"], item["id"]))

    def objects(self, category=None):
        with self._lock:
            return [
                {"id": entity_id, "loc": [round(value, 2) for value in entity["loc"]]}
                for entity_id, entity in self._state.items()
                if category is None or entity["type"] == category
            ]

    def health(self):
        with self._lock:
            age = None
            if self._last_update_time is not None:
                age = round(max(0.0, time.monotonic() - self._last_update_time), 2)
            return {
                "status": "healthy",
                "ros_connected": self._last_update_time is not None,
                "last_telemetry_age_sec": age,
            }

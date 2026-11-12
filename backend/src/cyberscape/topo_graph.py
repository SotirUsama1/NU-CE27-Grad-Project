"""Topological graph model with Dijkstra routing for the CyberScape orchestrator.

Loads the generated ``topo_graph.json`` and provides:
  * an in-memory adjacency list,
  * Dijkstra shortest-path search supporting excluded (quarantined) edges,
  * ``segment_edges(u, v)`` for the sequence of edges between two Passing Junctions,
  * ``location_summary()`` for prompt grounding (no external ROS dependencies).
"""

from __future__ import annotations

import heapq
import json
import os
from collections import defaultdict

DEFAULT_TOPO_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "data", "topo_graph.json"
)


class TopoGraph:
    """In-memory topological graph of the warehouse.

    Nodes are identified by string IDs. Each node carries coordinates,
    capacity, and a type (``aisle``, ``junction``, ``box``). Edges are
    undirected with a Euclidean weight, a normalized lock key, and a
    single-lane flag.
    """

    def __init__(self, topo_path: str | None = None):
        self.topo_path = topo_path or DEFAULT_TOPO_PATH
        self.nodes: dict[str, dict] = {}
        self.edges: list[dict] = []
        self._adjacency: dict[str, list[dict]] = defaultdict(list)
        self._edge_by_key: dict[str, dict] = {}
        self._loaded = False

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load(self, topo_path: str | None = None) -> TopoGraph:
        """Load the graph from a ``topo_graph.json`` file."""
        path = topo_path or self.topo_path
        with open(path, "r") as fh:
            data = json.load(fh)

        self.nodes = data["nodes"]
        self.edges = data["edges"]
        self._adjacency = defaultdict(list)
        self._edge_by_key = {}

        for edge in self.edges:
            u, v = edge["from"], edge["to"]
            key = edge["key"]
            self._adjacency[u].append(edge)
            self._adjacency[v].append({**edge, "from": v, "to": u})
            self._edge_by_key[key] = edge

        self._loaded = True
        return self

    def ensure_loaded(self) -> TopoGraph:
        """Load the graph on first use if not already loaded."""
        if not self._loaded:
            self.load()
        return self

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def node(self, node_id: str) -> dict:
        """Return the node record for ``node_id``."""
        self.ensure_loaded()
        if node_id not in self.nodes:
            raise KeyError(f"Unknown node: {node_id}")
        return self.nodes[node_id]

    def neighbors(self, node_id: str) -> list[dict]:
        """Return outgoing edge records for ``node_id``."""
        self.ensure_loaded()
        return list(self._adjacency.get(node_id, []))

    def edge_key(self, u: str, v: str) -> str:
        """Return the normalized undirected lock key for an edge."""
        return "lock:edge:" + "_".join(sorted([u, v]))
    # ------------------------------------------------------------------
    # Dijkstra shortest path
    # ------------------------------------------------------------------

    def dijkstra(
        self,
        source: str,
        target: str,
        excluded_edges: set[str] | None = None,
    ) -> tuple[float, list[str]]:
        """Dijkstra shortest path from ``source`` to ``target``.

        Args:
            source: Start node ID.
            target: Goal node ID.
            excluded_edges: Set of normalized lock keys to skip (quarantines).

        Returns:
            ``(total_weight, path)`` where ``path`` is a list of node IDs
            from source to target. Raises ``ValueError`` if no path exists.
        """
        self.ensure_loaded()
        excluded = excluded_edges or set()

        if source not in self.nodes:
            raise KeyError(f"Unknown source node: {source}")
        if target not in self.nodes:
            raise KeyError(f"Unknown target node: {target}")

        dist: dict[str, float] = {source: 0.0}
        prev: dict[str, str | None] = {source: None}
        visited: set[str] = set()
        heap: list[tuple[float, str]] = [(0.0, source)]

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)
            if u == target:
                break
            for edge in self._adjacency.get(u, []):
                v = edge["to"]
                if v in visited:
                    continue
                key = edge["key"]
                if key in excluded:
                    continue
                nd = d + edge["weight"]
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        if target not in dist:
            raise ValueError(f"No path from {source} to {target}")

        # Reconstruct path
        path: list[str] = []
        cur: str | None = target
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return dist[target], path

    def shortest_path(
        self,
        source: str,
        target: str,
        excluded_edges: set[str] | None = None,
    ) -> list[str]:
        """Return the node-ID shortest path from ``source`` to ``target``."""
        _, path = self.dijkstra(source, target, excluded_edges)
        return path

    def path_edges(
        self,
        source: str,
        target: str,
        excluded_edges: set[str] | None = None,
    ) -> list[dict]:
        """Return the ordered list of edge records along the shortest path.

        Each edge dict is oriented so that ``"from"`` is the current node and
        ``"to"`` is the next node along the path, matching the robot's actual
        direction of travel.
        """
        _, path = self.dijkstra(source, target, excluded_edges)
        edges: list[dict] = []
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            key = self.edge_key(u, v)
            stored = self._edge_by_key.get(key)
            if stored is None:
                raise ValueError(f"No edge between {u} and {v}")
            edge = dict(stored)
            edge["from"] = u
            edge["to"] = v
            edges.append(edge)
        return edges

    # ------------------------------------------------------------------
    # Segment edges between two Passing Junctions
    # ------------------------------------------------------------------

    def segment_edges(
        self,
        u: str,
        v: str,
        excluded_edges: set[str] | None = None,
    ) -> dict:
        """Return the sequence of edges and target junction between two nodes.

        Typically used between two Passing Junctions to plan a corridor
        traversal while respecting quarantined edges.

        Returns:
            ``{"edges": [...], "target": str, "total_weight": float}``
        """
        total_weight, _path = self.dijkstra(u, v, excluded_edges)
        edge_list = self.path_edges(u, v, excluded_edges)
        return {
            "edges": edge_list,
            "target": v,
            "total_weight": total_weight,
        }

    # ------------------------------------------------------------------
    # Location summary for prompt grounding
    # ------------------------------------------------------------------

    def location_summary(self, max_boxes_per_zone: int | None = None) -> dict:
        """Return a self-contained location index for LLM prompt grounding.

        Groups nodes by zone: ``boxes`` (storage locations), ``aisles``,
        ``junctions`` (Passing Junctions), and ``landmarks`` (docks, truck,
        home, racks, zone centers). No external ROS dependencies.

        Args:
            max_boxes_per_zone: If set, keep only the first N boxes per zone
                (sorted by label) to bound prompt size. ``None`` keeps all.
        """
        self.ensure_loaded()
        summary: dict[str, dict] = defaultdict(
            lambda: {"boxes": [], "aisles": [], "junctions": [], "landmarks": []}
        )

        landmark_types = {"dock", "home", "truck", "rack", "zone"}

        for node_id, node in self.nodes.items():
            zone = node.get("zone", "?")
            ntype = node.get("type", "?")
            entry = {
                "id": node_id,
                "x": node["x"],
                "y": node["y"],
            }
            if ntype == "box":
                entry["label"] = node.get("label", node_id)
                summary[zone]["boxes"].append(entry)
            elif ntype == "aisle":
                summary[zone]["aisles"].append(entry)
            elif ntype == "junction":
                summary[zone]["junctions"].append(entry)
            elif ntype in landmark_types:
                entry["label"] = node.get("label", node_id)
                entry["kind"] = ntype
                summary[zone]["landmarks"].append(entry)

        # Sort boxes within each zone by label for stable output
        for zone_data in summary.values():
            zone_data["boxes"].sort(key=lambda b: b.get("label", b["id"]))
            if max_boxes_per_zone is not None:
                zone_data["boxes"] = zone_data["boxes"][:max_boxes_per_zone]

        return dict(summary)


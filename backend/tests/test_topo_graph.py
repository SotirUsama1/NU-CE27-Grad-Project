"""Unit tests for the TopoGraph route engine.

Validates Dijkstra shortest paths, obstacle edge exclusions (quarantines),
normalized edge keys, and Passing Junction detection.
"""

from __future__ import annotations

import os
import sys

import pytest

# Ensure the backend package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from cyberscape.topo_graph import TopoGraph

TOPO_PATH = os.path.join(
    os.path.dirname(__file__), "..", "data", "topo_graph.json"
)


@pytest.fixture
def graph() -> TopoGraph:
    g = TopoGraph(TOPO_PATH).load()
    return g


def test_graph_loads(graph: TopoGraph) -> None:
    """The generated graph file loads with nodes and edges."""
    assert len(graph.nodes) > 0
    assert len(graph.edges) > 0


def test_node_types(graph: TopoGraph) -> None:
    """Nodes carry the expected type field."""
    types = {n.get("type") for n in graph.nodes.values()}
    assert "box" in types
    assert "aisle" in types


def test_edge_keys_normalized(graph: TopoGraph) -> None:
    """Every edge key uses the normalized undirected format."""
    for edge in graph.edges:
        u, v = edge["from"], edge["to"]
        expected = "lock:edge:" + "_".join(sorted([u, v]))
        assert edge["key"] == expected, f"Key mismatch: {edge['key']} != {expected}"


def test_edge_required_fields(graph: TopoGraph) -> None:
    """Every edge has weight, key, single_lane, from, to."""
    for edge in graph.edges:
        for field in ("from", "to", "weight", "key", "single_lane"):
            assert field in edge, f"Missing field {field} in edge {edge}"


def test_node_required_fields(graph: TopoGraph) -> None:
    """Every node has x, y, capacity, type."""
    for nid, node in graph.nodes.items():
        for field in ("x", "y", "capacity", "type"):
            assert field in node, f"Missing field {field} in node {nid}"


def test_dijkstra_basic_path(graph: TopoGraph) -> None:
    """Dijkstra finds a path between two nodes."""
    box_nodes = [n for n, d in graph.nodes.items() if d["type"] == "box"]
    aisle_nodes = [n for n, d in graph.nodes.items() if d["type"] == "aisle"]
    assert box_nodes and aisle_nodes

    weight, path = graph.dijkstra(box_nodes[0], aisle_nodes[0])
    assert weight > 0
    assert path[0] == box_nodes[0]
    assert path[-1] == aisle_nodes[0]
    # Path must be contiguous (consecutive nodes connected by edges)
    for i in range(len(path) - 1):
        key = graph.edge_key(path[i], path[i + 1])
        assert key in {e["key"] for e in graph.edges}


def test_dijkstra_excluded_edges(graph: TopoGraph) -> None:
    """Excluding an edge can force a longer route or make a path unreachable."""
    box_nodes = [n for n, d in graph.nodes.items() if d["type"] == "box"]
    aisle_nodes = [n for n, d in graph.nodes.items() if d["type"] == "aisle"]
    src = box_nodes[0]
    dst = aisle_nodes[0]

    weight_normal, path_normal = graph.dijkstra(src, dst)
    first_edge_key = graph.edge_key(path_normal[0], path_normal[1])

    # A box is typically a leaf node; excluding its only spur edge isolates it
    try:
        graph.dijkstra(src, dst, excluded_edges={first_edge_key})
        # If an alternate path exists, it must be >= the normal weight
        weight_alt, _path_alt = graph.dijkstra(src, dst, excluded_edges={first_edge_key})
        assert weight_alt >= weight_normal
    except ValueError:
        # Unreachable is acceptable for a leaf node
        pass


def test_dijkstra_unknown_node(graph: TopoGraph) -> None:
    """Dijkstra raises KeyError for unknown source or target."""
    with pytest.raises(KeyError):
        graph.dijkstra("nonexistent", "some_node")
    with pytest.raises(KeyError):
        graph.dijkstra("some_node", "nonexistent")


def test_segment_edges(graph: TopoGraph) -> None:
    """segment_edges returns edges, target, and total_weight."""
    box_nodes = [n for n, d in graph.nodes.items() if d["type"] == "box"]
    aisle_nodes = [n for n, d in graph.nodes.items() if d["type"] == "aisle"]
    result = graph.segment_edges(box_nodes[0], aisle_nodes[0])
    assert "edges" in result
    assert "target" in result
    assert "total_weight" in result
    assert result["target"] == aisle_nodes[0]
    assert result["total_weight"] > 0
    assert len(result["edges"]) > 0


def test_location_summary(graph: TopoGraph) -> None:
    """location_summary groups locations by zone with box/aisle/junction lists."""
    summary = graph.location_summary()
    assert isinstance(summary, dict)
    for zone, data in summary.items():
        assert "boxes" in data
        assert "aisles" in data
        assert "junctions" in data
        # Boxes should be sorted by label
        labels = [b.get("label", b["id"]) for b in data["boxes"]]
        assert labels == sorted(labels), f"Boxes not sorted in zone {zone}"


def test_location_summary_landmarks(graph: TopoGraph) -> None:
    """location_summary includes dock/home/truck/rack/zone landmarks."""
    summary = graph.location_summary()
    all_landmarks = {lm["id"] for data in summary.values() for lm in data["landmarks"]}
    assert "dock" in all_landmarks
    assert "truck" in all_landmarks
    assert "home" in all_landmarks
    # Landmark entries carry their node kind for prompt grounding
    kinds = {lm["kind"] for data in summary.values() for lm in data["landmarks"]}
    assert "dock" in kinds
    assert "rack" in kinds


def test_location_summary_box_cap(graph: TopoGraph) -> None:
    """max_boxes_per_zone bounds boxes per zone without dropping landmarks."""
    full = graph.location_summary()
    capped = graph.location_summary(max_boxes_per_zone=10)
    for zone, data in capped.items():
        assert len(data["boxes"]) <= 10
        assert data["landmarks"], f"Landmarks missing in zone {zone}"
    assert len(full["A"]["boxes"]) > len(capped["A"]["boxes"])


def test_shortest_path_convenience(graph: TopoGraph) -> None:
    """shortest_path convenience method returns node ID list."""
    box_nodes = [n for n, d in graph.nodes.items() if d["type"] == "box"]
    aisle_nodes = [n for n, d in graph.nodes.items() if d["type"] == "aisle"]
    path = graph.shortest_path(box_nodes[0], aisle_nodes[0])
    assert isinstance(path, list)
    assert path[0] == box_nodes[0]
    assert path[-1] == aisle_nodes[0]


def test_path_edges_convenience(graph: TopoGraph) -> None:
    """path_edges returns ordered edge records."""
    box_nodes = [n for n, d in graph.nodes.items() if d["type"] == "box"]
    aisle_nodes = [n for n, d in graph.nodes.items() if d["type"] == "aisle"]
    edges = graph.path_edges(box_nodes[0], aisle_nodes[0])
    assert isinstance(edges, list)
    assert len(edges) > 0
    for e in edges:
        assert "weight" in e and "key" in e


def test_path_edges_direction_matches_travel(graph: TopoGraph) -> None:
    """path_edges orients "from"/"to" to match the robot's direction of travel."""
    box_nodes = [n for n, d in graph.nodes.items() if d["type"] == "box"]
    aisle_nodes = [n for n, d in graph.nodes.items() if d["type"] == "aisle"]
    edges = graph.path_edges(box_nodes[0], aisle_nodes[0])
    # Reconstruct the path from edge directions
    path = [edges[0]["from"]]
    for e in edges:
        assert e["from"] == path[-1], f"Edge direction broken: {e['from']} != {path[-1]}"
        path.append(e["to"])
    assert path[0] == box_nodes[0]
    assert path[-1] == aisle_nodes[0]


def test_graph_is_single_connected_component(graph: TopoGraph) -> None:
    """The entire graph must be a single connected component."""
    graph.ensure_loaded()
    nodes = set(graph.nodes.keys())
    if not nodes:
        pytest.skip("No nodes in graph")
    # Build adjacency
    adj: dict[str, set[str]] = {nid: set() for nid in nodes}
    for edge in graph.edges:
        adj[edge["from"]].add(edge["to"])
        adj[edge["to"]].add(edge["from"])
    # BFS from an arbitrary node
    start = next(iter(nodes))
    visited = {start}
    stack = [start]
    while stack:
        cur = stack.pop()
        for nb in adj[cur]:
            if nb not in visited:
                visited.add(nb)
                stack.append(nb)
    assert visited == nodes, f"Graph is disconnected: {len(visited)}/{len(nodes)} nodes reachable"


def test_at_least_8_passing_junctions(graph: TopoGraph) -> None:
    """At least 8 Passing Junctions (capacity=2) must exist in the graph."""
    graph.ensure_loaded()
    junctions = [
        n for n, d in graph.nodes.items() if d["type"] == "junction"
    ]
    assert len(junctions) >= 8, f"Expected at least 8 junctions, found {len(junctions)}"
    for jid in junctions:
        assert graph.nodes[jid]["capacity"] == 2, f"Junction {jid} has wrong capacity"
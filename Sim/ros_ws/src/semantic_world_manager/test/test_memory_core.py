from pathlib import Path

from semantic_world_manager.memory_core import ThreadSafeMemoryCore


def make_core():
    core = ThreadSafeMemoryCore(target_frame="map")
    core.seed_entity("bin_a", "workcell_bin", [0.0, 0.0, 0.0], 0.0, [0.67, 0.67, 0.79])
    core.seed_entity("bin_b", "workcell_bin", [1.0, 0.0, 0.0], 0.0, [0.67, 0.67, 0.79])
    core.seed_entity("wall", "workcell", [5.0, 0.0, 0.0], 0.0, [22.12, 21.48, 7.67])
    return core


def test_seed_from_phase1_semantic_database():
    database_path = next(
        parent / "maps" / "Our_Structured_Warehouse_semantic.json"
        for parent in Path(__file__).resolve().parents
        if (parent / "maps" / "Our_Structured_Warehouse_semantic.json").is_file()
    )
    core = ThreadSafeMemoryCore(target_frame="map")
    core.seed_from_file(database_path)

    summary = core.summary()
    assert summary["total_objects"] == 7
    assert summary["categories"] == {"workcell": 1, "workcell_bin": 6}
    assert core.get_entity("workcell_bin")["size"] == [0.6684, 0.6684, 0.7909]


def test_update_transitions_seeded_entity_to_live():
    core = make_core()
    assert core.summary()["status"] == "static_seeded"

    assert core.update_entity_pose("bin_a", [2.123, 3.456, 0.0], 90.0)
    core.mark_telemetry_received()

    entity = core.get_entity("bin_a")
    assert entity["loc"] == [2.123, 3.456, 0.0]
    assert entity["status"] == "live_tracking"
    assert core.summary()["status"] == "live_tracking"


def test_spatial_query_filters_sorts_and_excludes_target():
    core = make_core()
    matches = core.spatial("bin_a", 1.5)
    assert matches == [{"id": "bin_b", "dist": 1.0}]
    assert core.spatial("missing", 1.5) is None


def test_category_listing_and_copies():
    core = make_core()
    items = core.objects("workcell_bin")
    assert [item["id"] for item in items] == ["bin_a", "bin_b"]
    entity = core.get_entity("bin_a")
    entity["loc"][0] = 99.0
    assert core.get_entity("bin_a")["loc"][0] == 0.0

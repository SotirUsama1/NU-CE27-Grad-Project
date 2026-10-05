from fastapi.testclient import TestClient

from semantic_world_manager.api_server import create_app
from semantic_world_manager.memory_core import ThreadSafeMemoryCore


def make_client():
    core = ThreadSafeMemoryCore(target_frame="map")
    core.seed_entity("workcell", "workcell", [0.0, 0.0, 0.0], 0.0, [22.12, 21.48, 7.67])
    core.seed_entity("workcell_bin", "workcell_bin", [3.004, 6.0, 0.004], 90.004, [0.6684, 0.6684, 0.7909])
    core.seed_entity("workcell_bin_clone", "workcell_bin", [4.0, 6.0, 0.0], 0.0, [0.67, 0.67, 0.79])
    return TestClient(create_app(core)), core


def test_summary_and_locate_endpoints():
    client, _ = make_client()
    summary = client.get("/api/scene/summary")
    assert summary.status_code == 200
    assert summary.json()["total_objects"] == 3
    assert summary.json()["categories"] == {"workcell": 1, "workcell_bin": 2}

    located = client.get("/api/scene/locate/workcell_bin")
    assert located.status_code == 200
    assert located.json() == {
        "id": "workcell_bin",
        "type": "workcell_bin",
        "loc": [3.0, 6.0, 0.0],
        "yaw_deg": 90.0,
        "size": [0.67, 0.67, 0.79],
        "frame_id": "map",
    }
    assert client.get("/api/scene/locate/missing").status_code == 404


def test_spatial_objects_and_health_endpoints():
    client, core = make_client()
    assert client.get("/api/scene/spatial", params={"target": "workcell_bin", "radius": 2}).json() == [
        {"id": "workcell_bin_clone", "dist": 1.0}
    ]
    assert len(client.get("/api/scene/objects", params={"category": "workcell_bin"}).json()) == 2
    assert client.get("/health").json()["ros_connected"] is False
    core.mark_telemetry_received()
    assert client.get("/health").json()["ros_connected"] is True

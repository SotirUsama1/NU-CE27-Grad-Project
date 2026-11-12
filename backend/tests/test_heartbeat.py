"""Unit and integration tests for the Phase 4 simulation-time liveness monitor.

Validates the two-tier heartbeat escalation:

* Tier 1 (3 s):  UNRESPONSIVE hold, self-healing on the next heartbeat.
* Tier 2 (10 s): QUARANTINED escalation + Redis quarantine keys, Dijkstra
  edge exclusion, and in-flight task release.

All timeout decisions are made against SIMULATION time - never the wall
clock - so the wall-clock test below proves a paused/slow RTF cannot
produce false positives.
"""

from __future__ import annotations

import time
from collections import defaultdict

import pytest
import pytest_asyncio

from cyberscape.heartbeat import HeartbeatMonitor
from cyberscape.redis_client import RedisClient
from cyberscape.topo_graph import TopoGraph


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class FakeDB:
    """In-memory stand-in for the Database methods the monitor uses."""

    def __init__(self):
        self.robots: dict[str, dict] = {}
        self.claimed_tasks: dict[str, list[str]] = defaultdict(list)
        self.unclaimed: list[str] = []
        self.telemetry: list[tuple] = []

    def _ensure(self, robot_id: str) -> dict:
        return self.robots.setdefault(
            robot_id,
            {
                "id": robot_id,
                "status": "IDLE",
                "current_node": None,
                "sim_time_last_seen": None,
            },
        )

    async def update_robot_telemetry(
        self,
        robot_id: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        sim_time: float | None = None,
        current_node: str | None = None,
    ) -> None:
        robot = self._ensure(robot_id)
        if sim_time is not None:
            robot["sim_time_last_seen"] = sim_time
        if current_node is not None:
            robot["current_node"] = current_node
        self.telemetry.append((robot_id, x, y, yaw, sim_time, current_node))

    async def get_robot(self, robot_id: str) -> dict | None:
        robot = self.robots.get(robot_id)
        return dict(robot) if robot else None

    async def set_robot_status(self, robot_id: str, status: str) -> None:
        self._ensure(robot_id)["status"] = status

    async def get_monitored_robots(self) -> list[dict]:
        return [
            dict(r)
            for r in self.robots.values()
            if r["status"] in ("IDLE", "BUSY", "UNRESPONSIVE")
        ]

    async def count_claimed_tasks(self, robot_id: str) -> int:
        return len(self.claimed_tasks.get(robot_id, []))

    async def unclaim_robot_tasks(self, robot_id: str) -> int:
        tasks = self.claimed_tasks.pop(robot_id, [])
        self.unclaimed.extend(tasks)
        return len(tasks)


class FakeRedis:
    """In-memory stand-in for the quarantine primitives."""

    def __init__(self):
        self.quarantined_nodes: set[str] = set()
        self.quarantined_edges: set[str] = set()
        self.set_calls: list[tuple] = []

    async def set_quarantine(self, node: str, edges: list[str]) -> None:
        self.set_calls.append((node, list(edges)))
        self.quarantined_nodes.add(node)
        self.quarantined_edges.update(edges)

    async def clear_quarantine(self, node: str, edges: list[str]) -> None:
        self.quarantined_nodes.discard(node)
        self.quarantined_edges.difference_update(edges)

    async def is_node_quarantined(self, node: str) -> bool:
        return node in self.quarantined_nodes


def _mini_graph() -> TopoGraph:
    """n_a -- n_b -- n_c, same shape as the dispatcher tests."""
    g = TopoGraph()
    g.nodes = {
        "n_a": {"x": 0.0, "y": 0.0, "capacity": 1, "type": "aisle", "zone": "A"},
        "n_b": {"x": 2.0, "y": 0.0, "capacity": 2, "type": "junction", "zone": "A"},
        "n_c": {"x": 4.0, "y": 0.0, "capacity": 1, "type": "aisle", "zone": "A"},
    }
    edges = [
        {"from": "n_a", "to": "n_b", "weight": 2.0,
         "key": "lock:edge:n_a_n_b", "single_lane": True},
        {"from": "n_b", "to": "n_c", "weight": 2.0,
         "key": "lock:edge:n_b_n_c", "single_lane": True},
    ]
    g.edges = edges
    g._adjacency = defaultdict(list)
    g._edge_by_key = {}
    for edge in edges:
        g._adjacency[edge["from"]].append(edge)
        g._adjacency[edge["to"]].append(
            {**edge, "from": edge["to"], "to": edge["from"]}
        )
        g._edge_by_key[edge["key"]] = edge
    g._loaded = True
    return g


def _monitor(db, redis, topo=None, unresponsive=3.0, quarantine=10.0):
    return HeartbeatMonitor(
        db=db,
        redis_client=redis,
        topo_graph=topo if topo is not None else _mini_graph(),
        unresponsive_timeout=unresponsive,
        quarantine_timeout=quarantine,
    )


# ---------------------------------------------------------------------------
# Tier 1: UNRESPONSIVE after 3 s (simulation time)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_tier1_unresponsive_after_3s_sim_time():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "BUSY", "current_node": "n_a",
        "sim_time_last_seen": 100.0,
    }
    m = _monitor(db, redis)

    # Exactly at the boundary: no escalation yet (strictly greater).
    assert await m.sweep(now_sim=103.0) == []
    assert db.robots["husky1"]["status"] == "BUSY"

    actions = await m.sweep(now_sim=104.0)  # 4 s > 3 s
    assert actions == ["husky1:UNRESPONSIVE"]
    assert db.robots["husky1"]["status"] == "UNRESPONSIVE"
    assert redis.quarantined_nodes == set(), "Tier 1 must NOT quarantine"
    assert db.unclaimed == [], "Tier 1 must NOT release in-flight tasks"


@pytest.mark.asyncio
async def test_tier1_not_triggered_below_timeout():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "IDLE", "current_node": "n_a",
        "sim_time_last_seen": 100.0,
    }
    m = _monitor(db, redis)

    actions = await m.sweep(now_sim=102.5)  # 2.5 s < 3 s
    assert actions == []
    assert db.robots["husky1"]["status"] == "IDLE"


@pytest.mark.asyncio
async def test_timeouts_use_simulation_time_not_wall_clock(monkeypatch):
    """A wall-clock implementation would compute an astronomic age.

    time.time() is patched to return ~year 33658; a monitor consulting the
    wall clock would instantly quarantine every robot. The sim-time monitor
    must take no action.
    """
    monkeypatch.setattr(time, "time", lambda: 1_000_000_000_000.0)
    db, redis = FakeDB(), FakeRedis()
    m = _monitor(db, redis)

    await m.record_heartbeat("husky1", sim_time=100.0)  # advances sim clock only
    db.robots["husky1"]["status"] = "BUSY"

    actions = await m.sweep()  # sim now = 100.0 -> age 0 s

    assert actions == []
    assert db.robots["husky1"]["status"] == "BUSY"


@pytest.mark.asyncio
async def test_no_sweep_action_before_any_heartbeat_observed():
    """A paused simulation (no heartbeat ever seen) can never time out."""
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "BUSY", "current_node": "n_a",
        "sim_time_last_seen": None,
    }
    m = _monitor(db, redis)

    assert await m.sweep() == []  # latest sim time unknown -> no-op
    assert db.robots["husky1"]["status"] == "BUSY"


@pytest.mark.asyncio
async def test_robots_without_sim_time_last_seen_are_skipped():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "IDLE", "current_node": "n_a",
        "sim_time_last_seen": None,
    }
    m = _monitor(db, redis)
    m._latest_sim_time = 500.0  # sim clock is running

    actions = await m.sweep()
    assert actions == []
    assert db.robots["husky1"]["status"] == "IDLE"


# ---------------------------------------------------------------------------
# Tier 2: QUARANTINED after 10 s + edge exclusion + task release
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_tier2_quarantine_after_10s_with_edge_exclusion():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "BUSY", "current_node": "n_b",
        "sim_time_last_seen": 100.0,
    }
    db.claimed_tasks["husky1"] = ["t1", "t2"]
    m = _monitor(db, redis)

    # At 10 s the robot is already past the Tier 1 window (3 s) and is held
    # in UNRESPONSIVE first; quarantine escalation needs strictly more than
    # 10 s.
    assert await m.sweep(now_sim=110.0) == ["husky1:UNRESPONSIVE"]
    assert db.robots["husky1"]["status"] == "UNRESPONSIVE"

    actions = await m.sweep(now_sim=112.0)  # 12 s > 10 s

    assert actions == ["husky1:QUARANTINED"]
    assert db.robots["husky1"]["status"] == "QUARANTINED"
    # In-flight primitives released back to the dispatch pool
    assert db.unclaimed == ["t1", "t2"]
    # Node + incident edges quarantined in Redis
    assert redis.quarantined_nodes == {"n_b"}
    assert redis.quarantined_edges == {
        "lock:edge:n_a_n_b", "lock:edge:n_b_n_c",
    }
    # Dijkstra exclusion set populated for routing
    assert m.excluded_edges == {"lock:edge:n_a_n_b", "lock:edge:n_b_n_c"}


@pytest.mark.asyncio
async def test_tier2_is_idempotent():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "BUSY", "current_node": "n_b",
        "sim_time_last_seen": 100.0,
    }
    db.claimed_tasks["husky1"] = ["t1"]
    m = _monitor(db, redis)

    assert await m.sweep(now_sim=112.0) == ["husky1:QUARANTINED"]
    assert await m.sweep(now_sim=115.0) == []  # already quarantined

    assert len(redis.set_calls) == 1, "Quarantine must not be re-applied"
    assert db.unclaimed == ["t1"], "Tasks must not be unclaimed twice"


@pytest.mark.asyncio
async def test_tier2_unknown_node_still_quarantines_robot():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "IDLE", "current_node": "ghost_node",
        "sim_time_last_seen": 100.0,
    }
    m = _monitor(db, redis)

    actions = await m.sweep(now_sim=111.0)

    assert actions == ["husky1:QUARANTINED"]
    assert db.robots["husky1"]["status"] == "QUARANTINED"
    assert redis.quarantined_nodes == {"ghost_node"}
    assert redis.quarantined_edges == set()
    assert m.excluded_edges == set()


@pytest.mark.asyncio
async def test_quarantined_robot_not_recovered_by_heartbeat():
    """Only an administrative override clears a Tier 2 quarantine."""
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "QUARANTINED", "current_node": "n_b",
        "sim_time_last_seen": 100.0,
    }
    m = _monitor(db, redis)

    await m.record_heartbeat("husky1", sim_time=120.0)

    assert db.robots["husky1"]["status"] == "QUARANTINED"


# ---------------------------------------------------------------------------
# Recovery from Tier 1
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_heartbeat_recovers_unresponsive_to_busy_when_tasks_claimed():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "UNRESPONSIVE", "current_node": "n_a",
        "sim_time_last_seen": 100.0,
    }
    db.claimed_tasks["husky1"] = ["t9"]
    m = _monitor(db, redis)

    await m.record_heartbeat("husky1", sim_time=105.0)

    assert db.robots["husky1"]["status"] == "BUSY"
    assert db.robots["husky1"]["sim_time_last_seen"] == 105.0


@pytest.mark.asyncio
async def test_heartbeat_recovers_unresponsive_to_idle_when_no_tasks():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "UNRESPONSIVE", "current_node": "n_a",
        "sim_time_last_seen": 100.0,
    }
    m = _monitor(db, redis)

    await m.record_heartbeat("husky1", sim_time=105.0)

    assert db.robots["husky1"]["status"] == "IDLE"


@pytest.mark.asyncio
async def test_sweep_recovers_unresponsive_once_sim_time_resumes():
    db, redis = FakeDB(), FakeRedis()
    db.robots["husky1"] = {
        "id": "husky1", "status": "UNRESPONSIVE", "current_node": "n_a",
        "sim_time_last_seen": 100.0,
    }
    m = _monitor(db, redis)

    actions = await m.sweep(now_sim=101.0)  # back under the 3 s window

    assert actions == ["husky1:RECOVERED"]
    assert db.robots["husky1"]["status"] == "IDLE"


@pytest.mark.asyncio
async def test_record_heartbeat_persists_telemetry():
    db, redis = FakeDB(), FakeRedis()
    m = _monitor(db, redis)

    await m.record_heartbeat(
        "husky1", sim_time=7.5, x=1.5, y=2.5, yaw=0.7, current_node="n_b"
    )

    assert db.telemetry == [("husky1", 1.5, 2.5, 0.7, 7.5, "n_b")]
    robot = await db.get_robot("husky1")
    assert robot["sim_time_last_seen"] == 7.5
    assert robot["current_node"] == "n_b"


# ---------------------------------------------------------------------------
# Integration: Tier 2 writes real Redis quarantine keys (live Redis)
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture
async def real_redis():
    client = RedisClient()
    await client.connect()
    try:
        await client.redis.ping()
    except Exception as exc:  # pragma: no cover - env dependent
        await client.disconnect()
        pytest.skip(f"Redis not reachable: {exc}")
    await client.redis.flushdb()
    yield client
    await client.disconnect()


@pytest.mark.asyncio
async def test_integration_tier2_quarantine_blocks_acquisition(real_redis):
    rc = real_redis
    db = FakeDB()
    db.robots["husky1"] = {
        "id": "husky1", "status": "BUSY", "current_node": "n_b",
        "sim_time_last_seen": 100.0,
    }
    db.claimed_tasks["husky1"] = ["t1"]
    m = _monitor(db, rc)

    actions = await m.sweep(now_sim=112.0)

    assert actions == ["husky1:QUARANTINED"]
    assert await rc.is_node_quarantined("n_b")
    assert await rc.redis.exists("quarantine:edge:lock:edge:n_a_n_b") == 1
    assert await rc.redis.exists("quarantine:edge:lock:edge:n_b_n_c") == 1

    # The quarantined corridor is refused by the locking layer...
    assert not await rc.acquire_segment_lock(
        "husky2", ["lock:edge:n_b_n_c"], "n_c", ttl=5.0
    )
    # ...as is the quarantined node itself.
    assert not await rc.acquire_segment_lock("husky2", [], "n_b", ttl=5.0)

    # Administrative override lifts the quarantine and reopens the corridor.
    await rc.clear_quarantine(
        "n_b", ["lock:edge:n_a_n_b", "lock:edge:n_b_n_c"]
    )
    assert await rc.acquire_segment_lock(
        "husky2", ["lock:edge:n_b_n_c"], "n_c", ttl=5.0
    )

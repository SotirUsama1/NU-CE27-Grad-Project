"""Unit and integration tests for the Phase 4 event-driven dispatcher.

Validates that the dispatcher claims tasks for IDLE robots, locks their
route segments atomically, reverts tasks to PENDING (with the robot pin
cleared) when segment locks are blocked or unroutable, and reacts to
Pub/Sub triggers with a 7-second fallback sweep.
"""

from __future__ import annotations

import asyncio
import time
from collections import defaultdict
from pathlib import Path

import asyncpg
import pytest
import pytest_asyncio
import redis.asyncio as aioredis

from cyberscape.config import settings
from cyberscape.db import Database
from cyberscape.dispatcher import (
    CHANNEL_SEGMENT_RELEASED,
    Dispatcher,
)
from cyberscape.redis_client import RedisClient
from cyberscape.topo_graph import TopoGraph

SCHEMA_PATH = Path(__file__).resolve().parent.parent / "sql" / "schema.sql"


# ---------------------------------------------------------------------------
# Fakes for unit tests
# ---------------------------------------------------------------------------


class FakeDB:
    def __init__(self):
        self.idle_robots: list[dict] = []
        self.claimable: dict[str, dict | None] = {}
        self.unclaimed: list[str] = []
        self.statuses: dict[str, str] = {}

    async def list_idle_robots(self):
        return [
            r for r in self.idle_robots
            if self.statuses.get(r["id"], "IDLE") == "IDLE"
        ]

    async def claim_next_task(self, robot_id):
        return self.claimable.pop(robot_id, None)

    async def set_robot_status(self, robot_id, status):
        self.statuses[robot_id] = status

    async def unclaim_task(self, task_id):
        self.unclaimed.append(task_id)


class FakeRedis:
    def __init__(self, acquire_result=True):
        self.acquire_result = acquire_result
        self.acquire_calls: list[tuple] = []

    async def acquire_segment_lock(self, robot_id, edge_keys, junction_node,
                                   ttl, junction_capacity=2):
        self.acquire_calls.append(
            (robot_id, list(edge_keys), junction_node, ttl, junction_capacity)
        )
        return self.acquire_result


def _mini_graph() -> TopoGraph:
    """n_a -- n_b -- n_c (n_c is a capacity-2 Passing Junction)."""
    g = TopoGraph()
    g.nodes = {
        "n_a": {"x": 0.0, "y": 0.0, "capacity": 1, "type": "aisle", "zone": "A"},
        "n_b": {"x": 2.0, "y": 0.0, "capacity": 1, "type": "aisle", "zone": "A"},
        "n_c": {"x": 4.0, "y": 0.0, "capacity": 2, "type": "junction", "zone": "A"},
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
        g._adjacency[edge["to"]].append({**edge, "from": edge["to"], "to": edge["from"]})
        g._edge_by_key[edge["key"]] = edge
    g._loaded = True
    return g


def _make_dispatcher(db, redis, topo=None) -> Dispatcher:
    return Dispatcher(db=db, redis_client=redis, topo_graph=topo)


# ---------------------------------------------------------------------------
# Unit tests (fakes)
# ---------------------------------------------------------------------------


def test_sweep_interval_defaults_to_config():
    d = _make_dispatcher(FakeDB(), FakeRedis())
    assert d.sweep_interval == settings.DISPATCH_SWEEP_INTERVAL_S == 7.0


@pytest.mark.asyncio
async def test_dispatch_success_sets_robot_busy_and_locks_route():
    db, fake_redis = FakeDB(), FakeRedis(acquire_result=True)
    db.idle_robots = [{"id": "husky1", "current_node": "n_a"}]
    db.claimable["husky1"] = {"id": "t1", "mission_id": "m1", "sequence_order": 1,
                              "primitive_type": "NAVIGATE_TO", "target_node": "n_c"}
    d = _make_dispatcher(db, fake_redis, _mini_graph())

    dispatched = await d.dispatch_once()

    assert dispatched == 1
    assert db.statuses["husky1"] == "BUSY"
    robot_id, edge_keys, junction, ttl, capacity = fake_redis.acquire_calls[0]
    assert robot_id == "husky1"
    assert edge_keys == ["lock:edge:n_a_n_b", "lock:edge:n_b_n_c"]
    assert junction == "n_c"
    assert capacity == 2
    assert ttl == settings.SEGMENT_LOCK_TTL_S
    assert db.unclaimed == []


@pytest.mark.asyncio
async def test_dispatch_blocked_locks_revert_task_to_pending():
    db, fake_redis = FakeDB(), FakeRedis(acquire_result=False)
    db.idle_robots = [{"id": "husky1", "current_node": "n_a"}]
    db.claimable["husky1"] = {"id": "t1", "mission_id": "m1", "sequence_order": 1,
                              "primitive_type": "NAVIGATE_TO", "target_node": "n_c"}
    d = _make_dispatcher(db, fake_redis, _mini_graph())

    dispatched = await d.dispatch_once()

    assert dispatched == 0
    assert "t1" in db.unclaimed, "Blocked task must be reverted to PENDING"
    assert "husky1" not in db.statuses, "Robot must stay IDLE when blocked"


@pytest.mark.asyncio
async def test_dispatch_unroutable_target_reverts_task():
    db, fake_redis = FakeDB(), FakeRedis()
    db.idle_robots = [{"id": "husky1", "current_node": "n_a"}]
    db.claimable["husky1"] = {"id": "t1", "mission_id": "m1", "sequence_order": 1,
                              "primitive_type": "NAVIGATE_TO", "target_node": "nowhere"}
    d = _make_dispatcher(db, fake_redis, _mini_graph())

    dispatched = await d.dispatch_once()

    assert dispatched == 0
    assert "t1" in db.unclaimed
    assert fake_redis.acquire_calls == []


@pytest.mark.asyncio
async def test_dispatch_with_no_idle_robots_is_noop():
    d = _make_dispatcher(FakeDB(), FakeRedis())
    assert await d.dispatch_once() == 0


@pytest.mark.asyncio
async def test_dispatch_unlocalized_robot_reverts_task():
    """A robot with current_node=None must NOT be dispatched with 0 edge locks."""
    db, fake_redis = FakeDB(), FakeRedis()
    db.idle_robots = [{"id": "husky1", "current_node": None}]
    db.claimable["husky1"] = {
        "id": "t1",
        "mission_id": "m1",
        "sequence_order": 1,
        "primitive_type": "NAVIGATE_TO",
        "target_node": "n_c",
    }
    d = _make_dispatcher(db, fake_redis, _mini_graph())

    dispatched = await d.dispatch_once()

    assert dispatched == 0
    assert "t1" in db.unclaimed
    assert fake_redis.acquire_calls == []
    assert d.stats["unroutable"] == 1


@pytest.mark.asyncio
async def test_dispatch_coalesces_triggers():
    """Concurrent dispatch triggers are coalesced without dropping passes."""
    db, fake_redis = FakeDB(), FakeRedis()
    db.idle_robots = [{"id": "husky1", "current_node": "n_a"}]
    db.claimable["husky1"] = {
        "id": "t1",
        "mission_id": "m1",
        "sequence_order": 1,
        "primitive_type": "NAVIGATE_TO",
        "target_node": "n_a",
    }
    d = _make_dispatcher(db, fake_redis, _mini_graph())

    # Trigger multiple concurrent dispatches
    res = await asyncio.gather(d.dispatch_once(), d.dispatch_once())
    assert sum(res) == 1
    assert db.statuses.get("husky1") == "BUSY"


# ---------------------------------------------------------------------------
# Integration tests (live Redis + live PostgreSQL)
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture
async def real_db():
    try:
        pool = await asyncpg.create_pool(dsn=settings.DATABASE_URL, min_size=2, max_size=10)
    except (OSError, asyncpg.PostgresError) as exc:  # pragma: no cover
        pytest.skip(f"PostgreSQL not reachable: {exc}")
    schema_sql = SCHEMA_PATH.read_text()
    async with pool.acquire() as conn:
        for statement in (s.strip() for s in schema_sql.split(";") if s.strip()):
            await conn.execute(statement)
        await conn.execute("TRUNCATE robots, missions, tasks RESTART IDENTITY CASCADE")
    yield pool
    await pool.close()


@pytest_asyncio.fixture
async def redis_client():
    client = RedisClient()
    await client.connect()
    try:
        await client.redis.ping()
    except (OSError, aioredis.RedisError) as exc:  # pragma: no cover
        await client.disconnect()
        pytest.skip(f"Redis not reachable: {exc}")
    await client.redis.flushdb()
    yield client
    await client.disconnect()


async def _seed(pool, robots, mission_id, task_id, target):
    async with pool.acquire() as conn:
        await conn.executemany(
            "INSERT INTO robots (id, status, current_node) VALUES ($1, 'IDLE', $2)",
            robots,
        )
        await conn.execute(
            "INSERT INTO missions (id, text) VALUES ($1, $2)",
            mission_id, f"mission {mission_id}",
        )
        await conn.execute(
            """
            INSERT INTO tasks (id, mission_id, sequence_order, primitive_type, target_node)
            VALUES ($1, $2, 1, 'NAVIGATE_TO', $3)
            """,
            task_id, mission_id, target,
        )


@pytest.mark.asyncio
async def test_integration_dispatch_claims_locks_and_marks_busy(real_db, redis_client):
    pool = real_db
    await _seed(pool, [("husky1", "n_a"), ("husky2", "n_b")], "m1", "t1", "n_c")
    db = Database()
    db.pool = pool
    d = _make_dispatcher(db, redis_client, _mini_graph())

    dispatched = await d.dispatch_once()

    assert dispatched == 1
    async with pool.acquire() as conn:
        task = await conn.fetchrow(
            "SELECT status, assigned_robot_id FROM tasks WHERE id = 't1'"
        )
        busy = await conn.fetchval("SELECT status FROM robots WHERE id = 'husky1'")
        idle = await conn.fetchval("SELECT status FROM robots WHERE id = 'husky2'")
    assert task["status"] == "CLAIMED"
    assert task["assigned_robot_id"] == "husky1"
    assert busy == "BUSY"
    assert idle == "IDLE"
    # Corridor locks are visible in Redis
    assert await redis_client.redis.get("lock:edge:n_a_n_b") == "husky1"
    assert await redis_client.redis.get("lock:edge:n_b_n_c") == "husky1"
    assert await redis_client.redis.hexists("lock:junction:n_c", "husky1") == 1


@pytest.mark.asyncio
async def test_integration_blocked_by_foreign_lock_reverts_task(real_db, redis_client):
    pool = real_db
    await _seed(pool, [("husky1", "n_a")], "m1", "t1", "n_c")

    # A foreign robot already holds the first corridor edge
    assert await redis_client.acquire_segment_lock(
        "other_robot", ["lock:edge:n_a_n_b"], "n_c", ttl=10.0
    )

    db = Database()
    db.pool = pool
    d = _make_dispatcher(db, redis_client, _mini_graph())
    dispatched = await d.dispatch_once()

    assert dispatched == 0
    async with pool.acquire() as conn:
        task = await conn.fetchrow(
            "SELECT status, assigned_robot_id FROM tasks WHERE id = 't1'"
        )
        robot_status = await conn.fetchval("SELECT status FROM robots WHERE id = 'husky1'")
    assert task["status"] == "PENDING", "Blocked task must revert to PENDING"
    assert task["assigned_robot_id"] is None, "Robot pin must be cleared on revert"
    assert robot_status == "IDLE"
    assert d.stats["blocked"] == 1

    # Once the foreign lock is released, a retry dispatches normally
    await redis_client.release_segment_lock("other_robot", ["lock:edge:n_a_n_b"], "n_c")
    dispatched = await d.dispatch_once()
    assert dispatched == 1


@pytest.mark.asyncio
async def test_integration_robot_already_at_target_reserves_node_only(real_db, redis_client):
    pool = real_db
    await _seed(pool, [("husky1", "n_c")], "m1", "t1", "n_c")
    db = Database()
    db.pool = pool
    d = _make_dispatcher(db, redis_client, _mini_graph())

    dispatched = await d.dispatch_once()

    assert dispatched == 1
    assert await redis_client.redis.hexists("lock:junction:n_c", "husky1") == 1
    assert await redis_client.redis.get("lock:edge:n_a_n_b") is None


@pytest.mark.asyncio
async def test_integration_segment_released_pubsub_triggers_dispatch(real_db, redis_client):
    pool = real_db
    await _seed(pool, [("husky1", "n_a")], "m1", "t1", "n_c")

    # Corridor initially blocked -> a dispatch would revert the task
    assert await redis_client.acquire_segment_lock(
        "other_robot", ["lock:edge:n_a_n_b", "lock:edge:n_b_n_c"], "n_c", ttl=10.0
    )
    db = Database()
    db.pool = pool
    d = _make_dispatcher(db, redis_client, _mini_graph())
    await d.start()
    try:
        await asyncio.sleep(0.5)  # allow pubsub subscription to settle
        await redis_client.release_segment_lock(
            "other_robot", ["lock:edge:n_a_n_b", "lock:edge:n_b_n_c"], "n_c"
        )
        # Publish the release event; re-publish while polling so a slow
        # subscription or a message published before it cannot lose the test.
        task_status = None
        robot_status = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            await redis_client.redis.publish(CHANNEL_SEGMENT_RELEASED, "n_c")
            await asyncio.sleep(0.1)
            async with pool.acquire() as conn:
                task_status = await conn.fetchval(
                    "SELECT status FROM tasks WHERE id = 't1'"
                )
                robot_status = await conn.fetchval(
                    "SELECT status FROM robots WHERE id = 'husky1'"
                )
                if task_status == "CLAIMED" and robot_status == "BUSY":
                    break

        async with pool.acquire() as conn:
            task = await conn.fetchrow(
                "SELECT status, assigned_robot_id FROM tasks WHERE id = 't1'"
            )
            robot_status = await conn.fetchval("SELECT status FROM robots WHERE id = 'husky1'")
        assert task_status == "CLAIMED", "Pub/Sub trigger must dispatch the task"
        assert task["assigned_robot_id"] == "husky1"
        assert robot_status == "BUSY"
    finally:
        await d.stop()

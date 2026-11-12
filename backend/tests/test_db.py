"""Concurrency unit tests for the CyberScape database layer.

Phase 3: Validates sequential task claiming, prerequisite gating, robot
session affinity, and SKIP LOCKED behaviour under simulated race conditions.

These tests mock ``asyncpg`` so they run without a live PostgreSQL instance.
"""

from __future__ import annotations

import asyncio
import json
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import asyncpg
import pytest
import pytest_asyncio

from cyberscape.config import settings
from cyberscape.db import Database

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def db() -> Database:
    return Database()


class _AcquireCM:
    """Stand-in for the async context manager returned by ``pool.acquire()``."""

    def __init__(self, conn):
        self._conn = conn

    async def __aenter__(self):
        return self._conn

    async def __aexit__(self, *exc_info) -> bool:
        return False


class _TransactionCM:
    """Stand-in for the async context manager returned by ``conn.transaction()``."""

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc_info) -> bool:
        return False


@pytest.fixture
def mock_pool(db: Database):
    """Return a mocked asyncpg pool and attach it to ``db``.

    Mirrors real asyncpg semantics: ``pool.acquire()`` and
    ``conn.transaction()`` are *synchronous* calls that return async
    context managers, while statement methods (execute/fetchrow) are
    awaitable ``AsyncMock``s.
    """
    conn = AsyncMock()
    conn.transaction = MagicMock(return_value=_TransactionCM())

    acquire_cm = _AcquireCM(conn)
    pool = MagicMock()
    pool.acquire = MagicMock(return_value=acquire_cm)

    db.pool = pool
    return pool, conn


# ---------------------------------------------------------------------------
# register_robot
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_register_robot_inserts(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.register_robot("husky1", current_node="aisle_A_n0")

    conn.execute.assert_awaited_once()
    args = conn.execute.await_args.args
    assert args[0].strip().upper().startswith("INSERT INTO ROBOTS")
    assert args[1] == "husky1"
    assert args[2] == "aisle_A_n0"

@pytest.mark.asyncio
async def test_register_robot_upserts_on_conflict(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.register_robot("husky1", current_node="box_A_001")
    await db.register_robot("husky1", current_node="aisle_B_n5")

    assert conn.execute.await_count == 2
    second_args = conn.execute.await_args_list[1].args
    assert "ON CONFLICT (id) DO UPDATE" in second_args[0]


# ---------------------------------------------------------------------------
# claim_next_task
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_claim_next_task_returns_first_claimable_task(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.fetchrow = AsyncMock(
        return_value={
            "id": "task_001",
            "mission_id": "mission_001",
            "sequence_order": 1,
            "primitive_type": "NAVIGATE_TO",
            "target_node": "box_A_001",
        }
    )
    conn.execute = AsyncMock()

    result = await db.claim_next_task("husky1")

    assert result is not None
    assert result["id"] == "task_001"
    assert result["sequence_order"] == 1
    assert conn.execute.await_count == 1
    update_call = conn.execute.await_args
    assert "SET status = 'CLAIMED'" in update_call.args[0]
    assert update_call.args[1] == "husky1"
    assert update_call.args[2] == "task_001"


@pytest.mark.asyncio
async def test_claim_next_task_returns_none_when_no_claimable_tasks(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.fetchrow = AsyncMock(return_value=None)

    result = await db.claim_next_task("husky1")

    assert result is None
    update_calls = [c for c in conn.execute.await_args_list if "UPDATE" in str(c.args[0])]
    assert len(update_calls) == 0


@pytest.mark.asyncio
async def test_claim_next_task_requires_predecessor_completed(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.fetchrow = AsyncMock(return_value=None)

    await db.claim_next_task("husky1")

    query = conn.fetchrow.await_args.args[0]
    assert "sequence_order - 1" in query
    assert "COMPLETED" in query
    assert "FOR UPDATE SKIP LOCKED" in query


@pytest.mark.asyncio
async def test_claim_next_task_binds_robot_for_mission_affinity(db: Database, mock_pool):
    """The affinity filter binds the claiming robot as $1: a robot may only
    take step N when it also executed step N-1 of that mission."""
    _pool, conn = mock_pool
    conn.fetchrow = AsyncMock(return_value=None)

    await db.claim_next_task("husky2")

    query, robot_id = conn.fetchrow.await_args.args
    assert "t2.assigned_robot_id = $1" in query
    assert robot_id == "husky2"


@pytest.mark.asyncio
async def test_claim_next_task_orders_fifo_oldest_mission_first(db: Database, mock_pool):
    """Tasks dispatch FIFO by created_at (oldest mission first), not by id."""
    _pool, conn = mock_pool
    conn.fetchrow = AsyncMock(return_value=None)

    await db.claim_next_task("husky1")

    query = conn.fetchrow.await_args.args[0]
    assert "ORDER BY t.created_at ASC, t.sequence_order ASC" in query
    assert "ORDER BY t.mission_id" not in query


@pytest.mark.asyncio
async def test_claim_next_task_skips_assigned_tasks(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.fetchrow = AsyncMock(return_value=None)

    await db.claim_next_task("husky2")

    query = conn.fetchrow.await_args.args[0]
    assert "assigned_robot_id IS NULL" in query


@pytest.mark.asyncio
async def test_claim_next_task_concurrent_skip_locked(db: Database, mock_pool):
    _pool, conn = mock_pool
    rows = [
        {
            "id": "task_001",
            "mission_id": "mission_001",
            "sequence_order": 1,
            "primitive_type": "NAVIGATE_TO",
            "target_node": "box_A_001",
        },
        {
            "id": "task_002",
            "mission_id": "mission_001",
            "sequence_order": 2,
            "primitive_type": "WAIT_FOR_LOAD",
            "target_node": "rack_A2",
        },
    ]
    conn.fetchrow = AsyncMock(side_effect=rows)
    conn.execute = AsyncMock()

    result_a = await db.claim_next_task("husky1")
    result_b = await db.claim_next_task("husky2")

    assert result_a is not None
    assert result_b is not None
    assert result_a["id"] != result_b["id"]


# ---------------------------------------------------------------------------
# update_task_status
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_update_task_status_completed_enforces_affinity(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.update_task_status("task_001", "COMPLETED", robot_id="husky1")

    call = conn.execute.await_args
    assert "SET status = $1" in call.args[0]
    assert "assigned_robot_id = $3" in call.args[0]
    assert call.args[1] == "COMPLETED"
    assert call.args[2] == "task_001"
    assert call.args[3] == "husky1"


@pytest.mark.asyncio
async def test_update_task_status_pending_to_claimed(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.update_task_status("task_001", "CLAIMED")

    call = conn.execute.await_args
    assert call.args[1] == "CLAIMED"
    assert call.args[2] == "task_001"


@pytest.mark.asyncio
async def test_update_task_status_to_pending_releases_robot(db: Database, mock_pool):
    """Resetting a task to PENDING must clear its robot pin (claim bug)."""
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.update_task_status("task_001", "PENDING")

    call = conn.execute.await_args
    assert "assigned_robot_id = NULL" in call.args[0]
    assert "claimed_at = NULL" in call.args[0]
    assert call.args[1] == "PENDING"
    assert call.args[2] == "task_001"


@pytest.mark.asyncio
async def test_unclaim_task_resets_to_pending_and_releases_robot(db: Database, mock_pool):
    """unclaim_task() makes a claimed task dispatchable again by anyone."""
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.unclaim_task("task_001")

    call = conn.execute.await_args
    assert "assigned_robot_id = NULL" in call.args[0]
    assert call.args[1] == "PENDING"
    assert call.args[2] == "task_001"


# ---------------------------------------------------------------------------
# update_robot_telemetry
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_update_robot_telemetry_updates_position(db: Database, mock_pool):
    _pool, conn = mock_pool
    conn.execute = AsyncMock()

    await db.update_robot_telemetry("husky1", x=10.5, y=20.3, yaw=1.57)

    call = conn.execute.await_args
    assert "SET x = $2, y = $3, yaw = $4" in call.args[0]
    assert call.args[1] == "husky1"
    assert call.args[2] == 10.5
    assert call.args[3] == 20.3
    assert call.args[4] == 1.57


# ---------------------------------------------------------------------------
# Pool connection errors
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_operations_raise_without_pool(db: Database):
    db.pool = None

    with pytest.raises(RuntimeError, match="Database pool is not connected"):
        await db.register_robot("husky1")

    with pytest.raises(RuntimeError, match="Database pool is not connected"):
        await db.claim_next_task("husky1")

    with pytest.raises(RuntimeError, match="Database pool is not connected"):
        await db.update_task_status("task_001", "COMPLETED", robot_id="husky1")

    with pytest.raises(RuntimeError, match="Database pool is not connected"):
        await db.update_robot_telemetry("husky1", 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Integration tests against a live PostgreSQL instance.
# These exercise the real FOR UPDATE SKIP LOCKED race semantics, prerequisite
# gating, and mission-to-robot affinity. They skip automatically when no
# database is reachable (e.g. ``docker compose up -d postgres`` not running).
# ---------------------------------------------------------------------------

SCHEMA_PATH = Path(__file__).resolve().parent.parent / "sql" / "schema.sql"


def _jsonb_codec_init(conn: asyncpg.Connection):
    """Decode JSONB (task params) as dicts on every pooled connection."""
    return conn.set_type_codec(
        "jsonb", encoder=json.dumps, decoder=json.loads, schema="pg_catalog"
    )


@pytest_asyncio.fixture
async def real_db() -> asyncpg.Pool:
    """Yield a pool against a live PostgreSQL with a fresh, empty schema."""
    try:
        pool = await asyncpg.create_pool(
            dsn=settings.DATABASE_URL, min_size=2, max_size=10, init=_jsonb_codec_init
        )
    except (OSError, asyncpg.PostgresError) as exc:
        pytest.skip(f"PostgreSQL not reachable: {exc}")

    schema_sql = SCHEMA_PATH.read_text()
    statements = [s.strip() for s in schema_sql.split(";") if s.strip()]
    async with pool.acquire() as conn:
        for statement in statements:
            await conn.execute(statement)
        await conn.execute("TRUNCATE robots, missions, tasks RESTART IDENTITY CASCADE")
    yield pool
    await pool.close()


def _make_db(pool: asyncpg.Pool) -> Database:
    db = Database()
    db.pool = pool
    return db


async def _seed_mission(
    pool: asyncpg.Pool,
    mission_id: str,
    task_ids: list[str],
    primitives: list[str],
    created_at: datetime | None = None,
    task_params: list[dict] | None = None,
) -> None:
    """Seed one mission with ``len(task_ids)`` tasks.

    ``created_at`` pins an explicit timestamp (FIFO tests); ``task_params``
    supplies per-task JSONB parameters (default ``{}``).
    """
    async with pool.acquire() as conn:
        await conn.execute(
            "INSERT INTO missions (id, text, created_at) VALUES ($1, $2, COALESCE($3, NOW()))",
            mission_id,
            f"mission {mission_id}",
            created_at,
        )
        rows = []
        for seq, (task_id, primitive) in enumerate(zip(task_ids, primitives), start=1):
            params = task_params[seq - 1] if task_params else {}
            rows.append(
                (task_id, mission_id, seq, primitive, "box_a_001", params, created_at)
            )
        await conn.executemany(
            """
            INSERT INTO tasks (id, mission_id, sequence_order, primitive_type,
                               target_node, params, created_at)
            VALUES ($1, $2, $3, $4, $5, $6, COALESCE($7, NOW()))
            """,
            rows,
        )


@pytest.mark.asyncio
async def test_integration_race_single_claimable_task(real_db: asyncpg.Pool):
    """8 concurrent claimers race for one claimable task; exactly one wins."""
    pool = real_db
    async with pool.acquire() as conn:
        await conn.executemany(
            "INSERT INTO robots (id) VALUES ($1)",
            [(f"husky{i}",) for i in range(1, 9)],
        )
    await _seed_mission(pool, "m1", ["t1", "t2", "t3"],
                        ["NAVIGATE_TO", "WAIT_FOR_LOAD", "NAVIGATE_TO"])

    async def claim(robot_id: str):
        return await _make_db(pool).claim_next_task(robot_id)

    results = await asyncio.gather(*(claim(f"husky{i}") for i in range(1, 9)))

    claimed = [r for r in results if r is not None]
    assert len(claimed) == 1, f"Expected exactly 1 winner, got {claimed}"
    assert claimed[0]["id"] == "t1"

    async with pool.acquire() as conn:
        winner = await conn.fetchrow(
            "SELECT status, assigned_robot_id FROM tasks WHERE id = 't1'"
        )
        assert winner["status"] == "CLAIMED"
        assert winner["assigned_robot_id"] in {f"husky{i}" for i in range(1, 9)}
        gated = await conn.fetch(
            "SELECT id, status, assigned_robot_id FROM tasks WHERE id <> 't1'"
        )
        assert all(
            r["status"] == "PENDING" and r["assigned_robot_id"] is None for r in gated
        )


@pytest.mark.asyncio
async def test_integration_concurrent_claims_distribute_distinct_tasks(real_db: asyncpg.Pool):
    """Two robots racing on two open step-1 tasks each get a distinct task."""
    pool = real_db
    async with pool.acquire() as conn:
        await conn.executemany(
            "INSERT INTO robots (id) VALUES ($1)", [("husky1",), ("husky2",)]
        )
    await _seed_mission(pool, "m1", ["m1_t1"], ["NAVIGATE_TO"])
    await _seed_mission(pool, "m2", ["m2_t1"], ["NAVIGATE_TO"])

    results = await asyncio.gather(
        _make_db(pool).claim_next_task("husky1"),
        _make_db(pool).claim_next_task("husky2"),
    )

    claimed = [r for r in results if r is not None]
    assert len(claimed) == 2
    assert {r["id"] for r in claimed} == {"m1_t1", "m2_t1"}
    assert len({r["id"] for r in claimed}) == 2


@pytest.mark.asyncio
async def test_integration_gating_and_mission_affinity(real_db: asyncpg.Pool):
    """End-to-end: gating blocks step 2 until step 1 COMPLETED, and only the
    robot that executed step 1 may claim step 2."""
    pool = real_db
    async with pool.acquire() as conn:
        await conn.executemany(
            "INSERT INTO robots (id) VALUES ($1)", [("husky1",), ("husky2",)]
        )
    await _seed_mission(pool, "m1", ["t1", "t2", "t3"],
                        ["NAVIGATE_TO", "WAIT_FOR_LOAD", "NAVIGATE_TO"])
    db = _make_db(pool)

    # Step 1: any robot may claim the first primitive
    t1 = await db.claim_next_task("husky1")
    assert t1 is not None and t1["id"] == "t1"

    # Gating: step 2 is unclaimable until step 1 is COMPLETED (for anyone)
    assert await db.claim_next_task("husky1") is None
    assert await db.claim_next_task("husky2") is None

    # Session affinity: a foreign robot cannot complete husky1's task
    await db.update_task_status("t1", "COMPLETED", robot_id="husky2")
    async with pool.acquire() as conn:
        row = await conn.fetchrow("SELECT status FROM tasks WHERE id = 't1'")
        assert row["status"] == "CLAIMED", "Foreign robot must not complete task"

    # The owning robot completes step 1
    await db.update_task_status("t1", "COMPLETED", robot_id="husky1")

    # Affinity: predecessor COMPLETED but by husky1, so husky2 is barred
    assert await db.claim_next_task("husky2") is None

    # The mission's executing robot claims step 2, then step 3
    t2 = await db.claim_next_task("husky1")
    assert t2 is not None and t2["id"] == "t2"
    await db.update_task_status("t2", "COMPLETED", robot_id="husky1")

    t3 = await db.claim_next_task("husky1")
    assert t3 is not None and t3["id"] == "t3"


@pytest.mark.asyncio
async def test_integration_register_and_telemetry(real_db: asyncpg.Pool):
    """register_robot upserts and update_robot_telemetry persists coordinates."""
    pool = real_db
    db = _make_db(pool)

    await db.register_robot("husky1", current_node="junction_A_1_1")
    await db.register_robot("husky1", current_node="junction_A_1_2")  # upsert
    await db.update_robot_telemetry("husky1", 1.0, 2.0, 3.0)

    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            "SELECT x, y, yaw, current_node FROM robots WHERE id = 'husky1'"
        )
        assert (row["x"], row["y"], row["yaw"]) == (1.0, 2.0, 3.0)
        assert row["current_node"] == "junction_A_1_2"


@pytest.mark.asyncio
async def test_integration_fifo_dispatch_oldest_mission_first(real_db: asyncpg.Pool):
    """Older missions (earlier created_at) dispatch first, regardless of id."""
    pool = real_db
    async with pool.acquire() as conn:
        await conn.execute("INSERT INTO robots (id) VALUES ('husky1')")
    # Seed order: m1 first, but m2 carries the older created_at.
    await _seed_mission(
        pool, "m1", ["m1_t1"], ["NAVIGATE_TO"],
        created_at=datetime(2026, 1, 1, 11, 0, tzinfo=timezone.utc),
    )
    await _seed_mission(
        pool, "m2", ["m2_t1"], ["NAVIGATE_TO"],
        created_at=datetime(2026, 1, 1, 10, 0, tzinfo=timezone.utc),
    )

    first = await _make_db(pool).claim_next_task("husky1")

    assert first is not None
    assert first["id"] == "m2_t1", "Older mission must dispatch first (FIFO)"


@pytest.mark.asyncio
async def test_integration_claim_unclaim_reclaim(real_db: asyncpg.Pool):
    """A task can be claimed, unclaimed (PENDING + NULL pin), and re-claimed."""
    pool = real_db
    async with pool.acquire() as conn:
        await conn.executemany(
            "INSERT INTO robots (id) VALUES ($1)", [("husky1",), ("husky2",)]
        )
    await _seed_mission(pool, "m1", ["t1", "t2"], ["NAVIGATE_TO", "WAIT_FOR_LOAD"])
    db = _make_db(pool)

    # Claim step 1
    claimed = await db.claim_next_task("husky1")
    assert claimed is not None and claimed["id"] == "t1"
    # Gating still blocks step 2 while step 1 is only CLAIMED
    assert await db.claim_next_task("husky2") is None

    # Unclaim: task resets to PENDING with no robot pin
    await db.unclaim_task("t1")
    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            "SELECT status, assigned_robot_id, claimed_at FROM tasks WHERE id = 't1'"
        )
        assert row["status"] == "PENDING"
        assert row["assigned_robot_id"] is None
        assert row["claimed_at"] is None

    # A different robot can claim the released task; affinity re-pins to it
    reclaimed = await db.claim_next_task("husky2")
    assert reclaimed is not None and reclaimed["id"] == "t1"
    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            "SELECT status, assigned_robot_id FROM tasks WHERE id = 't1'"
        )
        assert row["status"] == "CLAIMED"
        assert row["assigned_robot_id"] == "husky2"


@pytest.mark.asyncio
async def test_integration_task_params_round_trip(real_db: asyncpg.Pool):
    """JSONB params persist and are returned with the claimed task."""
    pool = real_db
    async with pool.acquire() as conn:
        await conn.execute("INSERT INTO robots (id) VALUES ('husky1')")
    await _seed_mission(
        pool, "m1", ["t1", "t2"], ["NAVIGATE_TO", "WAIT_FOR_LOAD"],
        task_params=[{"payload_kg": 12.5, "bay": "A2"}, {}],
    )
    db = _make_db(pool)

    claimed = await db.claim_next_task("husky1")
    assert claimed is not None and claimed["id"] == "t1"
    assert claimed["params"] == {"payload_kg": 12.5, "bay": "A2"}

    await db.update_task_status("t1", "COMPLETED", robot_id="husky1")
    claimed2 = await db.claim_next_task("husky1")
    assert claimed2 is not None and claimed2["id"] == "t2"
    assert claimed2["params"] == {}

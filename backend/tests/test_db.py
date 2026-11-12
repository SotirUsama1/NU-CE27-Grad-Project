"""Concurrency unit tests for the CyberScape database layer.

Phase 3: Validates sequential task claiming, prerequisite gating, robot
session affinity, and SKIP LOCKED behaviour under simulated race conditions.

These tests mock ``asyncpg`` so they run without a live PostgreSQL instance.
"""

from __future__ import annotations

from unittest.mock import AsyncMock

import pytest

from cyberscape.db import Database

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def db() -> Database:
    return Database()


@pytest.fixture
def mock_pool(db: Database):
    """Return a mocked asyncpg pool and attach it to ``db``."""
    pool = AsyncMock()
    conn = AsyncMock()
    cm = AsyncMock()
    cm.__aenter__ = AsyncMock(return_value=conn)
    cm.__aexit__ = AsyncMock(return_value=False)
    pool.acquire.return_value = cm
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
    update_call = conn.execute.await_args_list[1]
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

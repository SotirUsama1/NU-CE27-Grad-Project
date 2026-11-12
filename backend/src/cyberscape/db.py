"""Async database layer for the CyberScape backend.

Phase 3: Provides an asyncpg connection pool and task-claiming operations
with PostgreSQL ``SELECT FOR UPDATE SKIP LOCKED`` concurrency control.

Operations:
  * register_robot()        - Upsert a robot into the fleet table.
  * claim_next_task(robot_id) - Atomically claim the next claimable task,
      enforcing prerequisite gating (sequence_order - 1 = COMPLETED) and
      robot session affinity (assigned_robot_id).
  * update_task_status()    - Transition a task to a new status.
  * update_robot_telemetry() - Record robot position / telemetry.
"""

from __future__ import annotations

import json
from typing import Any

import asyncpg

from cyberscape.config import settings


class Database:
    """Thin asyncpg wrapper for the CyberScape orchestrator."""

    def __init__(self) -> None:
        self.pool: asyncpg.Pool | None = None

    async def connect(self) -> None:
        """Open the connection pool."""

        async def _init(conn: asyncpg.Connection) -> None:
            # Decode JSONB columns (task params) as dicts, not raw strings.
            await conn.set_type_codec(
                "jsonb", encoder=json.dumps, decoder=json.loads, schema="pg_catalog"
            )

        self.pool = await asyncpg.create_pool(
            dsn=settings.DATABASE_URL,
            min_size=1,
            max_size=10,
            init=_init,
        )

    async def disconnect(self) -> None:
        """Close the connection pool."""
        if self.pool is not None:
            await self.pool.close()
            self.pool = None

    async def register_robot(
        self,
        robot_id: str,
        current_node: str | None = None,
    ) -> None:
        """Insert or update a robot's fleet record."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO robots (id, status, current_node)
                VALUES ($1, 'IDLE', $2)
                ON CONFLICT (id) DO UPDATE
                    SET current_node = EXCLUDED.current_node,
                        updated_at = NOW()
                """,
                robot_id,
                current_node,
            )

    async def claim_next_task(
        self,
        robot_id: str,
    ) -> dict[str, Any] | None:
        """Atomically claim the next claimable task for ``robot_id``.

        Uses ``SELECT FOR UPDATE SKIP LOCKED`` so concurrent dispatchers
        never hand out the same task. A task is claimable when:
          * its status is ``PENDING`` and it is not yet assigned, and
          * it is the first task of its mission (sequence_order = 1), or
            the preceding task (sequence_order - 1) is ``COMPLETED`` by
            this same robot (mission-to-robot session affinity).

        Candidates are dispatched FIFO by ``created_at`` (oldest mission
        first), then by ``sequence_order``.

        On success the task is marked ``CLAIMED`` and pinned to
        ``assigned_robot_id`` (robot session affinity).
        """
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn, conn.transaction():
            row = await conn.fetchrow(
                """
                    SELECT t.id, t.mission_id, t.sequence_order,
                           t.primitive_type, t.target_node, t.params
                    FROM tasks t
                    WHERE t.status = 'PENDING'
                      AND t.assigned_robot_id IS NULL
                      AND (
                          t.sequence_order = 1
                          OR EXISTS (
                              SELECT 1 FROM tasks t2
                              WHERE t2.mission_id = t.mission_id
                                AND t2.sequence_order = t.sequence_order - 1
                                AND t2.status = 'COMPLETED'
                                AND t2.assigned_robot_id = $1
                          )
                      )
                    ORDER BY t.created_at ASC, t.sequence_order ASC
                    FOR UPDATE SKIP LOCKED
                    LIMIT 1
                    """,
                robot_id,
            )
            if row is None:
                return None

            await conn.execute(
                """
                    UPDATE tasks
                    SET status = 'CLAIMED',
                        assigned_robot_id = $1,
                        claimed_at = NOW(),
                        updated_at = NOW()
                    WHERE id = $2
                    """,
                robot_id,
                row["id"],
            )
            return dict(row)

    async def update_task_status(
        self,
        task_id: str,
        status: str,
        robot_id: str | None = None,
    ) -> None:
        """Transition ``task_id`` to ``status``.

        * ``COMPLETED`` enforces robot session affinity: only the claiming
          robot may complete its own task.
        * ``PENDING`` is a full reset (unclaim): the robot pin, claim stamp,
          and completion stamp are cleared, otherwise the task would remain
          invisible to the claim query (``assigned_robot_id IS NULL``) and
          become permanently unclaimable.
        """
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            if status == "COMPLETED":
                await conn.execute(
                    """
                    UPDATE tasks
                    SET status = $1,
                        completed_at = NOW(),
                        updated_at = NOW()
                    WHERE id = $2 AND assigned_robot_id = $3
                    """,
                    status,
                    task_id,
                    robot_id,
                )
            elif status == "PENDING":
                await conn.execute(
                    """
                    UPDATE tasks
                    SET status = $1,
                        assigned_robot_id = NULL,
                        claimed_at = NULL,
                        completed_at = NULL,
                        updated_at = NOW()
                    WHERE id = $2
                    """,
                    status,
                    task_id,
                )
            else:
                await conn.execute(
                    """
                    UPDATE tasks
                    SET status = $1,
                        updated_at = NOW()
                    WHERE id = $2
                    """,
                    status,
                    task_id,
                )

    async def unclaim_task(self, task_id: str) -> None:
        """Release ``task_id`` back to the dispatchable pool.

        Resets the task to ``PENDING`` and clears ``assigned_robot_id`` (and
        the claim/completion stamps) so it becomes claimable again by any
        robot. Used when segment locks cannot be acquired or on retry.
        """
        await self.update_task_status(task_id, "PENDING")

    async def update_robot_telemetry(
        self,
        robot_id: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        sim_time: float | None = None,
        current_node: str | None = None,
    ) -> None:
        """Record the latest robot position telemetry.

        ``sim_time`` is the SIMULATION timestamp (Gazebo ``/clock``) of the
        heartbeat, stored as ``sim_time_last_seen``. ``None`` fields leave the
        previously recorded value untouched (COALESCE).
        """
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE robots
                SET x = $2,
                    y = $3,
                    yaw = $4,
                    sim_time_last_seen = COALESCE($5::double precision, sim_time_last_seen),
                    current_node = COALESCE($6::text, current_node),
                    updated_at = NOW()
                WHERE id = $1
                """,
                robot_id,
                x,
                y,
                yaw,
                sim_time,
                current_node,
            )

    # ------------------------------------------------------------------
    # Fleet queries & state transitions (dispatcher / heartbeat support)
    # ------------------------------------------------------------------

    async def get_robot(self, robot_id: str) -> dict[str, Any] | None:
        """Return one robot record (or ``None`` if unknown)."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT id, status, current_node, x, y, yaw,
                       sim_time_last_seen, assigned_mission_id
                FROM robots WHERE id = $1
                """,
                robot_id,
            )
            return dict(row) if row else None

    async def list_idle_robots(self) -> list[dict[str, Any]]:
        """Return all IDLE robots with their last known topological node."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT id, current_node
                FROM robots
                WHERE status = 'IDLE'
                ORDER BY id
                """
            )
            return [dict(r) for r in rows]

    async def set_robot_status(self, robot_id: str, status: str) -> None:
        """Set a robot's fleet status (IDLE, BUSY, UNRESPONSIVE, ...)."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE robots SET status = $2, updated_at = NOW() WHERE id = $1
                """,
                robot_id,
                status,
            )

    async def get_monitored_robots(self) -> list[dict[str, Any]]:
        """Return robots subject to liveness monitoring (not QUARANTINED/MAINTENANCE)."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT id, status, current_node, sim_time_last_seen
                FROM robots
                WHERE status IN ('IDLE', 'BUSY', 'UNRESPONSIVE')
                ORDER BY id
                """
            )
            return [dict(r) for r in rows]

    async def count_claimed_tasks(self, robot_id: str) -> int:
        """Count tasks currently CLAIMED by ``robot_id``."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            return int(
                await conn.fetchval(
                    """
                    SELECT COUNT(*) FROM tasks
                    WHERE assigned_robot_id = $1 AND status = 'CLAIMED'
                    """,
                    robot_id,
                )
            )

    async def unclaim_robot_tasks(self, robot_id: str) -> int:
        """Release every CLAIMED task of ``robot_id`` back to the pool.

        Used when a robot becomes QUARANTINED: its in-flight primitives can
        no longer progress, so they are reset to PENDING (with the robot pin
        cleared) and re-dispatched to the rest of the fleet.
        """
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            result = await conn.execute(
                """
                UPDATE tasks
                SET status = 'PENDING',
                    assigned_robot_id = NULL,
                    claimed_at = NULL,
                    updated_at = NOW()
                WHERE assigned_robot_id = $1 AND status = 'CLAIMED'
                """,
                robot_id,
            )
            return int(result.split()[-1])

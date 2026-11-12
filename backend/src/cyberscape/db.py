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

from typing import Any

import asyncpg

from cyberscape.config import settings


class Database:
    """Thin asyncpg wrapper for the CyberScape orchestrator."""

    def __init__(self) -> None:
        self.pool: asyncpg.Pool | None = None

    async def connect(self) -> None:
        """Open the connection pool."""
        self.pool = await asyncpg.create_pool(
            dsn=settings.DATABASE_URL,
            min_size=1,
            max_size=10,
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
            the preceding task (sequence_order - 1) is ``COMPLETED``.

        On success the task is marked ``CLAIMED`` and pinned to
        ``assigned_robot_id`` (robot session affinity).
        """
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn, conn.transaction():
            row = await conn.fetchrow(
                """
                    SELECT t.id, t.mission_id, t.sequence_order,
                           t.primitive_type, t.target_node
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
                          )
                      )
                    ORDER BY t.mission_id, t.sequence_order
                    FOR UPDATE SKIP LOCKED
                    LIMIT 1
                    """
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

        When ``status`` is ``COMPLETED`` the robot session affinity is
        enforced: only the claiming robot may complete its own task.
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

    async def update_robot_telemetry(
        self,
        robot_id: str,
        x: float,
        y: float,
        yaw: float,
    ) -> None:
        """Record the latest robot position telemetry."""
        if self.pool is None:
            raise RuntimeError("Database pool is not connected")
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE robots
                SET x = $2, y = $3, yaw = $4, updated_at = NOW()
                WHERE id = $1
                """,
                robot_id,
                x,
                y,
                yaw,
            )

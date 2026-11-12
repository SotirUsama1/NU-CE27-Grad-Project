"""Deterministic mission sequencer node (Phase 6).

Maps a grounded intent into the strict primitive sequence

    NAVIGATE_TO -> WAIT_FOR_LOAD -> NAVIGATE_TO -> WAIT_FOR_UNLOAD ->
    REPORT_STATUS

(when the mission transports an item; plain missions skip the load/unload
pair) and persists it to PostgreSQL via ``db.py``. It also serves the
supervisory REPLAN path: it rebuilds the remaining sequence from the failed
task's position so the claim-gating prefix (preceding COMPLETED task) stays
intact, and wakes the dispatcher on ``cyberscape:mission:new``.
"""

from __future__ import annotations

import logging
from uuid import uuid4

logger = logging.getLogger(__name__)


def plan_steps(intent: dict) -> list[dict]:
    """Canonical primitive steps for one grounded intent."""
    item = intent.get("item")
    source_node = intent.get("source_node")
    destination_node = intent.get("destination_node")
    is_transport = (intent.get("mission_type") == "TRANSPORT") or bool(item)
    steps: list[dict] = []
    if source_node:
        steps.append({
            "primitive_type": "NAVIGATE_TO",
            "target_node": source_node,
            "params": {"purpose": "PICK_UP"},
        })
    if is_transport and source_node:
        steps.append({
            "primitive_type": "WAIT_FOR_LOAD",
            "target_node": source_node,
            "params": {"item": item} if item else {"cargo": "unspecified"},
        })
    steps.append({
        "primitive_type": "NAVIGATE_TO",
        "target_node": destination_node,
        "params": {"purpose": "DELIVER" if is_transport else "NAVIGATE"},
    })
    if is_transport:
        steps.append({
            "primitive_type": "WAIT_FOR_UNLOAD",
            "target_node": destination_node,
            "params": {"item": item} if item else {"cargo": "unspecified"},
        })
    steps.append({
        "primitive_type": "REPORT_STATUS",
        "target_node": destination_node,
        "params": {
            "message": (
                f"Mission complete at {intent.get('destination') or destination_node}"
            )
        },
    })
    return steps


def make_execution_node(db=None, redis_client=None):
    """Build the ``execution`` node closure over its dependencies."""

    async def wake_dispatcher(mission_id: str | None) -> None:
        if redis_client is not None and mission_id:
            # Same channel the worker subscribes to; the worker ignores the
            # echo (mission status is already PLANNED), the dispatcher wakes.
            await redis_client.publish_mission_new(mission_id)

    async def execution_node(state: dict) -> dict:
        mission_id = state.get("mission_id")
        failure = state.get("failure_context")
        if failure:
            return await _replan(state, mission_id, failure)
        return await _plan_fresh(state, mission_id)

    async def _plan_fresh(state: dict, mission_id: str | None) -> dict:
        intent = state.get("intent")
        if not intent or not intent.get("destination_node"):
            reason = state.get("error") or "mission intent is missing or ungrounded"
            if db is not None and mission_id:
                await db.update_mission_status(mission_id, "REJECTED")
            logger.warning("Mission %s not sequenced: %s", mission_id, reason)
            return {"tasks": [], "status": "REJECTED", "error": reason}
        rows: list[dict] = []
        if db is not None and mission_id:
            tasks_to_create = [
                {
                    "sequence_order": order,
                    "primitive_type": step["primitive_type"],
                    "target_node": step.get("target_node"),
                    "params": step.get("params") or {},
                }
                for order, step in enumerate(plan_steps(intent), start=1)
            ]
            if hasattr(db, "create_tasks_batch"):
                rows = await db.create_tasks_batch(
                    mission_id, tasks_to_create, mission_status="PLANNED"
                )
            elif hasattr(db, "pool") and db.pool is not None:
                async with db.pool.acquire() as conn, conn.transaction():
                    for t in tasks_to_create:
                        task_id = str(uuid4())
                        row = await conn.fetchrow(
                            """
                            INSERT INTO tasks
                                (id, mission_id, sequence_order, status, primitive_type,
                                 target_node, params)
                            VALUES ($1, $2, $3, 'PENDING', $4, $5, $6)
                            RETURNING id, mission_id, sequence_order, status, primitive_type,
                                      target_node, params
                            """,
                            task_id,
                            mission_id,
                            int(t["sequence_order"]),
                            t["primitive_type"],
                            t.get("target_node"),
                            t.get("params") or {},
                        )
                        rows.append(dict(row))
                    await conn.execute(
                        "UPDATE missions SET status = 'PLANNED', updated_at = NOW() WHERE id = $1",
                        mission_id,
                    )
            else:
                for t in tasks_to_create:
                    rows.append(await db.create_task(
                        mission_id, t["sequence_order"], t["primitive_type"],
                        t["target_node"], t["params"],
                    ))
                await db.update_mission_status(mission_id, "PLANNED")
        await wake_dispatcher(mission_id)
        logger.info("Mission %s sequenced into %d primitives", mission_id, len(rows))
        return {"tasks": rows, "status": "PLANNED"}

    async def _replan(state: dict, mission_id: str | None, failure: dict) -> dict:
        # The triage REPLANed: rebuild the remaining sequence from the failed
        # task's position. The intent comes from the state or the persisted
        # mission plan (the failure event carries no intent).
        intent = state.get("intent")
        if db is not None and mission_id and intent is None:
            intent = await db.get_mission_plan(mission_id)
        if not intent or not intent.get("destination_node") or db is None:
            reason = "replan impossible: no grounded mission plan available"
            logger.warning("Mission %s: %s", mission_id, reason)
            return {"tasks": [], "status": "REPLAN_FAILED", "error": reason}

        task_id = failure.get("task_id")
        failed = await db.get_task(task_id) if (db is not None and task_id) else None
        failed_order = int(failed["sequence_order"]) if failed else 1
        all_steps = plan_steps(intent)
        remaining = all_steps[failed_order - 1:] if failed_order <= len(all_steps) else all_steps

        if db is not None and mission_id:
            # COMPLETED tasks are kept (claim-gating prefix); the failed task
            # and the stale pending tail are replaced in their order slots.
            tasks_to_create = [
                {
                    "sequence_order": failed_order + offset,
                    "primitive_type": step["primitive_type"],
                    "target_node": step.get("target_node"),
                    "params": step.get("params") or {},
                }
                for offset, step in enumerate(remaining)
            ]
            if hasattr(db, "create_tasks_batch"):
                rows = await db.create_tasks_batch(
                    mission_id,
                    tasks_to_create,
                    mission_status="PLANNED",
                    delete_from_sequence_order=failed_order,
                )
            elif hasattr(db, "pool") and db.pool is not None:
                async with db.pool.acquire() as conn, conn.transaction():
                    await conn.execute(
                        """
                        DELETE FROM tasks
                        WHERE mission_id = $1 AND sequence_order >= $2
                          AND status <> 'COMPLETED'
                        """,
                        mission_id,
                        int(failed_order),
                    )
                    for t in tasks_to_create:
                        task_id = str(uuid4())
                        row = await conn.fetchrow(
                            """
                            INSERT INTO tasks
                                (id, mission_id, sequence_order, status, primitive_type,
                                 target_node, params)
                            VALUES ($1, $2, $3, 'PENDING', $4, $5, $6)
                            RETURNING id, mission_id, sequence_order, status, primitive_type,
                                      target_node, params
                            """,
                            task_id,
                            mission_id,
                            int(t["sequence_order"]),
                            t["primitive_type"],
                            t.get("target_node"),
                            t.get("params") or {},
                        )
                        rows.append(dict(row))
                    await conn.execute(
                        "UPDATE missions SET status = 'PLANNED', updated_at = NOW() WHERE id = $1",
                        mission_id,
                    )
            else:
                await db.delete_tasks_from(mission_id, failed_order)
                rows = []
                for t in tasks_to_create:
                    rows.append(await db.create_task(
                        mission_id, t["sequence_order"], t["primitive_type"],
                        t["target_node"], t["params"],
                    ))
                await db.update_mission_status(mission_id, "PLANNED")
        else:
            rows = []
        await wake_dispatcher(mission_id)
        logger.info(
            "Mission %s REPLANed from step %d (%d primitives, decision %s)",
            mission_id, failed_order, len(rows), state.get("decision") or "REPLAN",
        )
        return {"tasks": rows, "intent": intent, "status": "PLANNED"}

    return execution_node

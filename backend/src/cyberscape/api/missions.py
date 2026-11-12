"""Mission REST endpoints for the CyberScape orchestrator.

Phase 7 — 7.1

POST /missions
    Accept a natural-language mission string, persist a SUBMITTED record,
    and notify the Phase-6 LangGraph agent worker via Redis Pub/Sub.

GET /missions/{id}
    Return mission status and its constituent primitive tasks.
"""

from __future__ import annotations

import logging
from typing import Any

from fastapi import APIRouter, HTTPException, Request, status

from cyberscape.models import MissionRequest

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/missions", tags=["missions"])


# ---------------------------------------------------------------------------
# POST /missions
# ---------------------------------------------------------------------------


@router.post("", status_code=status.HTTP_201_CREATED)
async def submit_mission(
    body: MissionRequest,
    request: Request,
) -> dict[str, Any]:
    """Accept a natural-language mission and dispatch it to the agent worker.

    Returns the new mission record so callers can poll ``GET /missions/{id}``.
    """
    db = request.app.state.db
    redis = request.app.state.redis

    text = body.text.strip()
    if not text:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Mission text must not be empty.",
        )

    mission = await db.create_mission(text=text, status="SUBMITTED")
    logger.info("Mission %s SUBMITTED: %r", mission["id"], text[:80])

    # Notify the LangGraph agent worker; best-effort (worker has its own retry).
    try:
        await redis.publish_mission_new(mission["id"])
    except Exception:  # noqa: BLE001
        logger.warning(
            "Redis publish failed for mission %s; worker will pick it up on next sweep.",
            mission["id"],
        )

    return {
        "id": mission["id"],
        "text": mission["text"],
        "status": mission["status"],
        "created_at": mission["created_at"],
    }


# ---------------------------------------------------------------------------
# GET /missions/{mission_id}
# ---------------------------------------------------------------------------


@router.get("/{mission_id}")
async def get_mission(mission_id: str, request: Request) -> dict[str, Any]:
    """Return a mission record and all of its primitive tasks."""
    db = request.app.state.db

    mission = await db.get_mission(mission_id)
    if mission is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission '{mission_id}' not found.",
        )

    tasks = await db.list_mission_tasks(mission_id)

    return {
        "id": mission["id"],
        "text": mission["text"],
        "status": mission["status"],
        "created_at": mission["created_at"],
        "updated_at": mission["updated_at"],
        "tasks": [
            {
                "id": t["id"],
                "sequence_order": t["sequence_order"],
                "primitive_type": t["primitive_type"],
                "target_node": t["target_node"],
                "params": t["params"],
                "status": t["status"],
            }
            for t in tasks
        ],
    }

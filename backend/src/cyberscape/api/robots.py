"""Robot fleet REST endpoints for the CyberScape orchestrator.

Phase 7 — 7.2

GET  /robots              Fleet status listing.
POST /robots              Register a new robot at an initial topological node.
POST /robots/{id}/resume  Re-commission a MAINTENANCE robot back to IDLE.
"""

from __future__ import annotations

import logging
from typing import Any

from fastapi import APIRouter, HTTPException, Request, status
from pydantic import BaseModel

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/robots", tags=["robots"])


# ---------------------------------------------------------------------------
# Request schemas
# ---------------------------------------------------------------------------


class RegisterRobotRequest(BaseModel):
    """Body for POST /robots."""

    robot_id: str
    current_node: str | None = None


# ---------------------------------------------------------------------------
# GET /robots
# ---------------------------------------------------------------------------


@router.get("")
async def list_robots(request: Request) -> list[dict[str, Any]]:
    """Return the full fleet status listing."""
    db = request.app.state.db
    robots = await db.list_all_robots()
    return [
        {
            "id": r["id"],
            "status": r["status"],
            "current_node": r["current_node"],
            "x": r.get("x"),
            "y": r.get("y"),
            "yaw": r.get("yaw"),
            "assigned_mission_id": r.get("assigned_mission_id"),
            "updated_at": r.get("updated_at"),
        }
        for r in robots
    ]


# ---------------------------------------------------------------------------
# POST /robots
# ---------------------------------------------------------------------------


@router.post("", status_code=status.HTTP_201_CREATED)
async def register_robot(
    body: RegisterRobotRequest,
    request: Request,
) -> dict[str, Any]:
    """Register (or upsert) a robot into the fleet at an initial node."""
    db = request.app.state.db

    robot_id = body.robot_id.strip()
    if not robot_id:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="robot_id must not be empty.",
        )

    await db.register_robot(robot_id, current_node=body.current_node)
    logger.info("Robot %s registered at node=%s", robot_id, body.current_node)

    robot = await db.get_robot(robot_id)
    return {
        "id": robot["id"],
        "status": robot["status"],
        "current_node": robot["current_node"],
    }


# ---------------------------------------------------------------------------
# POST /robots/{robot_id}/resume
# ---------------------------------------------------------------------------


@router.post("/{robot_id}/resume")
async def resume_robot(robot_id: str, request: Request) -> dict[str, Any]:
    """Re-commission a MAINTENANCE robot back to IDLE.

    Only a robot in ``MAINTENANCE`` status may be resumed; any other status
    is an error so callers cannot accidentally reset an actively busy robot.
    """
    db = request.app.state.db
    redis = request.app.state.redis

    robot = await db.get_robot(robot_id)
    if robot is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot '{robot_id}' not found.",
        )

    if robot["status"] != "MAINTENANCE":
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=(
                f"Robot '{robot_id}' is in status '{robot['status']}', "
                "not MAINTENANCE. Only MAINTENANCE robots may be resumed."
            ),
        )

    await db.set_robot_status(robot_id, "IDLE")
    logger.info("Robot %s transitioned MAINTENANCE -> IDLE (resume)", robot_id)

    # Trigger the dispatcher so the now-IDLE robot can be assigned work
    # immediately rather than waiting for the next sweep.
    try:
        await redis.publish_robot_idle(robot_id)
    except Exception:  # noqa: BLE001
        logger.warning("Failed to notify dispatcher of %s IDLE; sweep will catch it.", robot_id)

    return {"id": robot_id, "status": "IDLE"}

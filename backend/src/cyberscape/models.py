"""Database models and Pydantic schemas for the CyberScape backend.

Phase 3: Defines enums and request/response schemas for robots, missions,
tasks, and telemetry. All enums are string-backed for safe PostgreSQL
storage and JSON serialization.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Any
from uuid import uuid4

from pydantic import BaseModel, Field

# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class RobotStatus(str, Enum):
    IDLE = "IDLE"
    BUSY = "BUSY"
    UNRESPONSIVE = "UNRESPONSIVE"
    QUARANTINED = "QUARANTINED"
    MAINTENANCE = "MAINTENANCE"


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    CLAIMED = "CLAIMED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class PrimitiveType(str, Enum):
    NAVIGATE_TO = "NAVIGATE_TO"
    WAIT_FOR_LOAD = "WAIT_FOR_LOAD"
    WAIT_FOR_UNLOAD = "WAIT_FOR_UNLOAD"
    REPORT_STATUS = "REPORT_STATUS"


# ---------------------------------------------------------------------------
# Pydantic schemas
# ---------------------------------------------------------------------------


class Robot(BaseModel):
    id: str
    status: RobotStatus = RobotStatus.IDLE
    current_node: str | None = None
    x: float | None = None
    y: float | None = None
    yaw: float | None = None
    assigned_mission_id: str | None = None
    updated_at: datetime | None = None
    created_at: datetime | None = None


class Mission(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid4()))
    text: str
    status: str = "SUBMITTED"
    created_at: datetime | None = None
    updated_at: datetime | None = None


class Task(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid4()))
    mission_id: str
    sequence_order: int
    status: TaskStatus = TaskStatus.PENDING
    primitive_type: PrimitiveType
    target_node: str | None = None
    params: dict[str, Any] = Field(default_factory=dict)
    assigned_robot_id: str | None = None
    claimed_at: datetime | None = None
    completed_at: datetime | None = None
    created_at: datetime | None = None
    updated_at: datetime | None = None


class MissionRequest(BaseModel):
    text: str


class OverrideRequest(BaseModel):
    node: str
    admin_token: str


class TelemetryPayload(BaseModel):
    robot_id: str
    x: float
    y: float
    yaw: float
    sim_time: str | None = None
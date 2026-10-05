from typing import Dict, List, Optional

from pydantic import BaseModel


class SceneSummary(BaseModel):
    total_objects: int
    categories: Dict[str, int]
    frame_id: str
    status: str


class LocatedEntity(BaseModel):
    id: str
    type: str
    loc: List[float]
    yaw_deg: float
    size: List[float]
    frame_id: str


class HealthStatus(BaseModel):
    status: str
    ros_connected: bool
    last_telemetry_age_sec: Optional[float]

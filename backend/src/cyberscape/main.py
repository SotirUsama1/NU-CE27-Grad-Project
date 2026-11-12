"""CyberScape FastAPI orchestrator application entry point.

Phase 7: Full application lifecycle.
  * Initializes database connection pool (Database) and Redis client.
  * Loads the warehouse topological graph (TopoGraph).
  * Starts simulation-time heartbeat monitor and event-driven dispatcher background tasks.
  * Optionally connects to rosbridge WebSocket (best effort for headless/test environments).
  * Enforces API-key authentication middleware on protected endpoints.
  * Mounts missions, robots, and override API routers.
"""

from __future__ import annotations

import contextlib
import logging
from collections.abc import AsyncIterator

from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse

from cyberscape.api import missions_router, override_router, robots_router
from cyberscape.config import settings
from cyberscape.db import Database
from cyberscape.dispatcher import Dispatcher
from cyberscape.heartbeat import HeartbeatMonitor
from cyberscape.redis_client import RedisClient
from cyberscape.rosbridge_client import RosBridgeClient
from cyberscape.topo_graph import TopoGraph

logger = logging.getLogger(__name__)

# Paths exempt from X-API-Key header authentication:
# - /health: liveness probe for Docker / Kubernetes / load balancers
# - /docs, /redoc, /openapi.json: interactive OpenAPI documentation
# - /override: emergency terminal endpoint authenticating via body.admin_token
UNAUTHENTICATED_PATHS = {
    "/health",
    "/docs",
    "/redoc",
    "/openapi.json",
    "/override",
}


@contextlib.asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Application lifespan: start up and shut down background services."""
    logger.info("Starting CyberScape orchestrator services...")

    # 1. Database connection pool
    db = Database()
    try:
        await db.connect()
        logger.info("Connected to PostgreSQL pool (%s)", settings.DATABASE_URL)
    except Exception as exc:  # noqa: BLE001
        logger.warning("PostgreSQL connection deferred (%s)", exc)
    app.state.db = db

    # 2. Redis client
    redis_client = RedisClient(settings.REDIS_URL)
    try:
        await redis_client.connect()
        logger.info("Connected to Redis (%s)", settings.REDIS_URL)
    except Exception as exc:  # noqa: BLE001
        logger.warning("Redis connection deferred (%s)", exc)
    app.state.redis = redis_client

    # 3. Topological Graph
    topo_graph = TopoGraph(settings.TOPO_MAP_PATH)
    try:
        topo_graph.ensure_loaded()
        logger.info("Loaded TopoGraph from %s (%d nodes)", settings.TOPO_MAP_PATH, len(topo_graph.nodes))
    except Exception as exc:  # noqa: BLE001
        logger.warning("TopoGraph loading deferred or partial (%s)", exc)
    app.state.topo_graph = topo_graph

    # 4. Heartbeat Monitor background task
    heartbeat_monitor = HeartbeatMonitor(
        db=db,
        redis_client=redis_client,
        topo_graph=topo_graph,
    )
    if db.pool is not None:
        await heartbeat_monitor.start()
        logger.info("HeartbeatMonitor background task started")
    app.state.heartbeat_monitor = heartbeat_monitor

    # 5. ROSBridge WebSocket client (best-effort; non-fatal if Gazebo/ROS is offline)
    async def _on_heartbeat(
        robot_id: str,
        sim_time: float,
        x: float | None,
        y: float | None,
        yaw: float | None,
    ) -> None:
        await heartbeat_monitor.record_heartbeat(
            robot_id, sim_time, x=x, y=y, yaw=yaw
        )

    async def _on_arrival(robot_id: str, node_id: str) -> None:
        await redis_client.publish_segment_released(node_id)

    async def _on_failure(failure_ctx: object) -> None:
        payload = failure_ctx.__dict__ if hasattr(failure_ctx, "__dict__") else dict(failure_ctx)  # type: ignore[arg-type]
        await redis_client.publish_nav2_failure(payload)

    rosbridge_client = RosBridgeClient(
        url=settings.ROSBRIDGE_WS_URL,
        topo_graph=topo_graph,
        on_heartbeat=_on_heartbeat,
        on_arrival=_on_arrival,
        on_failure=_on_failure,
        redis_client=redis_client,
    )
    try:
        await rosbridge_client.connect()
        logger.info("Connected to ROSBridge at %s", settings.ROSBRIDGE_WS_URL)
    except Exception as exc:  # noqa: BLE001
        logger.info("ROSBridge connection skipped (%s); running in headless mode", exc)
    app.state.rosbridge = rosbridge_client

    # 6. Event-driven Dispatcher background tasks (sweep + pubsub + rosbridge execution)
    dispatcher = Dispatcher(
        db=db,
        redis_client=redis_client,
        topo_graph=topo_graph,
        excluded_edges=heartbeat_monitor.excluded_edges,
        rosbridge_client=rosbridge_client if getattr(rosbridge_client, "connected", False) else None,
    )
    if db.pool is not None:
        await dispatcher.start()
        logger.info("Dispatcher background tasks started")
    app.state.dispatcher = dispatcher

    yield

    # Shutdown sequence
    logger.info("Shutting down CyberScape orchestrator services...")
    if getattr(rosbridge_client, "connected", False):
        with contextlib.suppress(Exception):
            await rosbridge_client.disconnect()

    with contextlib.suppress(Exception):
        await dispatcher.stop()

    with contextlib.suppress(Exception):
        await heartbeat_monitor.stop()

    with contextlib.suppress(Exception):
        await redis_client.disconnect()

    with contextlib.suppress(Exception):
        await db.disconnect()

    logger.info("All orchestrator services stopped.")


# ---------------------------------------------------------------------------
# FastAPI App Factory / Instance
# ---------------------------------------------------------------------------

app = FastAPI(
    title="CyberScape Orchestrator",
    description="Deterministic fast-loop orchestrator for multi-robot warehouse coordination.",
    version="0.1.0",
    lifespan=lifespan,
)


@app.middleware("http")
async def api_key_auth_middleware(request: Request, call_next):
    """Enforce X-API-Key header authentication on protected endpoints."""
    path = request.url.path.rstrip("/") or "/"
    if (
        path in UNAUTHENTICATED_PATHS
        or path.startswith(("/docs", "/redoc"))
    ):
        return await call_next(request)

    api_key = request.headers.get("x-api-key")
    if not api_key or api_key != settings.AUTH_API_KEY:
        return JSONResponse(
            status_code=status.HTTP_401_UNAUTHORIZED,
            content={"detail": "Invalid or missing API key."},
        )
    return await call_next(request)


# ---------------------------------------------------------------------------
# Route Mounting
# ---------------------------------------------------------------------------

app.include_router(missions_router)
app.include_router(robots_router)
app.include_router(override_router)


@app.get("/health", tags=["health"])
async def health() -> dict[str, str]:
    """Liveness probe used by container orchestration and load balancers."""
    return {"status": "ok"}

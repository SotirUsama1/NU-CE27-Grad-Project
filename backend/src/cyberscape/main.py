"""CyberScape FastAPI orchestrator application entry point.

Phase 1 skeleton. The full application -- database connection pools,
TopoGraph loading, API-key authentication middleware, mission/robot/override
routers, and the heartbeat + dispatcher background tasks -- is implemented
in Phase 7.
"""

from fastapi import FastAPI

app = FastAPI(
    title="CyberScape Orchestrator",
    description="Deterministic fast-loop orchestrator for multi-robot warehouse coordination.",
    version="0.1.0",
)


@app.get("/health", tags=["health"])
async def health() -> dict[str, str]:
    """Liveness probe used by container orchestration and load balancers."""
    return {"status": "ok"}
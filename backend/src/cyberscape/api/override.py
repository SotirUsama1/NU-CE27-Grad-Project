"""Safety-critical override endpoint for the CyberScape orchestrator.

Phase 7 — 7.3

POST /override
    Clear an active quarantine zone (node + its incident edges).
    Requires the ``admin_token`` field in the request body to match
    ``settings.AUTH_API_KEY``; a mismatch is a 403.

This is intentionally separate from the standard X-API-Key middleware so an
operator can clear a quarantine from an emergency terminal even without the
normal bearer credentials in a header.
"""

from __future__ import annotations

import logging
from typing import Any

from fastapi import APIRouter, HTTPException, Request, status

from cyberscape.config import settings
from cyberscape.models import OverrideRequest

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/override", tags=["override"])


@router.post("")
async def clear_quarantine(
    body: OverrideRequest,
    request: Request,
) -> dict[str, Any]:
    """Clear the quarantine on ``node`` and resume normal routing.

    Steps
    -----
    1. Validate admin_token.
    2. Look up the quarantined node's incident edge lock keys from the
       topological graph.
    3. Clear the quarantine markers in Redis (node + all incident edges).
    4. Remove the edges from the heartbeat monitor's excluded set so Dijkstra
       can route through them again immediately.
    5. Notify the dispatcher so IDLE robots can be re-scheduled right away.
    """
    # -------------------------------------------------------------------
    # 1. Auth
    # -------------------------------------------------------------------
    if body.admin_token != settings.AUTH_API_KEY:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Invalid admin token.",
        )

    node = body.node.strip()
    if not node:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="node must not be empty.",
        )

    redis = request.app.state.redis
    topo = request.app.state.topo_graph
    monitor = request.app.state.heartbeat_monitor

    # -------------------------------------------------------------------
    # 2. Confirm node is currently quarantined (best-effort check)
    # -------------------------------------------------------------------
    try:
        quarantined = await redis.is_node_quarantined(node)
    except Exception:  # noqa: BLE001
        quarantined = True  # assume it is and proceed anyway

    if not quarantined:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"Node '{node}' is not currently quarantined.",
        )

    # -------------------------------------------------------------------
    # 3. Resolve incident edge lock keys from the topological map
    # -------------------------------------------------------------------
    edge_keys: list[str] = []
    if topo is not None:
        try:
            graph = topo.ensure_loaded()
            edge_keys = [edge["key"] for edge in graph.neighbors(node)]
        except (KeyError, Exception):  # noqa: BLE001
            edge_keys = []

    # -------------------------------------------------------------------
    # 4. Clear quarantine in Redis
    # -------------------------------------------------------------------
    try:
        await redis.clear_quarantine(node, edge_keys)
    except Exception as exc:
        logger.error("Failed to clear quarantine for node %s: %s", node, exc)
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Redis unavailable; quarantine not cleared.",
        ) from exc

    logger.warning(
        "ADMIN OVERRIDE: quarantine cleared for node=%s (%d edges)", node, len(edge_keys)
    )

    # -------------------------------------------------------------------
    # 5. Sync the heartbeat monitor's excluded edges
    # -------------------------------------------------------------------
    if monitor is not None:
        monitor.excluded_edges.difference_update(edge_keys)

    # -------------------------------------------------------------------
    # 6. Trigger the dispatcher so routing can resume immediately
    # -------------------------------------------------------------------
    try:
        await redis.publish_segment_released(node)
    except Exception:  # noqa: BLE001
        logger.warning("Failed to notify dispatcher after clearing quarantine on %s", node)

    return {
        "cleared": True,
        "node": node,
        "edges_released": len(edge_keys),
    }

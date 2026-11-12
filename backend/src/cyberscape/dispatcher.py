"""Event-driven task dispatcher for the CyberScape orchestrator.

Phase 4: Claims tasks for IDLE robots, plans their route over the
topological graph, and reserves the corridor (edge locks + destination
junction capacity) atomically in Redis.

Triggers:
  * Pub/Sub messages on ``cyberscape:mission:new``      (new mission parsed),
                        ``cyberscape:robot:idle``       (robot became IDLE),
                        ``cyberscape:segment:released`` (corridor freed).
  * Fallback sweep every ``DISPATCH_SWEEP_INTERVAL_S`` (7 s).

If segment locks cannot be acquired (corridor busy / quarantined), the task
is reverted to PENDING with the robot pin cleared so another attempt (this
or a later sweep) can retry it.
"""

from __future__ import annotations

import asyncio
import logging

from cyberscape.config import settings
from cyberscape.db import Database
from cyberscape.redis_client import RedisClient

logger = logging.getLogger(__name__)

CHANNEL_MISSION_NEW = "cyberscape:mission:new"
CHANNEL_ROBOT_IDLE = "cyberscape:robot:idle"
CHANNEL_SEGMENT_RELEASED = "cyberscape:segment:released"
CHANNELS = (CHANNEL_MISSION_NEW, CHANNEL_ROBOT_IDLE, CHANNEL_SEGMENT_RELEASED)


class Dispatcher:
    """Claims and dispatches tasks to IDLE robots with segment locking."""

    def __init__(
        self,
        db: Database,
        redis_client: RedisClient,
        topo_graph=None,
        sweep_interval: float | None = None,
    ) -> None:
        self.db = db
        self.redis = redis_client
        self.topo_graph = topo_graph
        self.sweep_interval = (
            sweep_interval
            if sweep_interval is not None
            else settings.DISPATCH_SWEEP_INTERVAL_S
        )
        # Edge lock keys excluded from routing (wired to the heartbeat
        # monitor's quarantine set by the application in main.py).
        self.excluded_edges: set[str] = set()
        self.stats = {"dispatched": 0, "blocked": 0, "unroutable": 0}
        self._stopping = asyncio.Event()
        self._sweep_task: asyncio.Task | None = None
        self._pubsub_task: asyncio.Task | None = None
        self._dispatching = False

    # ------------------------------------------------------------------
    # Dispatching
    # ------------------------------------------------------------------

    async def dispatch_once(self) -> int:
        """One dispatch pass over all IDLE robots.

        Returns the number of robots that were dispatched (locked a
        segment and set BUSY). Re-entrant calls are ignored.
        """
        if self._dispatching:
            return 0
        self._dispatching = True
        try:
            return await self._dispatch_inner()
        finally:
            self._dispatching = False

    async def _dispatch_inner(self) -> int:
        robots = await self.db.list_idle_robots()
        dispatched = 0
        for robot in robots:
            task = await self.db.claim_next_task(robot["id"])
            if task is None:
                continue

            route = self._plan_segment(robot.get("current_node"), task.get("target_node"))
            if route is None:
                # No routable segment: release the claim so the sweep can
                # retry later (possibly after quarantines are cleared).
                await self.db.unclaim_task(task["id"])
                self.stats["unroutable"] += 1
                continue

            edge_keys, junction_node, capacity = route
            acquired = await self.redis.acquire_segment_lock(
                robot["id"],
                edge_keys,
                junction_node,
                ttl=settings.SEGMENT_LOCK_TTL_S,
                junction_capacity=capacity,
            )
            if acquired:
                await self.db.set_robot_status(robot["id"], "BUSY")
                dispatched += 1
                self.stats["dispatched"] += 1
                logger.info(
                    "Dispatched %s -> task %s (%d edges, junction %s)",
                    robot["id"],
                    task["id"],
                    len(edge_keys),
                    junction_node,
                )
            else:
                # Segment locks blocked: revert to PENDING and clear the
                # robot pin so the task can be re-claimed later.
                await self.db.unclaim_task(task["id"])
                self.stats["blocked"] += 1
                logger.info(
                    "Segment locks blocked for %s on task %s; reverted to PENDING",
                    robot["id"],
                    task["id"],
                )
        return dispatched

    def _plan_segment(
        self, source: str | None, target: str | None
    ) -> tuple[list[str], str, int] | None:
        """Plan one segment: edge lock keys + destination junction capacity.

        Returns ``None`` when the segment cannot be planned (unknown or
        unreachable target, no graph available).
        """
        if target is None:
            return None

        graph = self.topo_graph.ensure_loaded() if self.topo_graph is not None else None
        capacity = 2
        if graph is not None and target in graph.nodes:
            capacity = int(graph.nodes[target].get("capacity", 2))

        if not source or source == target:
            # Already at the destination: reserve its node capacity only.
            return [], target, capacity

        if graph is None or source not in graph.nodes or target not in graph.nodes:
            return None

        try:
            edges = graph.path_edges(source, target, excluded_edges=self.excluded_edges)
        except (ValueError, KeyError):
            return None
        return [edge["key"] for edge in edges], target, capacity

    # ------------------------------------------------------------------
    # Background loops
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Start the Pub/Sub listener and the fallback sweep task."""
        self._stopping.clear()
        self._pubsub_task = asyncio.create_task(self._pubsub_loop())
        self._sweep_task = asyncio.create_task(self._sweep_loop())

    async def stop(self) -> None:
        """Stop both background tasks."""
        self._stopping.set()
        tasks = [t for t in (self._sweep_task, self._pubsub_task) if t is not None]
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        self._sweep_task = None
        self._pubsub_task = None

    async def _sweep_loop(self) -> None:
        while not self._stopping.is_set():
            try:
                await asyncio.wait_for(
                    self._stopping.wait(), timeout=self.sweep_interval
                )
                break  # stopping set
            except asyncio.TimeoutError:
                pass
            try:
                await self.dispatch_once()
            except Exception:  # pragma: no cover - keep the dispatcher alive
                logger.exception("Dispatcher sweep failed")

    async def _pubsub_loop(self) -> None:
        redis = getattr(self.redis, "redis", None)
        if redis is None:
            logger.warning("Dispatcher Pub/Sub unavailable: Redis not connected")
            return
        try:
            pubsub = redis.pubsub()
            await pubsub.subscribe(*CHANNELS)
            async for message in pubsub.listen():
                if self._stopping.is_set():
                    break
                if message.get("type") == "message":
                    try:
                        await self.dispatch_once()
                    except Exception:  # pragma: no cover
                        logger.exception("Dispatcher trigger failed")
        except asyncio.CancelledError:
            raise
        finally:
            try:
                await pubsub.aclose()
            except Exception:  # pragma: no cover - redis-py version differences
                pass

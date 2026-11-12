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
import contextlib
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
        excluded_edges: set[str] | None = None,
        rosbridge_client=None,
    ) -> None:
        self.db = db
        self.redis = redis_client
        self.topo_graph = topo_graph
        self.rosbridge_client = rosbridge_client
        self.sweep_interval = (
            sweep_interval
            if sweep_interval is not None
            else settings.DISPATCH_SWEEP_INTERVAL_S
        )
        # Edge lock keys excluded from routing (wired to the heartbeat
        # monitor's quarantine set by the application in main.py).
        self.excluded_edges: set[str] = (
            excluded_edges if excluded_edges is not None else set()
        )
        self.stats = {"dispatched": 0, "blocked": 0, "unroutable": 0}
        self._stopping = asyncio.Event()
        self._sweep_task: asyncio.Task | None = None
        self._pubsub_task: asyncio.Task | None = None
        self._dispatching = False
        self._rerun_requested = False

    # ------------------------------------------------------------------
    # Dispatching
    # ------------------------------------------------------------------

    async def dispatch_once(self) -> int:
        """One dispatch pass over all IDLE robots.

        Returns the number of robots that were dispatched (locked a
        segment and set BUSY). Coalesces concurrent calls so triggers are
        never dropped.
        """
        if self._dispatching:
            self._rerun_requested = True
            return 0
        self._dispatching = True
        dispatched_total = 0
        try:
            while True:
                self._rerun_requested = False
                dispatched = await self._dispatch_inner()
                dispatched_total += dispatched
                if not self._rerun_requested or self._stopping.is_set():
                    break
            return dispatched_total
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
            try:
                acquired = await self.redis.acquire_segment_lock(
                    robot["id"],
                    edge_keys,
                    junction_node,
                    ttl=settings.SEGMENT_LOCK_TTL_S,
                    junction_capacity=capacity,
                )
            except Exception:
                await self.db.unclaim_task(task["id"])
                logger.exception(
                    "Error acquiring segment lock for %s on task %s; reverted to PENDING",
                    robot["id"],
                    task["id"],
                )
                continue

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
                if self.rosbridge_client is not None:
                    asyncio.create_task(
                        self._execute_claimed_task(
                            robot_id=robot["id"],
                            task=task,
                            edge_keys=edge_keys,
                            junction_node=junction_node,
                            source_node=robot.get("current_node"),
                        )
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
        unreachable target, unlocalized source, no graph available).
        """
        if target is None or source is None:
            return None

        graph = self.topo_graph.ensure_loaded() if self.topo_graph is not None else None
        capacity = 2
        if graph is not None and target in graph.nodes:
            capacity = int(graph.nodes[target].get("capacity", 2))

        if source == target:
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
            except (asyncio.TimeoutError, TimeoutError):  # noqa: UP041 - distinct on py3.10
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
        finally:
            with contextlib.suppress(Exception):
                await pubsub.aclose()

    async def _execute_claimed_task(
        self,
        robot_id: str,
        task: dict,
        edge_keys: list[str],
        junction_node: str,
        source_node: str | None,
    ) -> None:
        """Execute one claimed task through rosbridge and advance the mission pipeline."""
        primitive = task.get("primitive_type")
        target_node = task.get("target_node")
        task_id = task["id"]
        mission_id = task.get("mission_id")

        try:
            if primitive == "NAVIGATE_TO" and target_node and self.rosbridge_client is not None:
                from cyberscape.rosbridge_client import SegmentLease
                lease = SegmentLease(
                    robot_id=robot_id,
                    edge_keys=edge_keys,
                    junction_node=junction_node,
                )
                result = await self.rosbridge_client.dispatch_navigate_to_node(
                    namespace=robot_id,
                    node_id=target_node,
                    segment_lease=lease,
                    source_node=source_node,
                )
                if result.success:
                    await self.db.update_task_status(task_id, "COMPLETED", robot_id=robot_id)
                    await self.db.update_robot_telemetry(robot_id, current_node=target_node)
                    logger.info("Robot %s arrived at %s; task %s COMPLETED", robot_id, target_node, task_id)
                else:
                    await self.db.update_task_status(task_id, "FAILED")
                    logger.warning(
                        "Robot %s navigation to %s FAILED (status=%s)",
                        robot_id,
                        target_node,
                        result.status,
                    )
                    failure_ctx = {
                        "robot_id": robot_id,
                        "mission_id": mission_id,
                        "task_id": task_id,
                        "blocked_node": target_node,
                        "failure_type": "nav2_failure",
                        "error_message": f"Navigation failed or aborted (status={result.status})",
                    }
                    with contextlib.suppress(Exception):
                        await self.redis.publish_nav2_failure(failure_ctx)
            elif primitive in ("WAIT_FOR_LOAD", "WAIT_FOR_UNLOAD"):
                await asyncio.sleep(2.0)
                await self.db.update_task_status(task_id, "COMPLETED", robot_id=robot_id)
                logger.info(
                    "Robot %s finished %s at %s; task %s COMPLETED",
                    robot_id,
                    primitive,
                    target_node,
                    task_id,
                )
            elif primitive == "REPORT_STATUS":
                await self.db.update_task_status(task_id, "COMPLETED", robot_id=robot_id)
                logger.info("Robot %s finished REPORT_STATUS; task %s COMPLETED", robot_id, task_id)
                if mission_id:
                    tasks = await self.db.list_mission_tasks(mission_id)
                    if all(t.get("status") == "COMPLETED" for t in tasks):
                        await self.db.update_mission_status(mission_id, "COMPLETED")
                        logger.info("Mission %s fully COMPLETED!", mission_id)
            else:
                await self.db.update_task_status(task_id, "COMPLETED", robot_id=robot_id)
        except Exception:
            logger.exception("Error executing task %s on robot %s", task_id, robot_id)
            with contextlib.suppress(Exception):
                await self.db.update_task_status(task_id, "FAILED")
        finally:
            with contextlib.suppress(Exception):
                await self.redis.release_segment_lock(robot_id, edge_keys, junction_node)
            with contextlib.suppress(Exception):
                await self.db.set_robot_status(robot_id, "IDLE")
            with contextlib.suppress(Exception):
                await self.redis.publish_robot_idle(robot_id)

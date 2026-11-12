"""Simulation-time liveness monitor for the CyberScape orchestrator.

Phase 4: Robots publish heartbeats at 1 Hz carrying the SIMULATION clock
(Gazebo ``/clock``) timestamp. Timeouts are evaluated against simulation
time - never the wall clock - so a slow real-time factor (RTF lag) cannot
produce false "robot is dead" positives.

Two-tier liveness:
  * Tier 1 (``HEARTBEAT_TIMEOUT_S`` = 3 s):  hold in ``UNRESPONSIVE``.
    Automatic self-healing on the next heartbeat.
  * Tier 2 (``HEARTBEAT_QUARANTINE_S`` = 10 s): escalate to ``QUARANTINED``,
    quarantine the robot's last known node and its incident edges in Redis,
    add those edges to the Dijkstra exclusion set, and release the robot's
    in-flight task back to the dispatch pool. Only an administrative
    override clears a Tier 2 quarantine.
"""

from __future__ import annotations

import asyncio
import logging

from cyberscape.config import settings
from cyberscape.db import Database
from cyberscape.redis_client import RedisClient

logger = logging.getLogger(__name__)

MONITOR_SWEEP_INTERVAL_S = 1.0


class HeartbeatMonitor:
    """Evaluates robot liveness using simulation time."""

    def __init__(
        self,
        db: Database,
        redis_client: RedisClient,
        topo_graph=None,
        unresponsive_timeout: float | None = None,
        quarantine_timeout: float | None = None,
    ) -> None:
        self.db = db
        self.redis = redis_client
        self.topo_graph = topo_graph
        self.unresponsive_timeout = (
            unresponsive_timeout
            if unresponsive_timeout is not None
            else settings.HEARTBEAT_TIMEOUT_S
        )
        self.quarantine_timeout = (
            quarantine_timeout
            if quarantine_timeout is not None
            else settings.HEARTBEAT_QUARANTINE_S
        )
        # Edge lock keys excluded from Dijkstra routing while quarantined.
        # Wire this set into TopoGraph.dijkstra(excluded_edges=...) in main.
        self.excluded_edges: set[str] = set()
        self._latest_sim_time: float | None = None
        self._task: asyncio.Task | None = None
        self._stopping = asyncio.Event()

    # ------------------------------------------------------------------
    # Heartbeat ingestion (called by the ROS bridge on every heartbeat)
    # ------------------------------------------------------------------

    async def record_heartbeat(
        self,
        robot_id: str,
        sim_time: float,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        current_node: str | None = None,
    ) -> None:
        """Persist a heartbeat and self-heal a Tier 1 hold.

        ``sim_time`` advances the monitor's view of the simulation clock
        (it is global to the whole fleet).
        """
        sim_time = float(sim_time)
        if self._latest_sim_time is None or sim_time > self._latest_sim_time:
            self._latest_sim_time = sim_time

        await self.db.update_robot_telemetry(
            robot_id, x=x, y=y, yaw=yaw, sim_time=sim_time, current_node=current_node
        )

        robot = await self.db.get_robot(robot_id)
        if robot is not None and robot["status"] == "UNRESPONSIVE":
            claimed = await self.db.count_claimed_tasks(robot_id)
            recovered_status = "BUSY" if claimed else "IDLE"
            await self.db.set_robot_status(robot_id, recovered_status)
            logger.info(
                "Robot %s recovered from UNRESPONSIVE (sim %.3f) -> %s",
                robot_id,
                sim_time,
                recovered_status,
            )

    # ------------------------------------------------------------------
    # Liveness evaluation
    # ------------------------------------------------------------------

    async def sweep(self, now_sim: float | None = None) -> list[str]:
        """Evaluate every monitored robot against simulation time.

        Args:
            now_sim: Current simulation time. Defaults to the latest
                timestamp seen from any robot heartbeat; when no heartbeat
                has ever been observed the sweep is a no-op (a paused
                simulation can never time robots out).

        Returns:
            Human-readable list of transitions performed this sweep.
        """
        now = now_sim if now_sim is not None else self._latest_sim_time
        if now is None:
            return []

        robots = await self.db.get_monitored_robots()
        actions: list[str] = []
        for robot in robots:
            last_seen = robot.get("sim_time_last_seen")
            if last_seen is None:
                # Robot has not joined the simulation yet - not a timeout.
                continue
            age = now - float(last_seen)

            if age > self.quarantine_timeout:
                if robot["status"] != "QUARANTINED":
                    await self._escalate_quarantine(robot["id"], robot.get("current_node"))
                    actions.append(f"{robot['id']}:QUARANTINED")
            elif age > self.unresponsive_timeout:
                if robot["status"] in ("IDLE", "BUSY"):
                    await self.db.set_robot_status(robot["id"], "UNRESPONSIVE")
                    actions.append(f"{robot['id']}:UNRESPONSIVE")
            else:
                if robot["status"] == "UNRESPONSIVE":
                    claimed = await self.db.count_claimed_tasks(robot["id"])
                    status = "BUSY" if claimed else "IDLE"
                    await self.db.set_robot_status(robot["id"], status)
                    actions.append(f"{robot['id']}:RECOVERED")
        return actions

    async def _escalate_quarantine(self, robot_id: str, node: str | None) -> None:
        """Tier 2: quarantine the robot, its node, and incident edges."""
        await self.db.set_robot_status(robot_id, "QUARANTINED")
        await self.db.unclaim_robot_tasks(robot_id)

        edge_keys: list[str] = []
        if node is not None and self.topo_graph is not None:
            try:
                graph = self.topo_graph.ensure_loaded()
                edge_keys = [edge["key"] for edge in graph.neighbors(node)]
            except KeyError:
                edge_keys = []

        if node is not None:
            await self.redis.set_quarantine(node, edge_keys)
        self.excluded_edges.update(edge_keys)
        logger.warning(
            "Robot %s QUARANTINED at node %s (%d edges excluded from routing)",
            robot_id,
            node,
            len(edge_keys),
        )

    # ------------------------------------------------------------------
    # Background loop
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Start the periodic sweep background task."""
        self._stopping.clear()
        self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        """Stop the background task."""
        self._stopping.set()
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None

    async def _run(self) -> None:
        while not self._stopping.is_set():
            try:
                await self.sweep()
            except Exception:  # pragma: no cover - keep the monitor alive
                logger.exception("Heartbeat sweep failed")
            try:
                await asyncio.wait_for(
                    self._stopping.wait(), timeout=MONITOR_SWEEP_INTERVAL_S
                )
            except (asyncio.TimeoutError, TimeoutError):  # noqa: UP041 - distinct on py3.10
                continue

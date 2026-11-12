"""Standalone CyberScape agent worker process (Phase 6).

Consumes Redis Pub/Sub events and drives the LangGraph workflow:

* ``cyberscape:mission:new``   - a new natural-language mission
  (``{mission_id}`` from the API/CLI, or ``{text}`` to create one) is
  interpreted, grounded, sequenced into primitives, persisted, and handed
  to the dispatcher.
* ``cyberscape:failure:nav2``  - a Nav2 STATUS_ABORTED (6) failure event is
  triaged (REPLAN / RETRY / ESCALATE).

The worker shares ``cyberscape:mission:new`` with the dispatcher: the
execution node sets the mission status to PLANNED *before* publishing, and
the worker skips echoes whose mission status is no longer SUBMITTED - so
the publish wakes the dispatcher without the worker re-running the graph.

Run standalone:

    python -m cyberscape.langgraph.worker
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import signal

from cyberscape.db import Database
from cyberscape.langgraph.graph import build_graph, build_llm
from cyberscape.langgraph.state import WorkflowState
from cyberscape.redis_client import (
    CHANNEL_MISSION_NEW,
    CHANNEL_NAV2_FAILURE,
    RedisClient,
)
from cyberscape.rosbridge_client import FailureContext
from cyberscape.topo_graph import TopoGraph

logger = logging.getLogger(__name__)

#: Pub/Sub channels the worker subscribes to.
CHANNELS = (CHANNEL_MISSION_NEW, CHANNEL_NAV2_FAILURE)

#: Channel name -> handler name (matched on the suffix after ``cyberscape:``).
_MISSION = "mission:new"
_FAILURE = "failure:nav2"

#: ``FailureContext.to_dict()`` keys the worker copies into ``failure_context``.
_FAILURE_FIELDS = (
    "robot_id",
    "goal_id",
    "status",
    "node_id",
    "goal_pose",
    "values",
    "reason",
)


class MissionWorker:
    """Event-driven agent worker: Pub/Sub in, LangGraph workflow out."""

    def __init__(
        self,
        db=None,
        redis_client=None,
        topo: TopoGraph | None = None,
        llm=None,
        graph=None,
        sweep_interval: float = 10.0,
    ):
        self.db = db
        self.redis = redis_client
        self.sweep_interval = sweep_interval
        self.graph = graph if graph is not None else build_graph(
            db=db, topo=topo, llm=llm, redis_client=redis_client
        )
        self._stopping = asyncio.Event()
        self._stop_event = self._stopping
        self._pubsub_task: asyncio.Task | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def run_forever(self, reconnect_interval: float = 1.0) -> None:
        """Run the Pub/Sub listener and background sweep loop concurrently."""
        pubsub_task = asyncio.create_task(
            self._pubsub_loop(reconnect_interval=reconnect_interval)
        )
        sweep_task = asyncio.create_task(self._sweep_loop())
        try:
            await asyncio.gather(pubsub_task, sweep_task)
        except asyncio.CancelledError:
            pubsub_task.cancel()
            sweep_task.cancel()
            await asyncio.gather(pubsub_task, sweep_task, return_exceptions=True)

    async def _pubsub_loop(self, reconnect_interval: float = 1.0) -> None:
        """Subscribe to worker channels and process events with automatic reconnect."""
        while not self._stopping.is_set():
            redis = getattr(self.redis, "redis", None)
            if redis is None:
                logger.warning("Worker Pub/Sub unavailable: Redis not connected; waiting...")
                try:
                    await asyncio.sleep(reconnect_interval)
                except asyncio.CancelledError:
                    break
                continue

            pubsub = None
            try:
                pubsub = redis.pubsub()
                await pubsub.subscribe(*CHANNELS)
                logger.info("Agent worker listening on %s", ", ".join(CHANNELS))
                async for message in pubsub.listen():
                    if self._stopping.is_set():
                        break
                    if message.get("type") != "message":
                        continue
                    try:
                        await self.handle_message(message)
                    except Exception:  # pragma: no cover - keep the loop alive
                        logger.exception("Worker handler failed for message %r", message)
            except asyncio.CancelledError:
                break
            except Exception as exc:  # noqa: BLE001
                if self._stopping.is_set():
                    break
                logger.warning(
                    "Worker Pub/Sub disconnected (%s); reconnecting in %.1fs...",
                    exc,
                    reconnect_interval,
                )
                try:
                    await asyncio.sleep(reconnect_interval)
                except asyncio.CancelledError:
                    break
            finally:
                if pubsub is not None:
                    with contextlib.suppress(Exception):
                        await pubsub.aclose()

    async def _sweep_loop(self) -> None:
        """Periodic background sweep for orphaned SUBMITTED missions."""
        while not self._stopping.is_set():
            try:
                await asyncio.wait_for(
                    self._stopping.wait(), timeout=self.sweep_interval
                )
                break
            except (asyncio.TimeoutError, TimeoutError):  # noqa: UP041
                pass
            if self._stopping.is_set():
                break
            try:
                await self.sweep_submitted_missions()
            except Exception:  # pragma: no cover
                logger.exception("Worker mission sweep failed")

    async def sweep_submitted_missions(self) -> list[dict]:
        """Query for any missions in SUBMITTED state and process them.

        Ensures missions whose Redis notification was dropped or failed
        are never permanently orphaned.
        """
        if self.db is None:
            return []

        missions: list[dict] = []
        if hasattr(self.db, "list_submitted_missions"):
            try:
                missions = await self.db.list_submitted_missions()
            except Exception:
                logger.exception("Failed to query submitted missions during sweep")
                return []
        elif hasattr(self.db, "pool") and self.db.pool is not None:
            try:
                async with self.db.pool.acquire() as conn:
                    rows = await conn.fetch(
                        """
                        SELECT id, text, status, plan, created_at, updated_at
                        FROM missions
                        WHERE status = 'SUBMITTED'
                        ORDER BY created_at ASC
                        """
                    )
                    missions = [dict(r) for r in rows]
            except Exception:
                logger.exception("Failed to query submitted missions from pool during sweep")
                return []

        results = []
        for m in missions:
            if self._stopping.is_set():
                break
            m_id = m.get("id")
            text = m.get("text")
            logger.info("Sweep processing orphaned SUBMITTED mission: %s", m_id)
            try:
                state: WorkflowState = {"mission_id": m_id, "text": text}
                res = await self.graph.ainvoke(state)
                results.append(res)
            except Exception:
                logger.exception("Failed to process orphaned mission %s in sweep", m_id)
        return results

    def start(self, reconnect_interval: float = 1.0) -> asyncio.Task:
        """Run the worker (Pub/Sub loop + sweep loop) as a background task."""
        self._stopping.clear()
        self._pubsub_task = asyncio.create_task(
            self.run_forever(reconnect_interval=reconnect_interval)
        )
        return self._pubsub_task

    async def stop(self) -> None:
        """Signal all worker loops to stop and wait for them."""
        self._stopping.set()
        if self._pubsub_task is not None:
            self._pubsub_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._pubsub_task
            self._pubsub_task = None

    # ------------------------------------------------------------------
    # Event routing
    # ------------------------------------------------------------------

    async def handle_message(self, message) -> dict | None:
        """Decode one Pub/Sub message and run the matching workflow.

        Returns the final workflow state (or ``None`` when skipped).
        """
        channel = message.get("channel")
        if isinstance(channel, bytes):
            channel = channel.decode()
        payload = message.get("data")
        if isinstance(payload, bytes):
            payload = payload.decode()
        try:
            data = json.loads(payload) if isinstance(payload, str) else dict(payload)
        except (TypeError, ValueError):
            logger.warning("Worker dropping non-JSON payload on %s: %r", channel, payload)
            return None

        if isinstance(channel, str) and channel.endswith(_FAILURE):
            return await self._handle_failure(data)
        if isinstance(channel, str) and channel.endswith(_MISSION):
            return await self._handle_mission(data)
        logger.warning("Worker ignoring unknown channel %r", channel)
        return None

    # ------------------------------------------------------------------
    # Handlers
    # ------------------------------------------------------------------

    async def _handle_mission(self, data: dict) -> dict | None:
        """Run interpretation -> execution for one new mission."""
        mission_id = data.get("mission_id")
        text = data.get("text")

        if mission_id:
            # Echo guard: the worker shares this channel with the
            # dispatcher; only missions still awaiting a plan are ours.
            mission = await self.db.get_mission(mission_id) if self.db else None
            if mission is None:
                logger.warning("Worker skipping unknown mission %s", mission_id)
                return None
            if mission.get("status") != "SUBMITTED":
                logger.debug(
                    "Worker skipping echo for mission %s (status %s)",
                    mission_id, mission.get("status"),
                )
                return None
            text = text or mission.get("text")
        elif text:
            if self.db is not None:
                mission = await self.db.create_mission(text)
                mission_id = mission["id"]
        else:
            logger.warning("Worker dropping mission event with neither id nor text")
            return None

        state: WorkflowState = {"mission_id": mission_id, "text": text}
        result = await self.graph.ainvoke(state)
        logger.info("Mission %s processed -> %s", mission_id, result.get("status"))
        return result

    async def _handle_failure(self, data: dict) -> dict | None:
        """Run supervisory triage for one Nav2 failure event.

        The payload carries a ``FailureContext`` (flat, from
        ``FailureContext.to_dict()``, or nested under ``failure_context``)
        plus ``task_id``, ``mission_id`` and ``attempts``.
        """
        source = data.get("failure_context") if isinstance(data, dict) else None
        if not isinstance(source, dict):
            source = data
        failure_context: dict = {
            field: source.get(field) for field in _FAILURE_FIELDS
        }
        with contextlib.suppress(TypeError, ValueError):
            failure_context["status"] = int(source.get("status"))
        for key in ("task_id", "mission_id", "attempts"):
            if source.get(key) is not None:
                failure_context[key] = source[key]

        if not any(failure_context.get(field) is not None for field in _FAILURE_FIELDS):
            logger.warning("Worker dropping failure event without FailureContext")
            return None

        state: WorkflowState = {"failure_context": failure_context}
        result = await self.graph.ainvoke(state)
        logger.info(
            "Failure on robot %s triaged -> %s",
            failure_context.get("robot_id"), result.get("decision"),
        )
        return result


def build_failure_payload(
    failure: FailureContext, task_id: str, mission_id: str, attempts: int
) -> dict:
    """Wire-format for ``cyberscape:failure:nav2`` (FailureContext + refs)."""
    return {
        **failure.to_dict(),
        "task_id": task_id,
        "mission_id": mission_id,
        "attempts": attempts,
    }


async def run() -> None:
    """Connect the stack and run the worker until cancelled."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )
    db = Database()
    redis_client = RedisClient()
    await db.connect()
    await redis_client.connect()
    topo = TopoGraph()
    llm = build_llm()
    if llm is None:
        logger.info("No OPENAI_API_KEY configured: deterministic mode (rules only)")
    worker = MissionWorker(db=db, redis_client=redis_client, topo=topo, llm=llm)
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):  # not on Windows
            loop.add_signal_handler(sig, worker._stopping.set)
    try:
        await worker.run_forever()
    finally:
        with contextlib.suppress(Exception):
            await redis_client.disconnect()
        with contextlib.suppress(Exception):
            await db.disconnect()


def main() -> None:
    """Console entry point for the standalone agent worker."""
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logger.info("Agent worker stopped")


if __name__ == "__main__":
    main()

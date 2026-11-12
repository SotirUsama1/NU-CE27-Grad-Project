"""Supervisory exception triage node (Phase 6).

Evaluates Nav2 ``STATUS_ABORTED`` (6) events and picks the recovery
decision - ``REPLAN`` (fresh sequence from the failed step), ``RETRY``
(re-queue the same task) or ``ESCALATE`` (hold the robot for a human).
An LLM (langchain-openai) refines the triage when configured; a
deterministic keyword/attempt-budget heuristic keeps the system safe and
working with no API key. Non-abort statuses are never triaged.
"""

from __future__ import annotations

import logging
from typing import Any

from cyberscape.config import settings
from cyberscape.langgraph.interpretation import extract_json
from cyberscape.langgraph.prompts import build_supervisory_messages
from cyberscape.langgraph.state import Decision
from cyberscape.rosbridge_client import GoalStatus

logger = logging.getLogger(__name__)

#: Failure signatures that point at the route/goal rather than bad luck.
_BLOCKED_SIGNALS = (
    "blocked", "obstacle", "pinned", "unreachable", "no valid",
    "collision", "infeasible", "costmap", "goal in",
)


def fallback_decision(
    failure_context: dict[str, Any], attempts: int, max_retries: int
) -> str:
    """Deterministic triage when no LLM decided (or it misbehaved)."""
    if attempts >= max_retries:
        return Decision.ESCALATE
    haystack = str(failure_context).lower()
    if any(signal in haystack for signal in _BLOCKED_SIGNALS):
        return Decision.REPLAN
    return Decision.RETRY


def make_supervisory_node(db=None, llm=None, max_retries: int | None = None):
    """Build the ``supervisory`` node closure over its dependencies."""
    budget = settings.SUPERVISOR_MAX_RETRIES if max_retries is None else max_retries

    async def supervisory_node(state: dict) -> dict:
        failure = dict(state.get("failure_context") or {})
        status = failure.get("status")
        if status == GoalStatus.STATUS_CANCELED:
            logger.info(
                "Supervisory node received STATUS_CANCELED (5) for task %s; "
                "cleaning up without escalation",
                failure.get("task_id"),
            )
            if db is not None:
                await _cleanup_canceled_goal(db, failure)
            return {
                "decision": Decision.CANCELED,
                "decision_reason": (
                    "action goal was canceled (STATUS_CANCELED=5); "
                    "releasing robot without maintenance escalation"
                ),
                "status": "CANCELED",
            }

        if status != GoalStatus.STATUS_ABORTED:
            logger.warning(
                "Supervisory node got action status %r (only Nav2 STATUS_ABORTED (6) "
                "is triaged); ending without recovery",
                status,
            )
            return {
                "decision": Decision.ESCALATE,
                "decision_reason": (
                    f"unexpected action status {status!r} (only Nav2 "
                    "STATUS_ABORTED (6) is supervisory material)"
                ),
                "status": "ESCALATED",
            }

        attempts = int(failure.get("attempts") or 0)
        decision: str | None = None
        reason = ""
        if llm is not None:
            try:
                messages = build_supervisory_messages(failure, attempts, budget)
                response = await llm.ainvoke(messages)
                raw = extract_json(response.content)
                candidate = str(raw.get("decision", "")).strip().upper()
                if candidate in Decision.ALL:
                    decision = candidate
                    reason = str(raw.get("reason", "")) or reason
                else:
                    logger.warning("LLM triage returned invalid decision %r", candidate)
            except Exception as exc:  # noqa: BLE001
                logger.warning("LLM triage failed; using heuristics: %s", exc)
        if decision is None:
            decision = fallback_decision(failure, attempts, budget)
            reason = reason or (
                f"deterministic triage (attempts {attempts}/{budget}, "
                "no LLM decision available)"
            )

        if decision == Decision.ESCALATE and db is not None:
            await _hold_robot_for_human(db, failure)
        logger.info(
            "Triage for task %s: %s (%s)", failure.get("task_id"), decision, reason
        )
        return {"decision": decision, "decision_reason": reason}

    return supervisory_node


async def _hold_robot_for_human(db, failure: dict) -> None:
    """ESCALATE side effects: robot to MAINTENANCE, mission to ESCALATED."""
    robot_id = failure.get("robot_id")
    mission_id = failure.get("mission_id")
    try:
        if robot_id:
            await db.set_robot_status(robot_id, "MAINTENANCE")
        if mission_id:
            await db.update_mission_status(mission_id, "ESCALATED")
    except Exception:  # pragma: no cover - escalation must never crash triage
        logger.exception("Escalation side effects failed for %s", robot_id)


async def _cleanup_canceled_goal(db, failure: dict) -> None:
    """Handle STATUS_CANCELED: reset robot to IDLE (never MAINTENANCE) and cancel mission/task."""
    robot_id = failure.get("robot_id")
    mission_id = failure.get("mission_id")
    task_id = failure.get("task_id")
    try:
        if robot_id:
            await db.set_robot_status(robot_id, "IDLE")
        if task_id and hasattr(db, "update_task_status"):
            await db.update_task_status(task_id, "CANCELED")
        if mission_id and hasattr(db, "update_mission_status"):
            await db.update_mission_status(mission_id, "CANCELED")
    except Exception:  # pragma: no cover - cleanup must never crash triage
        logger.exception("Cancellation cleanup failed for %s", robot_id)


def make_retry_node(db=None):
    """Build the ``retry`` node: re-queue a FAILED task unchanged."""

    async def retry_node(state: dict) -> dict:
        failure = dict(state.get("failure_context") or {})
        task_id = failure.get("task_id")
        mission_id = failure.get("mission_id")
        if db is not None and task_id:
            task = await db.get_task(task_id)
            if task is not None and task.get("status") == "FAILED":
                attempts = await db.increment_task_attempts(task_id)
                # PENDING with the robot pin cleared: claimable again by any
                # robot that completed the preceding primitive.
                await db.unclaim_task(task_id)
                if mission_id:
                    await db.update_mission_status(mission_id, "PLANNED")
                logger.info(
                    "Task %s re-queued for retry (attempt %d)", task_id, attempts
                )
            else:
                logger.debug(
                    "Task %s not in FAILED state; nothing to re-queue", task_id
                )
        return {"status": "RETRYING"}

    return retry_node

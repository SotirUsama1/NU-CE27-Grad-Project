"""Shared workflow state and vocabulary for the CyberScape agent graph.

Phase 6: one ``WorkflowState`` TypedDict flows through every node; LangGraph
merges each node's returned partial dict into it. Node names and the triage
decision vocabulary live here so graph wiring, nodes and tests all agree.
"""

from __future__ import annotations

from typing import Any, TypedDict

# Graph node names (plan 6.1: interpretation/execution/supervisory + retry).
INTERPRETATION = "interpretation"
EXECUTION = "execution"
SUPERVISORY = "supervisory"
RETRY = "retry"

# State channel written by the supervisory node (routed on by graph.py).
DECISION = "decision"


class Decision:
    """Supervisory triage outcomes (plan 6.4)."""

    REPLAN = "REPLAN"      # plan a fresh sequence from the failed step onward
    RETRY = "RETRY"        # re-queue the same task (transient failure)
    ESCALATE = "ESCALATE"  # hold the robot for a human
    CANCELED = "CANCELED"  # goal was canceled; end graph gracefully without maintenance

    ALL = (REPLAN, RETRY, ESCALATE, CANCELED)


class WorkflowState(TypedDict, total=False):
    """State channels of the agent workflow graph.

    Keys are written by the node that owns them:

    * ``mission_id`` / ``text``  - the new-mission entry (worker/API),
    * ``failure_context``        - the failure entry: a Nav2 STATUS_ABORTED
      event dict carrying the ``FailureContext`` fields (robot_id, goal_id,
      status, node_id, goal_pose, values, reason) plus ``task_id``,
      ``mission_id`` and ``attempts``,
    * ``intent``                 - the grounded mission intent (interpretation),
    * ``tasks``                  - the persisted primitive task rows (execution),
    * ``decision`` / ``decision_reason`` - the triage outcome (supervisory),
    * ``error`` / ``status``     - failure text and a workflow summary stamp.
    """

    mission_id: str
    text: str
    failure_context: dict[str, Any]
    intent: dict[str, Any]
    decision: str
    decision_reason: str
    tasks: list[dict[str, Any]]
    error: str
    status: str

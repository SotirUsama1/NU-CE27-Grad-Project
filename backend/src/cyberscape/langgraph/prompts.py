"""Prompts for the CyberScape agent worker (Phase 6, English only).

Every prompt is grounded in the real warehouse layout: the interpreter's
system prompt embeds a landmark index rendered from
``TopoGraph.location_summary()``, so the LLM can only pick locations that
actually exist on the map instead of hallucinating names or coordinates.
"""

from __future__ import annotations

import json

INTERPRETATION_SYSTEM_TEMPLATE = """You are the mission interpreter of the CyberScape warehouse fleet (Autonomous Mobile Robots).

You translate a human mission statement into a strict JSON intent. Ground EVERY location in
the warehouse landmark index below: never invent places, node ids or coordinates, and never
output coordinates - the robots can only be sent to topological nodes from the index.

Warehouse landmark index (ground truth; copy source/destination verbatim from it):
{location_index}

Respond with STRICT JSON only - no prose, no markdown fences:
{{
  "mission_type": "TRANSPORT" when an item is moved, otherwise "NAVIGATE",
  "item": "the item/box label exactly as written in the index, or null",
  "source": "the pickup location: exact label or id from the index, or null",
  "destination": "the drop-off location: exact label or id from the index (never null)",
  "notes": "any constraint the user added, or null"
}}

Rules:
- "Move box A-001 from rack A2_0 to the shipping dock" -> item="box A-001", source="rack A2_0", destination="<the dock's label from the index>".
- With no source location, set source to null (the robot only navigates to the destination).
- Copy labels EXACTLY as they appear in the index."""


SUPERVISORY_SYSTEM_TEMPLATE = """You are the supervisory exception-triage agent of the CyberScape warehouse fleet.

A Nav2 ``NavigateToPose`` goal finished with ``action_msgs/msg/GoalStatus`` STATUS_ABORTED (6).
Decide the recovery action - EXACTLY one of:

- "REPLAN": the route or goal itself is impracticable (blocked corridor, unreachable or
  in-collision goal, obstacle pinning the robot). A fresh task sequence is planned from the
  failed step onward.
- "RETRY": the failure looks transient (controller hiccup, brief stall, costmap timing) and
  the attempt budget is not exhausted. The same task is re-queued unchanged.
- "ESCALATE": the failure repeated beyond the attempt budget, or it is safety-relevant.
  The robot is held in MAINTENANCE for a human.

Respond with STRICT JSON only - no prose, no markdown fences:
{{"decision": "REPLAN" | "RETRY" | "ESCALATE", "reason": "<one sentence>"}}

Failure context:
{failure_context}

Attempts so far: {attempts} (budget: {max_retries})
"""


def render_location_summary(summary: dict[str, dict]) -> str:
    """Human-readable landmark index from ``TopoGraph.location_summary()``.

    Boxes and landmarks render as ``label (id)``; junctions as bare ids.
    Unlabeled aisles are skipped - they are internal corridors, not places
    a mission would name.
    """
    sections: list[str] = []
    for zone in sorted(summary):
        data = summary[zone]
        lines = [f"Zone {zone}:"]
        for box in data.get("boxes") or []:
            lines.append(f"  - {box['label']} (id: {box['id']})")
        for landmark in data.get("landmarks") or []:
            lines.append(
                f"  - {landmark['label']} ({landmark.get('kind', 'landmark')}; id: {landmark['id']})"
            )
        for junction in data.get("junctions") or []:
            lines.append(f"  - junction {junction['id']}")
        if len(lines) == 1:
            lines.append("  - (no named locations)")
        sections.append("\n".join(lines))
    return "\n".join(sections)


def build_interpretation_messages(text: str, summary: dict[str, dict]) -> list:
    """(system, user) message pair for the mission interpreter."""
    system = INTERPRETATION_SYSTEM_TEMPLATE.format(
        location_index=render_location_summary(summary)
    )
    return [("system", system), ("user", f"Mission: {text}")]


def build_supervisory_messages(
    failure_context: dict, attempts: int, max_retries: int
) -> list:
    """(system, user) message pair for the supervisory triage."""
    system = SUPERVISORY_SYSTEM_TEMPLATE.format(
        failure_context=json.dumps(failure_context, sort_keys=True, default=str),
        attempts=attempts,
        max_retries=max_retries,
    )
    return [("system", system), ("user", "Triage this Nav2 abort.")]

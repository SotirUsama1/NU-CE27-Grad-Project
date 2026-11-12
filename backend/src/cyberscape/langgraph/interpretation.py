"""Natural-language mission interpretation node (Phase 6).

Parses a mission statement into a structured intent - with a
``langchain-openai`` LLM when one is configured, or a regex fallback when
not - and grounds every location against ``TopoGraph.location_summary()``:
a location the model invents cannot resolve to a topological node, so the
mission is REJECTED instead of being executed against hallucinated
coordinates.
"""

from __future__ import annotations

import json
import logging
import re
from typing import Any

from pydantic import BaseModel

from cyberscape.config import settings
from cyberscape.langgraph.prompts import build_interpretation_messages

logger = logging.getLogger(__name__)


class MissionIntent(BaseModel):
    """The interpreter's validated output before location grounding."""

    mission_type: str = "TRANSPORT"
    item: str | None = None
    source: str | None = None
    destination: str | None = None
    notes: str | None = None


_ITEM_VERB = r"(?:move|carry|bring|transport|take|shuttle|ferry)\b"
_GO_TO = r"(?:go|navigate|drive|head|travel)\s+to\b"


# ---------------------------------------------------------------------------
# JSON plumbing (shared with the supervisory triage)
# ---------------------------------------------------------------------------


def extract_json(content: Any) -> dict:
    """Pull the first JSON object out of an LLM response ``content``.

    Accepts plain strings (with or without ```json fences), dict content
    (structured output) and content-block lists. Raises ``ValueError`` for
    anything that is not a JSON object.
    """
    if isinstance(content, dict):
        return content
    if isinstance(content, (list, tuple)):
        content = " ".join(
            part.get("text", "") if isinstance(part, dict) else str(part)
            for part in content
        )
    text = str(content).strip()
    fenced = re.search(r"```(?:json)?\s*(.*?)```", text, flags=re.DOTALL)
    if fenced:
        text = fenced.group(1).strip()
    start, end = text.find("{"), text.rfind("}")
    if start == -1 or end <= start:
        raise ValueError("no JSON object found in the LLM response")
    data = json.loads(text[start:end + 1])
    if not isinstance(data, dict):
        raise TypeError("the LLM response is not a JSON object")
    return data


# ---------------------------------------------------------------------------
# Grounding: names -> topological node ids
# ---------------------------------------------------------------------------


def normalize_location_name(name: Any) -> str:
    """Canonical form of a location mention (lowercased, no leading 'the')."""
    lowered = re.sub(r"\s+", " ", str(name or "").strip().lower())
    return lowered.removeprefix("the ")


def build_grounding_index(summary: dict[str, dict]) -> dict[str, tuple[str, str]]:
    """normalized location -> (node_id, kind); the first label wins ties."""
    index: dict[str, tuple[str, str]] = {}
    for zone_data in summary.values():
        entries: list[tuple[str, str, str]] = []
        for box in zone_data.get("boxes") or []:
            entries.append((box["label"], box["id"], "box"))
        for landmark in zone_data.get("landmarks") or []:
            entries.append((landmark["label"], landmark["id"],
                            landmark.get("kind", "landmark")))
        for junction in zone_data.get("junctions") or []:
            entries.append((junction["id"], junction["id"], "junction"))
        for label, node_id, kind in entries:
            index.setdefault(normalize_location_name(label), (node_id, kind))
    return index


def ground_location(
    index: dict[str, tuple[str, str]], name: str | None
) -> tuple[str, str] | None:
    """Resolve a location mention to ``(node_id, kind)``, or ``None``.

    Exact (normalized) match first, then containment - preferring the
    shortest matching label, so "rack A2" resolves to ``rack_a2_0`` (the
    first sorted bay) rather than something accidental.
    """
    if not name:
        return None
    key = normalize_location_name(name)
    hit = index.get(key)
    if hit:
        return hit
    candidates = [
        (known, resolved) for known, resolved in index.items()
        if known and (key in known or known in key)
    ]
    if candidates:
        return min(candidates, key=lambda kv: len(kv[0]))[1]
    return None


def regex_intent(text: str) -> dict:
    """Deterministic intent extraction for LLM-free operation.

    Handles the canonical phrasings ("move X from A to B", "go to A");
    raises ``ValueError`` for anything else so the mission is rejected
    instead of guessed.
    """
    flat = " ".join(str(text or "").split())
    moved = re.search(
        _ITEM_VERB + r"\s+(?P<item>.+?)\s+from\s+(?P<source>.+?)\s+to\s+"
        r"(?P<destination>.+?)\s*[.?!]?$",
        flat, flags=re.IGNORECASE,
    )
    if moved:
        return {
            "mission_type": "TRANSPORT", "item": moved.group("item"),
            "source": moved.group("source"), "destination": moved.group("destination"),
            "notes": None,
        }
    go = re.search(
        _GO_TO + r"\s+(?P<destination>.+?)\s*[.?!]?$", flat, flags=re.IGNORECASE
    )
    if go:
        return {
            "mission_type": "NAVIGATE", "item": None, "source": None,
            "destination": go.group("destination"), "notes": None,
        }
    raise ValueError(
        "no LLM configured and the mission text matches no known pattern"
    )


# ---------------------------------------------------------------------------
# The node
# ---------------------------------------------------------------------------


def make_interpretation_node(
    topo,
    db=None,
    llm=None,
    max_boxes_per_zone: int | None = None,
):
    """Build the ``interpretation`` node closure over its dependencies.

    ``llm`` is a langchain chat model (or None for the regex fallback);
    ``db`` persists the grounded plan and mission status when provided.
    """
    box_cap = (
        settings.PROMPT_MAX_BOXES_PER_ZONE
        if max_boxes_per_zone is None else max_boxes_per_zone
    )

    async def reject(state: dict, reason: str) -> dict:
        mission_id = state.get("mission_id")
        if db is not None and mission_id:
            await db.update_mission_status(mission_id, "REJECTED")
        logger.warning("Mission %s rejected: %s", mission_id, reason)
        return {"intent": None, "error": reason, "status": "REJECTED"}

    async def interpretation_node(state: dict) -> dict:
        summary = topo.location_summary(max_boxes_per_zone=box_cap)
        text = state.get("text") or ""
        try:
            if llm is not None:
                messages = build_interpretation_messages(text, summary)
                response = await llm.ainvoke(messages)
                raw = extract_json(response.content)
            else:
                raw = regex_intent(text)
            parsed = MissionIntent.model_validate(raw)
        except Exception as exc:  # noqa: BLE001
            return await reject(state, f"could not parse the mission: {exc}")

        index = build_grounding_index(summary)
        source = ground_location(index, parsed.source)
        destination = ground_location(index, parsed.destination)
        if parsed.destination and destination is None:
            return await reject(
                state,
                f"destination {parsed.destination!r} is not a known warehouse location",
            )
        if parsed.source and source is None:
            return await reject(
                state,
                f"source {parsed.source!r} is not a known warehouse location",
            )
        if source is None and destination is None:
            return await reject(state, "the mission names no resolvable location")

        grounded = {
            "mission_type": parsed.mission_type,
            "item": parsed.item,
            "source": parsed.source,
            "destination": parsed.destination,
            "notes": parsed.notes,
            "source_node": source[0] if source else None,
            "source_kind": source[1] if source else None,
            "destination_node": destination[0] if destination else None,
            "destination_kind": destination[1] if destination else None,
        }
        mission_id = state.get("mission_id")
        if db is not None and mission_id:
            await db.set_mission_plan(mission_id, grounded)
            await db.update_mission_status(mission_id, "PARSED")
        return {"intent": grounded, "error": None}

    return interpretation_node

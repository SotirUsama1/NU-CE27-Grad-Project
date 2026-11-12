"""LangGraph workflow wiring for the CyberScape agent worker (Phase 6).

One ``StateGraph`` handles both entry paths:

* a NEW MISSION (``text`` present) -> ``interpretation`` (parse + ground)
  -> ``execution`` (persist primitives, wake the dispatcher)
* a FAILURE (``failure_context`` present, a Nav2 STATUS_ABORTED event)
  -> ``supervisory`` (triage) -> REPLAN to ``execution``, RETRY to
  ``retry``, ESCALATE to ``END``.

Nodes are plain closures built in ``cyberscape.langgraph.{interpretation,
execution,supervisory}``; this module only wires state, routing and edges.
"""

from __future__ import annotations

import logging
from typing import Any

from langgraph.graph import END, START, StateGraph

from cyberscape.config import settings
from cyberscape.langgraph.execution import make_execution_node
from cyberscape.langgraph.interpretation import make_interpretation_node
from cyberscape.langgraph.state import (
    DECISION,
    EXECUTION,
    INTERPRETATION,
    RETRY,
    SUPERVISORY,
    Decision,
    WorkflowState,
)
from cyberscape.langgraph.supervisory import (
    make_retry_node,
    make_supervisory_node,
)
from cyberscape.topo_graph import TopoGraph

logger = logging.getLogger(__name__)


def route_entry(state: WorkflowState) -> str:
    """Conditional entry: failure events go to triage, missions to parsing."""
    if state.get("failure_context"):
        return SUPERVISORY
    return INTERPRETATION


def route_after_supervisory(state: WorkflowState) -> str:
    """Supervisory triage outcome -> REPLAN/RETRY/ESCALATE edges."""
    decision = state.get(DECISION) or Decision.ESCALATE
    if decision == Decision.REPLAN:
        return EXECUTION
    if decision == Decision.RETRY:
        return RETRY
    if decision == Decision.CANCELED:
        logger.info("Supervisory decision CANCELED ends the workflow gracefully (END)")
        return END
    logger.warning("Supervisory decision %s ends the workflow (ESCALATE)", decision)
    return END


def build_llm():
    """Build the chat model for interpretation/triage, or ``None``.

    Returns a ``langchain-openai`` ``ChatOpenAI`` only when an API key is
    configured; an empty key (CI default) or a missing langchain-openai
    install keeps the whole workflow deterministic (regex parsing +
    heuristic triage).
    """
    if not settings.OPENAI_API_KEY:
        return None
    try:
        from langchain_openai import ChatOpenAI
    except ImportError:  # pragma: no cover - depends on the local install
        logger.warning("OPENAI_API_KEY set but langchain-openai is not installed")
        return None
    kwargs: dict[str, Any] = {
        "model": settings.OPENAI_MODEL,
        "temperature": 0,
        "api_key": settings.OPENAI_API_KEY,
    }
    if settings.OPENAI_BASE_URL:
        kwargs["base_url"] = settings.OPENAI_BASE_URL
    return ChatOpenAI(**kwargs)


def build_graph(
    db=None,
    topo: TopoGraph | None = None,
    llm=None,
    redis_client=None,
    max_boxes_per_zone: int | None = None,
    max_retries: int | None = None,
):
    """Compile the CyberScape agent workflow graph.

    Args:
        db: ``cyberscape.db.Database`` (or compatible) for persistence.
        topo: ``TopoGraph`` grounding the interpreter; defaults to a lazily
            loaded graph from the configured topo file.
        llm: chat model for interpretation and triage (``None`` = rules only).
        redis_client: publishes ``cyberscape:mission:new`` after sequencing.
        max_boxes_per_zone: prompt grounding cap (default from settings).
        max_retries: supervisory attempt budget (default from settings).
    """
    topo = (topo if topo is not None else TopoGraph()).ensure_loaded()

    workflow = StateGraph(WorkflowState)

    workflow.add_node(
        INTERPRETATION,
        make_interpretation_node(
            topo, db=db, llm=llm, max_boxes_per_zone=max_boxes_per_zone
        ),
    )
    workflow.add_node(EXECUTION, make_execution_node(db=db, redis_client=redis_client))
    workflow.add_node(
        SUPERVISORY, make_supervisory_node(db=db, llm=llm, max_retries=max_retries)
    )
    workflow.add_node(RETRY, make_retry_node(db=db))

    # Conditional entry (plan 6.1): failure -> supervisory, else interpretation.
    workflow.add_conditional_edges(
        START,
        route_entry,
        {
            SUPERVISORY: SUPERVISORY,
            INTERPRETATION: INTERPRETATION,
        },
    )

    # interpretation -> execution -> END
    workflow.add_edge(INTERPRETATION, EXECUTION)
    workflow.add_edge(EXECUTION, END)

    # supervisory -> [execution | retry | END]
    workflow.add_conditional_edges(
        SUPERVISORY,
        route_after_supervisory,
        {
            EXECUTION: EXECUTION,
            RETRY: RETRY,
            END: END,
        },
    )
    workflow.add_edge(RETRY, END)

    return workflow.compile()

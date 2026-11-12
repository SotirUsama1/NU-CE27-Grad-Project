"""Async ROSBridge WebSocket client for the CyberScape orchestrator.

Phase 5: connects to the rosbridge_server started by
``Sim/ros_ws/src/warehouse_navigation/launch/multi_robot.launch.py``
(``ws://localhost:9090``) and speaks the rosbridge JSON protocol:

  * subscribes to ``/{namespace}/amcl_pose``
    (``geometry_msgs/msg/PoseWithCovarianceStamped``) for the continuous AMCL
    pose and junction-arrival detection (within 0.5 m of the goal node),
  * subscribes to ``/{namespace}/heartbeat`` (``std_msgs/msg/Header``;
    ``stamp`` = simulation time, ``frame_id`` = nearest topological node -
    see ``heartbeat_publisher.py``),
  * dispatches Nav2 ``NavigateToPose`` goals junction-by-junction through the
    rosbridge action protocol (``send_action_goal`` / ``action_result`` /
    ``action_feedback`` / ``cancel_action_goal`` ops, verified against
    rosbridge_suite 2.0.8).

Goal statuses are bound to ROS 2 ``action_msgs/msg/GoalStatus``:

  * ``STATUS_SUCCEEDED = 4``  -> the primitive advanced (task sequence moves on),
  * ``STATUS_ABORTED  = 6``  -> a failure context is emitted for the
    Supervisory Layer (LangGraph exception triage, Phase 6).

(The constants were verified against the ROS 2 Humble message definition in the
simulation container: UNKNOWN=0, ACCEPTED=1, EXECUTING=2, CANCELING=3,
SUCCEEDED=4, CANCELED=5, ABORTED=6. The ROS-side navigator.py imports the enum
directly; the backend has no ROS runtime, so the enum is mirrored here and
cross-checked by an optional test.)
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any
from uuid import uuid4

import websockets
from redis.exceptions import RedisError
from websockets.exceptions import WebSocketException

from cyberscape.config import settings

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# ROS 2 message bindings (no ROS runtime on the backend host)
# ---------------------------------------------------------------------------


class GoalStatus:
    """Mirror of ROS 2 ``action_msgs/msg/GoalStatus`` (verified vs Humble binary)."""

    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


#: NavigateToPose action type, as rosbridge expects it in ``action_type``.
NAVIGATE_TO_POSE_TYPE = "nav2_msgs/action/NavigateToPose"

#: rosbridge ops (rosbridge_suite 2.0.8 protocol).
OP_SEND_ACTION_GOAL = "send_action_goal"
OP_CANCEL_ACTION_GOAL = "cancel_action_goal"
OP_ACTION_RESULT = "action_result"
OP_ACTION_FEEDBACK = "action_feedback"
OP_SUBSCRIBE = "subscribe"
OP_UNSUBSCRIBE = "unsubscribe"
OP_PUBLISH = "publish"

#: Junction arrival: an AMCL pose within this distance of the goal node counts.
ARRIVAL_THRESHOLD_M = 0.5

#: asyncio.TimeoutError is NOT an alias of the builtin TimeoutError on Python
#: 3.10 (this runtime), so wait_for timeouts must catch both; rewriting the
#: pair to the builtin alone (ruff UP041) would break timeouts on 3.10.
_TIMEOUT_ERRORS: tuple[type[BaseException], ...] = (asyncio.TimeoutError, TimeoutError)


@dataclass
class SegmentLease:
    """The Redis segment lock held while a robot traverses its corridor.

    Passed to ``dispatch_navigate_to_pose`` so the client keeps the lease
    alive with rolling renewals for the whole traversal (plan Gap 5).
    """

    robot_id: str
    edge_keys: list[str]
    junction_node: str | None = None


@dataclass
class NavigationResult:
    """Outcome of one NavigateToPose goal dispatched through rosbridge."""

    goal_id: str
    action: str
    status: int | None            # action_msgs/msg/GoalStatus value (None on timeout)
    success: bool                 # status == STATUS_SUCCEEDED
    values: dict = field(default_factory=dict)   # NavigateToPose.Result as sent by rosbridge
    timed_out: bool = False

    @property
    def aborted(self) -> bool:
        """True when Nav2 itself gave up (STATUS_ABORTED) - supervisory material."""
        return self.status == GoalStatus.STATUS_ABORTED


@dataclass
class FailureContext:
    """What the Supervisory Layer needs to triage a failed navigation (plan 5.4)."""

    robot_id: str
    goal_id: str
    status: int
    node_id: str | None
    goal_pose: dict
    values: dict
    reason: str

    def to_dict(self) -> dict:
        return {
            "robot_id": self.robot_id, "goal_id": self.goal_id, "status": self.status,
            "node_id": self.node_id, "goal_pose": self.goal_pose, "values": self.values,
            "reason": self.reason,
        }


# ---------------------------------------------------------------------------
# Pure serialization / parsing helpers (unit-tested without a connection)
# ---------------------------------------------------------------------------


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rad) of a geometry_msgs/msg/Quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> dict:
    """Geometry_msgs orientation dict equivalent to ``yaw`` (rad)."""
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def build_goal_pose(
    x: float, y: float, yaw: float = 0.0, frame_id: str = "map", stamp: dict | None = None
) -> dict:
    """JSON for a PoseStamped goal (the ``pose`` field of NavigateToPose.Goal)."""
    return {
        "header": {
            "stamp": stamp or {"sec": 0, "nanosec": 0},
            "frame_id": frame_id,
        },
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": quaternion_from_yaw(yaw),
        },
    }


def build_navigate_to_pose_goal(
    x: float, y: float, yaw: float = 0.0, frame_id: str = "map", stamp: dict | None = None
) -> dict:
    """JSON args of a NavigateToPose.Goal (``pose`` + default behavior tree)."""
    return {
        "pose": build_goal_pose(x, y, yaw, frame_id=frame_id, stamp=stamp),
        "behavior_tree": "",
    }


def build_send_action_goal(
    goal_id: str, action: str, goal_args: dict, feedback: bool = False
) -> dict:
    """rosbridge ``send_action_goal`` op (send_action_goal.py capability)."""
    return {
        "op": OP_SEND_ACTION_GOAL,
        "id": goal_id,
        "action": action,
        "action_type": NAVIGATE_TO_POSE_TYPE,
        "args": goal_args,
        "feedback": feedback,
    }


def build_cancel_action_goal(goal_id: str, action: str) -> dict:
    """rosbridge ``cancel_action_goal`` op."""
    return {"op": OP_CANCEL_ACTION_GOAL, "id": goal_id, "action": action}


def build_subscribe(topic: str, message_type: str, subscription_id: str | None = None) -> dict:
    """rosbridge ``subscribe`` op for one topic."""
    op: dict[str, Any] = {"op": OP_SUBSCRIBE, "topic": topic, "type": message_type}
    if subscription_id:
        op["id"] = subscription_id
    return op


def build_unsubscribe(topic: str, subscription_id: str | None = None) -> dict:
    """rosbridge ``unsubscribe`` op for one topic."""
    op: dict[str, Any] = {"op": OP_UNSUBSCRIBE, "topic": topic}
    if subscription_id:
        op["id"] = subscription_id
    return op


def parse_pose(msg: dict) -> tuple[float, float, float]:
    """(x, y, yaw) from a PoseWithCovarianceStamped JSON payload."""
    pose = msg["pose"]["pose"]
    position = pose["position"]
    q = pose["orientation"]
    return (
        float(position["x"]),
        float(position["y"]),
        yaw_from_quaternion(float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])),
    )


def parse_header(msg: dict) -> tuple[float, str]:
    """(sim_time, frame_id) from a std_msgs/msg/Header JSON payload.

    ``sim_time`` is the header stamp as seconds (builtin_interfaces/msg/Time) -
    the simulation timestamp the heartbeat publisher carries.
    """
    stamp = msg.get("stamp") or {}
    sim_time = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) / 1e9
    return sim_time, str(msg.get("frame_id", ""))


# ---------------------------------------------------------------------------
# The client
# ---------------------------------------------------------------------------


class RosBridgeClient:
    """Async WebSocket client for one rosbridge_server (the whole fleet)."""

    def __init__(
        self,
        url: str = "ws://localhost:9090",
        topo_graph=None,
        on_success: Callable[[str, NavigationResult], Any] | None = None,
        on_failure: Callable[[FailureContext], Any] | None = None,
        on_heartbeat: Callable[[str, float, float | None, float | None, float | None], Any] | None = None,
        on_arrival: Callable[[str, str], Any] | None = None,
        redis_client=None,
        lease_renew_interval: float = 2.0,
    ) -> None:
        self.url = url
        self.topo_graph = topo_graph
        self.on_success = on_success      # STATUS_SUCCEEDED -> advance the task
        self.on_failure = on_failure      # STATUS_ABORTED -> supervisory layer
        # Gap 2: (robot_id, sim_time, x, y, yaw) per heartbeat, for
        # HeartbeatMonitor.record_heartbeat (x/y/yaw come from the latest
        # amcl_pose; the node id stays in self.heartbeats[robot]["node"]).
        self.on_heartbeat = on_heartbeat
        # Gap 3: (robot_id, node_id) when an amcl_pose lands within 0.5 m of
        # the goal junction - the orchestrator releases the traversed segment
        # lock and publishes cyberscape:segment:released.
        self.on_arrival = on_arrival
        # Gap 5: RedisClient whose renew_segment_lease() keeps the corridor
        # reserved while a goal is in flight (interval < SEGMENT_LOCK_TTL_S).
        self.redis_client = redis_client
        if lease_renew_interval <= 0:
            raise ValueError("lease_renew_interval must be positive")
        self.lease_renew_interval = lease_renew_interval
        self.ws = None
        self._recv_task: asyncio.Task | None = None
        self._subscriptions: dict[str, str] = {}      # topic -> subscription id
        self._pending: dict[str, asyncio.Future] = {}  # goal id -> result future
        self.poses: dict[str, tuple[float, float, float]] = {}   # namespace -> (x, y, yaw)
        self.heartbeats: dict[str, dict[str, Any]] = {}          # namespace -> {"sim_time", "node"}
        self._targets: dict[str, str] = {}             # namespace -> goal node id
        self.arrived: dict[str, str | None] = {}       # namespace -> node reached (or None)
        self.arrivals: asyncio.Queue = asyncio.Queue() # (namespace, node_id) junction arrivals
        self.failures: asyncio.Queue = asyncio.Queue() # FailureContext of every STATUS_ABORTED
        self._callback_tasks: set[asyncio.Task] = set()  # strong refs, see _spawn_callback

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        """Open the WebSocket and start the receive loop."""
        if self.ws is not None:
            return
        self.ws = await websockets.connect(self.url)
        self._recv_task = asyncio.create_task(self._recv_loop())

    async def disconnect(self) -> None:
        """Stop the receive loop and close the WebSocket."""
        ws = self.ws
        if self._recv_task is not None:
            self._recv_task.cancel()
            await asyncio.gather(self._recv_task, return_exceptions=True)
            self._recv_task = None
        self.ws = None
        if ws is not None:
            try:
                await ws.close()
            except (OSError, WebSocketException) as exc:  # close is best effort
                logger.debug("Ignored error closing rosbridge WebSocket: %s", exc)
        self._subscriptions.clear()

    @property
    def connected(self) -> bool:
        return self.ws is not None

    async def _send(self, op: dict) -> None:
        if self.ws is None:
            raise RuntimeError("RosBridgeClient is not connected")
        await self.ws.send(json.dumps(op))

    # ------------------------------------------------------------------
    # Receive loop & routing
    # ------------------------------------------------------------------

    async def _recv_loop(self) -> None:
        try:
            async for raw in self.ws:
                try:
                    message = json.loads(raw)
                except (TypeError, ValueError):
                    logger.warning("Dropping non-JSON rosbridge frame: %.120r", raw)
                    continue
                self._route(message)
        except asyncio.CancelledError:
            raise
        except (OSError, WebSocketException) as exc:  # pragma: no cover - connection level
            logger.warning("rosbridge receive loop stopped: %s", exc)
        finally:
            self.ws = None
            for future in self._pending.values():
                if not future.done():
                    future.set_exception(ConnectionError("rosbridge connection closed"))
            self._pending.clear()

    def _route(self, message: dict) -> None:
        op = message.get("op")
        if op == OP_PUBLISH:
            self._on_topic_message(message)
        elif op == OP_ACTION_RESULT:
            self._on_action_result(message)
        elif op == OP_ACTION_FEEDBACK:
            self._on_action_feedback(message)
        # status/error ops are logged and ignored: dispatch timeouts surface them
        elif op in ("status", "error"):
            logger.debug("rosbridge %s: %s", op, message.get("message", ""))

    # ------------------------------------------------------------------
    # Topic subscriptions (amcl_pose + heartbeat)
    # ------------------------------------------------------------------

    async def subscribe_pose(self, namespace: str) -> None:
        """Subscribe to /{namespace}/amcl_pose for pose & arrival detection."""
        await self._subscribe_topic(
            f"/{namespace}/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"
        )

    async def subscribe_heartbeat(self, namespace: str) -> None:
        """Subscribe to /{namespace}/heartbeat (sim-time Header)."""
        await self._subscribe_topic(f"/{namespace}/heartbeat", "std_msgs/msg/Header")

    async def _subscribe_topic(self, topic: str, message_type: str) -> None:
        subscription_id = str(uuid4())
        self._subscriptions[topic] = subscription_id
        await self._send(build_subscribe(topic, message_type, subscription_id))

    async def unsubscribe(self, topic: str) -> None:
        if self._subscriptions.pop(topic, None) is not None:
            await self._send(build_unsubscribe(topic))

    def _namespace_of(self, topic: str) -> str | None:
        """/husky1/amcl_pose -> husky1 (fleet topics are always /{ns}/{tail})."""
        parts = [p for p in topic.split("/") if p]
        if len(parts) >= 2:
            return parts[0]
        return None

    def _on_topic_message(self, message: dict) -> None:
        topic = message.get("topic", "")
        payload = message.get("msg") or {}
        namespace = self._namespace_of(topic)
        if namespace is None:
            return
        if topic.endswith("/amcl_pose"):
            try:
                pose = parse_pose(payload)
            except (KeyError, TypeError, ValueError):
                logger.warning("Malformed amcl_pose on %s", topic)
                return
            self.poses[namespace] = pose
            self._check_arrival(namespace, pose)
        elif topic.endswith("/heartbeat"):
            try:
                sim_time, node = parse_header(payload)
            except (TypeError, ValueError):
                logger.warning("Malformed heartbeat on %s", topic)
                return
            self.heartbeats[namespace] = {"sim_time": sim_time, "node": node}
            # Gap 2: hand the liveness data straight to the monitor. The
            # 1 Hz Header carries no pose, so the coordinates come from the
            # latest amcl_pose of the same robot (None until one arrived).
            if self.on_heartbeat is not None:
                pose = self.poses.get(namespace)
                x, y, yaw = pose if pose is not None else (None, None, None)
                self._spawn_callback(self.on_heartbeat, namespace, sim_time, x, y, yaw)

    # ------------------------------------------------------------------
    # Junction arrival detection (< 0.5 m from the goal node)
    # ------------------------------------------------------------------

    def node_xy(self, node_id: str) -> tuple[float, float] | None:
        """(x, y) of a topological node, or None if unknown."""
        if self.topo_graph is None:
            return None
        try:
            tg = getattr(self.topo_graph, "ensure_loaded", None)
            graph = tg() if callable(tg) else self.topo_graph
            nodes = getattr(graph, "nodes", {})
            node = nodes[node_id]
            # If the target is a physical rack, navigate to its accessible aisle waypoint
            if node.get("type") == "rack":
                edges = getattr(graph, "edges", [])
                for edge in edges:
                    if edge.get("from") == node_id and edge.get("to") in nodes:
                        neighbor = nodes[edge["to"]]
                        return float(neighbor["x"]), float(neighbor["y"])
                    if edge.get("to") == node_id and edge.get("from") in nodes:
                        neighbor = nodes[edge["from"]]
                        return float(neighbor["x"]), float(neighbor["y"])
            return float(node["x"]), float(node["y"])
        except (KeyError, TypeError, ValueError):
            return None

    def arrival_distance(self, namespace: str, node_id: str) -> float | None:
        """Straight-line distance (m) from the robot's last AMCL pose to a node."""
        pose = self.poses.get(namespace)
        xy = self.node_xy(node_id)
        if pose is None or xy is None:
            return None
        return math.dist(pose[:2], xy)

    def at_junction(self, namespace: str, node_id: str, threshold: float = ARRIVAL_THRESHOLD_M) -> bool:
        """True when the robot is within ``threshold`` m of ``node_id``."""
        distance = self.arrival_distance(namespace, node_id)
        return distance is not None and distance < threshold

    def _check_arrival(self, namespace: str, pose: tuple[float, float, float]) -> None:
        """Record a junction arrival when the active goal's node is reached."""
        node_id = self._targets.get(namespace)
        if node_id is None:
            return
        xy = self.node_xy(node_id)
        if xy is None or math.dist(pose[:2], xy) >= ARRIVAL_THRESHOLD_M:
            return
        if self.arrived.get(namespace) != node_id:
            self.arrived[namespace] = node_id
            self.arrivals.put_nowait((namespace, node_id))
            logger.info("%s arrived at junction %s", namespace, node_id)
            # Gap 3: let the orchestrator release the traversed segment lock
            # in Redis and broadcast cyberscape:segment:released.
            if self.on_arrival is not None:
                self._spawn_callback(self.on_arrival, namespace, node_id)

    # ------------------------------------------------------------------
    # NavigateToPose dispatch (rosbridge action protocol)
    # ------------------------------------------------------------------

    def _goal_stamp(self, namespace: str) -> dict:
        """Sim-time header stamp from the robot's last heartbeat (0s fallback).

        Nav2 resolves the goal with the latest transform when the stamp is 0,
        so the fallback is safe; the heartbeat stamp is simply more truthful.
        """
        heartbeat = self.heartbeats.get(namespace)
        if heartbeat and heartbeat.get("sim_time"):
            sim_time = float(heartbeat["sim_time"])
            sec = int(sim_time)
            return {"sec": sec, "nanosec": round((sim_time - sec) * 1e9)}
        return {"sec": 0, "nanosec": 0}

    async def dispatch_navigate_to_pose(
        self,
        namespace: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        goal_id: str | None = None,
        timeout: float = 600.0,
        feedback: bool = False,
        segment_lease: SegmentLease | None = None,
    ) -> NavigationResult:
        """Send one NavigateToPose goal and await its terminal GoalStatus.

        STATUS_SUCCEEDED (4) resolves ``success=True`` (task advances);
        STATUS_ABORTED (6) resolves ``success=False`` and emits a FailureContext
        to the Supervisory Layer; a timeout first cancels the goal in Nav2
        (Gap 4: no ghost AMR keeps driving unattended) and then resolves
        ``timed_out=True``.

        With ``segment_lease`` and a ``redis_client``, a background task
        renews the corridor's segment lock every ``lease_renew_interval``
        seconds until the goal finishes (Gap 5).
        """
        action = f"/{namespace}/navigate_to_pose"
        goal_id = goal_id or f"cyberscape:{namespace}:{uuid4().hex[:8]}"
        goal_args = build_navigate_to_pose_goal(
            x, y, yaw, stamp=self._goal_stamp(namespace)
        )
        future: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[goal_id] = future
        stop_renewal = asyncio.Event()
        renew_task: asyncio.Task | None = None
        if segment_lease is not None and self.redis_client is None:
            logger.warning(
                "segment_lease given for %s without redis_client: no lease renewals",
                namespace,
            )
        try:
            await self._send(build_send_action_goal(goal_id, action, goal_args, feedback))
            if segment_lease is not None and self.redis_client is not None:
                # Gap 5: keep the corridor reserved for the whole traversal.
                renew_task = asyncio.create_task(
                    self._renew_lease_loop(segment_lease, stop_renewal)
                )
            try:
                status, values = await asyncio.wait_for(future, timeout)
            except _TIMEOUT_ERRORS:
                # Gap 4: a timed-out goal keeps running in Nav2 unless it is
                # canceled - the AMR would be a ghost moving with no lease,
                # no tracking and nobody waiting for the result.
                logger.warning(
                    "NavigateToPose %s timed out after %.0fs; canceling in Nav2",
                    goal_id, timeout,
                )
                try:
                    await self.cancel_navigate(goal_id, namespace)
                except (OSError, WebSocketException, RuntimeError) as exc:
                    logger.error("Could not cancel timed-out goal %s: %s", goal_id, exc)
                return NavigationResult(
                    goal_id=goal_id, action=action, status=None, success=False,
                    values={}, timed_out=True,
                )
        finally:
            self._pending.pop(goal_id, None)
            stop_renewal.set()
            if renew_task is not None:
                await asyncio.gather(renew_task, return_exceptions=True)

        result = NavigationResult(
            goal_id=goal_id, action=action, status=status,
            success=status == GoalStatus.STATUS_SUCCEEDED, values=values,
        )
        if result.success:
            logger.info("%s NavigateToPose %s SUCCEEDED", namespace, goal_id)
            if self.on_success is not None:
                await _maybe_async(self.on_success, namespace, result)
        else:
            logger.warning(
                "%s NavigateToPose %s finished with status %s", namespace, goal_id, status
            )
            if status == GoalStatus.STATUS_ABORTED:
                self._emit_failure(
                    FailureContext(
                        robot_id=namespace, goal_id=goal_id,
                        status=status, node_id=self._targets.get(namespace),
                        goal_pose=goal_args["pose"], values=values,
                        reason="Nav2 aborted the NavigateToPose goal",
                    )
                )
        return result

    async def _renew_lease_loop(self, lease: SegmentLease, stop: asyncio.Event) -> None:
        """Roll the corridor's segment lease forward while the goal runs (Gap 5).

        The Redis lock taken before dispatch expires after
        ``SEGMENT_LOCK_TTL_S``; without renewal a slow traversal would lose
        the reservation and a second robot could be sent into the same
        single-lane edge mid-run. Renewal stops as soon as ``stop`` is set -
        the dispatch is over and the orchestrator owns the lease again.
        """
        while not stop.is_set():
            try:
                await asyncio.wait_for(stop.wait(), timeout=self.lease_renew_interval)
            except _TIMEOUT_ERRORS:
                pass  # interval elapsed: renew now
            if stop.is_set():
                return
            try:
                renewed = await self.redis_client.renew_segment_lease(
                    lease.robot_id,
                    lease.edge_keys,
                    ttl=settings.SEGMENT_LOCK_TTL_S,
                    junction_node=lease.junction_node,
                )
            except (OSError, RedisError) as exc:
                logger.warning(
                    "Segment lease renewal errored for %s on %s: %s",
                    lease.robot_id, lease.edge_keys, exc,
                )
                continue
            if not renewed:
                logger.warning(
                    "Segment lease renewal rejected for %s on %s: lock lapsed",
                    lease.robot_id, lease.edge_keys,
                )

    def _travel_heading(
        self, namespace: str, target_xy: tuple[float, float], source_node: str | None = None
    ) -> float:
        """Yaw facing along the travel edge into ``target_xy`` (Gap 6).

        The robot drives toward the junction already facing it instead of
        arriving sideways and rotating in place: the heading is the direction
        of the edge being traversed - from ``source_node`` when the caller
        knows the previous junction, else from the robot's latest AMCL pose,
        else 0.0 (unknown start; Nav2 then resolves the approach itself).
        """
        source_xy: tuple[float, float] | None = None
        if source_node is not None:
            source_xy = self.node_xy(source_node)
        if source_xy is None:
            pose = self.poses.get(namespace)
            source_xy = pose[:2] if pose is not None else None
        if source_xy is None:
            return 0.0
        return math.atan2(target_xy[1] - source_xy[1], target_xy[0] - source_xy[0])

    async def dispatch_navigate_to_node(
        self,
        namespace: str,
        node_id: str,
        timeout: float = 600.0,
        segment_lease: SegmentLease | None = None,
        source_node: str | None = None,
    ) -> NavigationResult:
        """Drive to one topological junction (plan 5.4: junction-by-junction).

        Sets the junction as the namespace's arrival target (0.5 m detection on
        every amcl_pose) and dispatches the goal to its coordinates facing
        along the travel edge (Gap 6), renewing ``segment_lease`` under way
        (Gap 5).
        """
        xy = self.node_xy(node_id)
        if xy is None:
            raise KeyError(f"Unknown topological node: {node_id}")
        self._targets[namespace] = node_id
        self.arrived[namespace] = None
        try:
            yaw = self._travel_heading(namespace, xy, source_node=source_node)
            return await self.dispatch_navigate_to_pose(
                namespace, xy[0], xy[1], yaw=yaw, timeout=timeout,
                segment_lease=segment_lease,
            )
        finally:
            self._targets.pop(namespace, None)

    async def cancel_navigate(self, goal_id: str, namespace: str) -> None:
        """Cancel a running NavigateToPose goal."""
        await self._send(build_cancel_action_goal(goal_id, f"/{namespace}/navigate_to_pose"))

    # ------------------------------------------------------------------
    # Action result / feedback handling
    # ------------------------------------------------------------------

    def _on_action_result(self, message: dict) -> None:
        goal_id = message.get("id")
        future = self._pending.get(goal_id)
        status = message.get("status")
        values = message.get("values") or {}
        if not isinstance(values, dict):
            values = {}
        if future is None or future.done():
            logger.debug("action_result for unknown/finished goal %s", goal_id)
            return
        if message.get("result") is False:
            # rosbridge-side failure (e.g. unknown action type): GoalStatus.STATUS_UNKNOWN
            status = GoalStatus.STATUS_UNKNOWN
            values = {"error": values}
        future.set_result((status, values))

    def _on_action_feedback(self, message: dict) -> None:
        # NavigateToPose feedback (distance_remaining, recoveries) is logged only;
        # the result status drives the task sequence.
        logger.debug(
            "action_feedback %s: %.160s", message.get("id"), message.get("values", {})
        )

    def _emit_failure(self, context: FailureContext) -> None:
        """Hand the failure to the Supervisory Layer (queue + callback)."""
        self.failures.put_nowait(context)
        logger.warning(
            "FAILURE %s goal=%s node=%s: %s",
            context.robot_id, context.goal_id, context.node_id, context.reason,
        )
        if self.on_failure is not None:
            self._spawn_callback(self.on_failure, context)

    def _spawn_callback(self, callback: Callable[..., Any], *args) -> None:
        """Fire a client callback without ever blocking the receive loop.

        The task is kept referenced until done (asyncio holds only a weak
        reference to a bare task) and its exceptions are logged by
        ``_maybe_async`` instead of surfacing in the receive loop.
        """
        task = asyncio.get_running_loop().create_task(_maybe_async(callback, *args))
        self._callback_tasks.add(task)
        task.add_done_callback(self._callback_tasks.discard)


async def _maybe_async(callback, *args) -> None:
    """Await async callbacks, call sync ones directly; never raise into the client."""
    try:
        result = callback(*args)
        if asyncio.iscoroutine(result):
            await result
    except Exception:  # pragma: no cover - user callbacks must not break dispatch
        logger.exception("rosbridge client callback failed")

"""Unit tests for the Phase 5 rosbridge client (mocked WebSocket).

Covers, per plan 5.5:
  * message serialization (goal pose quaternion math, rosbridge ops),
  * ROS 2 goal status binding - action_msgs/msg/GoalStatus
    STATUS_SUCCEEDED = 4 advances the task, STATUS_ABORTED = 6 emits a
    failure context (verified against the ROS 2 Humble binary),
  * arrival distance calculations (junction arrival within 0.5 m),
  * the full dispatch flow over a mocked WebSocket connection,
  * the integration-gap behaviors: the heartbeat pipeline (Gap 2), the
    junction-arrival hook (Gap 3), cancel-on-timeout (Gap 4), rolling segment
    lease renewal (Gap 5) and travel-edge heading (Gap 6).
"""

from __future__ import annotations

import asyncio
import json
import math
from collections import deque
from contextlib import asynccontextmanager

import pytest
import pytest_asyncio
import websockets
from redis.exceptions import RedisError

from cyberscape.config import settings
from cyberscape.rosbridge_client import (
    ARRIVAL_THRESHOLD_M,
    NAVIGATE_TO_POSE_TYPE,
    GoalStatus,
    NavigationResult,
    RosBridgeClient,
    SegmentLease,
    build_cancel_action_goal,
    build_goal_pose,
    build_navigate_to_pose_goal,
    build_send_action_goal,
    build_subscribe,
    build_unsubscribe,
    parse_header,
    parse_pose,
    quaternion_from_yaw,
    yaw_from_quaternion,
)

# ---------------------------------------------------------------------------
# Fixtures & fakes
# ---------------------------------------------------------------------------


class MiniTopo:
    """n_a at (0, 0), n_b at (3, 4): 5 m apart, straight-line only."""

    def __init__(self):
        self.nodes = {
            "n_a": {"x": 0.0, "y": 0.0, "type": "junction"},
            "n_b": {"x": 3.0, "y": 4.0, "type": "junction"},
        }


class FakeWebSocket:
    """Minimal stand-in for a websockets connection (async iteration + send)."""

    def __init__(self):
        self.sent: list[dict] = []
        self._frames: deque = deque()
        self._event = asyncio.Event()
        self.closed = False

    def feed(self, message: str) -> None:
        self._frames.append(message)
        self._event.set()

    def send_frame(self, op: dict) -> None:
        self.feed(json.dumps(op))

    def last_send(self) -> dict:
        return self.sent[-1]

    async def send(self, raw: str) -> None:
        self.sent.append(json.loads(raw))

    async def close(self) -> None:
        self.closed = True
        self._event.set()

    def __aiter__(self):
        return self

    async def __anext__(self) -> str:
        await self._event.wait()
        if self.closed and not self._frames:
            raise StopAsyncIteration
        item = self._frames.popleft()
        if not self._frames:
            self._event.clear()
        return item


class FakeLeaseRedis:
    """Records renew_segment_lease calls (Gap 5)."""

    def __init__(self):
        self.renewals: list[tuple[str, list[str], float, str | None]] = []

    async def renew_segment_lease(self, robot_id, edge_keys, ttl=None, junction_node=None):
        self.renewals.append((robot_id, list(edge_keys), ttl, junction_node))
        return True


@pytest.fixture
def fake_ws(monkeypatch):
    """Patch websockets.connect to hand back one shared FakeWebSocket."""
    fake = FakeWebSocket()

    async def fake_connect(url, *args, **kwargs):
        assert url == "ws://test:9090"
        return fake

    monkeypatch.setattr(websockets, "connect", fake_connect)
    return fake


@asynccontextmanager
async def connected_client(**kwargs):
    """A connected RosBridgeClient with non-default wiring (the gap callbacks).

    Requires the ``fake_ws`` fixture (the patched ``websockets.connect``) in
    the test that opens it.
    """
    kwargs.setdefault("url", "ws://test:9090")
    kwargs.setdefault("topo_graph", MiniTopo())
    c = RosBridgeClient(**kwargs)
    await c.connect()
    try:
        yield c
    finally:
        await c.disconnect()


@pytest_asyncio.fixture
async def client(fake_ws):
    async with connected_client() as c:
        yield c


def pose_message(x: float, y: float, yaw: float = 0.0, frame_id: str = "map") -> dict:
    """A realistic PoseWithCovarianceStamped JSON payload (as rosbridge sends it)."""
    return {
        "header": {"stamp": {"sec": 100, "nanosec": 250_000_000}, "frame_id": frame_id},
        "pose": {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": quaternion_from_yaw(yaw),
            },
            "covariance": [0.0] * 36,
        },
    }


def heartbeat_message(sim_time: float, node: str) -> dict:
    sec = int(sim_time)
    return {
        "stamp": {"sec": sec, "nanosec": round((sim_time - sec) * 1e9)},
        "frame_id": node,
    }


def action_result(goal_id: str, status: int, result: bool = True) -> dict:
    return {
        "op": "action_result", "id": goal_id, "status": status,
        "values": {"result": "ok"} if result else {"result": "error"},
        "result": result,
    }


async def wait_until(predicate, timeout: float = 2.0) -> bool:
    """Poll a predicate so callback-driven assertions are deterministic."""
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        if predicate():
            return True
        await asyncio.sleep(0.01)
    return predicate()


# ---------------------------------------------------------------------------
# Message serialization
# ---------------------------------------------------------------------------


def test_goal_pose_serialization_at_yaw_zero():
    pose = build_goal_pose(1.25, -2.5, 0.0, frame_id="map", stamp={"sec": 7, "nanosec": 1})
    assert pose["header"]["frame_id"] == "map"
    assert pose["header"]["stamp"] == {"sec": 7, "nanosec": 1}
    assert pose["pose"]["position"] == {"x": 1.25, "y": -2.5, "z": 0.0}
    assert pose["pose"]["orientation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def test_goal_pose_serialization_at_quarter_turn():
    """yaw = pi/2 -> orientation z = sin(pi/4), w = cos(pi/4)."""
    orientation = quaternion_from_yaw(math.pi / 2)
    assert orientation["z"] == pytest.approx(math.sqrt(2) / 2)
    assert orientation["w"] == pytest.approx(math.sqrt(2) / 2)
    assert yaw_from_quaternion(**orientation) == pytest.approx(math.pi / 2)


def test_goal_pose_stamp_defaults_to_zero():
    pose = build_goal_pose(0.0, 0.0)
    assert pose["header"]["stamp"] == {"sec": 0, "nanosec": 0}


def test_goal_pose_yaw_round_trip_over_wrapped_angles():
    for yaw in (0.0, 0.3, math.pi / 2, math.pi, -2.5, -math.pi):
        assert yaw_from_quaternion(**quaternion_from_yaw(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_navigate_to_pose_goal_has_pose_and_default_behavior_tree():
    goal = build_navigate_to_pose_goal(2.0, 3.0, 0.5)
    assert goal["behavior_tree"] == ""
    assert goal["pose"]["pose"]["position"]["x"] == 2.0
    assert goal["pose"]["pose"]["position"]["y"] == 3.0


def test_send_action_goal_op_serialization():
    op = build_send_action_goal("g1", "/husky1/navigate_to_pose", {"pose": {}}, feedback=True)
    assert op == {
        "op": "send_action_goal",
        "id": "g1",
        "action": "/husky1/navigate_to_pose",
        "action_type": NAVIGATE_TO_POSE_TYPE,
        "args": {"pose": {}},
        "feedback": True,
    }
    assert NAVIGATE_TO_POSE_TYPE == "nav2_msgs/action/NavigateToPose"


def test_subscribe_and_unsubscribe_ops():
    sub = build_subscribe("/husky1/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", "s1")
    assert sub == {
        "op": "subscribe", "topic": "/husky1/amcl_pose",
        "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "id": "s1",
    }
    unsub = build_unsubscribe("/husky1/amcl_pose", "s1")
    assert unsub == {"op": "unsubscribe", "topic": "/husky1/amcl_pose", "id": "s1"}


def test_cancel_action_goal_op():
    op = build_cancel_action_goal("g9", "/husky2/navigate_to_pose")
    assert op == {"op": "cancel_action_goal", "id": "g9", "action": "/husky2/navigate_to_pose"}


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def test_parse_pose_extracts_xy_and_yaw():
    x, y, yaw = parse_pose(pose_message(10.0, 20.0, math.pi / 2))
    assert (x, y) == (10.0, 20.0)
    assert yaw == pytest.approx(math.pi / 2)


def test_parse_header_extracts_sim_time_and_node():
    sim_time, node = parse_header(heartbeat_message(42.75, "junction_A_1_1"))
    assert sim_time == pytest.approx(42.75)
    assert node == "junction_A_1_1"


def test_parse_header_defaults_without_stamp():
    assert parse_header({"frame_id": "n_a"}) == (0.0, "n_a")


# ---------------------------------------------------------------------------
# ROS 2 GoalStatus binding (action_msgs/msg/GoalStatus)
# ---------------------------------------------------------------------------


def test_goal_status_constants_match_ros2_humble():
    """Plan 5.4 numbers, verified against the ROS 2 Humble message definition."""
    assert GoalStatus.STATUS_SUCCEEDED == 4
    assert GoalStatus.STATUS_ABORTED == 6


def test_goal_status_full_enum_ordering():
    assert [GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED] == [0, 1, 2, 3, 4, 5, 6]


def test_goal_status_matches_action_msgs_if_ros_available():
    """Cross-check the mirror against the real enum when a ROS runtime exists.

    Skipped on the plain backend host (no action_msgs); run inside the ROS 2
    container to prove the binding.
    """
    pytest.importorskip("action_msgs.msg")
    from action_msgs.msg import GoalStatus as RealGoalStatus

    for name in ("STATUS_UNKNOWN", "STATUS_ACCEPTED", "STATUS_EXECUTING", "STATUS_CANCELING",
                 "STATUS_SUCCEEDED", "STATUS_CANCELED", "STATUS_ABORTED"):
        assert getattr(GoalStatus, name) == getattr(RealGoalStatus, name)


def test_navigation_result_semantics():
    succeeded = NavigationResult(goal_id="g", action="/a", status=4, success=True)
    aborted = NavigationResult(goal_id="g", action="/a", status=6, success=False)
    assert succeeded.success and not succeeded.aborted
    assert aborted.aborted and not aborted.success


# ---------------------------------------------------------------------------
# Dispatch flow over the mocked WebSocket
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_subscribe_ops_sent_on_connect(client, fake_ws):
    await client.subscribe_pose("husky1")
    await client.subscribe_heartbeat("husky1")

    assert fake_ws.sent[0]["op"] == "subscribe"
    assert fake_ws.sent[0]["topic"] == "/husky1/amcl_pose"
    assert fake_ws.sent[1]["topic"] == "/husky1/heartbeat"
    assert fake_ws.sent[1]["type"] == "std_msgs/msg/Header"
    assert set(client._subscriptions) == {"/husky1/amcl_pose", "/husky1/heartbeat"}


@pytest.mark.asyncio
async def test_heartbeat_and_pose_messages_update_state(client, fake_ws):
    await client.subscribe_pose("husky1")
    await client.subscribe_heartbeat("husky1")

    fake_ws.send_frame({"op": "publish", "topic": "/husky1/heartbeat",
                        "msg": heartbeat_message(12.5, "n_b")})
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(1.0, 2.0, 0.5)})
    await wait_until(lambda: "husky1" in client.heartbeats and "husky1" in client.poses)

    assert client.heartbeats["husky1"] == {"sim_time": pytest.approx(12.5), "node": "n_b"}
    x, y, yaw = client.poses["husky1"]
    assert (x, y) == (1.0, 2.0)
    assert yaw == pytest.approx(0.5)


@pytest.mark.asyncio
async def test_heartbeat_callback_receives_sim_time_and_merged_pose(client, fake_ws):
    """Gap 2: on_heartbeat gets (robot_id, sim_time, x, y, yaw) per heartbeat."""
    beats = []

    async def on_heartbeat(robot_id, sim_time, x, y, yaw):
        beats.append((robot_id, sim_time, x, y, yaw))

    client.on_heartbeat = on_heartbeat

    # First beat before any AMCL pose: coordinates are None
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/heartbeat",
                        "msg": heartbeat_message(12.5, "n_b")})
    assert await wait_until(lambda: len(beats) == 1)
    assert beats[0] == ("husky1", pytest.approx(12.5), None, None, None)

    # After a pose, the next beat carries the merged AMCL coordinates
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(1.0, 2.0, 0.5)})
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/heartbeat",
                        "msg": heartbeat_message(13.0, "n_b")})
    assert await wait_until(lambda: len(beats) == 2)
    assert beats[1] == ("husky1", pytest.approx(13.0), 1.0, 2.0, pytest.approx(0.5))


@pytest.mark.asyncio
async def test_dispatch_success_status_4_advances_task(client, fake_ws):
    """STATUS_SUCCEEDED = 4: success result + on_success (advance) callback."""
    advanced = []
    client.on_success = lambda ns, result: advanced.append((ns, result))

    task = asyncio.create_task(
        client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, yaw=0.0, timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    sent = fake_ws.sent[0]
    assert sent["op"] == "send_action_goal"
    assert sent["action"] == "/husky1/navigate_to_pose"
    assert sent["action_type"] == NAVIGATE_TO_POSE_TYPE
    assert sent["args"]["pose"]["pose"]["position"] == {"x": 3.0, "y": 4.0, "z": 0.0}

    fake_ws.send_frame(action_result(sent["id"], GoalStatus.STATUS_SUCCEEDED))
    result = await task

    assert result.success is True
    assert result.status == GoalStatus.STATUS_SUCCEEDED == 4
    assert result.aborted is False
    assert advanced == [("husky1", result)], "STATUS_SUCCEEDED must advance the task"
    assert client.failures.empty()


@pytest.mark.asyncio
async def test_dispatch_aborted_status_6_emits_failure_context(client, fake_ws):
    """STATUS_ABORTED = 6: failure result + FailureContext for the Supervisory Layer."""
    contexts = []

    async def on_failure(context):
        contexts.append(context)

    client.on_failure = on_failure

    task = asyncio.create_task(
        client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_ABORTED))
    result = await task

    assert result.success is False
    assert result.aborted is True
    assert result.status == GoalStatus.STATUS_ABORTED == 6

    context = await asyncio.wait_for(client.failures.get(), 2.0)
    assert context.robot_id == "husky1"
    assert context.status == GoalStatus.STATUS_ABORTED
    assert context.reason
    assert context.to_dict()["status"] == 6
    assert await wait_until(lambda: len(contexts) == 1), "on_failure callback must fire"
    assert contexts[0] is context


@pytest.mark.asyncio
async def test_dispatch_canceled_status_5_is_not_supervisory_failure(client, fake_ws):
    """Only STATUS_ABORTED (6) is supervisory material; CANCELED (5) is not."""
    task = asyncio.create_task(
        client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_CANCELED))
    result = await task

    assert result.success is False
    assert result.aborted is False
    assert client.failures.empty()


@pytest.mark.asyncio
async def test_dispatch_timeout_cancels_goal_before_returning(client, fake_ws):
    """Gap 4: a timed-out goal is canceled in Nav2 - no ghost AMR keeps moving."""
    result = await client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, timeout=0.05)

    assert result.timed_out is True
    assert result.status is None
    assert result.success is False
    assert client.failures.empty()

    # The last op on the wire is the cancel for exactly this goal
    assert fake_ws.sent[-1] == {
        "op": "cancel_action_goal", "id": result.goal_id,
        "action": "/husky1/navigate_to_pose",
    }
    assert [message["op"] for message in fake_ws.sent] == [
        "send_action_goal", "cancel_action_goal",
    ]


@pytest.mark.asyncio
async def test_dispatch_success_sends_no_cancel(client, fake_ws):
    """A normally finishing goal must not be canceled."""
    task = asyncio.create_task(
        client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))

    result = await task
    assert result.success is True
    assert all(message["op"] != "cancel_action_goal" for message in fake_ws.sent)


@pytest.mark.asyncio
async def test_segment_lease_renewed_while_goal_in_flight(fake_ws):
    """Gap 5: the corridor lease rolls forward during traversal, then stops."""
    lease_redis = FakeLeaseRedis()
    lease = SegmentLease(
        robot_id="husky1", edge_keys=["lock:edge:a_b"], junction_node="n_b"
    )

    async with connected_client(redis_client=lease_redis, lease_renew_interval=0.05) as c:
        task = asyncio.create_task(
            c.dispatch_navigate_to_pose(
                "husky1", 3.0, 4.0, timeout=5.0, segment_lease=lease
            )
        )
        await wait_until(lambda: len(fake_ws.sent) == 1)
        assert await wait_until(lambda: len(lease_redis.renewals) >= 1)
        robot_id, edge_keys, ttl, junction = lease_redis.renewals[0]
        assert robot_id == "husky1"
        assert edge_keys == ["lock:edge:a_b"]
        assert ttl == pytest.approx(settings.SEGMENT_LOCK_TTL_S)
        assert junction == "n_b"

        fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
        result = await task
        assert result.success is True

        count_after = len(lease_redis.renewals)
        await asyncio.sleep(0.15)          # three more renewal intervals
        assert len(lease_redis.renewals) == count_after, "renewals must stop"


@pytest.mark.asyncio
async def test_lease_renewal_errors_do_not_break_dispatch(fake_ws):
    """A Redis error mid-traversal is logged, never fatal to the goal."""

    class FailingLeaseRedis:
        def __init__(self):
            self.calls = 0

        async def renew_segment_lease(self, *args, **kwargs):
            self.calls += 1
            raise RedisError("connection reset")

    lease_redis = FailingLeaseRedis()
    lease = SegmentLease(robot_id="husky1", edge_keys=["lock:edge:a_b"])

    async with connected_client(redis_client=lease_redis, lease_renew_interval=0.05) as c:
        task = asyncio.create_task(
            c.dispatch_navigate_to_pose(
                "husky1", 3.0, 4.0, timeout=5.0, segment_lease=lease
            )
        )
        await wait_until(lambda: len(fake_ws.sent) == 1)
        assert await wait_until(lambda: lease_redis.calls >= 1)

        fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
        result = await task
        assert result.success is True, "Redis trouble must not fail the navigation"


@pytest.mark.asyncio
async def test_dispatch_rosbridge_side_failure_maps_to_unknown(client, fake_ws):
    """result: False from rosbridge (e.g. unknown action type) -> STATUS_UNKNOWN."""
    task = asyncio.create_task(
        client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_UNKNOWN,
                                     result=False))
    result = await task

    assert result.success is False
    assert result.status == GoalStatus.STATUS_UNKNOWN
    assert client.failures.empty(), "rosbridge-side errors are not Nav2 aborts"


@pytest.mark.asyncio
async def test_goal_stamp_uses_heartbeat_sim_time(client, fake_ws):
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/heartbeat",
                        "msg": heartbeat_message(12.5, "n_b")})
    await wait_until(lambda: "husky1" in client.heartbeats)

    task = asyncio.create_task(
        client.dispatch_navigate_to_pose("husky1", 3.0, 4.0, timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    stamp = fake_ws.sent[0]["args"]["pose"]["header"]["stamp"]
    assert stamp == {"sec": 12, "nanosec": 500_000_000}

    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
    await task


@pytest.mark.asyncio
async def test_dispatch_to_node_sets_junction_target(client, fake_ws):
    """dispatch_navigate_to_node drives to the node's graph coordinates."""
    task = asyncio.create_task(
        client.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    sent = fake_ws.sent[0]
    assert sent["args"]["pose"]["pose"]["position"] == {"x": 3.0, "y": 4.0, "z": 0.0}
    assert client._targets["husky1"] == "n_b"

    fake_ws.send_frame(action_result(sent["id"], GoalStatus.STATUS_SUCCEEDED))
    result = await task
    assert result.success is True


@pytest.mark.asyncio
async def test_node_goal_faces_along_travel_edge(client, fake_ws):
    """Gap 6: yaw = atan2(dy, dx) of the traversed edge, from the robot's pose."""
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(0.0, 0.0, 0.0)})
    await wait_until(lambda: "husky1" in client.poses)

    task = asyncio.create_task(
        client.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    orientation = fake_ws.sent[0]["args"]["pose"]["pose"]["orientation"]
    # Edge n_a(0,0) -> n_b(3,4): heading atan2(4, 3); half-angle sin = 1/sqrt(5)
    assert orientation["z"] == pytest.approx(1.0 / math.sqrt(5))
    assert orientation["w"] == pytest.approx(2.0 / math.sqrt(5))
    assert yaw_from_quaternion(**orientation) == pytest.approx(math.atan2(4.0, 3.0))

    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
    result = await task
    assert result.success is True


@pytest.mark.asyncio
async def test_node_goal_heading_prefers_explicit_source_node(client, fake_ws):
    """Gap 6: source_node (the edge being traversed) wins over the raw pose."""
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(10.0, -10.0, 0.0)})
    await wait_until(lambda: "husky1" in client.poses)

    task = asyncio.create_task(
        client.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0, source_node="n_a")
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    orientation = fake_ws.sent[0]["args"]["pose"]["pose"]["orientation"]
    # Heading along n_a(0,0) -> n_b(3,4), NOT from the pose at (10, -10)
    assert orientation["z"] == pytest.approx(1.0 / math.sqrt(5))
    assert orientation["w"] == pytest.approx(2.0 / math.sqrt(5))

    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
    await task


@pytest.mark.asyncio
async def test_node_goal_heading_defaults_to_zero_without_start(client, fake_ws):
    """Gap 6: no pose and no source_node -> yaw 0.0 (Nav2-safe fallback)."""
    task = asyncio.create_task(
        client.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)
    orientation = fake_ws.sent[0]["args"]["pose"]["pose"]["orientation"]
    assert orientation == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
    await task


@pytest.mark.asyncio
async def test_dispatch_to_unknown_node_raises(client):
    with pytest.raises(KeyError, match="Unknown topological node"):
        await client.dispatch_navigate_to_node("husky1", "no_such_node", timeout=1.0)


@pytest.mark.asyncio
async def test_cancel_action_goal_op_sent(client, fake_ws):
    await client.cancel_navigate("g42", "husky1")
    assert fake_ws.last_send() == {
        "op": "cancel_action_goal", "id": "g42", "action": "/husky1/navigate_to_pose"
    }


# ---------------------------------------------------------------------------
# Junction arrival detection (amcl_pose within 0.5 m)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_arrival_distance_and_at_junction(client, fake_ws):
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(2.7, 4.0)})   # 0.3 m from n_b (3, 4)
    await wait_until(lambda: "husky1" in client.poses)

    assert client.arrival_distance("husky1", "n_b") == pytest.approx(0.3)
    assert client.at_junction("husky1", "n_b") is True
    assert client.arrival_distance("husky1", "n_a") == pytest.approx(math.dist((2.7, 4.0), (0.0, 0.0)))
    assert client.at_junction("husky1", "n_a") is False


@pytest.mark.asyncio
async def test_arrival_distance_none_without_pose_or_unknown_node(client):
    assert client.arrival_distance("husky1", "n_b") is None
    assert client.at_junction("husky1", "n_b") is False
    assert client.node_xy("missing") is None


@pytest.mark.asyncio
async def test_arrival_event_fires_within_threshold(client, fake_ws):
    """A pose < 0.5 m from the goal node records one junction arrival."""
    task = asyncio.create_task(
        client.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)

    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(2.55, 4.0)})          # 0.45 m: arrival
    assert await wait_until(lambda: client.arrived.get("husky1") == "n_b")
    namespace, node = await asyncio.wait_for(client.arrivals.get(), 2.0)
    assert (namespace, node) == ("husky1", "n_b")

    # A second pose inside the threshold must not duplicate the arrival
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(2.6, 4.0)})
    await asyncio.sleep(0.05)
    assert client.arrivals.empty()

    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
    result = await task
    assert result.success is True


@pytest.mark.asyncio
async def test_arrival_callback_fires_for_segment_lock_release(fake_ws):
    """Gap 3: on_arrival(robot_id, node_id) when the pose lands within 0.5 m."""
    arrivals = []

    async def on_arrival(robot_id, node_id):
        arrivals.append((robot_id, node_id))

    async with connected_client(on_arrival=on_arrival) as c:
        task = asyncio.create_task(
            c.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0)
        )
        await wait_until(lambda: len(fake_ws.sent) == 1)

        fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                            "msg": pose_message(2.6, 4.0)})     # 0.4 m from n_b
        assert await wait_until(lambda: arrivals == [("husky1", "n_b")])

        # Deduped: a second in-threshold pose must not re-fire the callback
        fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                            "msg": pose_message(2.7, 4.0)})
        await asyncio.sleep(0.05)
        assert arrivals == [("husky1", "n_b")]

        fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
        result = await task
        assert result.success is True


@pytest.mark.asyncio
async def test_no_arrival_event_beyond_threshold(client, fake_ws):
    task = asyncio.create_task(
        client.dispatch_navigate_to_node("husky1", "n_b", timeout=5.0)
    )
    await wait_until(lambda: len(fake_ws.sent) == 1)

    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": pose_message(2.0, 4.0)})           # 1.0 m away
    await asyncio.sleep(0.05)
    assert client.arrivals.empty()
    assert client.arrived.get("husky1") is None

    fake_ws.send_frame(action_result(fake_ws.sent[0]["id"], GoalStatus.STATUS_SUCCEEDED))
    await task


@pytest.mark.asyncio
async def test_arrival_threshold_is_half_a_meter():
    assert ARRIVAL_THRESHOLD_M == 0.5


# ---------------------------------------------------------------------------
# Robustness
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_malformed_and_foreign_messages_are_ignored(client, fake_ws):
    fake_ws.feed("not json at all")
    fake_ws.send_frame({"op": "publish", "topic": "/amcl_pose",           # no namespace
                        "msg": pose_message(1.0, 1.0)})
    fake_ws.send_frame({"op": "publish", "topic": "/husky1/amcl_pose",
                        "msg": {"pose": {"pose": {}}}})                   # malformed
    fake_ws.send_frame({"op": "some_unknown_op", "data": 1})
    await asyncio.sleep(0.05)

    assert client.poses == {}
    assert client.heartbeats == {}
    assert fake_ws.closed is False, "the receive loop must survive bad frames"


@pytest.mark.asyncio
async def test_send_without_connection_raises():
    c = RosBridgeClient(url="ws://test:9090")
    with pytest.raises(RuntimeError, match="not connected"):
        await c._send({"op": "subscribe", "topic": "/t", "type": "std_msgs/msg/Header"})


@pytest.mark.asyncio
async def test_disconnect_closes_websocket_and_stops_loop(client, fake_ws):
    assert client.connected
    await client.disconnect()
    assert client.connected is False
    assert fake_ws.closed is True
    assert client._subscriptions == {}

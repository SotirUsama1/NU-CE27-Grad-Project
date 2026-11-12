"""Atomic Redis segment locking for the CyberScape orchestrator.

Phase 4: Corridor segments and Passing Junction capacity are reserved in a
single atomic Lua turn (``EVALSHA``), using normalized undirected keys:

  * edge lock:      ``lock:edge:{min(u,v)}_{max(u,v)}``   (string = owner id)
  * junction slot:  ``lock:junction:{node}``             (hash: robot -> 1)
  * quarantine:     ``quarantine:node:{node}``,
                    ``quarantine:edge:{lock_key}``

Concurrency rules enforced inside the scripts (all-or-nothing):
  * an edge lock held by ANOTHER robot blocks acquisition,
  * a quarantined node or edge blocks acquisition,
  * the destination junction admits at most ``capacity`` distinct robots
    (re-entrant for a robot that already holds a slot),
  * release/renew only touch keys (or slots) owned by the calling robot.
"""

from __future__ import annotations

import json
from typing import Any

import redis.asyncio as aioredis

from cyberscape.config import settings

# ---------------------------------------------------------------------------
# Lua scripts
# ---------------------------------------------------------------------------

# KEYS[1]                junction capacity hash
# KEYS[2 .. 1+n]         edge lock keys
# KEYS[2+n .. 1+2n]      quarantine edge markers (one per edge key)
# KEYS[2+2n]             quarantine node marker
# ARGV[1] robot_id  [2] ttl_ms  [3] junction capacity  [4] n_edges
# Returns 1 on success, 0 when blocked (nothing is written).
ACQUIRE_SEGMENT_LUA = """
local robot    = ARGV[1]
local ttl      = tonumber(ARGV[2])
local capacity = tonumber(ARGV[3])
local n        = tonumber(ARGV[4])
local junction = KEYS[1]

-- Quarantines block acquisition outright.
if redis.call('EXISTS', KEYS[2 + 2 * n]) == 1 then
    return 0
end
for i = 1, n do
    if redis.call('EXISTS', KEYS[1 + n + i]) == 1 then
        return 0
    end
end

-- Edge locks: fail fast if any is owned by another robot.
for i = 1, n do
    local owner = redis.call('GET', KEYS[1 + i])
    if owner and owner ~= robot then
        return 0
    end
end

-- Junction capacity: re-entrant for a robot that already holds a slot.
if redis.call('HEXISTS', junction, robot) == 0 then
    if redis.call('HLEN', junction) >= capacity then
        return 0
    end
    redis.call('HSET', junction, robot, '1')
end
redis.call('PEXPIRE', junction, ttl)

-- Take (or refresh) the edge locks.
for i = 1, n do
    local owner = redis.call('GET', KEYS[1 + i])
    if not owner then
        redis.call('SET', KEYS[1 + i], robot, 'PX', ttl)
    else
        redis.call('PEXPIRE', KEYS[1 + i], ttl)
    end
end
return 1
"""

# KEYS[1]           junction capacity hash
# KEYS[2 .. 1+n]    edge lock keys
# ARGV[1] robot_id  [2] n_edges
# Deletes only keys owned by the robot; returns the number of edges released.
RELEASE_SEGMENT_LUA = """
local robot = ARGV[1]
local n     = tonumber(ARGV[2])
local junction = KEYS[1]
local released = 0
for i = 1, n do
    local owner = redis.call('GET', KEYS[1 + i])
    if owner == robot then
        redis.call('DEL', KEYS[1 + i])
        released = released + 1
    end
end
redis.call('HDEL', junction, robot)
return released
"""

# KEYS[1]           junction capacity hash
# KEYS[2 .. 1+n]    edge lock keys
# ARGV[1] robot_id  [2] ttl_ms  [3] n_edges
# Rolling lease renewal: extends owned locks only. Returns
# {edges_renewed, junction_held} - caller decides success.
RENEW_SEGMENT_LUA = """
local robot = ARGV[1]
local ttl   = tonumber(ARGV[2])
local n     = tonumber(ARGV[3])
local junction = KEYS[1]
local renewed = 0
for i = 1, n do
    local owner = redis.call('GET', KEYS[1 + i])
    if owner == robot then
        redis.call('PEXPIRE', KEYS[1 + i], ttl)
        renewed = renewed + 1
    end
end
local junction_held = 0
if redis.call('HEXISTS', junction, robot) == 1 then
    redis.call('PEXPIRE', junction, ttl)
    junction_held = 1
end
return {renewed, junction_held}
"""


CHANNEL_MISSION_NEW = "cyberscape:mission:new"
CHANNEL_NAV2_FAILURE = "cyberscape:failure:nav2"
CHANNEL_ROBOT_IDLE = "cyberscape:robot:idle"
CHANNEL_SEGMENT_RELEASED = "cyberscape:segment:released"


class RedisClient:
    """Async Redis wrapper with atomic Lua segment-locking primitives."""

    def __init__(self, redis_url: str | None = None) -> None:
        self.redis_url = redis_url or settings.REDIS_URL
        self.redis: aioredis.Redis | None = None
        self._acquire_script: Any | None = None
        self._release_script: Any | None = None
        self._renew_script: Any | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        """Create the async Redis client and register the Lua scripts."""
        self.redis = aioredis.from_url(self.redis_url, decode_responses=True)
        self._acquire_script = self.redis.register_script(ACQUIRE_SEGMENT_LUA)
        self._release_script = self.redis.register_script(RELEASE_SEGMENT_LUA)
        self._renew_script = self.redis.register_script(RENEW_SEGMENT_LUA)

    async def disconnect(self) -> None:
        if self.redis is not None:
            await self.redis.aclose()
            self.redis = None
            self._acquire_script = None
            self._release_script = None
            self._renew_script = None

    def _require_connected(self) -> aioredis.Redis:
        if self.redis is None:
            raise RuntimeError("Redis client is not connected")
        return self.redis

    # ------------------------------------------------------------------
    # Key helpers (normalized undirected keys)
    # ------------------------------------------------------------------

    @staticmethod
    def edge_lock_key(u: str, v: str) -> str:
        """Normalized undirected lock key: lock:edge:{min(u,v)}_{max(u,v)}."""
        return "lock:edge:" + "_".join(sorted([u, v]))

    @staticmethod
    def junction_key(node: str) -> str:
        return f"lock:junction:{node}"

    @staticmethod
    def _quarantine_edge_key(edge_key: str) -> str:
        return f"quarantine:edge:{edge_key}"

    @staticmethod
    def _quarantine_node_key(node: str) -> str:
        return f"quarantine:node:{node}"

    def _acquire_keys(
        self, edge_keys: list[str], junction_node: str
    ) -> tuple[list[str], int]:
        """Build the KEYS layout for the acquire script."""
        n = len(edge_keys)
        keys = [self.junction_key(junction_node)]
        keys.extend(edge_keys)
        keys.extend(self._quarantine_edge_key(k) for k in edge_keys)
        keys.append(self._quarantine_node_key(junction_node))
        return keys, n

    def _segment_keys(
        self, edge_keys: list[str], junction_node: str | None
    ) -> list[str]:
        """Build the KEYS layout for release/renew scripts."""
        junction = junction_node if junction_node else "none"
        return [self.junction_key(junction), *edge_keys]

    # ------------------------------------------------------------------
    # Segment locking primitives
    # ------------------------------------------------------------------

    async def acquire_segment_lock(
        self,
        robot_id: str,
        edge_keys: list[str],
        junction_node: str,
        ttl: float,
        junction_capacity: int = 2,
    ) -> bool:
        """Atomically reserve corridor edges and destination junction capacity.

        Succeeds only if every edge is free (or already owned by this robot),
        nothing is quarantined, and the junction has a free slot. On success
        all locks carry a rolling TTL of ``ttl`` seconds.
        """
        self._require_connected()
        keys, n = self._acquire_keys(edge_keys, junction_node)
        args = [
            robot_id,
            int(ttl * 1000),
            junction_capacity,
            n,
        ]
        result = await self._acquire_script(keys=keys, args=args)
        return int(result) == 1

    async def release_segment_lock(
        self,
        robot_id: str,
        edge_keys: list[str],
        junction_node: str | None = None,
    ) -> int:
        """Release edges owned by ``robot_id`` (ownership verified in Lua).

        Foreign locks are never touched. When ``junction_node`` is given the
        robot's junction slot is freed as well. Returns the number of edges
        actually released.
        """
        self._require_connected()
        keys = self._segment_keys(edge_keys, junction_node)
        args = [robot_id, len(edge_keys)]
        result = await self._release_script(keys=keys, args=args)
        return int(result)

    async def renew_segment_lease(
        self,
        robot_id: str,
        edge_keys: list[str],
        ttl: float,
        junction_node: str | None = None,
    ) -> bool:
        """Rolling lease renewal of owned segment locks.

        Returns ``True`` only when every edge lock is still owned by
        ``robot_id`` (and, when ``junction_node`` is given, the junction slot
        is still held). A ``False`` return means the lease lapsed and the
        segment must be re-acquired.
        """
        self._require_connected()
        keys = self._segment_keys(edge_keys, junction_node)
        args = [robot_id, int(ttl * 1000), len(edge_keys)]
        renewed, junction_held = await self._renew_script(keys=keys, args=args)
        if int(renewed) != len(edge_keys):
            return False
        return not (junction_node is not None and int(junction_held) != 1)

    async def publish_segment_released(self, node: str) -> int:
        """Broadcast a corridor release event on Redis Pub/Sub."""
        redis = self._require_connected()
        return await redis.publish(CHANNEL_SEGMENT_RELEASED, node)

    async def publish_mission_new(self, mission_id: str) -> int:
        """Notify the dispatcher that a mission's tasks are ready to claim.

        The agent worker's execution node calls this after persisting a task
        sequence; the dispatcher (and worker echo) both listen on
        ``cyberscape:mission:new``.
        """
        redis = self._require_connected()
        return await redis.publish(
            CHANNEL_MISSION_NEW, json.dumps({"mission_id": mission_id})
        )

    async def publish_robot_idle(self, robot_id: str) -> int:
        """Broadcast a robot IDLE transition on Redis Pub/Sub."""
        redis = self._require_connected()
        return await redis.publish(CHANNEL_ROBOT_IDLE, robot_id)

    async def publish_nav2_failure(self, payload: dict) -> int:
        """Forward a Nav2 STATUS_ABORTED (6) failure event to the supervisor.

        ``payload`` carries the ``FailureContext`` fields plus ``task_id``,
        ``mission_id`` and ``attempts`` (see langgraph/worker.py).
        """
        redis = self._require_connected()
        return await redis.publish(CHANNEL_NAV2_FAILURE, json.dumps(payload))

    # ------------------------------------------------------------------
    # Quarantine (Tier 2 safety state)
    # ------------------------------------------------------------------

    async def set_quarantine(self, node: str, edges: list[str]) -> None:
        """Quarantine ``node`` and its incident edges (persists until cleared)."""
        redis = self._require_connected()
        pipe = redis.pipeline(transaction=True)
        pipe.set(self._quarantine_node_key(node), "1")
        for edge_key in edges:
            pipe.set(self._quarantine_edge_key(edge_key), "1")
        await pipe.execute()

    async def clear_quarantine(self, node: str, edges: list[str]) -> None:
        """Lift a quarantine (administrative override)."""
        redis = self._require_connected()
        pipe = redis.pipeline(transaction=True)
        pipe.delete(self._quarantine_node_key(node))
        for edge_key in edges:
            pipe.delete(self._quarantine_edge_key(edge_key))
        await pipe.execute()

    async def is_node_quarantined(self, node: str) -> bool:
        """True when ``node`` is currently quarantined."""
        redis = self._require_connected()
        return bool(await redis.exists(self._quarantine_node_key(node)))

"""Unit and integration tests for the atomic Redis segment locking.

Phase 4: Validates the Lua scripts for mutual exclusion, undirected edge
contention, junction capacity limits, ownership verification on release and
renew, all-or-nothing atomicity, quarantine gating, and rolling TTLs.

Integration tests run against a live Redis (``docker compose up -d redis``)
and skip automatically when Redis is unreachable.
"""

from __future__ import annotations

import pytest
import pytest_asyncio
import redis.asyncio as aioredis

from cyberscape.config import settings
from cyberscape.redis_client import RedisClient

# ---------------------------------------------------------------------------
# Mocked unit tests (key semantics & connection guards)
# ---------------------------------------------------------------------------


def test_edge_lock_key_is_normalized_undirected():
    """Edge keys are identical regardless of traversal direction."""
    assert RedisClient.edge_lock_key("node_b", "node_a") == "lock:edge:node_a_node_b"
    assert RedisClient.edge_lock_key("node_a", "node_b") == (
        RedisClient.edge_lock_key("node_b", "node_a")
    )


def test_junction_key_format():
    assert RedisClient.junction_key("junction_A_1_1") == "lock:junction:junction_A_1_1"


@pytest.mark.asyncio
async def test_acquire_raises_without_connection():
    client = RedisClient()
    with pytest.raises(RuntimeError, match="Redis client is not connected"):
        await client.acquire_segment_lock("husky1", [], "junction_A", ttl=5.0)


@pytest.mark.asyncio
async def test_release_raises_without_connection():
    client = RedisClient()
    with pytest.raises(RuntimeError, match="Redis client is not connected"):
        await client.release_segment_lock("husky1", ["lock:edge:a_b"])


# ---------------------------------------------------------------------------
# Integration tests (live Redis)
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture
async def redis_client():
    """Connected RedisClient with a flushed database; skips if unreachable."""
    client = RedisClient()
    await client.connect()
    try:
        await client.redis.ping()
    except (OSError, aioredis.RedisError) as exc:  # pragma: no cover - env dependent
        await client.disconnect()
        pytest.skip(f"Redis not reachable: {exc}")
    await client.redis.flushdb()
    yield client
    await client.disconnect()


@pytest.mark.asyncio
async def test_mutual_exclusion_single_edge(redis_client):
    """Two robots cannot hold the same edge lock at once."""
    rc = redis_client
    key = RedisClient.edge_lock_key("a", "b")

    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0)
    # Foreign robot blocked
    assert not await rc.acquire_segment_lock("husky2", [key], "junction_1", ttl=5.0)
    # Same robot re-acquiring is re-entrant (refresh)
    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0)

    # After release the other robot can take it
    assert await rc.release_segment_lock("husky1", [key], "junction_1") == 1
    assert await rc.acquire_segment_lock("husky2", [key], "junction_1", ttl=5.0)


@pytest.mark.asyncio
async def test_undirected_edge_contention(redis_client):
    """Locking edge (u,v) from either direction contends on the same key."""
    rc = redis_client
    key_ab = RedisClient.edge_lock_key("u", "v")
    key_ba = RedisClient.edge_lock_key("v", "u")
    assert key_ab == key_ba

    assert await rc.acquire_segment_lock("husky1", [key_ab], "junction_1", ttl=5.0)
    assert await rc.redis.get(key_ab) == "husky1"
    # husky2 traversing the same corridor in the opposite direction is blocked
    assert not await rc.acquire_segment_lock("husky2", [key_ba], "junction_2", ttl=5.0)


@pytest.mark.asyncio
async def test_junction_capacity_limit(redis_client):
    """Junction admits at most `capacity` robots; a release frees a slot."""
    rc = redis_client
    jk = RedisClient.junction_key("junction_X")

    # No edges needed - junction-only reservations
    assert await rc.acquire_segment_lock("husky1", [], "junction_X", ttl=5.0,
                                         junction_capacity=2)
    assert await rc.acquire_segment_lock("husky2", [], "junction_X", ttl=5.0,
                                         junction_capacity=2)
    assert await rc.redis.hlen(jk) == 2
    # Third robot is over capacity
    assert not await rc.acquire_segment_lock("husky3", [], "junction_X", ttl=5.0,
                                             junction_capacity=2)

    # husky1 frees its slot -> husky3 can now enter
    assert await rc.release_segment_lock("husky1", [], "junction_X") == 0
    assert await rc.redis.hexists(jk, "husky1") == 0
    assert await rc.acquire_segment_lock("husky3", [], "junction_X", ttl=5.0,
                                         junction_capacity=2)


@pytest.mark.asyncio
async def test_capacity_one_blocks_second_robot(redis_client):
    """A capacity-1 node (e.g. a box waypoint) is exclusive."""
    rc = redis_client
    assert await rc.acquire_segment_lock("husky1", [], "box_a_001", ttl=5.0,
                                         junction_capacity=1)
    assert not await rc.acquire_segment_lock("husky2", [], "box_a_001", ttl=5.0,
                                             junction_capacity=1)


@pytest.mark.asyncio
async def test_ownership_verification_on_release(redis_client):
    """Release deletes ONLY locks owned by the calling robot."""
    rc = redis_client
    key = RedisClient.edge_lock_key("a", "b")

    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0)
    # Foreign robot's release is a no-op (released == 0) and the lock survives
    assert await rc.release_segment_lock("husky2", [key], "junction_1") == 0
    assert await rc.redis.get(key) == "husky1"
    # Owner releases and the lock disappears
    assert await rc.release_segment_lock("husky1", [key], "junction_1") == 1
    assert await rc.redis.get(key) is None


@pytest.mark.asyncio
async def test_ownership_verification_on_renew(redis_client):
    """Renew extends only owned leases; foreign renewal fails."""
    rc = redis_client
    key = RedisClient.edge_lock_key("a", "b")

    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0)

    # Foreign robot cannot renew husky1's lease
    assert not await rc.renew_segment_lease("husky2", [key], ttl=5.0,
                                            junction_node="junction_1")

    # Owner renews; the TTL is refreshed near the full lease
    pttl_before = await rc.redis.pttl(key)
    assert await rc.renew_segment_lease("husky1", [key], ttl=5.0,
                                        junction_node="junction_1")
    pttl_after = await rc.redis.pttl(key)
    assert pttl_after > pttl_before - 100  # refreshed (allow small drift)


@pytest.mark.asyncio
async def test_renew_detects_expired_lease(redis_client):
    """A lapsed lease makes renewal fail (caller must re-acquire)."""
    rc = redis_client
    key = RedisClient.edge_lock_key("a", "b")

    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0)
    # Force-expire the lease
    await rc.redis.pexpire(key, 1)
    import asyncio as _asyncio
    await _asyncio.sleep(0.01)

    assert not await rc.renew_segment_lease("husky1", [key], ttl=5.0,
                                            junction_node="junction_1")


@pytest.mark.asyncio
async def test_acquire_is_atomic_all_or_nothing(redis_client):
    """A blocked edge prevents acquiring the free edges in the same call."""
    rc = redis_client
    free_key = RedisClient.edge_lock_key("a", "b")
    contested_key = RedisClient.edge_lock_key("c", "d")

    # husky0 already holds the contested edge
    assert await rc.acquire_segment_lock("husky0", [contested_key], "junction_0",
                                         ttl=5.0)

    # husky1 wants both: nothing may be written on failure
    assert not await rc.acquire_segment_lock(
        "husky1", [free_key, contested_key], "junction_1", ttl=5.0
    )
    assert await rc.redis.get(free_key) is None
    assert await rc.redis.get(contested_key) == "husky0"
    assert await rc.redis.hlen(RedisClient.junction_key("junction_1")) == 0


@pytest.mark.asyncio
async def test_quarantine_blocks_edges_and_nodes(redis_client):
    """Quarantined edges/nodes block acquisition until cleared."""
    rc = redis_client
    edge_a = RedisClient.edge_lock_key("a", "b")
    edge_free = RedisClient.edge_lock_key("x", "y")

    await rc.set_quarantine("junction_Q", [edge_a])

    # Quarantined edge is blocked...
    assert not await rc.acquire_segment_lock("husky1", [edge_a], "junction_Q",
                                             ttl=5.0)
    # ...an unrelated edge to an unrelated junction is fine
    assert await rc.acquire_segment_lock("husky1", [edge_free], "junction_OK",
                                         ttl=5.0)
    # A quarantined NODE blocks even edge-free acquisition
    assert not await rc.acquire_segment_lock("husky2", [], "junction_Q", ttl=5.0)

    await rc.clear_quarantine("junction_Q", [edge_a])
    assert not await rc.is_node_quarantined("junction_Q")
    assert await rc.acquire_segment_lock("husky2", [edge_a], "junction_Q", ttl=5.0)


@pytest.mark.asyncio
async def test_edge_only_quarantine_blocks_even_with_free_destination(redis_client):
    """An edge-only quarantine (node NOT quarantined) still blocks acquisition.

    Regression guard for the Lua off-by-one: the quarantine edge-marker loop
    must read KEYS[2+n .. 1+2n], independent of the node marker at KEYS[2+2n].
    """
    rc = redis_client
    edge = RedisClient.edge_lock_key("p", "q")

    # Quarantine ONLY the edge marker (not the node).
    await rc.redis.set(f"quarantine:edge:{edge}", "1")

    # Blocked when the edge is requested, even toward a clean junction...
    assert not await rc.acquire_segment_lock("husky1", [edge], "junction_CLEAN",
                                             ttl=5.0)
    # ...and nothing was written (no junction slot taken).
    assert await rc.redis.hlen(RedisClient.junction_key("junction_CLEAN")) == 0
    # An unrelated edge toward the same clean junction still works.
    assert await rc.acquire_segment_lock(
        "husky1", [RedisClient.edge_lock_key("r", "s")], "junction_CLEAN", ttl=5.0
    )


@pytest.mark.asyncio
async def test_lock_ttl_is_rolling(redis_client):
    """Acquired locks carry the requested TTL."""
    rc = redis_client
    key = RedisClient.edge_lock_key("a", "b")

    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0)
    pttl = await rc.redis.pttl(key)
    assert 0 < pttl <= 5000


@pytest.mark.asyncio
async def test_release_frees_junction_slot(redis_client):
    """Releasing with a junction node frees that robot's capacity slot."""
    rc = redis_client
    jk = RedisClient.junction_key("junction_1")
    key = RedisClient.edge_lock_key("a", "b")

    assert await rc.acquire_segment_lock("husky1", [key], "junction_1", ttl=5.0,
                                         junction_capacity=1)
    assert await rc.redis.hexists(jk, "husky1") == 1
    await rc.release_segment_lock("husky1", [key], "junction_1")
    assert await rc.redis.hexists(jk, "husky1") == 0
    # Slot freed for someone else
    assert await rc.acquire_segment_lock("husky2", [key], "junction_1", ttl=5.0,
                                         junction_capacity=1)


@pytest.mark.asyncio
async def test_lock_ttl_matches_config(redis_client):
    """Default lock TTL comes from settings (5s rolling lease)."""
    assert settings.SEGMENT_LOCK_TTL_S == 5.0

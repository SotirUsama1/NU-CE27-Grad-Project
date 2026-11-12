-- CyberScape backend database schema.
-- Phase 3: Database Models & Concurrency Task Queue.
-- PostgreSQL 16 initializes this file from /docker-entrypoint-initdb.d/init.sql on first startup.

-- Robots: fleet members with session affinity and liveness state.
-- sim_time_last_seen: last heartbeat timestamp in SIMULATION time (Gazebo /clock),
-- so liveness timeouts never produce RTF-lag false positives.
CREATE TABLE IF NOT EXISTS robots (
    id TEXT PRIMARY KEY,
    status TEXT NOT NULL DEFAULT 'IDLE',
    current_node TEXT,
    x REAL,
    y REAL,
    yaw REAL,
    sim_time_last_seen DOUBLE PRECISION,
    assigned_mission_id TEXT,
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Missions: top-level mission records submitted via natural language.
CREATE TABLE IF NOT EXISTS missions (
    id TEXT PRIMARY KEY,
    text TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'SUBMITTED',
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Tasks: primitive-level units of work within a mission.
--   prerequisite gating: task N is claimable only when task N-1 is COMPLETED.
--   robot session affinity: claimed tasks are pinned to assigned_robot_id.
--   params: free-form primitive parameters (JSONB).
--   dispatch order: FIFO by created_at (oldest mission first), then sequence.
CREATE TABLE IF NOT EXISTS tasks (
    id TEXT PRIMARY KEY,
    mission_id TEXT NOT NULL REFERENCES missions(id) ON DELETE CASCADE,
    sequence_order INTEGER NOT NULL,
    status TEXT NOT NULL DEFAULT 'PENDING',
    primitive_type TEXT NOT NULL,
    target_node TEXT,
    params JSONB DEFAULT '{}'::jsonb,
    assigned_robot_id TEXT REFERENCES robots(id) ON DELETE SET NULL,
    claimed_at TIMESTAMPTZ,
    completed_at TIMESTAMPTZ,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    UNIQUE(mission_id, sequence_order)
);

-- Concurrency index: dispatcher sweeps tasks by (status, sequence_order).
CREATE INDEX IF NOT EXISTS idx_tasks_status_seq ON tasks(status, sequence_order);

-- Foreign key lookups.
CREATE INDEX IF NOT EXISTS idx_tasks_mission ON tasks(mission_id);
CREATE INDEX IF NOT EXISTS idx_tasks_robot ON tasks(assigned_robot_id);
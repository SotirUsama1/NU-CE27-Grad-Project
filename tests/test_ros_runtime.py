"""Exercise the installed ROS entry point and a real Uvicorn HTTP server."""

import json
import os
from pathlib import Path
import socket
import subprocess
import time
from urllib.error import URLError
from urllib.request import urlopen

import pytest


pytest.importorskip("rclpy", reason="Run this smoke test in the ROS build job")


def test_installed_node_serves_seeded_scene(tmp_path):
    project_root = Path(__file__).resolve().parents[1]
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]

    log_path = tmp_path / "world-state-node.log"
    env = dict(os.environ, WORKSPACE_ROOT=str(project_root), ROS_LOCALHOST_ONLY="1")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [
                "ros2", "run", "semantic_world_manager", "world_state_node",
                "--ros-args", "-p", "api_host:=127.0.0.1", "-p", f"api_port:={port}",
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                try:
                    with urlopen(f"http://127.0.0.1:{port}/health", timeout=1) as response:
                        health = json.load(response)
                    break
                except (URLError, TimeoutError):
                    time.sleep(0.1)
            else:
                pytest.fail(f"HTTP server did not become ready:\n{log_path.read_text()}")

            assert health["ros_connected"] is False
            with urlopen(f"http://127.0.0.1:{port}/api/scene/summary", timeout=2) as response:
                summary = json.load(response)
            with (project_root / "maps/Our_Structured_Warehouse_semantic.json").open() as source:
                expected = json.load(source)
            assert summary["total_objects"] == len(expected)
            assert summary["status"] == "static_seeded"
        finally:
            # ros2 run starts a child; terminate the entire process group.
            import signal
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

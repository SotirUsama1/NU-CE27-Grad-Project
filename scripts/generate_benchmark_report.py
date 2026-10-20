#!/usr/bin/env python3
"""Aggregates and formats Nav2 stress test benchmark results into Markdown tables."""
import json
from pathlib import Path
import sys

def main():
    results_dir = Path("/home/ubuntu/Documents/Grad/NU-CE27-Grad-Project/Sim/ros_ws/log/nav2_benchmark/stress_results")
    if not results_dir.exists():
        print("No results directory found.")
        return

    json_files = sorted(results_dir.glob("stress_*.json"))
    if not json_files:
        print("No stress test JSON files found.")
        return

    print("# Nav2 Local Planner Stress Benchmark Report\n")
    print("| Controller | Robot | Total Legs | Passed | Pass Rate | Total Recoveries | Duration (min) |")
    print("|:-----------|:------|:-----------|:-------|:----------|:-----------------|:---------------|")

    summaries = []
    for f in json_files:
        try:
            data = json.loads(f.read_text())
            summaries.append((f.stem, data))
            ctrl = data.get('controller', 'unknown').upper()
            robot = data.get('robot', 'unknown').upper()
            total = data.get('total_legs', 0)
            passed = data.get('passed_legs', 0)
            rate = f"{data.get('pass_rate_pct', 0.0)}%"
            recov = data.get('total_recoveries', 0)
            dur = data.get('total_wall_time_mins', 0.0)
            print(f"| {ctrl} | {robot} | {total} | {passed} | {rate} | {recov} | {dur:.1f} |")
        except Exception as e:
            print(f"Error reading {f}: {e}")

    print("\n## Waypoint-by-Waypoint Breakdown\n")
    for name, data in summaries:
        ctrl = data.get('controller', 'unknown').upper()
        robot = data.get('robot', 'unknown').upper()
        print(f"### Controller: {ctrl} ({robot})\n")
        print("| Lap | Waypoint | Status | Time (s) | Recoveries | GT Error XY (m) |")
        print("|:---|:---------|:-------|:---------|:-----------|:----------------|")
        for r in data.get('results', []):
            lap = r.get('lap', '-')
            wp = r.get('waypoint', '-')
            st = "PASS" if r.get('pass') else "**FAIL**"
            elapsed = r.get('elapsed_s', '-')
            rec = r.get('recoveries', 0)
            xy = r.get('gt_error_xy', '-')
            print(f"| {lap} | {wp} | {st} | {elapsed} | {rec} | {xy} |")
        print("\n")

if __name__ == '__main__':
    main()

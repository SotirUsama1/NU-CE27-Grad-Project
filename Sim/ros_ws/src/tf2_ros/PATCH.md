# Patched tf2_ros (warehouse project)

This folder is ROS 2 Humble's `tf2_ros` **0.25.23** (from https://github.com/ros2/geometry2, tag `0.25.23`,
the same version as in our Docker image) with **one bug fixed** in `src/buffer.cpp`. Because it is built in
our workspace, every node started after `source /main/ros_ws/install/setup.bash` (AMCL, the Nav2 costmaps, ...)
loads this library instead of `/opt/ros/humble/lib/libtf2_ros.so`.

## The bug: a deadlock that froze AMCL

`tf2_ros::Buffer::waitForTransform` held its `timer_to_request_map_mutex_` while calling
`addTransformableRequest`, which needs BufferCore's request lock. When a transform arrives, BufferCore holds
its request lock and calls the request's callback, which takes `timer_to_request_map_mutex_`: the opposite
order. If a laser scan (AMCL's scan thread) and a TF message (the TransformListener's thread) arrive at the
same moment, each thread holds one lock and waits for the other, forever.

Seen in our simulation in about 1 of 3 robot start-ups: AMCL published `map -> odom` once and never again,
so Nav2 never got the robot's position. A `gdb` stack dump of the stuck process showed exactly these two
threads waiting on each other. The code is the same on the humble branch and in rolling (checked 2026-11-09).

## The fix

`waitForTransform` now registers the request **without** holding `timer_to_request_map_mutex_`. The race that
lock protected against (the callback running before the request's timeout timer is stored) is handled by
parking such an early answer in a small table (`g_early_results`); `waitForTransform` picks it up right after
registering. Timeouts and `cancel()` remove any parked answer, so the table cannot grow.

The table is at file scope on purpose: the size and layout of the `Buffer` class must not change, because
Nav2 and the other libraries were compiled against the original header. Headers are untouched.
All changed lines are marked `WAREHOUSE PATCH`.

## When to remove or update it

- If the Docker image moves to another tf2_ros version, replace this folder with that version's source and
  re-apply the patch (or drop it if upstream fixed the lock order). Never mix versions: the patched library
  must match the headers the rest of ROS was built with.
- To check the fix is active: `ldd /opt/ros/humble/lib/libamcl_core.so | grep tf2_ros` must point into
  `/main/ros_ws/install/tf2_ros/`.

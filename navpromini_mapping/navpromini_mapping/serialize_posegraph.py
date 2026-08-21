#!/usr/bin/env python3
"""Serialize slam_toolbox's pose graph alongside a saved map.

    ros2 run navpromini_mapping serialize_posegraph /path/to/<map_name>

Writes `<map_name>.posegraph` and `<map_name>.data` next to the map's
`.pgm`/`.yaml`, by calling slam_toolbox's `/slam_toolbox/serialize_map`.

WHY THIS EXISTS
---------------
A saved `.pgm` is a picture of the map, not the map. It cannot be extended:
once SLAM stops, the graph of scans and their relative constraints is gone,
so the only way to incorporate a changed environment is to re-map from
scratch — which produces a NEW ORIGIN, silently invalidating every pose
stored in map frame. On this robot that means the dock pose
(~/.navpromini_dock_pose.json) and every Flutter bookmark: the robot would
drive confidently to the wrong place.

Keeping the pose graph is what preserves the option of *continuing* a map
later — extending it in the same frame, so stored poses stay valid. It costs
nothing today and cannot be recovered retroactively, which is exactly why it
is worth doing now rather than when it is needed.

SAFETY CONTRACT
---------------
This is strictly additive and ALWAYS exits 0. Saving the map is the
operator's actual goal; serialization is a bonus. If slam_toolbox is not
running (e.g. the map was saved while in navigation mode), or the service
fails, this says so and exits successfully so the map save is not reported as
a failure. Never let a nice-to-have fail the thing it was attached to.
"""

from __future__ import annotations

import os
import sys

SERVICE = '/slam_toolbox/serialize_map'
WAIT_SEC = 5.0
CALL_TIMEOUT_SEC = 60.0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    args = [a for a in argv if not a.startswith('--')]
    if not args:
        print('serialize_posegraph: no output path given — nothing to do')
        return 0
    path = os.path.abspath(os.path.expanduser(args[0]))
    for ext in ('.yaml', '.pgm', '.posegraph', '.data'):
        if path.endswith(ext):
            path = path[: -len(ext)]

    try:
        import rclpy
        from rclpy.node import Node
        from slam_toolbox.srv import SerializePoseGraph
    except Exception as exc:  # noqa: BLE001
        print(f'serialize_posegraph: slam_toolbox interfaces unavailable ({exc}) '
              '— map saved, pose graph skipped')
        return 0

    rclpy.init(args=None)
    node = Node('navpromini_serialize_posegraph')
    try:
        client = node.create_client(SerializePoseGraph, SERVICE)
        if not client.wait_for_service(timeout_sec=WAIT_SEC):
            node.get_logger().info(
                f'{SERVICE} not available — slam_toolbox is probably not running '
                '(map saved from navigation mode?). Map saved, pose graph skipped.'
            )
            return 0

        req = SerializePoseGraph.Request()
        req.filename = path
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_SEC)

        if not future.done() or future.result() is None:
            node.get_logger().warn(
                'serialize_map did not return in time — map saved, pose graph skipped')
            return 0

        result = future.result()
        if getattr(result, 'result', 0) == 0:
            node.get_logger().info(
                f'Pose graph serialized: {path}.posegraph / .data — this map can '
                'be extended later in the same frame')
        else:
            node.get_logger().warn(
                f'serialize_map returned {result.result} (failed to write '
                f'{path}) — map saved, pose graph skipped')
        return 0
    except Exception as exc:  # noqa: BLE001 — must never fail the map save
        node.get_logger().warn(f'pose graph serialization failed: {exc!r}')
        return 0
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())

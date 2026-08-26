#!/usr/bin/env python3
"""Robot-side persistence for named waypoints and saved missions — the
shared, durable copy so any client (this Flutter app, a different device
running it, a future web API) sees the same data instead of each client
carrying its own local-only copy.

Was waypoint_store_node.py, handling only bookmarks — generalized to also
cover missions on the same pattern before either was ever deployed live, so
there's no migration step, just one node with two independent topics:

  /waypoints (std_msgs/String, JSON, reliable + transient_local)
      Whole-blob replace of every bookmark on every map. Shape matches
      Flutter's SettingsProvider._bookmarks field-for-field:
        {"<mapName>": [{"id": "...", "icon": 12345, "name": "...",
          "positionX": 0.0, "positionY": 0.0, "positionZ": 0.0,
          "theta": 0.0, "isGoalActive": false, "isDock": false}, ...], ...}

  /missions (std_msgs/String, JSON, reliable + transient_local)
      Whole-blob replace of every saved mission. Shape matches
      SettingsProvider._missions (keyed "<mapName>::<missionName>") with
      each value being Mission.toJson() from lib/modals/mission.dart:
        {"<mapName>::<missionName>": {"mission_name": "...",
          "mission_description": "...", "map_name": "...",
          "items": [...]}, ...}

Both topics share the same idiom as dock_manager_node's /dock_pose: a late
subscriber gets the last published value immediately (transient_local, no
polling), and publishing a new blob on the topic sets/replaces the whole
stored set for that data type. Each is deliberately whole-blob replace, not
per-item add/remove — no ordering or partial-update ambiguity to get wrong,
and the Flutter app already keeps the complete set in memory, so sending it
all every time costs nothing extra there. A finer-grained API can be added
later without changing either topic's contract if that turns out to matter
at a much larger item count.
"""

from __future__ import annotations

import json
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

WAYPOINTS_FILE = os.path.expanduser('~/.navpromini_waypoints.json')
MISSIONS_FILE = os.path.expanduser('~/.navpromini_missions.json')

# Publish side: transient_local so a client that (re)subscribes after the
# store was set still gets it without polling (same pattern as dock_pose).
_PUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Subscribe side: plain volatile — this is an incoming "set" request, not
# state a late subscriber needs replayed to it.
_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class _JsonTopicStore:
    """One persisted JSON blob synced over one topic — the shared shape
    both /waypoints and /missions use. Owns its own publisher, subscriber,
    and file; the node just instantiates one of these per data type.
    """

    def __init__(self, node: Node, topic_name: str, file_path: str, label: str) -> None:
        self._node = node
        self._file_path = file_path
        self._label = label

        self._pub = node.create_publisher(String, topic_name, _PUB_QOS)
        node.create_subscription(String, topic_name, self._on_set, _SUB_QOS)

        loaded = self._load()
        if loaded is not None:
            self._raw = loaded
            self._publish(loaded)
            node.get_logger().info(
                f'{label}: {len(json.loads(loaded))} entr(y/ies) loaded from {file_path}')
        else:
            self._raw = '{}'
            node.get_logger().info(f'{label}: no saved data yet')

    def _load(self) -> Optional[str]:
        if not os.path.exists(self._file_path):
            return None
        try:
            with open(self._file_path, 'r') as f:
                raw = f.read()
            json.loads(raw)  # validate before trusting it
            return raw
        except (OSError, ValueError) as exc:
            self._node.get_logger().warn(f'Failed to load {self._file_path}: {exc}')
            return None

    def _persist(self, raw: str) -> None:
        try:
            tmp_path = self._file_path + '.tmp'
            with open(tmp_path, 'w') as f:
                f.write(raw)
            os.replace(tmp_path, self._file_path)
        except OSError as exc:
            self._node.get_logger().warn(f'Failed to persist {self._file_path}: {exc}')

    def _publish(self, raw: str) -> None:
        self._pub.publish(String(data=raw))

    def _on_set(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            if not isinstance(parsed, dict):
                raise ValueError('top level must be a JSON object')
        except ValueError as exc:
            self._node.get_logger().warn(
                f'Rejected malformed {self._label} payload: {exc}')
            return

        if msg.data == self._raw:
            return  # no-op — avoid a needless disk write + republish loop
        self._raw = msg.data
        self._persist(msg.data)
        # Always republish: this topic has exactly one writer role
        # (whole-blob replace), so there's no "already own topic's cache"
        # subtlety to preserve — every subscriber should see every update.
        self._publish(msg.data)
        self._node.get_logger().info(f'{self._label} updated — {len(parsed)} entr(y/ies)')


class AppDataStoreNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_app_data_store')
        self._waypoints = _JsonTopicStore(
            self, 'waypoints', WAYPOINTS_FILE, 'waypoints')
        self._missions = _JsonTopicStore(
            self, 'missions', MISSIONS_FILE, 'missions')
        self.get_logger().info('app_data_store ready — /waypoints, /missions')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AppDataStoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

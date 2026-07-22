#!/usr/bin/env python3
"""Site-agnostic RMF dispenser/ingestor adapter.

Discovers workcell GUIDs from an RMF nav graph (``pickup_dispenser`` /
``dropoff_ingestor`` vertex properties) and answers the standard
``/dispenser_*`` and ``/ingestor_*`` topics.

In Gazebo deployments that already spawn TeleportDispenser / TeleportIngestor
models, disable this node — those plugins own the same topics.
"""

from __future__ import annotations

import os
import threading
import time
from collections import defaultdict
from typing import Dict, Iterable, List, Set

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult, IngestorState


def _guids_from_nav_graph(path: str) -> tuple[Set[str], Set[str]]:
    dispensers: Set[str] = set()
    ingestors: Set[str] = set()
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    levels = data.get('levels') or {}
    if not isinstance(levels, dict):
        return dispensers, ingestors
    for level in levels.values():
        if not isinstance(level, dict):
            continue
        for vertex in level.get('vertices') or []:
            # Nav graphs: [x, y, {props}]. Building YAML verts are longer.
            if not isinstance(vertex, list) or not vertex:
                continue
            props = vertex[-1] if isinstance(vertex[-1], dict) else {}
            pickup = props.get('pickup_dispenser')
            dropoff = props.get('dropoff_ingestor')
            if isinstance(pickup, str) and pickup.strip():
                dispensers.add(pickup.strip())
            if isinstance(dropoff, str) and dropoff.strip():
                ingestors.add(dropoff.strip())
    return dispensers, ingestors


class WorkcellAdapter(Node):
    def __init__(self):
        super().__init__('rmf_workcell_adapter')
        self.declare_parameter('nav_graph_file', '')
        self.declare_parameter('transfer_delay_sec', 2.0)
        self.declare_parameter('state_hz', 1.0)

        graph = self.get_parameter('nav_graph_file').value
        if not graph:
            pkg = get_package_share_directory('navpromini_rmf_sim')
            graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
        if not os.path.isfile(graph):
            raise FileNotFoundError(f'nav_graph_file not found: {graph}')

        self._dispensers, self._ingestors = _guids_from_nav_graph(graph)
        if not self._dispensers and not self._ingestors:
            self.get_logger().warn(
                f'No pickup_dispenser/dropoff_ingestor GUIDs in {graph}'
            )

        self._lock = threading.Lock()
        self._disp_queues: Dict[str, List[str]] = defaultdict(list)
        self._ing_queues: Dict[str, List[str]] = defaultdict(list)
        self._seen_disp: Set[str] = set()
        self._seen_ing: Set[str] = set()

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Fleet latches requests (TRANSIENT_LOCAL); match so late starts still
        # receive the active Loaditem / Dropoff request.
        req_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._disp_state = self.create_publisher(
            DispenserState, 'dispenser_states', latched
        )
        self._disp_result = self.create_publisher(
            DispenserResult, 'dispenser_results', 10
        )
        self._ing_state = self.create_publisher(
            IngestorState, 'ingestor_states', latched
        )
        self._ing_result = self.create_publisher(
            IngestorResult, 'ingestor_results', 10
        )
        self.create_subscription(
            DispenserRequest, 'dispenser_requests', self._on_disp, req_qos
        )
        self.create_subscription(
            IngestorRequest, 'ingestor_requests', self._on_ing, req_qos
        )
        hz = max(0.1, float(self.get_parameter('state_hz').value))
        self.create_timer(1.0 / hz, self._publish_states)
        self.get_logger().info(
            f'workcell adapter graph={graph} '
            f'dispensers={sorted(self._dispensers)} '
            f'ingestors={sorted(self._ingestors)} '
            f'delay={float(self.get_parameter("transfer_delay_sec").value):.1f}s'
        )

    def _now(self):
        return self.get_clock().now().to_msg()

    def _delay(self) -> float:
        return max(0.0, float(self.get_parameter('transfer_delay_sec').value))

    def _known(self, guid: str, known: Iterable[str]) -> bool:
        if guid in known:
            return True
        lower = {g.lower() for g in known}
        return guid.lower() in lower

    def _publish_states(self) -> None:
        now = self._now()
        with self._lock:
            disp_guids = set(self._dispensers) | set(self._disp_queues)
            ing_guids = set(self._ingestors) | set(self._ing_queues)
            disp_queues = {g: list(q) for g, q in self._disp_queues.items()}
            ing_queues = {g: list(q) for g, q in self._ing_queues.items()}

        for guid in sorted(disp_guids):
            queue = disp_queues.get(guid, [])
            msg = DispenserState()
            msg.time = now
            msg.guid = guid
            msg.mode = DispenserState.BUSY if queue else DispenserState.IDLE
            msg.request_guid_queue = queue
            msg.seconds_remaining = self._delay() if queue else 0.0
            self._disp_state.publish(msg)
        for guid in sorted(ing_guids):
            queue = ing_queues.get(guid, [])
            msg = IngestorState()
            msg.time = now
            msg.guid = guid
            msg.mode = IngestorState.BUSY if queue else IngestorState.IDLE
            msg.request_guid_queue = queue
            msg.seconds_remaining = self._delay() if queue else 0.0
            self._ing_state.publish(msg)

    def _on_disp(self, req: DispenserRequest) -> None:
        with self._lock:
            if req.request_guid in self._seen_disp:
                return
            self._seen_disp.add(req.request_guid)

        if not self._known(req.target_guid, self._dispensers):
            # Still complete: dashboard/API often uses office-demo handler names
            # (e.g. coke_dispenser) that differ from pickup_dispenser on the graph.
            self.get_logger().warn(
                f'unknown dispenser guid={req.target_guid!r} '
                f'(known={sorted(self._dispensers)}); completing anyway'
            )

        delay = self._delay()
        self.get_logger().info(
            f'dispenser {req.target_guid} request={req.request_guid} '
            f'— ACK then SUCCESS in {delay:.1f}s'
        )
        with self._lock:
            self._disp_queues[req.target_guid].append(req.request_guid)

        ack = DispenserResult()
        ack.time = self._now()
        ack.request_guid = req.request_guid
        ack.source_guid = req.target_guid
        ack.status = DispenserResult.ACKNOWLEDGED
        self._disp_result.publish(ack)

        def _finish():
            time.sleep(delay)
            out = DispenserResult()
            out.time = self._now()
            out.request_guid = req.request_guid
            out.source_guid = req.target_guid
            out.status = DispenserResult.SUCCESS
            self._disp_result.publish(out)
            with self._lock:
                q = self._disp_queues.get(req.target_guid, [])
                if req.request_guid in q:
                    q.remove(req.request_guid)
                if not q:
                    self._disp_queues.pop(req.target_guid, None)

        threading.Thread(target=_finish, daemon=True).start()

    def _on_ing(self, req: IngestorRequest) -> None:
        with self._lock:
            if req.request_guid in self._seen_ing:
                return
            self._seen_ing.add(req.request_guid)

        if not self._known(req.target_guid, self._ingestors):
            self.get_logger().warn(
                f'unknown ingestor guid={req.target_guid!r} '
                f'(known={sorted(self._ingestors)}); completing anyway'
            )

        delay = self._delay()
        self.get_logger().info(
            f'ingestor {req.target_guid} request={req.request_guid} '
            f'— ACK then SUCCESS in {delay:.1f}s'
        )
        with self._lock:
            self._ing_queues[req.target_guid].append(req.request_guid)

        ack = IngestorResult()
        ack.time = self._now()
        ack.request_guid = req.request_guid
        ack.source_guid = req.target_guid
        ack.status = IngestorResult.ACKNOWLEDGED
        self._ing_result.publish(ack)

        def _finish():
            time.sleep(delay)
            out = IngestorResult()
            out.time = self._now()
            out.request_guid = req.request_guid
            out.source_guid = req.target_guid
            out.status = IngestorResult.SUCCESS
            self._ing_result.publish(out)
            with self._lock:
                q = self._ing_queues.get(req.target_guid, [])
                if req.request_guid in q:
                    q.remove(req.request_guid)
                if not q:
                    self._ing_queues.pop(req.target_guid, None)

        threading.Thread(target=_finish, daemon=True).start()


def main() -> None:
    rclpy.init()
    node = WorkcellAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

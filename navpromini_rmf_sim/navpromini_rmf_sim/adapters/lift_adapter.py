#!/usr/bin/env python3
"""Optional Python lift *node* fallback when Gazebo liblift is unavailable.

Prefer the book architecture instead:
  https://osrf.github.io/ros2multirobotbook/integration_lifts.html

  - Lift **node**: Gazebo ``liblift.so`` (``rmf_building_sim_gz_plugins``)
  - Lift **adapter**: ``lift_supervisor`` (``rmf_fleet_adapter``)

Only use this module with ``use_python_lift_adapter:=true`` and
``lifts.*.plugins: false``. It publishes ``/lift_states``, answers
``/lift_requests``, and moves cabin/shaft doors via ``gz set_pose``.
"""

from __future__ import annotations

import math
import os
import subprocess
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rmf_lift_msgs.msg import LiftRequest, LiftState


def _lifts_from_nav_graph(path: str) -> Tuple[Dict[str, List[str]], Dict[str, dict]]:
    """Return (lift_name -> floors, lift_name -> {x,y,yaw,dims})."""
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    floors_by_lift: Dict[str, List[str]] = {}
    meta: Dict[str, dict] = {}
    levels = data.get('levels') or {}
    if isinstance(levels, dict):
        for level_name, level in levels.items():
            if not isinstance(level, dict):
                continue
            for vertex in level.get('vertices') or []:
                if not isinstance(vertex, list) or not vertex:
                    continue
                props = vertex[-1] if isinstance(vertex[-1], dict) else {}
                lift = props.get('lift') or props.get('lift_cabin')
                if not isinstance(lift, str) or not lift.strip():
                    continue
                name = lift.strip()
                floors_by_lift.setdefault(name, [])
                if level_name not in floors_by_lift[name]:
                    floors_by_lift[name].append(str(level_name))
    for name, floors in floors_by_lift.items():
        floors_by_lift[name] = sorted(floors)
    for name, cfg in (data.get('lifts') or {}).items():
        name = str(name)
        floors_by_lift.setdefault(name, ['L1', 'L2'])
        if isinstance(cfg, dict):
            pos = cfg.get('position') or [0.0, 0.0, 0.0]
            dims = cfg.get('dims') or [1.5, 1.5]
            meta[name] = {
                'x': float(pos[0]),
                'y': float(pos[1]),
                'yaw': float(pos[2]) if len(pos) > 2 else 0.0,
                'width': float(dims[0]),
                'depth': float(dims[1]) if len(dims) > 1 else float(dims[0]),
            }
    return floors_by_lift, meta


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _gz_set_pose(
    name: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    world: str,
    logger=None,
) -> bool:
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    req = (
        f"name: '{name}', "
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
    )
    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '500',
        '--req', req,
    ]
    try:
        completed = subprocess.run(
            cmd, check=False, capture_output=True, text=True, timeout=1.0
        )
        ok = 'data: true' in (completed.stdout or '')
        if not ok and logger is not None:
            logger.debug(
                f'set_pose {name}: {(completed.stdout or completed.stderr or "")[:80]}'
            )
        return ok
    except Exception as exc:
        if logger is not None:
            logger.debug(f'set_pose {name} error: {exc}')
        return False


@dataclass
class _DoorLeaf:
    entity: str
    closed: Tuple[float, float, float]
    open: Tuple[float, float, float]
    yaw: float


@dataclass
class _Lift:
    name: str
    floors: List[str]
    x: float
    y: float
    yaw: float
    width: float
    depth: float
    floor_height: Dict[str, float]
    world: str
    door_leaves: Dict[str, List[_DoorLeaf]] = field(default_factory=dict)
    cabin_entity: str = ''
    current: str = 'L1'
    destination: str = 'L1'
    door: int = LiftState.DOOR_CLOSED
    motion: int = LiftState.MOTION_STOPPED
    mode: int = LiftState.MODE_AGV
    session_id: str = ''
    lock: threading.Lock = field(default_factory=threading.Lock)
    timer: Optional[threading.Timer] = None

    def snapshot(self, stamp) -> LiftState:
        msg = LiftState()
        msg.lift_time = stamp
        msg.lift_name = self.name
        msg.available_floors = list(self.floors)
        with self.lock:
            msg.current_floor = self.current
            msg.destination_floor = self.destination
            msg.door_state = self.door
            msg.motion_state = self.motion
            msg.current_mode = self.mode
            msg.session_id = self.session_id
        msg.available_modes = [LiftState.MODE_AGV, LiftState.MODE_HUMAN]
        return msg


def _build_door_leaves(
    lift: str,
    floors: List[str],
    x: float,
    y: float,
    yaw: float,
    depth: float,
    floor_height: Dict[str, float],
) -> Dict[str, List[_DoorLeaf]]:
    """Shaft **model** closed/open poses (world frame).

    Harmonic rejects nested cabin-door link set_pose (entity id 0). Only the
    top-level ``ShaftDoor_<lift>_<floor>_LiftDoor`` model moves reliably.
    Cabin door visuals are stripped in ``generate_site_assets`` so lidar is
    not blocked after the shaft model parks aside.
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Whole door assembly parks clear of the 1 m opening along local +X.
    slide = 1.2

    def local_to_world(lx: float, ly: float, z: float) -> Tuple[float, float, float]:
        return (x + cy * lx - sy * ly, y + sy * lx + cy * ly, z)

    out: Dict[str, List[_DoorLeaf]] = {}
    for floor in floors:
        z0 = float(floor_height.get(floor, 0.0))
        # Model pose is at floor height (links carry the 1.25 m visual offset).
        ly = depth * 0.5 + 0.05
        shaft = f'ShaftDoor_{lift}_{floor}_LiftDoor'
        out[floor] = [
            _DoorLeaf(
                entity=shaft,
                closed=local_to_world(0.0, ly, z0),
                open=local_to_world(slide, ly, z0),
                yaw=yaw,
            ),
        ]
    return out


class LiftAdapter(Node):
    def __init__(self):
        super().__init__('rmf_lift_adapter')
        self.declare_parameter('nav_graph_file', '')
        self.declare_parameter('initial_floor', 'L1')
        self.declare_parameter('travel_delay_sec', 3.0)
        self.declare_parameter('door_delay_sec', 1.5)
        self.declare_parameter('state_hz', 2.0)
        self.declare_parameter('world_name', 'sim_world')
        self.declare_parameter('floor_height_L1', 0.0)
        self.declare_parameter('floor_height_L2', 3.0)
        # Cabin set_pose (floor change) works on model name "Lift1".
        # Shaft door *models* set_pose OK; nested cabin-door *links* fail
        # (entity id 0). Cabin visuals are removed at world sanitize time.
        self.declare_parameter('move_gazebo_cabin', True)
        self.declare_parameter('move_gazebo_doors', True)
        # Back-compat: move_gazebo=true enables cabin only.
        self.declare_parameter('move_gazebo', True)

        graph = self.get_parameter('nav_graph_file').value
        if not graph:
            pkg = get_package_share_directory('navpromini_rmf_sim')
            graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
        if not os.path.isfile(graph):
            raise FileNotFoundError(f'nav_graph_file not found: {graph}')

        initial = str(self.get_parameter('initial_floor').value)
        world = str(self.get_parameter('world_name').value)
        discovered, meta = _lifts_from_nav_graph(graph)
        if not discovered:
            self.get_logger().warn(f'No lifts found in {graph}')

        floor_height = {
            'L1': float(self.get_parameter('floor_height_L1').value),
            'L2': float(self.get_parameter('floor_height_L2').value),
        }
        move_legacy = bool(self.get_parameter('move_gazebo').value)
        self._move_cabin = move_legacy and bool(
            self.get_parameter('move_gazebo_cabin').value
        )
        self._move_doors = bool(self.get_parameter('move_gazebo_doors').value)
        self._world = world
        self._lifts: Dict[str, _Lift] = {}
        for name, floors in discovered.items():
            m = meta.get(name, {
                'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'width': 1.5, 'depth': 1.5,
            })
            lift = _Lift(
                name=name,
                floors=floors,
                x=m['x'],
                y=m['y'],
                yaw=m['yaw'],
                width=m['width'],
                depth=m['depth'],
                floor_height=floor_height,
                world=world,
                cabin_entity=name,
                current=initial if initial in floors else floors[0],
                destination=initial if initial in floors else floors[0],
                door=LiftState.DOOR_CLOSED,
            )
            if self._move_doors:
                lift.door_leaves = _build_door_leaves(
                    name, floors, lift.x, lift.y, lift.yaw, lift.depth, floor_height
                )
            self._lifts[name] = lift
            if self._move_cabin:
                threading.Thread(
                    target=lambda L=lift: self._apply_cabin(L),
                    daemon=True,
                ).start()

        # RELIABLE so api-server / dashboard subscribers match; BEST_EFFORT
        # subscribers still receive from a RELIABLE publisher.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_pub = self.create_publisher(
            LiftState, 'lift_states', state_qos
        )
        self.create_subscription(
            LiftRequest, 'lift_requests', self._on_request, 10
        )
        hz = max(0.2, float(self.get_parameter('state_hz').value))
        self.create_timer(1.0 / hz, self._publish_states)
        self.get_logger().info(
            f'lift adapter graph={graph} lifts='
            f'{ {n: l.floors for n, l in self._lifts.items()} } '
            f'initial={initial} cabin={self._move_cabin} doors={self._move_doors}'
        )

    def _now(self):
        return self.get_clock().now().to_msg()

    def _publish_states(self) -> None:
        stamp = self._now()
        for lift in self._lifts.values():
            self._state_pub.publish(lift.snapshot(stamp))

    def _apply_cabin(self, lift: _Lift) -> None:
        if not self._move_cabin:
            return
        z = float(lift.floor_height.get(lift.current, 0.0))
        _gz_set_pose(
            lift.cabin_entity, lift.x, lift.y, z, lift.yaw, lift.world, self.get_logger()
        )

    def _apply_doors(self, lift: _Lift, open_doors: bool) -> None:
        # Park / restore shaft door *models* (not nested cabin links).
        if not self._move_doors:
            return
        for leaf in lift.door_leaves.get(lift.current, []):
            x, y, z = leaf.open if open_doors else leaf.closed
            _gz_set_pose(
                leaf.entity, x, y, z, leaf.yaw, lift.world, self.get_logger()
            )

    def _cancel_timer(self, lift: _Lift) -> None:
        if lift.timer is not None:
            lift.timer.cancel()
            lift.timer = None

    def _schedule(self, lift: _Lift, delay: float, fn) -> None:
        self._cancel_timer(lift)

        def _run():
            fn()

        lift.timer = threading.Timer(delay, _run)
        lift.timer.daemon = True
        lift.timer.start()

    def _on_request(self, req: LiftRequest) -> None:
        lift = self._lifts.get(req.lift_name)
        if lift is None:
            return
        travel = max(0.1, float(self.get_parameter('travel_delay_sec').value))
        door_delay = max(0.2, float(self.get_parameter('door_delay_sec').value))

        with lift.lock:
            if req.request_type == LiftRequest.REQUEST_END_SESSION:
                lift.session_id = ''
                lift.mode = LiftState.MODE_AGV
                lift.door = LiftState.DOOR_MOVING
                lift.destination = lift.current
                lift.motion = LiftState.MOTION_STOPPED
                self.get_logger().info(f'{lift.name}: end session — closing doors')

                def _closed():
                    with lift.lock:
                        lift.door = LiftState.DOOR_CLOSED
                    self._apply_doors(lift, open_doors=False)

                self._schedule(lift, door_delay, _closed)
                return

            lift.session_id = req.session_id or lift.session_id
            lift.mode = (
                LiftState.MODE_HUMAN
                if req.request_type == LiftRequest.REQUEST_HUMAN_MODE
                else LiftState.MODE_AGV
            )
            dest = req.destination_floor or lift.current
            if dest not in lift.floors:
                self.get_logger().warn(
                    f'{lift.name}: ignore unknown floor {dest}'
                )
                return
            lift.destination = dest
            # AGV sessions always need doors open at the cabin floor.
            want_open = True
            if lift.mode != LiftState.MODE_AGV:
                want_open = req.door_state == LiftRequest.DOOR_OPEN

            if dest == lift.current:
                # Ignore repeats while already open / opening for this session.
                if (
                    lift.session_id
                    and lift.session_id == (req.session_id or lift.session_id)
                    and lift.door in (LiftState.DOOR_OPEN, LiftState.DOOR_MOVING)
                    and lift.motion == LiftState.MOTION_STOPPED
                    and want_open
                ):
                    return

                # Same floor: MOVING → OPEN so fleet waits on door_state.
                lift.motion = LiftState.MOTION_STOPPED
                lift.door = LiftState.DOOR_MOVING
                self.get_logger().info(
                    f'{lift.name}: opening doors on {dest} '
                    f'(session={lift.session_id})'
                )

                def _opened():
                    self._apply_doors(lift, open_doors=want_open)
                    with lift.lock:
                        lift.door = (
                            LiftState.DOOR_OPEN if want_open else LiftState.DOOR_CLOSED
                        )
                    self.get_logger().info(
                        f'{lift.name}: doors '
                        f'{"OPEN" if want_open else "CLOSED"} on {dest}'
                    )

                self._schedule(lift, door_delay, _opened)
                return

            # Travel to another floor.
            lift.door = LiftState.DOOR_MOVING
            lift.motion = (
                LiftState.MOTION_UP
                if lift.floors.index(dest) > lift.floors.index(lift.current)
                else LiftState.MOTION_DOWN
            )
            self.get_logger().info(
                f'{lift.name}: {lift.current} -> {dest} '
                f'(session={lift.session_id})'
            )
            self._apply_doors(lift, open_doors=False)

            def _arrive():
                with lift.lock:
                    lift.current = dest
                    lift.destination = dest
                    lift.motion = LiftState.MOTION_STOPPED
                    lift.door = LiftState.DOOR_MOVING
                self._apply_cabin(lift)

                def _doors():
                    self._apply_doors(lift, open_doors=want_open)
                    with lift.lock:
                        lift.door = (
                            LiftState.DOOR_OPEN if want_open else LiftState.DOOR_CLOSED
                        )

                self._schedule(lift, door_delay, _doors)

            self._schedule(lift, travel, _arrive)


def main() -> None:
    rclpy.init()
    node = LiftAdapter()
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

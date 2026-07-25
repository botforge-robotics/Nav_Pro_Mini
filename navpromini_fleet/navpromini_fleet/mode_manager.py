#!/usr/bin/env python3
"""Exclusive robot mode manager: HARDWARE | MAPPING | NAV_READY | NAV_ACTIVE.

Listens to:
  mapping_cmd  (std_msgs/String JSON)  — start | stop | save
  fleet_cmd    (std_msgs/String JSON)  — start_nav | stop_nav | claim_map | factory_reset | set_pose

Invariant: never run SLAM and Nav2/AMCL together (single map→odom owner).
Hardware bringup (lidar/odom/micro-ROS) stays up under systemd navpro-robot.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from navpromini_fleet.fleet_config import (
    DEFAULT_FLEET_PATH,
    DEFAULT_MAPS_DIR,
    load_fleet_config,
    save_fleet_config,
)

MODES = ('HARDWARE', 'MAPPING', 'NAV_READY', 'NAV_ACTIVE')

# Match fleet mapping-bridge / zenoh (TRANSIENT_LOCAL). Volatile pubs are dropped.
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _maps_dir() -> Path:
    return Path(os.environ.get('NAVPRO_MAPS_DIR', str(DEFAULT_MAPS_DIR)))


class ModeManager(Node):
    def __init__(self) -> None:
        super().__init__('navpro_mode_manager')
        self.declare_parameter('config_path', str(DEFAULT_FLEET_PATH))
        self.declare_parameter('maps_dir', str(_maps_dir()))
        self.declare_parameter('slam_launch', 'navpromini_mapping slam.launch.py')
        self.declare_parameter('nav_launch', 'navpromini_navigation navigation.launch.py')
        self.declare_parameter('map_saver_launch', 'navpromini_mapping map_saver.launch.py')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self._lock = threading.Lock()
        self._mode = 'HARDWARE'
        self._slam_proc: Optional[subprocess.Popen[str]] = None
        self._nav_proc: Optional[subprocess.Popen[str]] = None
        self._level_id: str = ''
        self._map_name: str = 'map'
        self._pending_pose: Optional[dict[str, float]] = None
        self._pose_flush_left = 0
        self._last_status_state = 'idle'
        self._log_handles: list[Any] = []
        self._last_scan: Optional[LaserScan] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, 'mapping_status', STATUS_QOS)
        # Compact SLAM preview for zenoh (full /map can collide with RMF BuildingMap).
        self._grid_pub = self.create_publisher(OccupancyGrid, 'mapping_grid', MAP_QOS)
        self._pose_pub = self.create_publisher(PoseStamped, 'mapping_pose', 10)
        self._scan_pub = self.create_publisher(String, 'mapping_scan', 10)
        self._mode_pub = self.create_publisher(String, 'navpro/nav_mode', 10)
        self._display_pub = self.create_publisher(String, 'navpro/display_state', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self.create_subscription(String, 'mapping_cmd', self._on_mapping_cmd, 10)
        self.create_subscription(String, 'fleet_cmd', self._on_fleet_cmd, 10)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, MAP_QOS)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        self.create_timer(1.0, self._heartbeat_status)
        self.create_timer(0.5, self._pose_tick)
        self.create_timer(0.1, self._preview_tick)  # 10 Hz pose + lidar overlay
        self._publish_mode('HARDWARE')
        self.get_logger().info('mode_manager ready (HARDWARE)')

    # ----- helpers -----

    def _publish_mode(self, mode: str) -> None:
        self._mode = mode
        m = String()
        m.data = mode
        self._mode_pub.publish(m)
        # Display mapping
        display = {
            'HARDWARE': 'need_map',
            'MAPPING': 'mapping',
            'NAV_READY': 'ready',
            'NAV_ACTIVE': 'nav',
        }.get(mode, 'boot')
        d = String()
        d.data = display
        self._display_pub.publish(d)
        cfg = load_fleet_config(Path(str(self.get_parameter('config_path').value)))
        if cfg is not None:
            cfg.nav_mode = mode
            try:
                save_fleet_config(cfg, Path(str(self.get_parameter('config_path').value)))
            except OSError as exc:
                self.get_logger().warn(f'could not persist nav_mode: {exc}')

    def _publish_mapping_status(self, state: str, error: str = '', **extra: Any) -> None:
        # GUI / Redis expect starting|active|saving|saved|idle|error (not "mapping").
        if state == 'mapping':
            state = 'active'
        self._last_status_state = state
        payload = {'state': state, 'mode': self._mode, **extra}
        if error:
            payload['error'] = error
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Relay SLAM /map → /mapping_grid while mapping (GUI preview over zenoh)."""
        if self._mode != 'MAPPING':
            return
        self._grid_pub.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _lookup_base_in_map(self) -> Optional[tuple[float, float, float]]:
        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        try:
            tf = self._tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        t = tf.transform.translation
        yaw = _yaw_from_quat(tf.transform.rotation)
        return float(t.x), float(t.y), float(yaw)

    def _preview_tick(self) -> None:
        """Publish mapping_pose + mapping_scan for GUI overlays while SLAM is live."""
        if self._mode != 'MAPPING':
            return
        pose = self._lookup_base_in_map()
        if pose is None:
            return
        x, y, yaw = pose
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = str(self.get_parameter('map_frame').value)
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        self._pose_pub.publish(ps)

        scan = self._last_scan
        if scan is None:
            return
        # Transform sparse lidar hits into map frame for the Mapping UI overlay.
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                scan.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException:
            return
        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tyaw = _yaw_from_quat(tf.transform.rotation)
        c, s = math.cos(tyaw), math.sin(tyaw)
        xy: list[float] = []
        angle = float(scan.angle_min)
        step = max(1, len(scan.ranges) // 180)
        for i in range(0, len(scan.ranges), step):
            r = float(scan.ranges[i])
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                angle += float(scan.angle_increment) * step
                continue
            # Point in laser frame
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            mx = c * lx - s * ly + tx
            my = s * lx + c * ly + ty
            xy.extend((mx, my))
            angle += float(scan.angle_increment) * step
            if len(xy) >= 800:
                break
        if len(xy) < 2:
            return
        msg = String()
        msg.data = json.dumps({'xy': xy, 'ts': time.time()}, separators=(',', ':'))
        self._scan_pub.publish(msg)

    def _ros_env(self) -> dict[str, str]:
        env = os.environ.copy()
        env['ROS_LOCALHOST_ONLY'] = env.get('ROS_LOCALHOST_ONLY', '1')
        return env

    def _launch(self, args: list[str]) -> subprocess.Popen[str]:
        """Spawn ros2 launch; log to file (PIPE without a reader deadlocks children)."""
        cmd = ['ros2', 'launch', *args]
        self.get_logger().info(f'spawn: {" ".join(cmd)}')
        log_dir = Path('/var/log/navpro')
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            log_dir = Path('/tmp')
        label = args[0] if args else 'child'
        log_path = log_dir / f'{label}-{int(time.time())}.log'
        log_f = open(log_path, 'w', encoding='utf-8')  # noqa: SIM115
        self._log_handles.append(log_f)
        self.get_logger().info(f'{label} logs → {log_path}')
        return subprocess.Popen(
            cmd,
            env=self._ros_env(),
            stdout=log_f,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    def _stop_proc(self, proc: Optional[subprocess.Popen[str]], label: str) -> None:
        if proc is None:
            return
        if proc.poll() is not None:
            return
        self.get_logger().info(f'stopping {label} pid={proc.pid}')
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=5)

    def _stop_slam(self) -> None:
        proc = self._slam_proc
        self._slam_proc = None
        self._stop_proc(proc, 'slam')

    def _stop_nav(self) -> None:
        proc = self._nav_proc
        self._nav_proc = None
        self._stop_proc(proc, 'nav')

    def _ensure_maps_dir(self) -> Path:
        d = Path(str(self.get_parameter('maps_dir').value))
        d.mkdir(parents=True, exist_ok=True)
        return d

    # ----- mapping_cmd -----

    def _on_mapping_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().warn('mapping_cmd: invalid JSON')
            return
        action = str(payload.get('action') or '').lower()
        level_id = str(payload.get('levelId') or payload.get('level_id') or '')
        with self._lock:
            if action == 'start':
                self._cmd_start_slam(level_id)
            elif action == 'stop':
                self._cmd_stop_slam()
            elif action == 'save':
                self._cmd_save_map(level_id or self._level_id, str(payload.get('mapName') or self._map_name))
            else:
                self.get_logger().warn(f'unknown mapping action: {action}')

    def _cmd_start_slam(self, level_id: str) -> None:
        if self._mode == 'MAPPING' and self._slam_proc and self._slam_proc.poll() is None:
            self._publish_mapping_status('active', levelId=level_id or self._level_id)
            return
        self._stop_nav()
        self._stop_slam()
        self._level_id = level_id
        self._publish_mapping_status('starting', levelId=level_id)
        # slam.launch.py: use_sim_time:=false use_rviz:=false
        self._slam_proc = self._launch([
            'navpromini_mapping', 'slam.launch.py',
            'use_sim_time:=false', 'use_rviz:=false', 'autostart:=true',
        ])
        self._publish_mode('MAPPING')
        self._publish_mapping_status('active', levelId=level_id)
        self.get_logger().info('MAPPING started')

    def _cmd_stop_slam(self) -> None:
        # Flip display/mode first so OLED/LED leave "Mapping..." immediately.
        # Stopping slam can block for many seconds; kill off the executor using
        # a captured proc handle so a later start cannot be nulled by this thread.
        proc = self._slam_proc
        self._slam_proc = None
        self._publish_mode('HARDWARE')
        self._publish_mapping_status('idle')
        self.get_logger().info('SLAM stop requested → HARDWARE')
        if proc is not None:
            threading.Thread(
                target=self._stop_proc,
                args=(proc, 'slam'),
                name='stop-slam',
                daemon=True,
            ).start()

    def _cmd_save_map(self, level_id: str, map_name: str) -> None:
        if self._mode != 'MAPPING':
            self._publish_mapping_status('error', error='save requires MAPPING mode')
            return
        maps = self._ensure_maps_dir()
        # map_saver writes to package maps by default — override via env for saver script
        out_stem = maps / map_name
        self._publish_mapping_status('saving', levelId=level_id, mapName=map_name)
        # Use map_saver_cli directly so we control output path
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', str(out_stem),
            '--ros-args', '-p', 'map_subscribe_transient_local:=true',
        ]
        try:
            r = subprocess.run(cmd, env=self._ros_env(), capture_output=True, text=True, timeout=60)
            if r.returncode != 0:
                self._publish_mapping_status('error', error=r.stderr[-300:] or r.stdout[-300:])
                return
        except (subprocess.TimeoutExpired, OSError) as exc:
            self._publish_mapping_status('error', error=str(exc))
            return
        self._level_id = level_id
        self._map_name = map_name
        pgm = Path(str(out_stem) + '.pgm')
        yml = Path(str(out_stem) + '.yaml')
        if level_id and pgm.is_file() and yml.is_file():
            try:
                from navpromini_fleet.upload_map import upload_occupancy
                cfg = load_fleet_config(Path(str(self.get_parameter('config_path').value)))
                if cfg is not None and cfg.provisioning_token:
                    upload_occupancy(level_id, pgm, yml, cfg.api_base, cfg.provisioning_token)
                    self.get_logger().info(f'uploaded occupancy to level {level_id}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'map upload failed: {exc}')
                self._publish_mapping_status(
                    'error',
                    error=f'saved locally but upload failed: {exc}',
                    levelId=level_id,
                    mapName=map_name,
                    pgm=str(pgm),
                    yaml=str(yml),
                )
                return
        self._publish_mapping_status(
            'saved',
            levelId=level_id,
            mapName=map_name,
            pgm=str(pgm),
            yaml=str(yml),
        )
        self.get_logger().info(f'map saved → {out_stem}.{{pgm,yaml}}')

    # ----- fleet_cmd -----

    def _on_fleet_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().warn('fleet_cmd: invalid JSON')
            return
        action = str(payload.get('action') or '').lower()
        with self._lock:
            if action in ('start_nav', 'nav_start', 'start'):
                self._cmd_start_nav(payload)
            elif action in ('stop_nav', 'nav_stop', 'stop'):
                self._cmd_stop_nav()
            elif action in ('claim_map', 'sync_map', 'map_claim'):
                self._cmd_claim_map(payload)
            elif action == 'factory_reset':
                self._cmd_factory_reset()
            elif action in ('set_pose', 'initial_pose'):
                pose = payload.get('pose') if isinstance(payload.get('pose'), dict) else payload
                self._pending_pose = {
                    'x': float(pose.get('x') or 0.0),
                    'y': float(pose.get('y') or 0.0),
                    'yaw': float(pose.get('yaw') or 0.0),
                }
                self._pose_flush_left = 20
                self._publish_initial_pose(self._pending_pose)
            else:
                self.get_logger().warn(f'unknown fleet action: {action}')

    def _claim_from_fleet(
        self,
        level_id: str,
        map_name: str = 'active',
    ) -> Optional[Path]:
        """Download GUI occupancy; return local yaml path or None on failure."""
        if not level_id:
            return None
        maps = self._ensure_maps_dir()
        try:
            from navpromini_fleet.map_claim import claim_map

            cfg = load_fleet_config(Path(str(self.get_parameter('config_path').value)))
            if cfg is None or not cfg.provisioning_token:
                self.get_logger().error('claim_map: missing fleet config/token')
                return None
            result = claim_map(
                level_id, maps, cfg.api_base, cfg.provisioning_token, map_name
            )
            self._level_id = level_id
            self._map_name = map_name
            self.get_logger().info(
                f"claimed map level={level_id} name={map_name} "
                f"rev={result.get('mapRevision') or '?'} → {result['path']}"
            )
            return Path(result['path'])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'map claim failed: {exc}')
            self._publish_mapping_status('error', error=f'map claim failed: {exc}')
            return None

    def _cmd_claim_map(self, payload: dict[str, Any]) -> None:
        """Pull fleet GUI map; if Nav2 is up, reload it with the new files."""
        level_id = str(payload.get('levelId') or payload.get('level_id') or self._level_id or '')
        map_name = str(payload.get('mapName') or payload.get('map_name') or 'active')
        if not level_id:
            self.get_logger().warn('claim_map: no levelId')
            return
        yaml_path = self._claim_from_fleet(level_id, map_name)
        if yaml_path is None:
            return
        # Stash files only when not navigating; restart Nav2 so map_server picks origin/pgm.
        if self._mode == 'NAV_ACTIVE' and self._nav_proc and self._nav_proc.poll() is None:
            pose = payload.get('pose') or payload.get('initialPose')
            restart: dict[str, Any] = {
                'levelId': level_id,
                'mapName': map_name,
                'mapYaml': str(yaml_path),
            }
            if isinstance(pose, dict):
                restart['pose'] = pose
            self.get_logger().info('claim_map: reloading Nav2 with new occupancy')
            self._cmd_start_nav(restart)
        else:
            self._publish_mapping_status('idle', levelId=level_id, mapName=map_name)

    def _cmd_start_nav(self, payload: dict[str, Any]) -> None:
        self._stop_slam()
        map_name = str(
            payload.get('mapName') or payload.get('map_name') or self._map_name or 'active'
        )
        maps = self._ensure_maps_dir()
        level_id = str(payload.get('levelId') or payload.get('level_id') or self._level_id or '')

        # Always claim occupancy from fleet GUI when levelId present
        if level_id:
            claimed = self._claim_from_fleet(level_id, map_name)
            if claimed is None and not (maps / f'{map_name}.yaml').is_file():
                return

        yaml_path = maps / f'{map_name}.yaml'
        if payload.get('mapYaml'):
            yaml_path = Path(str(payload['mapYaml']))
        if not yaml_path.is_file():
            alt = Path(map_name)
            if alt.is_file():
                yaml_path = alt
            else:
                self.get_logger().error(f'nav map missing: {yaml_path}')
                self._publish_mapping_status('error', error=f'map not found: {yaml_path}')
                return

        self._stop_nav()
        map_arg = str(yaml_path) if yaml_path.suffix == '.yaml' else map_name
        self._nav_proc = self._launch([
            'navpromini_navigation', 'navigation.launch.py',
            f'map_name:={map_arg}',
            'use_sim_time:=false',
            'use_rviz:=false',
            'autostart:=true',
        ])
        time.sleep(2.0)
        pose = payload.get('pose') or payload.get('initialPose')
        if pose is None and all(k in payload for k in ('x', 'y')):
            pose = {
                'x': float(payload['x']),
                'y': float(payload['y']),
                'yaw': float(payload.get('yaw') or 0.0),
            }
        if isinstance(pose, dict):
            self._pending_pose = {
                'x': float(pose.get('x') or 0.0),
                'y': float(pose.get('y') or 0.0),
                'yaw': float(pose.get('yaw') or 0.0),
            }
            # Republish for ~10s: AMCL often activates after the first shot and
            # would otherwise miss a one-shot initialpose (no map TF → Nav2 fail).
            self._pose_flush_left = 20
            self._publish_initial_pose(self._pending_pose)
        self._publish_mode('NAV_ACTIVE')
        self._publish_mapping_status('idle')
        self.get_logger().info(f'NAV_ACTIVE map={map_arg}')

    def _pose_tick(self) -> None:
        if self._pose_flush_left <= 0 or self._pending_pose is None:
            return
        self._publish_initial_pose(self._pending_pose)
        self._pose_flush_left -= 1
        if self._pose_flush_left == 0:
            self._pending_pose = None

    def _publish_initial_pose(self, payload: dict[str, Any]) -> None:
        import math
        x = float(payload.get('x') or 0.0)
        y = float(payload.get('y') or 0.0)
        yaw = float(payload.get('yaw') or 0.0)
        msg = PoseWithCovarianceStamped()
        # Stamp 0 = "use latest TF" — avoids AMCL extrapolation warnings that
        # can drop early initialpose while clocks/odom catch up.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Modest covariance
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self._initialpose_pub.publish(msg)
        self.get_logger().info(f'initialpose x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

    def _cmd_stop_nav(self) -> None:
        self._stop_nav()
        self._publish_mode('HARDWARE')
        self.get_logger().info('Nav stopped → HARDWARE')

    def _cmd_factory_reset(self) -> None:
        self.get_logger().warn('factory_reset requested')
        self._stop_nav()
        self._stop_slam()
        cfg_path = Path(str(self.get_parameter('config_path').value))
        cfg = load_fleet_config(cfg_path)
        # ACK before wipe so server sees confirmation
        if cfg and cfg.robot_id and cfg.provisioning_token:
            try:
                import urllib.request
                req = urllib.request.Request(
                    f'{cfg.api_base}/robots/{cfg.robot_id}/factory-reset-ack',
                    data=b'{}',
                    headers={
                        'Content-Type': 'application/json',
                        'X-Provisioning-Token': cfg.provisioning_token,
                    },
                    method='POST',
                )
                urllib.request.urlopen(req, timeout=5)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'factory_reset ack failed: {exc}')

        # Wipe claimed maps so next provision does not reuse stale occupancy.
        try:
            maps = self._ensure_maps_dir()
            for p in maps.iterdir():
                if p.is_file():
                    p.unlink(missing_ok=True)
                elif p.is_dir():
                    subprocess.run(['rm', '-rf', str(p)], check=False)
            self.get_logger().info(f'factory_reset cleared maps under {maps}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'factory_reset map wipe: {exc}')

        # Drop site Wi‑Fi connection so reboot returns to setup hotspot.
        try:
            subprocess.run(
                ['nmcli', '-t', '-f', 'NAME,TYPE', 'connection', 'show'],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
            # Delete common site connection + any non-AP wifi profiles we created.
            for conn in ('navpro-site-wifi',):
                subprocess.run(
                    ['nmcli', 'connection', 'delete', conn],
                    check=False,
                    capture_output=True,
                    timeout=10,
                )
            # Best-effort: disconnect active wifi so hotspot can claim the iface.
            subprocess.run(
                ['nmcli', 'device', 'disconnect', 'wlan0'],
                check=False,
                capture_output=True,
                timeout=10,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'factory_reset wifi wipe: {exc}')

        try:
            if cfg_path.is_file():
                cfg_path.unlink()
        except OSError as exc:
            self.get_logger().error(f'factory_reset unlink: {exc}')
        self._publish_mapping_status('factory_reset')
        subprocess.Popen(['systemctl', 'reboot'], env=self._ros_env())

    def _heartbeat_status(self) -> None:
        # Keep display in sync (late subscribers / missed one-shot after provision).
        display = {
            'HARDWARE': 'need_map',
            'MAPPING': 'mapping',
            'NAV_READY': 'ready',
            'NAV_ACTIVE': 'nav',
        }.get(self._mode, 'boot')
        d = String()
        d.data = display
        self._display_pub.publish(d)

        # Republish mapping_status so late zenoh/bridge subscribers leave "starting".
        if self._mode == 'MAPPING':
            self._publish_mapping_status(
                'active' if self._last_status_state in ('starting', 'active', 'mapping') else self._last_status_state,
                levelId=self._level_id,
            )

        # Detect crashed children
        if self._mode == 'MAPPING' and self._slam_proc and self._slam_proc.poll() is not None:
            self.get_logger().warn('slam process exited unexpectedly')
            self._slam_proc = None
            self._publish_mode('HARDWARE')
            self._publish_mapping_status('idle')
        if self._mode in ('NAV_READY', 'NAV_ACTIVE') and self._nav_proc and self._nav_proc.poll() is not None:
            self.get_logger().warn('nav process exited unexpectedly')
            self._nav_proc = None
            self._publish_mode('HARDWARE')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:  # noqa: SLF001
            node._stop_nav()  # noqa: SLF001
            node._stop_slam()  # noqa: SLF001
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

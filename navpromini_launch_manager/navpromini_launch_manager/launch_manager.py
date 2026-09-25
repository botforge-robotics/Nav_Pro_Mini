import uuid
import rclpy
from rclpy.node import Node
from navpromini_launch_manager_interfaces.srv import LaunchWithArgs, StopLaunch, GetMapList, DeleteMap
import subprocess
import threading
from collections import OrderedDict
import os
import signal
from pathlib import Path
from ament_index_python import get_package_share_directory, PackageNotFoundError
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from std_msgs.msg import Int32


FILTER_TOKENS = ('_keepout', '_mask', '_filter', '_speed', '_zone', '_restricted', '_costmap')


def is_valid_base_map(stem: str) -> bool:
    if not stem or stem.startswith('.'):
        return False
    lower = stem.lower()
    return not any(tok in lower for tok in FILTER_TOKENS)


class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.launch_service = self.create_service(
            LaunchWithArgs, 'launch_with_args', self.launch_callback)
        self.stop_service = self.create_service(
            StopLaunch, 'stop_launch', self.stop_callback)
        self.map_list_service = self.create_service(
            GetMapList, 'get_map_list', self.get_map_list_callback
        )
        self.delete_map_service = self.create_service(
            DeleteMap, 'delete_map', self.delete_map_callback)
        self.active_launches = OrderedDict()
        # Set logger to debug level for verbose internal state reporting
        try:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        except Exception:
            # Fallback in case the API is unavailable in some distributions
            pass

        self.get_logger().info('Launch Manager ready with start/stop capabilities')
        self._shutting_down = False  # Add shutdown flag

        # Subscribe to client count to monitor active UI/clients
        self.client_count_sub = self.create_subscription(
            Int32,
            '/client_count',  # Topic name
            self.client_count_callback,
            10
        )
        # Debounce timer for the zero-client auto-stop below — see
        # client_count_callback for why this exists.
        self._zero_client_timer = None

    def check_package_exists(self, package_name):
        """Check if ROS package exists using ament index"""
        self.get_logger().debug(f"Checking package existence: {package_name}")
        try:
            get_package_share_directory(package_name)
            self.get_logger().debug(f"Package '{package_name}' found")
            return True
        except PackageNotFoundError:
            self.get_logger().debug(f"Package '{package_name}' not found")
            return False

    def check_launch_file_exists(self, package_name, launch_file):
        """Check if launch file exists using substitution-based path joining"""
        self.get_logger().debug(
            f"Validating launch file '{launch_file}' in package '{package_name}'")
        try:
            package_share = get_package_share_directory(package_name)
            launch_path = Path(package_share) / 'launch' / launch_file
            exists = launch_path.exists()
            self.get_logger().debug(
                f"Launch file path: {launch_path} exists: {exists}")
            return exists
        except Exception:
            self.get_logger().debug("Exception while checking launch file existence", exc_info=True)
            return False

    def launch_callback(self, request, response):
        self.get_logger().debug(
            f"launch_callback called with package={request.package}, launch_file={request.launch_file}, arguments='{request.arguments}'")
        try:
            # Validate package existence
            if not self.check_package_exists(request.package):
                response.success = False
                response.message = f"Package '{request.package}' not found"
                response.unique_id = ""
                self.get_logger().error(response.message)
                return response

            # Validate launch file existence
            if not self.check_launch_file_exists(request.package, request.launch_file):
                response.success = False
                response.message = f"Launch file '{request.launch_file}' not found in package '{request.package}'"
                response.unique_id = ""
                self.get_logger().error(response.message)
                return response

            # Generate unique ID and prepare command
            unique_id = str(uuid.uuid4())
            cmd = ['ros2', 'launch', request.package, request.launch_file]

            # Handle arguments if provided
            if request.arguments.strip():
                try:
                    args_list = request.arguments.split()
                    cmd.extend(args_list)
                except Exception as e:
                    response.success = False
                    response.message = f"Invalid arguments format: {str(e)}"
                    response.unique_id = ""
                    self.get_logger().error(response.message)
                    return response

            # Check if this is a map saving operation by launch file name
            is_map_saver = request.launch_file in ('save_map.launch.py', 'map_saver.launch.py')

            # For map saving, run synchronously
            if is_map_saver:
                self.get_logger().info("Starting synchronous map save operation")
                self.get_logger().debug(
                    f"Executing map save command: {' '.join(cmd)}")

                # -------------------------------------------------
                # Pre-save: capture existing maps (if path detectable)
                # -------------------------------------------------
                verification_path = None
                args_dict = {}
                try:
                    for token in args_list if 'args_list' in locals() else []:
                        if ':=' in token:
                            k, v = token.split(':=', 1)
                            args_dict[k] = v

                    # Common key names for map directory
                    map_dir_key = next(
                        (k for k in ['path', 'map_path', 'map_dir'] if k in args_dict), None)
                    if map_dir_key:
                        # If the provided value already looks like 'package/subdir', use as-is,
                        # otherwise prefix with the launch package name.
                        val = args_dict[map_dir_key]
                        verification_path = val if '/' in val else f"{request.package}/{val}"
                    else:
                        verification_path = f"{request.package}/maps"
                except Exception:
                    pass

                pre_maps = None
                if verification_path:
                    try:
                        pre_maps = self._retrieve_map_list(verification_path)
                    except Exception as e:
                        self.get_logger().warning(
                            f"Pre-save map list retrieval failed: {e}")

                # -------------------------------------------------
                # Execute the save map launch synchronously
                # -------------------------------------------------
                process = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    check=False
                )

                # Log process completion details
                self.get_logger().debug(
                    f"Map save process finished with return code {process.returncode}")
                self.get_logger().debug(f"Map save stdout: {process.stdout}")
                self.get_logger().debug(f"Map save stderr: {process.stderr}")

                # -------------------------------------------------
                # Post-save verification
                # -------------------------------------------------
                target_map_name = args_dict.get('map_name', '').strip()
                if target_map_name.endswith('.yaml'):
                    target_map_name = target_map_name[:-5]
                if target_map_name.endswith('.pgm'):
                    target_map_name = target_map_name[:-4]

                post_maps = None
                if verification_path:
                    try:
                        post_maps = self._retrieve_map_list(verification_path)
                    except Exception as e:
                        self.get_logger().warning(
                            f"Post-save map list retrieval failed: {e}")

                existed_before = (
                    target_map_name != '' and
                    pre_maps is not None and
                    target_map_name in pre_maps
                )
                exists_now = (
                    target_map_name != '' and
                    post_maps is not None and
                    target_map_name in post_maps
                )

                if process.returncode == 0:
                    if target_map_name:
                        if exists_now and not existed_before:
                            response.success = True
                            response.message = "Map save successful"
                        elif exists_now and existed_before:
                            response.success = False
                            response.message = "already exist"
                        else:
                            response.success = False
                            response.message = f"Map save failed: map '{target_map_name}' was not created"
                    else:
                        map_list_change = (pre_maps is not None and post_maps is not None and len(post_maps) > len(pre_maps))
                        response.success = map_list_change
                        response.message = "Map save successful" if map_list_change else "Map save failed"
                else:
                    response.success = False
                    response.message = f"Map save failed with return code {process.returncode}"

                response.unique_id = unique_id

                # Additional high-level log for success/failure
                if response.success:
                    self.get_logger().info("Map saved and verified successfully")
                else:
                    self.get_logger().error("Map save verification failed; see debug logs for details")

                return response

            # For normal async launches - terminate existing duplicate launch first
            for existing_id, existing_info in list(self.active_launches.items()):
                if existing_info.get('package') == request.package and existing_info.get('launch_file') == request.launch_file:
                    self.get_logger().info(
                        f"Stopping existing launch {existing_id} ({request.launch_file}) before starting new instance")
                    try:
                        p = existing_info['process']
                        os.killpg(os.getpgid(p.pid), signal.SIGINT)
                        try:
                            p.wait(timeout=3.0)
                        except subprocess.TimeoutExpired:
                            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    except Exception as err:
                        self.get_logger().warning(f"Error stopping prior launch: {err}")
                    self.active_launches.pop(existing_id, None)

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                # Merged into stdout: one pipe, one reader thread below,
                # and neither stream is ever left undrained.
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True
            )

            self.get_logger().debug(
                f"Subprocess started with PID {process.pid}")

            # Check if process started successfully
            if process.poll() is not None:
                output = process.stdout.read()
                response.success = False
                response.message = f"Process failed to start: {output}"
                response.unique_id = ""
                self.get_logger().error(response.message)
                return response

            # Continuously drain stdout/stderr into this node's own logger
            # for the rest of this launch's life — previously nobody ever
            # read this pipe once the process was confirmed alive, which
            # meant every node started this way (Nav2's planner/controller/
            # bt_navigator, dock_manager, camera, tag_dock, ...) was
            # completely invisible in journalctl: systemd only captures a
            # direct child's *inherited* stdout/stderr fd, not one manually
            # piped here and never drained. Beyond visibility, an unread
            # pipe also risks the child blocking on write() once the OS
            # pipe buffer (64KB) fills — a real hang risk for something as
            # long-running and chatty as the nav stack. See
            # _stream_launch_output for the reader itself.
            threading.Thread(
                target=self._stream_launch_output,
                args=(unique_id, process),
                daemon=True,
            ).start()

            # Store process information for async launches
            self.active_launches[unique_id] = {
                'process': process,
                'package': request.package,
                'launch_file': request.launch_file,
                'cmd': ' '.join(cmd),
                'start_time': self.get_clock().now().to_msg()
            }

            self.get_logger().debug(
                f"Active launches updated: {list(self.active_launches.keys())}")

            # Set success response
            response.success = True
            response.message = f"Launch initiated with ID: {unique_id}"
            response.unique_id = unique_id
            self.get_logger().info(
                f"Started launch {unique_id}: {' '.join(cmd)}")

        except Exception as e:
            self.get_logger().debug("Unexpected exception in launch_callback", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            response.unique_id = ""
            self.get_logger().error(response.message)

        return response

    def _stream_launch_output(self, unique_id: str, process: subprocess.Popen) -> None:
        """Reader-thread body: forwards a launched process's combined
        stdout/stderr to this node's own logger, line by line, tagged with
        the launch id so it's obvious which launch a given line came from
        when several are active. `.readline()` blocks, hence its own
        thread — rclpy's logger is safe to call off the main thread. Exits
        on its own once the process closes its stdout (normal exit, or
        after stop_callback/_on_zero_client_grace_elapsed kills it) — no
        separate shutdown signalling needed.
        """
        tag = unique_id[:8]
        try:
            for line in iter(process.stdout.readline, ''):
                self.get_logger().info(f'[{tag}] {line.rstrip()}')
        except Exception as e:  # noqa: BLE001 — a dead logger thread should
            # never take the launch down with it; just say so and stop.
            self.get_logger().debug(
                f"Output reader for {tag} stopped: {e}")
        finally:
            try:
                process.stdout.close()
            except Exception:
                pass

    def _terminate_process(self, process, cmd_str=""):
        """Gracefully terminate a process group with fast fallback to kill."""
        try:
            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                process.wait(timeout=4.0)
                return True, "Process stopped gracefully"
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f"Graceful SIGINT timed out for {cmd_str}, sending SIGTERM")
                os.killpg(pgid, signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                    return True, "Process terminated with SIGTERM"
                except subprocess.TimeoutExpired:
                    self.get_logger().error(f"SIGTERM timed out for {cmd_str}, killing with SIGKILL")
                    os.killpg(pgid, signal.SIGKILL)
                    try:
                        process.wait(timeout=1.0)
                    except Exception:
                        pass
                    return True, "Process killed with SIGKILL"
        except ProcessLookupError:
            return True, "Process already terminated"
        except Exception as e:
            return False, f"Stop failed: {str(e)}"

    def stop_callback(self, request, response):
        self.get_logger().info(
            f"stop_callback called with unique_id='{request.unique_id}'")
        unique_id = (request.unique_id or "").strip()

        to_stop = []
        if unique_id in ('all', '*', '', 'active'):
            # Stop all active launches
            to_stop = list(self.active_launches.items())
        elif unique_id in ('mapping', 'navigation'):
            for uid, info in list(self.active_launches.items()):
                if unique_id in info.get('launch_file', '') or unique_id in info.get('package', ''):
                    to_stop.append((uid, info))
        elif unique_id in self.active_launches:
            to_stop = [(unique_id, self.active_launches[unique_id])]
        elif len(self.active_launches) == 1:
            # Only one active launch running, stop it even if ID mismatched
            to_stop = list(self.active_launches.items())
            self.get_logger().info(f"Targeting single active launch {to_stop[0][0]} for request '{unique_id}'")

        if not to_stop:
            response.success = True
            response.message = f"No matching active launches found for ID: {unique_id}"
            self.get_logger().info(response.message)
            return response

        all_success = True
        messages = []
        for uid, launch_info in to_stop:
            self.active_launches.pop(uid, None)
            success, msg = self._terminate_process(launch_info['process'], launch_info.get('cmd', ''))
            if not success:
                all_success = False
            messages.append(f"[{uid[:8]}] {msg}")
            self.get_logger().info(f"Stopped {uid}: {msg}")

        response.success = all_success
        response.message = "; ".join(messages)
        self.get_logger().info(f"Stop result: {response.message}")
        return response

    def get_map_list_callback(self, request, response):
        try:
            map_files = self._retrieve_map_list(request.path)
            response.maplist = map_files
            response.success = True
            response.message = f"Found {len(map_files)} maps in {request.path}"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.maplist = []
            self.get_logger().error(f"Map list error: {str(e)}")

        return response

    # -------------------------------------------------
    # Internal utility: retrieve map list for a given package/path
    # -------------------------------------------------
    def _retrieve_map_list(self, path: str):
        """Return list of map (yaml) names for a given 'package/relative_path'.

        Raises ValueError on any validation failure.
        """
        self.get_logger().debug(f"_retrieve_map_list called with path={path}")

        if '/' not in path:
            raise ValueError("Path format must be 'package_name/path_to_maps'")

        package_name, map_path = path.split('/', 1)

        # Get package share directory using ament index
        try:
            package_share = get_package_share_directory(package_name)
        except PackageNotFoundError:
            raise ValueError(f"Package '{package_name}' not found")

        # Build full path using pathlib
        maps_dir = Path(package_share) / map_path

        if not maps_dir.exists():
            raise ValueError(f"Map directory not found: {maps_dir}")

        # Find all YAML files and extract names, excluding internal filter masks and hidden files
        map_files = [
            f.stem for f in maps_dir.glob('*.yaml')
            if f.is_file() and is_valid_base_map(f.stem)
        ]

        if not map_files:
            raise ValueError("No .yaml map files found in directory")

        self.get_logger().debug(f"Maps found: {map_files}")
        return map_files

    def delete_map_callback(self, request, response):
        self.get_logger().debug(
            f"delete_map_callback called with map_path={request.map_path}, map_name={request.map_name}")
        # 1) Path format
        if '/' not in request.map_path:
            response.success = False
            response.message = "Invalid map_path. Use 'package_name/path_to_maps'"
            return response

        package_name, rel_path = request.map_path.split('/', 1)

        # 2) Package exists?
        try:
            pkg_share = get_package_share_directory(package_name)
        except PackageNotFoundError:
            response.success = False
            response.message = f"Package '{package_name}' not found"
            return response

        # 3) Directory exists?
        maps_dir = Path(pkg_share) / rel_path
        if not maps_dir.is_dir():
            response.success = False
            response.message = f"Maps directory not found: {maps_dir}"
            return response

        # 4) Delete base .yaml and .pgm files, and ANY associated filter/mask files
        deleted = []
        errors = []
        prefix = request.map_name

        try:
            for fpath in maps_dir.iterdir():
                if fpath.is_file() and (fpath.name.startswith(f"{prefix}_") or fpath.name.startswith(f"{prefix}.")):
                    try:
                        fpath.unlink()
                        deleted.append(fpath.name)
                    except Exception as e:
                        errors.append(f"Failed to delete {fpath.name}: {e}")
        except Exception as e:
            errors.append(f"Error reading {maps_dir}: {e}")

        # 5) Prepare response
        if deleted:
            response.success = True
            response.message = f"Deleted: {', '.join(deleted)}"
            if errors:
                response.message += f"; warnings: {', '.join(errors)}"
        else:
            response.success = False
            response.message = '; '.join(errors)

        self.get_logger().debug(
            f"Delete results - deleted: {deleted}, errors: {errors}")

        return response

    def destroy_node(self):
        self.get_logger().debug("destroy_node called")
        self._shutting_down = True
        self.get_logger().info("Shutting down and cleaning up launches")
        # Cleanup all active processes
        for uid, info in list(self.active_launches.items()):
            try:
                self.get_logger().info(f"Terminating {uid}")
                os.killpg(os.getpgid(info['process'].pid), signal.SIGTERM)
                info['process'].wait(timeout=1)
            except Exception as e:
                self.get_logger().error(f"Cleanup error for {uid}: {str(e)}")
                self.get_logger().debug("Exception during destroy_node cleanup", exc_info=True)
        self.active_launches.clear()
        super().destroy_node()

    def __del__(self):
        self.get_logger().debug("__del__ called")
        if not self._shutting_down:
            self.get_logger().warning("Destructor called without proper shutdown")
            for uid, info in self.active_launches.items():
                info['process'].terminate()
            self.active_launches.clear()

    # -------------------------------------------------
    # Callback: Client count monitoring
    # -------------------------------------------------
    def client_count_callback(self, msg: Int32):
        """Stop all active launches if no clients stay disconnected for a
        grace period — not the instant the count touches zero.

        The assumption is that an external entity publishes the current
        number of connected clients (e.g., web UI sessions), and that
        "nobody's connected" for real means stop the robot to free
        resources. But rosbridge's client count also drops to zero for a
        page refresh, an app relaunch, or a momentary network blip — all of
        which reconnect within a couple of seconds — and stopping
        instantly on that blip was killing a navigation session the
        operator had just started, every single time they reloaded the
        page. Debounced: a zero count arms a one-shot timer instead of
        stopping immediately, and any reconnect within the grace window
        cancels it. A genuine "operator walked away" still stops the robot,
        just after this window instead of on the first missed heartbeat.
        """
        self.get_logger().debug(
            f"client_count_callback received count={msg.data}")
        # Client disconnects should not terminate active autonomous navigation/missions
        pass

    # How long to wait, after the client count hits zero, before actually
    # stopping active launches — long enough to comfortably cover a page
    # refresh/app relaunch's reconnect, short enough that a genuinely
    # abandoned session still stops in reasonable time.
    _ZERO_CLIENT_GRACE_SEC = 20.0

    def _on_zero_client_grace_elapsed(self):
        # Self-cancelling: create_timer is periodic, this is meant as a
        # one-shot.
        timer, self._zero_client_timer = self._zero_client_timer, None
        if timer is not None:
            timer.cancel()

        if not self.active_launches:
            return  # already stopped some other way in the meantime

        self.get_logger().info(
            "Zero-client grace period elapsed with no reconnect. "
            "Stopping all active launches.")
        # Iterate over a copy to avoid modification during iteration
        for uid, info in list(self.active_launches.items()):
            try:
                self.get_logger().info(f"Auto-stopping launch {uid}")
                process = info['process']
                # Re-use the same stop logic: SIGINT then fallback force
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                try:
                    process.wait(timeout=30.0)
                    self.get_logger().info(
                        f"Launch {uid} stopped gracefully")
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(
                        f"Graceful stop of {uid} timed out, forcing")
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(process.pid),
                                  signal.SIGKILL)
                        self.get_logger().error(
                            f"Launch {uid} killed with SIGKILL")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to auto-stop {uid}: {str(e)}")
            finally:
                # Ensure removal from active list
                self.active_launches.pop(uid, None)


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManager()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

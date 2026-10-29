#!/usr/bin/env python3
# A simple GUI for Gaussian-LIC on payload recordings, in the browser (viser): pick a ROS 2 bag, watch the coloured
# point cloud of the map grow while it is processed (/gaussian_lic/map, as in RViz2), and look at the Gaussian splat
# map once Gaussian-LIC has saved it.
#
#   ros2 launch gaussian_lic gui.launch.xml        then open http://localhost:8080
#
# Recordings with raw Ouster packets are converted first (replay_ouster_bag.py), into <name>_replayed next to them,
# which later runs reuse. Runs with a Python that has viser installed, see README.

import collections
import glob
import os
import re
import shutil
import signal
import subprocess
import threading
import time
import webbrowser

import numpy as np
import rclpy
import rosbag2_py
import viser
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from splat_viewer import quat_wxyz_to_matrix, read_ply, splat_covariances, view_behind_camera

INSTRUCTIONS = '''
1. Put your recordings (ROS 2 bag folders or `.mcap` files) in the folder below and click **Refresh**.
2. Pick a recording and click **Start**. Recordings with raw Ouster packets are converted first, into a copy
   named `<name>_replayed` next to them (once, this takes as long as the recording).
3. The coloured point cloud of the map builds up while the recording is processed. At the end Gaussian-LIC saves
   its map, which takes a minute or two, and the Gaussian splats appear.

Drag to rotate, right-drag to move, scroll to zoom.
'''
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def bag_uri(path):
    """What rosbag2 opens for a bag folder or .mcap file, None if it is neither"""
    if os.path.isfile(path):
        return path if path.endswith('.mcap') else None
    if not os.path.isdir(path):
        return None
    if os.path.isfile(os.path.join(path, 'metadata.yaml')):
        files = rosbag2_py.Info().read_metadata(path, '').relative_file_paths
        if all(os.path.isfile(os.path.join(path, f)) for f in files):
            return path
    # no metadata.yaml, or one listing files that are not there (a recording cut short): open its only file
    mcaps = glob.glob(os.path.join(path, '*.mcap'))
    return mcaps[0] if len(mcaps) == 1 else None


class Bag:
    """A recording: kind is 'ready' (has /ouster/points), 'raw' (Ouster packets, converted first) or None (unusable)"""

    def __init__(self, path, uri, metadata):
        self.name, self.uri = os.path.basename(path), uri
        self.converted = re.sub(r'\.mcap$', '', path) + '_replayed'
        self.start_ns, self.duration = metadata.starting_time.nanoseconds, metadata.duration.nanoseconds / 1e9
        topics = {t.topic_metadata.name: t.message_count for t in metadata.topics_with_message_count}
        self.kind = None
        if topics.get('/camera/left/image_raw'):
            if topics.get('/ouster/points') and topics.get('/ouster/imu'):
                self.kind = 'ready'
            elif topics.get('/ouster/lidar_packets'):
                self.kind = 'raw'

    def label(self):
        if self.kind == 'ready':
            what = 'ready'
        elif getattr(load_bag(self.converted), 'kind', None) == 'ready':
            what = f'uses {os.path.basename(self.converted)}'
        else:
            what = 'raw Ouster packets, converted first'
        return f'{self.name}  ({self.duration:.0f} s, {what})'


def load_bag(path):
    try:
        uri = bag_uri(path)
        return Bag(path, uri, rosbag2_py.Info().read_metadata(uri, '')) if uri else None
    except Exception:  # not a readable bag
        return None


def find_bags(folder):
    """The usable recordings directly in folder, newest first"""
    bags = []
    for entry in os.scandir(folder):
        if not entry.name.startswith('.') and (bag := load_bag(entry.path)) and bag.kind:
            bags.append((entry.stat().st_mtime, bag))
    return [bag for _, bag in sorted(bags, key=lambda b: -b[0])]


def terminate(proc, timeout):
    """Ctrl-C to the process group, killed if it is not done after timeout seconds"""
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def host_path(path):
    """path as seen outside the container: docker/run.sh mounts the result folder at /results"""
    path, host = os.path.realpath(path), os.environ.get('GAUSSIAN_LIC_HOST_RESULT')
    return host + path[len('/results'):] if host and path.startswith('/results/') else path


class Gui(Node):
    def __init__(self):
        super().__init__('gaussian_lic_gui')
        port = self.declare_parameter('port', 8080).value
        folder = self.declare_parameter('bag_folder', os.path.expanduser('~')).value
        self.result_path = self.declare_parameter('result_path', '').value
        self.fov = np.deg2rad(self.declare_parameter('camera_vertical_fov_deg', 38.5).value)
        self.aspect = self.declare_parameter('camera_aspect', 1.5517).value

        self.lock = threading.Lock()
        self.bags, self.worker, self.proc = {}, None, None
        self.stage, self.stage_time, self.message, self.folder_note, self.run_bag = 'idle', time.time(), '', '', None
        self.stopping, self.stop_reason, self.failure, self.psnr, self.coco_done, self.keyframes = \
            False, None, None, None, None, 0
        self.log, self.log_shown = collections.deque(maxlen=300), None
        self.cloud, self.splats, self.trajectory_handle, self.frustum = None, None, None, None
        self.reset_run()

        self.server = viser.ViserServer(port=port, verbose=False)
        self.server.scene.set_up_direction('+z')
        gui = self.server.gui
        gui.configure_theme(control_width='large', show_share_button=False)
        gui.add_markdown('## Gaussian-LIC\n' + INSTRUCTIONS)
        self.folder = gui.add_text('Recordings folder', folder)
        self.refresh_button = gui.add_button('Refresh', icon=viser.Icon.REFRESH)
        self.recording = gui.add_dropdown('Recording', ('none found',), disabled=True)
        self.start_button = gui.add_button('Start', color='green', icon=viser.Icon.PLAYER_PLAY)
        self.stop_button = gui.add_button('Stop', color='red', icon=viser.Icon.PLAYER_STOP, visible=False)
        self.status = gui.add_markdown('')
        self.progress = gui.add_progress_bar(0, visible=False)
        with gui.add_folder('View'):
            self.show_cloud = gui.add_checkbox('Point cloud', True)
            self.show_splats = gui.add_checkbox('Gaussian splats', True)
            self.last_button = gui.add_button('Show the last result', icon=viser.Icon.PHOTO,
                                              hint='the Gaussian map saved by the latest run')
        with gui.add_folder('Log', expand_by_default=False):
            self.log_text = gui.add_markdown('')

        self.refresh_button.on_click(lambda _: self.refresh())
        self.start_button.on_click(lambda _: self.start())
        self.stop_button.on_click(lambda _: self.stop())
        self.last_button.on_click(lambda _: self.load_result())
        self.show_cloud.on_update(lambda _: self.set_visible(cloud=self.show_cloud.value))
        self.show_splats.on_update(lambda _: self.set_visible(splats=self.show_splats.value))
        self.refresh()

        self.create_subscription(PointCloud2, '/gaussian_lic/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseStamped, '/pose_for_gs', self.on_pose, 100)
        self.create_timer(0.5, self.update)
        self.get_logger().info(f'Gaussian-LIC GUI: open http://localhost:{port} in a browser')
        if self.declare_parameter('open_browser', True).value:
            webbrowser.open(f'http://localhost:{port}')  # does nothing in the container, docker/gui.sh opens it

    # ---- recordings and runs

    def refresh(self):
        try:
            bags = find_bags(os.path.expanduser(self.folder.value))
        except OSError as e:
            bags, self.folder_note = [], f'Cannot read the folder: {e.strerror}.'
        else:
            self.folder_note = '' if bags else f'No recordings found in `{self.folder.value}`.'
        self.bags = {bag.label(): bag for bag in bags}
        previous = self.recording.value
        self.recording.options = tuple(self.bags) or ('none found',)
        self.recording.value = previous if previous in self.bags else self.recording.options[0]
        self.recording.disabled = self.start_button.disabled = not self.bags
        self.last_button.disabled = not os.path.isfile(os.path.join(self.result_path, 'point_cloud.ply'))

    def start(self):
        bag = self.bags.get(self.recording.value)
        with self.lock:
            if bag is None or self.worker is not None:
                return
            self.stopping, self.stop_reason, self.failure, self.psnr, self.coco_done, self.keyframes = \
                False, None, None, None, None, 0
            self.worker = threading.Thread(target=self.run, args=(bag,), daemon=True)
        for handle in (self.start_button, self.refresh_button, self.recording, self.last_button):
            handle.disabled = True
        self.start_button.visible, self.stop_button.visible, self.stop_button.disabled = False, True, False
        self.worker.start()

    def stop(self, reason=None):
        with self.lock:
            if self.worker is None or self.stopping:
                return
            self.stopping, self.stop_reason, proc = True, reason, self.proc
        self.stop_button.disabled = True
        if proc is not None:
            threading.Thread(target=terminate, args=(proc, 40), daemon=True).start()

    def run(self, bag):
        try:
            if bag.kind == 'raw':
                converted = load_bag(bag.converted)
                if getattr(converted, 'kind', None) != 'ready':
                    if os.path.exists(bag.converted):
                        return self.finish('failed', f'`{bag.converted}` exists but is not a converted recording: '
                                                     'delete it and start again.')
                    self.set_stage('converting', bag)
                    code = self.execute(['ros2', 'run', 'gaussian_lic', 'replay_ouster_bag.py', bag.uri, bag.converted])
                    converted = load_bag(bag.converted) if code == 0 and not self.stopping else None
                    if getattr(converted, 'kind', None) != 'ready':
                        shutil.rmtree(bag.converted, ignore_errors=True)  # the partial copy made by this run
                        return self.finish_stopped() if self.stopping else \
                            self.finish('failed', 'Converting the recording failed, see the log.')
                bag = converted

            self.reset_scene()
            self.set_stage('starting', bag)
            started = time.time()
            self.execute(['ros2', 'launch', 'gaussian_lic', 'payload.launch.xml', f'bag_path:={bag.uri}',
                          'rviz:=false', 'viewer:=false', f'result_path:={self.result_path}'])
            if self.stopping:
                return self.finish_stopped()
            ply = os.path.join(self.result_path, 'point_cloud.ply')
            if not os.path.isfile(ply) or os.path.getmtime(ply) < started:
                return self.finish('failed', (self.failure or 'Gaussian-LIC did not save a map') + ', see the log.')
            n = self.load_result()
            psnr = f', novel-view PSNR {self.psnr} dB' if self.psnr else ''
            self.finish('done', f'**Done**: {n / 1e6:.2f}M Gaussians{psnr}. '
                                f'Saved in `{host_path(self.result_path)}`.')
        except Exception as e:  # shown in the GUI rather than lost in a thread
            self.finish('failed', f'{type(e).__name__}: {e}')

    def execute(self, cmd):
        """Runs cmd in its own process group, its output to the terminal and the log; returns its exit code"""
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
        with self.lock:
            if self.stopping:
                return -1
            self.log.append('$ ' + ' '.join(cmd))
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                         errors='replace', env=env, start_new_session=True)
        for line in self.proc.stdout:
            print(line, end='', flush=True)
            self.on_line(ANSI.sub('', line.rstrip()))
        code = self.proc.wait()
        with self.lock:
            self.proc = None
        return code

    def on_line(self, line):
        if line:
            self.log.append(line)
        if 'Cur Frame' in line:
            self.keyframes += 1
        elif 'Runtime Statistics' in line:
            self.set_stage('finishing')
        elif m := re.search(r'\[In-Sequence Novel View PSNR\] ([\d.]+)', line):
            self.psnr = m.group(1)
        elif 'All Done.' in line:  # Coco-LIC is through the recording
            self.coco_done = time.time()
        elif 'process has died' in line:
            self.failure = 'A node crashed: ' + line.split(']: ')[0].split('[')[-1]

    def set_stage(self, stage, bag=None):
        with self.lock:
            self.stage, self.stage_time = stage, time.time()
            self.run_bag = bag or self.run_bag

    def finish(self, stage, message):
        with self.lock:
            self.stage, self.message, self.worker = stage, message, None
        self.start_button.visible, self.stop_button.visible = True, False
        self.refresh_button.disabled = False
        self.refresh()

    def finish_stopped(self):
        if self.stop_reason:
            return self.finish('failed', self.stop_reason)
        return self.finish('stopped', 'Stopped.')

    def shutdown(self):
        self.stop()
        with self.lock:
            proc, worker = self.proc, self.worker
        if proc is not None:
            terminate(proc, 20)
        if worker is not None:
            worker.join(10)
        self.server.stop()

    # ---- the scene

    def reset_run(self):
        self.map_msg, self.map_time, self.n_points, self.extent = None, 0.0, 0, None
        self.trajectory, self.first_pose, self.data_time, self.view_set, self.drawn = [], None, None, False, 0

    def reset_scene(self):
        with self.lock:
            handles = [self.cloud, self.splats, self.trajectory_handle, self.frustum]
            self.cloud, self.splats, self.trajectory_handle, self.frustum = None, None, None, None
            self.reset_run()
        for handle in handles:
            if handle is not None:
                handle.remove()
        self.set_visible(cloud=True, splats=True)

    def set_visible(self, cloud=None, splats=None):
        if cloud is not None:
            self.show_cloud.value = cloud
            if self.cloud is not None:
                self.cloud.visible = cloud
        if splats is not None:
            self.show_splats.value = splats
            if self.splats is not None:
                self.splats.visible = splats

    def on_map(self, msg):
        with self.lock:
            self.map_msg = msg  # the whole map, replaced about every second

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        pose = (np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z]))
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        with self.lock:
            if self.first_pose is None:
                self.first_pose = pose
            self.trajectory.append(pose[0])
            self.latest_wxyz = pose[1]
            if self.run_bag is not None:
                self.data_time = (stamp_ns - self.run_bag.start_ns) / 1e9

    def draw_cloud(self, msg):
        """The Gaussian centres in their base colour, as RViz2 shows /gaussian_lic/map"""
        cloud = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'))
        points = np.stack([cloud['x'], cloud['y'], cloud['z']], 1).astype(np.float32)
        rgb = np.ascontiguousarray(cloud['rgb']).view(np.uint32)
        colors = np.stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255], 1).astype(np.uint8)
        if self.extent is None:  # the first map sets the view
            center = np.median(points, axis=0)
            self.extent = (center, np.percentile(np.linalg.norm(points - center, axis=1), 90))
        self.cloud = self.server.scene.add_point_cloud('/point_cloud', points, colors, point_size=0.02,
                                                       precision='float32', visible=self.show_cloud.value)
        self.n_points = len(points)

    def set_view(self, position, target):
        self.server.initial_camera.position, self.server.initial_camera.look_at = position, target
        for client in self.server.get_clients().values():
            client.camera.position, client.camera.look_at = position, target

    def load_result(self):
        """Shows the Gaussian map saved by the latest run; returns its number of Gaussians"""
        d = read_ply(os.path.join(self.result_path, 'point_cloud.ply'))
        splats = self.server.scene.add_gaussian_splats('/gaussian_map', centers=d[:, :3],
                                                       covariances=splat_covariances(d),
                                                       rgbs=d[:, 3:6], opacities=d[:, 6:7])
        with self.lock:
            self.splats = splats
            first_pose = self.first_pose
        self.set_visible(cloud=False, splats=True)  # the splats cover the point cloud; tick it to see both
        if first_pose is not None:
            self.set_view(*view_behind_camera(*first_pose))
        else:  # from the world origin, where the recording starts, towards the middle of the map
            self.set_view(np.zeros(3), np.median(d[:, :3], axis=0))
        return len(d)

    # ---- updated twice a second

    def update(self):
        with self.lock:
            map_msg = self.map_msg if time.time() - self.map_time > 2.0 else None  # ~15 MB per million points
            if map_msg is not None:
                self.map_msg, self.map_time = None, time.time()
            trajectory, first_pose = list(self.trajectory), self.first_pose
            stage, bag = self.stage, self.run_bag
        if map_msg is not None:
            self.draw_cloud(map_msg)
        if len(trajectory) > self.drawn:  # the camera at its latest pose, and its trajectory
            self.drawn = len(trajectory)
            with self.lock:
                position, wxyz = trajectory[-1], self.latest_wxyz
            self.frustum = self.server.scene.add_camera_frustum(
                '/camera', fov=self.fov, aspect=self.aspect, scale=0.3, color=(255, 80, 80),
                wxyz=wxyz, position=position)
            if len(trajectory) >= 2:
                t = np.array(trajectory)
                self.trajectory_handle = self.server.scene.add_line_segments(
                    '/trajectory', points=np.stack([t[:-1], t[1:]], axis=1), colors=(255, 80, 255),
                    thickness=3.0, thickness_units='screen')
        if first_pose is not None and self.extent is not None and not self.view_set:
            self.view_set = True  # once per run: the first map from above, seen from behind the camera
            center, radius = self.extent
            forward = quat_wxyz_to_matrix(first_pose[1][None])[0][:, 2] * [1, 1, 0]
            forward /= max(np.linalg.norm(forward), 1e-6)
            distance = max(3 * radius, 10.0)
            self.set_view(center - 0.6 * distance * forward + [0, 0, 0.8 * distance], center)

        if stage == 'starting' and self.n_points:
            self.set_stage('mapping')
            stage = 'mapping'
        if stage in ('starting', 'mapping') and self.coco_done and not self.keyframes \
                and time.time() - self.coco_done > 30:
            self.stop('Coco-LIC went through the recording without Gaussian-LIC getting a frame: '
                      'Coco-LIC most likely did not initialise, see the log.')
        self.show_status(stage, bag)

    def show_status(self, stage, bag):
        elapsed, progress, animated = time.time() - self.stage_time, None, False
        if self.stopping and self.worker is not None:
            text, progress, animated = 'Stopping ...', 100, True
        elif stage == 'converting':
            expected = bag.duration + 10  # the recording is replayed in real time
            text, progress = (f'**Converting** the Ouster packets of {bag.name}: '
                              f'{elapsed:.0f} of about {expected:.0f} s'), 100 * elapsed / expected
        elif stage == 'starting':
            text, progress, animated = '**Starting** Coco-LIC and Gaussian-LIC ...', 100, True
        elif stage == 'mapping':
            t = max(self.data_time or 0.0, 0.0)
            text = (f'**Mapping** {bag.name}: {t:.0f} of {bag.duration:.0f} s, '
                    f'{self.n_points / 1e6:.2f}M points in the map, {self.keyframes} keyframes')
            progress = 100 * t / bag.duration
        elif stage == 'finishing':
            text, progress, animated = ('**Saving the Gaussian map**: Gaussian-LIC renders every frame to '
                                        f'evaluate it ({elapsed:.0f} s, takes a minute or two)'), 100, True
        elif stage == 'failed':
            text = f'**Failed**: {self.message}'
        elif stage in ('done', 'stopped'):
            text = self.message
        else:
            text = self.folder_note or 'Pick a recording and click **Start**.'
        if self.status.content != text:
            self.status.content = text
        self.progress.visible = progress is not None
        if progress is not None:
            self.progress.value, self.progress.animated = min(progress, 100), animated

        log = '\n'.join(list(self.log)[-30:])
        if log != self.log_shown:
            self.log_shown, self.log_text.content = log, f'```\n{log}\n```'


def main():
    # handle Ctrl-C ourselves: the pipeline runs in its own process group and has to be stopped first
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    node = Gui()
    while rclpy.ok() and not stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Live 3D view of the Gaussian-LIC map: renders /gaussian_lic/splats as Gaussian splats in the browser (viser),
# together with the current camera (/pose_for_gs) and its trajectory. With the parameter ply set, it shows a
# saved map (point_cloud.ply from the result folder) instead, see view_map.launch.xml.
# Runs with a Python that has viser installed, see README.

import signal
import threading

import numpy as np
import rclpy
import viser
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

FIELDS = ('x', 'y', 'z', 'red', 'green', 'blue', 'opacity',
          'scale_0', 'scale_1', 'scale_2', 'rot_0', 'rot_1', 'rot_2', 'rot_3')
C0 = 0.28209479177387814  # zeroth-order spherical harmonic


def read_ply(path):
    """Gaussians saved by Gaussian-LIC, one row each in the layout of FIELDS (activated like /gaussian_lic/splats)"""
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply' or f.readline().strip() != b'format binary_little_endian 1.0':
            raise ValueError(f'{path} is not a binary little-endian PLY file')
        names, count = [], 0
        while (line := f.readline().decode().split()) != ['end_header']:
            if line[:2] == ['element', 'vertex']:
                count = int(line[2])
            elif line[0] == 'property':
                if line[1] != 'float':
                    raise ValueError(f'{path}: unexpected property {" ".join(line)}')
                names.append(line[2])
        v = np.fromfile(f, dtype=[(n, '<f4') for n in names], count=count)

    def cols(*ns):
        return np.stack([v[n] for n in ns], 1)
    return np.hstack([cols('x', 'y', 'z'),
                      np.clip(cols('f_dc_0', 'f_dc_1', 'f_dc_2') * C0 + 0.5, 0, 1),
                      1 / (1 + np.exp(-cols('opacity'))),
                      np.exp(cols('scale_0', 'scale_1', 'scale_2')),
                      cols('rot_0', 'rot_1', 'rot_2', 'rot_3')])


def quat_wxyz_to_matrix(q):
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], 1)


def splat_covariances(d):
    """3D covariances of Gaussians in the layout of FIELDS"""
    R = quat_wxyz_to_matrix(d[:, 10:14] / np.linalg.norm(d[:, 10:14], axis=1, keepdims=True))
    return np.einsum('nij,nj,nkj->nik', R, d[:, 7:10] ** 2, R)


def view_behind_camera(position, wxyz):
    """(position, look_at) of a view just behind a camera, looking along its optical axis"""
    forward = quat_wxyz_to_matrix(np.array([wxyz]))[0][:, 2]
    return position - 0.5 * forward, position + 3.0 * forward


class SplatViewer(Node):
    def __init__(self):
        super().__init__('splat_viewer')
        port = self.declare_parameter('port', 8080).value
        ply = self.declare_parameter('ply', '').value
        self.fov = np.deg2rad(self.declare_parameter('camera_vertical_fov_deg', 40.0).value)
        self.aspect = self.declare_parameter('camera_aspect', 4.0 / 3.0).value

        self.server = viser.ViserServer(port=port, verbose=False)
        self.server.scene.set_up_direction('+z')
        self.trajectory = []
        self.camera_pose = None  # first (position, wxyz), where the initial view starts
        self.view_set = False

        if ply:
            self.show(read_ply(ply))
        else:
            self.create_subscription(PointCloud2, '/gaussian_lic/splats', self.on_splats,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PoseStamped, '/pose_for_gs', self.on_pose, 100)
        self.get_logger().info(f'Gaussian splat viewer: open http://localhost:{port} in a browser')

    def on_splats(self, msg):
        self.show(point_cloud2.read_points_numpy(msg, field_names=FIELDS))

    def show(self, d):
        centers = d[:, 0:3]
        self.server.scene.add_gaussian_splats('/gaussian_map', centers=centers, covariances=splat_covariances(d),
                                              rgbs=d[:, 3:6], opacities=d[:, 6:7])
        self.get_logger().info(f'showing {len(d)} Gaussians')

        if not self.view_set:  # set the view once, then leave it to the user
            self.view_set = True
            if self.camera_pose is not None:
                position, target = view_behind_camera(*self.camera_pose)
            else:  # from the world origin, where the recording starts, towards the middle of the map
                position, target = np.zeros(3), np.median(centers, axis=0)
            self.server.initial_camera.look_at = target
            self.server.initial_camera.position = position
            for client in self.server.get_clients().values():
                client.camera.look_at = target
                client.camera.position = position

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        position = np.array([p.x, p.y, p.z])
        if self.camera_pose is None:
            self.camera_pose = (position, np.array([q.w, q.x, q.y, q.z]))
        self.server.scene.add_camera_frustum('/camera', fov=self.fov, aspect=self.aspect, scale=0.3,
                                             color=(255, 80, 80), wxyz=(q.w, q.x, q.y, q.z), position=position)
        self.trajectory.append(position)
        if len(self.trajectory) >= 2 and len(self.trajectory) % 5 == 0:
            t = np.array(self.trajectory)
            self.server.scene.add_line_segments('/trajectory', points=np.stack([t[:-1], t[1:]], axis=1),
                                                colors=(255, 80, 255), thickness=3.0, thickness_units='screen')


def main():
    # handle Ctrl-C ourselves: launch forwards a second SIGINT, which must not kill the process mid-shutdown
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    node = SplatViewer()
    while rclpy.ok() and not stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.server.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

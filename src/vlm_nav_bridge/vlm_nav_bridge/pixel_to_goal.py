#!/usr/bin/env python3
"""
pixel_to_goal — mini-project 4.1

Takes a pixel, turns it into a Nav2 goal.

    (u, v) + depth  ->  3D point in camera optical frame     [YOUR MATH]
                    ->  3D point in map                      (tf2)
                    ->  RViz Marker                          (verify visually!)
                    ->  standoff pose 0.6 m short, facing it [YOUR MATH]
                    ->  nav2_simple_commander.goToPose()

Two functions are left for you: deproject() and standoff_pose().
Everything else is plumbing. Look for TODO(Oli).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import (
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

# Registers geometry_msgs types with tf2's transform machinery.
# Never referenced directly. Remove it and buffer.transform() raises a
# type error that says nothing about a missing import.
import tf2_geometry_msgs  # noqa: F401

from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator


class PixelToGoal(Node):

    def __init__(self):
        super().__init__('pixel_to_goal')

        # ---- parameters -------------------------------------------------
        # Topic and frame names differ between TB3 models and between sim
        # and hardware. Check yours with:
        #     ros2 topic list | grep -i depth
        #     ros2 run tf2_tools view_frames
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/depth/camera_info')
        self.declare_parameter('optical_frame', 'camera_depth_optical_frame')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pixel_u', 320)
        self.declare_parameter('pixel_v', 240)
        self.declare_parameter('standoff_radius', 0.6)
        self.declare_parameter('patch', 2)          # median over (2p+1)^2 px
        self.declare_parameter('send_goal', True)   # False = marker only

        g = lambda n: self.get_parameter(n).value
        self.depth_topic = g('depth_topic')
        self.info_topic = g('info_topic')
        self.optical_frame = g('optical_frame')
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.u = int(g('pixel_u'))
        self.v = int(g('pixel_v'))
        self.radius = float(g('standoff_radius'))
        self.patch = int(g('patch'))
        self.send_goal = bool(g('send_goal'))

        # ---- intrinsics, filled by the camera_info callback -------------
        self.fx = self.fy = self.cx = self.cy = None

        # ---- tf2 --------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.done = False          # one-shot: don't spam goals every frame

        # ---- io ---------------------------------------------------------
        # qos_profile_sensor_data is BEST_EFFORT. A best-effort subscriber
        # can receive from a reliable publisher, but not the reverse, so
        # this is the safe choice for both image and camera_info.
        self.create_subscription(
            CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(Marker, '~/target_marker', 1)

        self.nav = BasicNavigator()

        self.get_logger().info(
            f'pixel_to_goal up. pixel=({self.u},{self.v}) '
            f'standoff={self.radius} m  send_goal={self.send_goal}')

    # =====================================================================
    #  YOUR MATH  -  the two functions you derived
    # =====================================================================

    def deproject(self, u, v, d):
        """
        TODO(Oli): pixel + depth  ->  3D point in the camera OPTICAL frame.

        You have, as instance attributes:
            self.fx, self.fy    focal lengths, in PIXELS
            self.cx, self.cy    principal point, in PIXELS

        Args:
            u   column index (horizontal)
            v   row index (vertical)
            d   depth at that pixel, already converted to METRES

        Returns:
            (X, Y, Z) as floats, metres, in camera_depth_optical_frame
            (+Z out of the lens, +X right, +Y down)

        Three lines. You wrote them on paper.
        """
        raise NotImplementedError('deproject() — write the three lines')

    def standoff_pose(self, mug_xy, robot_xy, radius):
        """
        TODO(Oli): pick a spot short of the target, facing it.

        Args:
            mug_xy    (Mx, My)  target position in the map frame
            robot_xy  (Rx, Ry)  robot position in the map frame
            radius    standoff distance in metres

        Returns:
            (Sx, Sy, yaw)   position in map, and heading in radians

        Reminders:
          - back off FROM the target TOWARD the robot
          - normalise before scaling
          - yaw is measured from +X in the map frame
          - if the robot is already inside the radius, return None and let
            the caller bail rather than sending a goal that reverses
        """
        raise NotImplementedError('standoff_pose() — write the vector math')

    # =====================================================================
    #  PLUMBING  -  below here is done
    # =====================================================================

    def on_info(self, msg: CameraInfo):
        if self.fx is not None:
            return
        k = msg.k                       # row-major 3x3
        self.fx, self.fy = k[0], k[4]
        self.cx, self.cy = k[2], k[5]
        self.get_logger().info(
            f'intrinsics: fx={self.fx:.2f} fy={self.fy:.2f} '
            f'cx={self.cx:.2f} cy={self.cy:.2f}  ({msg.width}x{msg.height})')

    def depth_at(self, img, u, v):
        """Median depth in metres over a small patch, ignoring holes.

        Two things happen here:
          - unit conversion. 16UC1 depth is in MILLIMETRES, 32FC1 in METRES.
          - hole rejection. Depth is 0 or NaN on reflective, transparent and
            too-near surfaces. One pixel is not enough; take the median of
            the valid ones nearby.
        """
        h, w = img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            raise ValueError(f'pixel ({u},{v}) outside {w}x{h} image')

        p = self.patch
        # NOTE the index order: [row, col] == [v, u]. Backwards from how you
        # write the coordinate. On a square image, getting this wrong is silent.
        win = img[max(0, v - p):v + p + 1, max(0, u - p):u + p + 1].astype(np.float64)

        if img.dtype == np.uint16:      # 16UC1 -> millimetres
            win = win / 1000.0

        valid = win[np.isfinite(win) & (win > 0.0)]
        if valid.size == 0:
            raise ValueError(f'no valid depth near ({u},{v}) — hole or out of range')
        return float(np.median(valid))

    def publish_marker(self, p: PointStamped):
        m = Marker()
        m.header = p.header
        m.ns = 'pixel_to_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = p.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.05, 0.85, 0.95, 0.95
        self.marker_pub.publish(m)

    def robot_xy(self, stamp):
        tf = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, stamp,
            timeout=Duration(seconds=0.2))
        t = tf.transform.translation
        return (t.x, t.y)

    def on_depth(self, msg: Image):
        if self.done or self.fx is None:
            return

        # ---- 1. depth at the pixel --------------------------------------
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            d = self.depth_at(img, self.u, self.v)
        except Exception as e:
            self.get_logger().warn(f'depth: {e}')
            return

        # ---- 2. deproject  [your math] ----------------------------------
        X, Y, Z = self.deproject(self.u, self.v, d)
        if not all(math.isfinite(c) for c in (X, Y, Z)):
            self.get_logger().warn('deproject returned non-finite values')
            return

        p = PointStamped()
        p.header.frame_id = self.optical_frame   # because you used K
        p.header.stamp = msg.header.stamp        # the image's stamp, not now()
        p.point.x, p.point.y, p.point.z = X, Y, Z

        # ---- 3. into the map frame --------------------------------------
        try:
            p_map = self.tf_buffer.transform(
                p, self.map_frame, timeout=Duration(seconds=0.2))
            r_xy = self.robot_xy(msg.header.stamp)
        except ConnectivityException:
            self.get_logger().warn(
                f'no path {self.map_frame} <- {self.optical_frame}. '
                'Is localization (AMCL) running?')
            return
        except ExtrapolationException:
            self.get_logger().warn('no transform at that stamp — dropping frame')
            return
        except LookupException as e:
            self.get_logger().error(f'unknown frame: {e}')
            return

        self.get_logger().info(
            f'depth={d:.3f} m  optical=({X:.3f},{Y:.3f},{Z:.3f})  '
            f'map=({p_map.point.x:.3f},{p_map.point.y:.3f},{p_map.point.z:.3f})')

        # ---- 4. marker — LOOK AT THIS BEFORE TRUSTING ANYTHING ----------
        self.publish_marker(p_map)

        if not self.send_goal:
            self.get_logger().info('send_goal=false — marker only, stopping here')
            self.done = True
            return

        # ---- 5. standoff pose  [your math] ------------------------------
        result = self.standoff_pose(
            (p_map.point.x, p_map.point.y), r_xy, self.radius)
        if result is None:
            self.get_logger().warn('already inside the standoff radius — no goal sent')
            self.done = True
            return
        Sx, Sy, yaw = result

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(Sx)
        goal.pose.position.y = float(Sy)
        goal.pose.position.z = 0.0
        # yaw-only rotation about +Z:  q = (axis * sin(t/2), cos(t/2))
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'goal: ({Sx:.3f},{Sy:.3f}) yaw={math.degrees(yaw):.1f} deg')

        self.done = True
        self.nav.waitUntilNav2Active()
        self.nav.goToPose(goal)

        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb:
                self.get_logger().info(
                    f'{fb.distance_remaining:.2f} m remaining', throttle_duration_sec=1.0)

        self.get_logger().info(f'result: {self.nav.getResult()}')


def main():
    rclpy.init()
    node = PixelToGoal()
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

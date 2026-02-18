#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R
from mocap4r2_msgs.msg import RigidBodies
import argparse

from .config import MocapCfg

def safe_quat(q):
    q = np.array(q, dtype=float)
    if not np.all(np.isfinite(q)):
        return None
    n = np.linalg.norm(q)
    if n < 1e-8:
        return None
    return q / n

class MocapPoseToOdom(Node):
    def __init__(self):
        super().__init__('mocap_to_odom')
        self.sub = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_callback,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.pos_buffer = [deque(maxlen=MocapCfg.buffer_size) for _ in range(3)]
        self.quat_buffer = [deque(maxlen=MocapCfg.buffer_size) for _ in range(4)]

        self.t_prev = None
        self.p_prev_filt = None
        self.q_prev_filt = None

    def rigid_bodies_callback(self, msg: RigidBodies):

        if not msg.rigidbodies:
            return

        rb = msg.rigidbodies[0]
        t = self.get_clock().now().nanoseconds * 1e-9

        ### Position
        p = np.array([
            rb.pose.position.x,
            rb.pose.position.y,
            rb.pose.position.z,
        ], dtype=float)

        for i in range(3):
            self.pos_buffer[i].append(p[i])

        p_filt = np.array([np.mean(self.pos_buffer[i]) for i in range(3)], dtype=float)

        ### Rotation
        q = np.array([
            rb.pose.orientation.x,
            rb.pose.orientation.y,
            rb.pose.orientation.z,
            rb.pose.orientation.w,
        ], dtype=float)

        q = safe_quat(q)
        if q is None:
            return

        if self.q_prev_filt is not None:
            if np.dot(self.q_prev_filt, q) < 0:
                q = -q

        for i in range(4):
            self.quat_buffer[i].append(q[i])

        q_filt = np.array([np.mean(self.quat_buffer[i]) for i in range(4)], dtype=float)
        q_filt = safe_quat(q_filt)
        
        if self.t_prev is None:
            self.t_prev = t
            self.p_prev_filt = p_filt
            self.q_prev_filt = q_filt
            return

        ### Elapsed time
        dt = t - self.t_prev
        
        if dt <= 0.0:
            raise ValueError(f"dt must be > 0, got {dt}")

        ### Velocity  
        v = (p_filt - self.p_prev_filt) / dt

        q_prev = self.q_prev_filt

        rotation_prev = R.from_quat(q_prev)
        rotation_curr = R.from_quat(q_filt)

        relative_rotation = rotation_curr * rotation_prev.inv()
        relative_rotvec = relative_rotation.as_rotvec()
        angular_velocity = relative_rotvec / dt

        ### Odom Message
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = MocapCfg.frame
        odom.child_frame_id = 'base_link'

        odom.pose.pose = rb.pose
        odom.pose.pose.position.x = float(p_filt[0])
        odom.pose.pose.position.y = float(p_filt[1])
        odom.pose.pose.position.z = float(p_filt[2])

        odom.pose.pose.orientation.x = float(q_filt[0])
        odom.pose.pose.orientation.y = float(q_filt[1])
        odom.pose.pose.orientation.z = float(q_filt[2])
        odom.pose.pose.orientation.w = float(q_filt[3])

        odom.twist.twist.linear.x = float(v[0])
        odom.twist.twist.linear.y = float(v[1])
        odom.twist.twist.linear.z = float(v[2])

        odom.twist.twist.angular.x = float(angular_velocity[0])
        odom.twist.twist.angular.y = float(angular_velocity[1])
        odom.twist.twist.angular.z = float(angular_velocity[2])

        self.t_prev = t
        self.p_prev_filt = p_filt
        self.q_prev_filt = q_filt

        self.pub.publish(odom)

def main():
    rclpy.init()
    node = MocapPoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

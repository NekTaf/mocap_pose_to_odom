#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R
from mocap4r2_msgs.msg import RigidBodies
import argparse

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('pose_buffer_n', default=10, type=int)
known_args, _ = parser.parse_known_args()
pose_buffer_n_arg = known_args.pose_buffer_n

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
        self.sub = self.create_subscription(RigidBodies, 'rigid_bodies', self.callback, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)

        self.pose_buffer = deque(maxlen=pose_buffer_n_arg)  

    def callback(self, msg: RigidBodies):
        self.pose_buffer.append(msg)
        if len(self.pose_buffer) < 2:
            return

        first = self.pose_buffer[0]
        last  = self.pose_buffer[-1]

        dt = (last.header.stamp.sec - first.header.stamp.sec) + \
             (last.header.stamp.nanosec - first.header.stamp.nanosec) * 1e-9
        if dt <= 0.0:
            return

        p1 = np.array([
            first.rigidbodies[0].pose.position.x,
            first.rigidbodies[0].pose.position.y, 
            first.rigidbodies[0].pose.position.z
            ])
        
        p2 = np.array([
            last.rigidbodies[0].pose.position.x,
            last.rigidbodies[0].pose.position.y,
            last.rigidbodies[0].pose.position.z
            ])
        
        v  = (p2 - p1) / dt

        q1 = [
            first.rigidbodies[0].pose.orientation.x,
            first.rigidbodies[0].pose.orientation.y,
            first.rigidbodies[0].pose.orientation.z,
            first.rigidbodies[0].pose.orientation.w
            ]
        
        q2 = [
            last.rigidbodies[0].pose.orientation.x,
            last.rigidbodies[0].pose.orientation.y, 
            last.rigidbodies[0].pose.orientation.z,
            last.rigidbodies[0].pose.orientation.w
            ]
        
        
        q1 = safe_quat(q1)
        q2 = safe_quat(q2)
        
        if q1 is None or q2 is None:
            return
        
        if np.dot(q1, q2) < 0:
            q2 = -q2

        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        dq = r2 * r1.inv()         
        rotvec = dq.as_rotvec()     
        omega  = rotvec / dt        
        
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.rigidbodies[0].pose

        odom.twist.twist.linear.x  = float(v[0])
        odom.twist.twist.linear.y  = float(v[1])
        odom.twist.twist.linear.z  = float(v[2])
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])

        self.pub.publish(odom)

def main():
    rclpy.init()
    node = MocapPoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

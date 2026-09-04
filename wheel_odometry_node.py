#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# use the classes made by us
try:
    from  robot_kinematics.py import (
        DiffDriveKinematics, 
        MecanumKinematics, 
        ThreeWheelOmniKinematics, 
        FourWheelOmniKinematics
    )
    KINEMATICS_AVAILABLE = True
except ImportError:
    KINEMATICS_AVAILABLE = False
     

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('drive_type', 'diff_drive')
        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('wheel_radius', 0.1)

        drive_type = self.get_parameter('drive_type').value
        L = self.get_parameter('track_width').value
        W = self.get_parameter('wheelbase').value
        R = self.get_parameter('wheel_radius').value

        if not KINEMATICS_AVAILABLE:
            self.get_logger().warn("robot_kinematics.py not found! Using MockKinematics for testing.")
            self.kinematics = MockKinematics(L, W, R)
        else:
            if drive_type == 'diff_drive':
                self.kinematics = DiffDriveKinematics(L, W, R)
            elif drive_type == 'mecanum':
                self.kinematics = MecanumKinematics(L, W, R)
            elif drive_type == '3_wheel_omni':
                self.kinematics = ThreeWheelOmniKinematics(L, W, R)
            elif drive_type == '4_wheel_omni':
                self.kinematics = FourWheelOmniKinematics(L, W, R)
            else:
                self.get_logger().error(f"Unknown drive_type: {drive_type}")
                raise ValueError("Invalid drive type")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/encoder_speeds',
            self.encoder_callback,
            10
        )
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Wheel Odometry Node initialized with {drive_type} kinematics.")

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        w = msg.data
        
        #Protect your node from crashing
        try:
            vx, vy, wz = self.kinematics.forward(w)
        except Exception as e:
            self.get_logger().error(f"Error in kinematics forward() method: {e}")
            return # Skip this odometry cycle if their math fails

        # Local to Global Frame Rotation
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.get_quaternion_from_euler(0, 0, self.theta)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        q = Quaternion()
        q.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        q.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        q.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
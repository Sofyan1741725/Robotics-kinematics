import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


from .robot_kinematics import (
    DiffDriveKinematics,
    MecanumKinematics,
    ThreeWheelOmniKinematics,
    FourWheelOmniKinematics
)


class KinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')

        self.declare_parameter('drive_type', 'diff_drive')
        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('wheelbase', 0.6)
        self.declare_parameter('wheel_radius', 0.1)


        drive_type = self.get_parameter('drive_type').value
        track_width = self.get_parameter('track_width').value
        wheelbase = self.get_parameter('wheelbase').value
        wheel_radius = self.get_parameter('wheel_radius').value

        if drive_type == 'diff_drive':
            self.kinematics = DiffDriveKinematics(
            track_width,
            wheelbase,
            wheel_radius
         )

        elif drive_type == 'mecanum': 
            self.kinematics = MecanumKinematics(
            track_width,
            wheelbase,
            wheel_radius
         )

        elif drive_type == 'three_wheel_omni':
            self.kinematics = ThreeWheelOmniKinematics(
            track_width,
            wheelbase,
            wheel_radius
         )
        elif drive_type == 'four_wheel_omni':
            self.kinematics = FourWheelOmniKinematics(
            track_width,
            wheelbase,
            wheel_radius
         )

        else:
            raise ValueError(f'Unknown drive type: {drive_type}')


         # Subscriber
        self.subscription = self.create_subscription(
             Twist,
             '/cmd_vel',
             self.cmd_vel_callback,
             10
        )


         # Publisher
        self.publisher = self.create_publisher(
            Float64MultiArray,
           '/wheel_setpoints',
            10
        )



    def cmd_vel_callback(self, msg):

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        wheel_speeds = self.kinematics.inverse(vx, vy, wz)

        output_msg = Float64MultiArray()
        output_msg.data = wheel_speeds

        self.publisher.publish(output_msg) 





# main function to run the node
def main(args=None):

    rclpy.init(args=args)

    node = KinematicsNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    


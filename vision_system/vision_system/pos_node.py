import rclpy
from rclpy.node import Node
from stewart_interfaces.msg import Pose
import numpy as np
from vision_system import RotationMatrix

class PositionNode(Node):

    def __init__(self):
        super().__init__('position_node')

        # Subscriber to vision node
        self.subscription = self.create_subscription(
            Pose,
            'topic',
            self.callback,
            10
        )

        # Publisher for estimated platform position
        self.publisher_ = self.create_publisher(Pose, 'platform_pose', 10)

        # Store initial pose
        self.initial_t = None
        self.initial_R = None

    def callback(self, msg):

        t_current = np.array([msg.pos.x, msg.pos.y, msg.pos.z])
        roll = msg.ori.x
        pitch = msg.ori.y
        yaw = msg.ori.z

        R_current = RotationMatrix.euler_to_R(roll, pitch, yaw)

        # Initial Frame
        if self.initial_t is None:
            self.initial_t = t_current
            self.initial_R = R_current
            self.get_logger().info("Initial pose locked")
            return

        # Current Position
        delta_t = t_current - self.initial_t

        # Current Rotation
        delta_R = self.initial_R.T @ R_current
        d_roll, d_pitch, d_yaw = RotationMatrix.R_to_euler(delta_R)

        # Outgoing Message
        msg_out = Pose()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "world_frame"

        msg_out.pos.x = float(delta_t[0])
        msg_out.pos.y = float(delta_t[1])
        msg_out.pos.z = float(delta_t[2])

        msg_out.ori.x = float(d_roll)
        msg_out.ori.y = float(d_pitch)
        msg_out.ori.z = float(d_yaw)

        # Publish
        self.publisher_.publish(msg_out)

        # Debug Print
        self.get_logger().info(
            f"Platform Pose: ({delta_t[0]:.2f}, {delta_t[1]:.2f}, {delta_t[2]:.2f}) | "
            f"RPY: ({np.rad2deg(d_roll):.1f}, {np.rad2deg(d_pitch):.1f}, {np.rad2deg(d_yaw):.1f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
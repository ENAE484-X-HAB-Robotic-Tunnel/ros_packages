import rclpy
from rclpy.node import Node
from stewart_interfaces.msg import Pose
import tf_transformations
from vision_system import RotationMatrix
import math
import numpy as np
from vision_system.state_estimation import Hatch, Platform, Tag, Camera, Renderer

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_system')
        self.publisher_ = self.create_publisher(Pose, 'topic', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Camera Parameters
        fx, fy = 800, 800 
        cx, cy = 300, 200
        cam_R = np.eye(3); 
        cam_t = np.array([0, 0, 0])
        dist_coeffs = np.array([[0.07863, -0.20173, -0.00186, 0.00294, 0.21325]], dtype=np.float64)
        cam0 = Camera(0, [fx, fy, cx, cy], dist_coeffs, cam_R, cam_t)
        self.cams = [cam0]

        # Tag Setup
        tag0 = Tag(0, 0.13, 1*math.pi/3)
        tag1 = Tag(1, 0.13, 3*math.pi/3)
        tag2 = Tag(2, 0.13, 5*math.pi/3)
        self.tags = [tag0, tag1, tag2]

        # Hatch and Platform Setup
        self.hatch    = Hatch(self.tags)
        self.platform = Platform(self.cams, self.hatch)

    def timer_callback(self):
        msg = self.update_target()
        if msg is not None:
            self.publisher_.publish(msg)
            roll = np.rad2deg(msg.ori.x)
            pitch = np.rad2deg(msg.ori.y)
            yaw = np.rad2deg(msg.ori.z)
            self.get_logger().info(f"Published Pose: pos = ({msg.pos.x:.2f},{msg.pos.y:.2f}, {msg.pos.z:.2f}),"
                                    f"rpy = ({roll:.1f}, {pitch:.1f}, {yaw:.1f}) deg")
    def update_target(self):
        target_R, target_t = self.platform.updateTarget()

        # Consideration if no tag is detected (R and t are None)
        if target_R is None or target_t is None:
            return None
        
        x,y,z = target_t
        roll, pitch, yaw = RotationMatrix.R_to_euler(target_R)

        # Creating Message
        msg = Pose()

        # Creating the header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "SP_frame"

        # Message Point
        msg.pos.x = x
        msg.pos.y = y
        msg.pos.z = z

        # Message Orientation
        msg.ori.x = roll
        msg.ori.y = pitch
        msg.ori.z = yaw

        return msg
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



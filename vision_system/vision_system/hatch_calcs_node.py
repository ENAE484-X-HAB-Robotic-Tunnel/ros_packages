import rclpy
from rclpy.node import Node
from stewart_interfaces.msg import Pose
import tf_transformations
from vision_system import RotationMatrix
import math
import numpy as np
from vision_system.state_estimation import Hatch, Platform, Tag, Camera, Renderer

class MathNode(Node):

    def __init__(self):
        super().__init__('math_node')
        self.subscription = self.create_subscription(Pose, '/tag_detections',self.callback, 10)

        # Tag Setup
        tag0 = Tag(0, 0.13, 1*math.pi/3)
        tag1 = Tag(1, 0.13, 3*math.pi/3)
        tag2 = Tag(2, 0.13, 5*math.pi/3)
        self.tags = [tag0, tag1, tag2]

        self.hatch    = Hatch(self.tags)

        cam_R = np.eye(3)
        cam_t = np.array([0, 0, 0])
        self.camera = Camera(0, [800,800,300,200], np.zeros((1,5)), cam_R, cam_t)

        self.platform = Platform([self.camera], self.hatch)

        self.buffer = []


    def callback(self, msg):
        
        target_R, target_t = self.platform.updateTarget()

        avg_R, avg_t = self.platform.averagePoses(poses)

        roll, pitch, yaw = RotationMatrix.R_to_euler(avg_R)

        msg_out = Pose()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "platform_frame"

        msg_out.pos.x, msg_out.pos.y, msg_out.pos.z = avg_t
        msg_out.ori.x = roll
        msg_out.ori.y = pitch
        msg_out.ori.z = yaw

        self.publisher_.publish(msg_out)
        return msg
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



import rclpy
from rclpy.node import Node
from stewart_interfaces.msg import Pose
from vision_system import RotationMatrix
import math
import numpy as np
from vision_system.state_estimation import Tag, Camera

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_system')
        self.publisher_ = self.create_publisher(Pose, '/tag_detections', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Camera Parameters
        fx, fy = 800, 800 
        cx, cy = 300, 200
        cam_R = np.eye(3); 
        cam_t = np.array([0, 0, 0])
        dist_coeffs = np.array([[0.07863, -0.20173, -0.00186, 0.00294, 0.21325]], dtype=np.float64)
        cam0 = Camera(0, [fx, fy, cx, cy], dist_coeffs, cam_R, cam_t)
        self.cams = [cam0]

    def timer_callback(self):
        ret, frame, detections = self.camera.read()
        for det in detections:
            msg = Pose()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.header.frame_id = str(det.tag_id)
            
            # position
            msg.pos.x = float(det.pose_t[0])
            msg.pos.y = float(det.pose_t[1])
            msg.pos.z = float(det.pose_t[2])

            # orientation (rvec)
            msg.ori.x = float(det.pose_R[0][0])
            msg.ori.y = float(det.pose_R[1][0])
            msg.ori.z = float(det.pose_R[2][0])

            

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



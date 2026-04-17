"""
This is a service that solves for the platform's
leg lengths given a pose
"""

from stewart_interfaces.srv import IkRequest
from stewart_helpers.StewartPlatform import StewartPlatform
import stewart_helpers.kinematics as kine

# NOTE: Remove the double folder structure before building

import rclpy
from rclpy.node import Node
import numpy as np

class IkService(Node):
    def __init__(self):
        super().__init__('ik_service')
        self.srv = self.create_service(IkRequest, 'ik_request', self.ik_request_callback)
        self.platform = StewartPlatform(base_r = 7, plat_r = 5, offset_deg = 7)
        self.platform.set_Pose(X_base=[0, 0, 0, 0, 90, 0])
        
    
    def ik_request_callback(self, request, response):
        """
        Get lengths from ik solver, parse into response.lengths
        """
        self.get_logger().info("Service request recieved from client")
        # response.lengths.leg1 = calc_leg1
        goal_point = request.pose.pos
        goal_vector3 = request.pose.ori

        X_goal = np.array([goal_point.x, goal_point.y, goal_point.z, 
            goal_vector3.x, goal_vector3.y, goal_vector3.z])
        
        lengths, _, _, _ = kine.solve_ik(self.platform.X_base, X_goal, self.platform)

        response.lengths.leg0 = lengths[0]
        response.lengths.leg1 = lengths[1]
        response.lengths.leg2 = lengths[2]
        response.lengths.leg3 = lengths[3]
        response.lengths.leg4 = lengths[4]
        response.lengths.leg5 = lengths[5]

        self.get_logger().info(f"leg lengts: {lengths}")

        return response
    
def main():
    rclpy.init()
    ik_service = IkService()

    rclpy.spin(ik_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()   

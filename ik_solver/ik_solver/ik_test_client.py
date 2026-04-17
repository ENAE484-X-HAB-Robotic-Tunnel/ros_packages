import sys
import numpy as np
from stewart_interfaces.srv import IkRequest


import rclpy
from rclpy.node import Node

class IkMinimalClientAsync(Node):
    def __init__(self):
        super().__init__('ik_minimal_client_async')
        self.cli = self.create_client(IkRequest, 'ik_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable. waiting again...')
        self.req = IkRequest.Request()

    def send_request(self):
        X_goal = np.zeros(6)
        # default goal pose

        # actual goal pose
        X_goal[0] = int(sys.argv[1])
        X_goal[1] = int(sys.argv[2])
        X_goal[2] = int(sys.argv[3])
        X_goal[3] = int(sys.argv[4])
        X_goal[4] = int(sys.argv[5])
        X_goal[5] = int(sys.argv[6])

        self.req.pose.pos.x = X_goal[0]
        self.req.pose.pos.y = X_goal[1]
        self.req.pose.pos.z = X_goal[2]
        self.req.pose.ori.x = X_goal[3]
        self.req.pose.ori.y = X_goal[4]
        self.req.pose.ori.z = X_goal[5]

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    ik_client = IkMinimalClientAsync()
    ik_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(ik_client)
        if ik_client.future.done():
            try:
                response = ik_client.future.result()
            except Exception as e:
                ik_client.get_logger().info('Service call failed %r' %(e,))
            else:
                ik_client.get_logger().info(f'Result of Ik Service {response.lengths}')
            break

    ik_client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

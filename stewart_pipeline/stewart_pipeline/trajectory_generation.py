import sys
import threading
import tty
import termios

import numpy as np
import rclpy
from rclpy.node import Node

from stewart_interfaces.msg import Velocity
from stewart_interfaces.srv import IkRequest


class TrajNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.publisher = self.create_publisher(
            Velocity,
            'motor_velocity',
            1
        )

        self.cli = self.create_client(IkRequest, 'ik_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = IkRequest.Request()

        self.X_goal = np.array([float(sys.argv[i]) for i in range(1, 7)])
        self.X_init = np.zeros(6)

        self.trajectory = []
        self.traj_index = 0
        self.last_leg_lengths = None

        # start background thread for keyboard listener
        self.kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.kb_thread.start()


    def send_request(self, X_step):
        self.req.pose.pos.x = X_step[0]
        self.req.pose.pos.y = X_step[1]
        self.req.pose.pos.z = X_step[2]
        self.req.pose.ori.x = X_step[3]
        self.req.pose.ori.y = X_step[4]
        self.req.pose.ori.z = X_step[5]

        event = threading.Event()
        result_container = [None]

        # manager async call
        def done_callback(future):
            result_container[0] = future.result()
            event.set()  # unblock the background thread

        future = self.cli.call_async(self.req)
        future.add_done_callback(done_callback)

        event.wait()  # block keyboard thread until main executor resolves the future
        return result_container[0]


    def trigger_publisher(self):
        # trigger publisher on keyboard press

        # check for edge cases
        if not self.trajectory:
            self.get_logger().warn('Trajectory has not been generated yet.')
            return

        if self.traj_index >= len(self.trajectory):
            self.get_logger().info('End of trajectory reached.')
            return

        # count step
        X_step = self.trajectory[self.traj_index]
        self.get_logger().info(f'Step {self.traj_index + 1}/{len(self.trajectory)}: {X_step}')

        response = self.send_request(X_step)

        if response is None:
            self.get_logger().error('IK service returned no response.')
            return

        msg = Velocity()
        # setup and publish velocity message
        msg.velocity = [
            response.lengths.leg0,
            response.lengths.leg1,
            response.lengths.leg2,
            response.lengths.leg3,
            response.lengths.leg4,
            response.lengths.leg5,
        ]
        self.publisher.publish(msg)

        self.get_logger().info(f'Published leg lengths: {msg.velocity}')
        self.traj_index += 1


    def generate_trajectory(self, X_init, X_goal, N=10):
        # return a straight line extension from init to goal with N steps

        X_init = np.asarray(X_init, dtype=float)
        X_goal = np.asarray(X_goal, dtype=float)

        # linspace rows: shape (N, 6)
        # start=1/N so we step *toward* the goal rather than sitting at init
        alphas = np.linspace(1.0 / N, 1.0, N)
        trajectory = [X_init + alpha * (X_goal - X_init) for alpha in alphas]

        self.get_logger().info(f'Generated trajectory with {N} steps.')
        return trajectory

    # ------------------------------------------------------------------
    # Keyboard listener (runs in its own thread)
    # ------------------------------------------------------------------

    def _keyboard_listener(self):
        """Block on stdin; call trigger_publisher() on every Space press."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        self.get_logger().info('Keyboard listener ready — press SPACE to step through trajectory.')
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self.trigger_publisher()
                elif ch in ('\x03', '\x04'):   # Ctrl-C / Ctrl-D → clean exit
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    if len(sys.argv) < 7:
        print('Usage: traj_node <x> <y> <z> <roll> <pitch> <yaw>')
        sys.exit(1)

    rclpy.init(args=args)
    traj_gen = TrajNode()

    X_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # x y z  roll pitch yaw (rad)
    traj_gen.X_init = np.asarray(X_init)

    traj_gen.trajectory = traj_gen.generate_trajectory(
        traj_gen.X_init, traj_gen.X_goal, N=10
    )

    try:
        rclpy.spin(traj_gen)
    except KeyboardInterrupt:
        pass
    finally:
        traj_gen.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import sys
import rclpy
from rclpy.node import Node
from stewart_interfaces.msg import Pose

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer

class PosGUI(Node,QWidget):
    def __init__(self):
        Node.__init__(self, 'position_gui')
        QWidget.__init__(self)

        # GUI
        self.setWindowTitle("Current Platform Pose")
        self.setFixedSize(400, 200)
        self.pos_label = QLabel("Position: (0.00, 0.00, 0.00)")
        self.ori_label = QLabel("RPY: (0.00, 0.00, 0.00)")

        layout = QVBoxLayout()
        layout.addWidget(self.pos_label)
        layout.addWidget(self.ori_label)
        self.setLayout(layout)

        # Subscription
        self.subscription = self.create_subscription(
            Pose,
            'platform_pose',
            self.callback,
            10
        )

        # PyQt timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(20)

    def callback(self,msg: Pose):
        
        x = msg.pos.x
        y = msg.pos.y
        z = msg.pos.z
        roll = msg.ori.x
        pitch = msg.ori.y
        yaw = msg.ori.z

        self.pos_label.setText(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
        self.ori_label.setText(f"RPY: ({roll:.2f}, {pitch:.2f}, {yaw:.2f})")

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0)

def main():
    rclpy.init()

    app = QApplication(sys.argv)
    gui = PosGUI()
    gui.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()


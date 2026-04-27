# Custom message for Poses. ROS2 builtin pose uses Quaternions
# and we are using rpy
# Here x = roll, y = yaw, z = pitch
# You can use this message as such: (python)
# from stewart_interfaces.msgs import Poses
# pose = Pose()
# pose.pos.x = x
# ...

std_msgs/Header header
geometry_msgs/Point pos
geometry_msgs/Vector3 ori

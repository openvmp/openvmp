from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from control_msgs.action import FollowJointTrajectory


class ActionClientNode(Node):
    def __init__(self, name="action_client"):
        super().__init__(name)
        self.declare_parameter("unit", Parameter.Type.STRING)
        id = self.get_parameter("unit").value

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/openvmp/robot_" + id + "/trajectory_controller/follow_joint_trajectory",
        )

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from control_msgs.action import FollowJointTrajectory

from brake.srv import Command as BrakeCommand


class ActionClientNode(Node):
    def __init__(self, name="action_client"):
        super().__init__(name)
        self.declare_parameter("unit", Parameter.Type.STRING)
        id = self.get_parameter("unit").value

        self.brake_clients = {}
        self.brake_clients["front_turn_table_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_turn_table_joint_brake/command",
        )
        self.brake_clients["front_body_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_body_joint_brake/command",
        )
        self.brake_clients["front_left_arm_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_left_arm_joint_brake/command",
        )
        self.brake_clients["front_left_arm_inner_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_left_arm_inner_joint_brake/command",
        )
        self.brake_clients["front_left_arm_wheel_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_left_arm_wheel_joint_brake/command",
        )
        self.brake_clients["front_right_arm_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_right_arm_joint_brake/command",
        )
        self.brake_clients["front_right_arm_inner_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_right_arm_inner_joint_brake/command",
        )
        self.brake_clients["front_right_arm_wheel_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_front_right_arm_wheel_joint_brake/command",
        )
        self.brake_clients["rear_turn_table_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_turn_table_joint_brake/command",
        )
        self.brake_clients["rear_body_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_body_joint_brake/command",
        )
        self.brake_clients["rear_left_arm_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_left_arm_joint_brake/command",
        )
        self.brake_clients["rear_left_arm_inner_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_left_arm_inner_joint_brake/command",
        )
        self.brake_clients["rear_left_arm_wheel_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_left_arm_wheel_joint_brake/command",
        )
        self.brake_clients["rear_right_arm_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_right_arm_joint_brake/command",
        )
        self.brake_clients["rear_right_arm_inner_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_right_arm_inner_joint_brake/command",
        )
        self.brake_clients["rear_right_arm_wheel_joint"] = self.create_client(
            BrakeCommand,
            "/openvmp/robot_" + id + "/joint_rear_right_arm_inner_joint_brake/command",
        )
        for joint in self.brake_clients:
            self.brake_clients[joint].wait_for_service()

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/openvmp/robot_" + id + "/trajectory_controller/follow_joint_trajectory",
        )

    def brakes_engage_all(self):
        self.brakes_engage_all_internal_(True)

    def brakes_disengage_all(self):
        self.brakes_engage_all_internal_(False)

    def brakes_engage_all_internal_(self, engage):
        for joint in self.brake_clients:
            req = BrakeCommand.Request()
            req.engage = engage
            f = self.brake_clients[joint].call_async(req)
            rclpy.spin_until_future_complete(self, f)

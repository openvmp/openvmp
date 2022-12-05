#!/usr/local/bin/python3

# import time
import math
import rclpy


from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from openvmp_motion_control_py.action import ActionClientNode


class WalkActionClientNode(ActionClientNode):
    def __init__(self):
        super().__init__("walk")

    def send_goal(self):
        goal = FollowJointTrajectory.Goal()

        upside_down = 0  # use '1' for upside down
        upside_down_coeff = 1 - 2 * upside_down
        shoulder_lift = 0.2
        shoulder_turn = 0.3 + 0.1 * upside_down  # wider legs allow for bigger turn
        arm_reach = 0.4

        half_body_base = upside_down * 3.141592653589979
        arm_inner_angle = -1.57 * upside_down_coeff

        position1 = [
            0,
            half_body_base,
            0,
            arm_inner_angle,
            0,
            arm_inner_angle,
            0,
            half_body_base,
            0,
            -arm_inner_angle,
            0,
            -arm_inner_angle,
        ]
        position2 = [
            0,
            half_body_base - shoulder_lift,
            0,
            arm_inner_angle + shoulder_lift,
            0,
            arm_inner_angle - shoulder_lift,
            0,
            half_body_base - shoulder_lift,
            0,
            -arm_inner_angle + shoulder_lift,
            0,
            -arm_inner_angle - shoulder_lift,
        ]
        position3 = [
            shoulder_turn,
            half_body_base,
            arm_reach,
            arm_inner_angle + shoulder_turn * math.sin(arm_reach),
            arm_reach,
            arm_inner_angle + shoulder_turn * math.sin(arm_reach),
            shoulder_turn,
            half_body_base,
            arm_reach,
            -arm_inner_angle - shoulder_turn * math.sin(arm_reach),
            arm_reach,
            -arm_inner_angle - shoulder_turn * math.sin(arm_reach),
        ]
        position4 = [
            0,
            half_body_base + shoulder_lift,
            0,
            arm_inner_angle - shoulder_lift,
            0,
            arm_inner_angle + shoulder_lift,
            0,
            half_body_base + shoulder_lift,
            0,
            -arm_inner_angle - shoulder_lift,
            0,
            -arm_inner_angle + shoulder_lift,
        ]
        position5 = [
            -shoulder_turn,
            half_body_base,
            -arm_reach,
            arm_inner_angle + shoulder_turn * math.sin(arm_reach),
            -arm_reach,
            arm_inner_angle + shoulder_turn * math.sin(arm_reach),
            -shoulder_turn,
            half_body_base,
            -arm_reach,
            -arm_inner_angle - shoulder_turn * math.sin(arm_reach),
            -arm_reach,
            -arm_inner_angle - shoulder_turn * math.sin(arm_reach),
        ]

        goal.trajectory.joint_names.append("front_turn_table_joint")
        goal.trajectory.joint_names.append("front_body_joint")
        goal.trajectory.joint_names.append("front_left_arm_joint")
        goal.trajectory.joint_names.append("front_left_arm_inner_joint")
        goal.trajectory.joint_names.append("front_right_arm_joint")
        goal.trajectory.joint_names.append("front_right_arm_inner_joint")
        goal.trajectory.joint_names.append("rear_turn_table_joint")
        goal.trajectory.joint_names.append("rear_body_joint")
        goal.trajectory.joint_names.append("rear_left_arm_joint")
        goal.trajectory.joint_names.append("rear_left_arm_inner_joint")
        goal.trajectory.joint_names.append("rear_right_arm_joint")
        goal.trajectory.joint_names.append("rear_right_arm_inner_joint")
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=position1,
                velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                time_from_start=Duration(sec=1),
            ),
        ]
        cycles = 3
        for i in range(0, cycles):
            goal.trajectory.points.extend(
                [
                    JointTrajectoryPoint(
                        positions=position2,
                        velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        time_from_start=Duration(sec=2 + 4 * i),
                    ),
                    JointTrajectoryPoint(
                        positions=position3,
                        # velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        time_from_start=Duration(sec=2 + 4 * i + 1),
                    ),
                    JointTrajectoryPoint(
                        positions=position4,
                        velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        time_from_start=Duration(sec=2 + 4 * i + 2),
                    ),
                    JointTrajectoryPoint(
                        positions=position5,
                        # velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        time_from_start=Duration(sec=2 + 4 * i + 3),
                    ),
                ]
            )
        goal.trajectory.points.extend(
            [
                JointTrajectoryPoint(
                    positions=position2,
                    velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    time_from_start=Duration(sec=2 + 4 * cycles + 1),
                ),
                JointTrajectoryPoint(
                    positions=position1,
                    velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    time_from_start=Duration(sec=2 + 4 * cycles + 2),
                ),
            ]
        )

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    action_client = WalkActionClientNode()
    # action_client.brakes_disengage_all()
    future = action_client.send_goal()
    # time.sleep(5)
    # print(asyncio.run(future))
    # print(future.result())
    rclpy.spin_until_future_complete(action_client, future)
    # action_client.brakes_engage_all()
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

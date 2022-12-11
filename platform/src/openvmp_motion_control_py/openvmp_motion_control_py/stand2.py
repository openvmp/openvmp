# import time
import asyncio
import rclpy


from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from openvmp_motion_control_py.action import ActionClientNode


class StandActionClientNode(ActionClientNode):
    def __init__(self):
        super().__init__("stand")

    def send_goal(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names.append("front_turn_table_joint")
        goal.trajectory.joint_names.append("front_body_joint")
        goal.trajectory.joint_names.append("front_left_arm_joint")
        goal.trajectory.joint_names.append("front_left_arm_inner_joint")
        # goal.trajectory.joint_names.append("front_left_arm_wheel_joint")
        goal.trajectory.joint_names.append("front_right_arm_joint")
        goal.trajectory.joint_names.append("front_right_arm_inner_joint")
        # goal.trajectory.joint_names.append("front_right_arm_wheel_joint")
        goal.trajectory.joint_names.append("rear_turn_table_joint")
        goal.trajectory.joint_names.append("rear_body_joint")
        goal.trajectory.joint_names.append("rear_left_arm_joint")
        goal.trajectory.joint_names.append("rear_left_arm_inner_joint")
        # goal.trajectory.joint_names.append("rear_left_arm_wheel_joint")
        goal.trajectory.joint_names.append("rear_right_arm_joint")
        goal.trajectory.joint_names.append("rear_right_arm_inner_joint")
        # goal.trajectory.joint_names.append("rear_right_arm_wheel_joint")
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[
                    0,
                    0,
                    0,
                    0.7,
                    # 0,
                    0,
                    0.7,
                    # 0,
                    0,
                    0,
                    0,
                    0.7,
                    # 0,
                    0,
                    0.7,
                    # 0,
                ],
                # velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                time_from_start=Duration(sec=2),
            ),
            # JointTrajectoryPoint(
            #     positions=[
            #         0,
            #         3.1415,
            #         0,
            #         1.57,
            #         0,
            #         0,
            #         1.57,
            #         0,
            #         0,
            #         3.1415,
            #         0,
            #         -1.57,
            #         0,
            #         0,
            #         -1.57,
            #         0,
            #     ],
            #     velocities=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            #     time_from_start=Duration(sec=4),
            # ),
        ]

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    action_client = StandActionClientNode()
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

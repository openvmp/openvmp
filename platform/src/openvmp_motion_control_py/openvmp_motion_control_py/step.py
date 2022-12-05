#!/usr/local/bin/python3

# import time
import math
import rclpy


from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from openvmp_motion_control_py.action import ActionClientNode


class StepActionClientNode(ActionClientNode):
    def __init__(self):
        super().__init__("step")

    def send_goal(self):
        goal = FollowJointTrajectory.Goal()

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    action_client = StepActionClientNode()
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

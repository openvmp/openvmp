import random
import string


def generate_id():
    return random.choice(string.ascii_uppercase) + "".join(
        random.choice(string.ascii_uppercase + string.digits) for _ in range(3)
    )


def generate_prefix(robot_id):
    return "/openvmp/robot_" + robot_id

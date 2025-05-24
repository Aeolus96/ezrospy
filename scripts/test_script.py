#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus

# End of Imports ------------------------------------------------------------------------------------------------------

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# Test stuff here

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus(verbose=True)

    initial_time = robot.get_clock().now()
    duration = 10.0
    rate = robot.create_rate(1)
    while rclpy.ok():
        robot.yolo_look_for("stop")
        if (robot.get_clock().now() - initial_time).nanoseconds / 1e9 > duration:
            break
        rate.sleep()


# Main Executer (No need to change) -----------------------------------------------------------------------------------
def main(args=None):  # <<< ROS entry point
    try:
        rclpy.init(args=args)
        script()
        rclpy.shutdown()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus

# End of Imports ------------------------------------------------------------------------------------------------------

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# Test Q.0 E-Stop Manual
# 1. Test Goal
    # This test is intended to evaluate safety features of Manual E-Stop.
# 2. Test Setup
    # The following items shall be placed on the road:
    # o Barrel 1 on the side of the road to indicate a starting point at which vehicle is stationary
    # o Barrel 2 on the side of the road to indicate the position where E-Stop button is pressed
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed
    # 5. Judge manually pushes E-Stop and reaches full stop within 3ft (+- 2 inches) form the Barrel 2
    # 6. End test run
# 4. Evaluation
    # Pass Criteria - vehicle reaches full stop within 3 ft (+- 2 inches) from Barrel 2.

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus()
    robot.print_title("Test Q.1 E-Stop Manual")

    robot.print_title("Test Completed")


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

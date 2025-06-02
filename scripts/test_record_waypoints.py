#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus
from modules.ezros_tools import YAMLReader

# End of Imports ------------------------------------------------------------------------------------------------------

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# Test Q.1 E-Stop Manual
# 1. Test Goal
# This test is intended to evaluate safety features of Manual E-Stop.
# 2. Test Setup
# The following items shall be placed on the road:
# o Barrel 1 on the side of the road to indicate a starting point at which vehicle is stationary
# o Barrel 2 on the side of the road to indicate the position where E-Stop button is pressed
# o Barrel 3 on the side of the road to indicate the maxim distance for the vehicle to come to the complete
# stop. The distance between Barrel 2 and Barrel 3 is 14 feet
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle maintains the target speed
# 5. Judge manually pushes E-Stop at Barrel 2
# 6. Vehicle comes to full stop before reaching Barrel 3.
# 7. End test run
# 4. Evaluation
# Pass Criteria - vehicle is able to stop before reaching Barrel 3

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

RECORD_DURATION = 30
WAYPOINT_YAML_PATH = "/home/dev/waypoints/main_S_right_turn_W.yaml"


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus(verbose=False)
    robot.print_title("Record Waypoints")
    # robot.drive_for(speed=1, angle=robot.lane_center, duration=30)
    yml = YAMLReader(WAYPOINT_YAML_PATH)

    for i in range(RECORD_DURATION * 5):
        robot.update_current_waypoint()
        yml[f"waypoint{i}"] = (
            {"lat": robot.waypoint.latitude},
            {"long": robot.waypoint.longitude},
            {"heading": robot.heading if robot.heading is not None else 0},
        )
        # print(robot.waypoint)
        print("heading: ", robot.heading)

        time.sleep(0.2)

    yml.write(WAYPOINT_YAML_PATH)
    robot.print_highlights(f"Saved waypoints to {WAYPOINT_YAML_PATH}.")

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

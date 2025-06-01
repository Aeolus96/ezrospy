#!/usr/bin/env python3

# Test Q.1 Lane Keeping (Go Straight)
# 1. Test Goal
# This test is intended to evaluate if the vehicle is able to stay within lane boundaries, without wheels
# crossing the line or driving on the line.
# 2. Test Setup
# The following items shall be placed on the road:
# - Barrel 1 on the side of the road to indicate a starting point at which vehicle is stationary
# - Barrel 2 about 50 ft away to indicate an ending point.
# - A duct tape’s mark placed 3 ft from the Barrel 2
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle maintains the target speed (between 4 – 5 mph)
# 5. Vehicle reaches full stop within 3 ft (+- 2 inches) from the Barrel 2
# 6. End test run
# 4. Evaluation
# Pass Criteria - vehicle stays within lane boundaries without wheels crossing the lines. Vehicle
# reaches full stop within 3 ft (+- 2 inches) from Barrel 2.

#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus()
    robot.print_title("Test Lane Keeping (Go Straight, Intersection)")

    WAYPOINT_YAML_PATH = "/home/dev/waypoints/N_FW_intersection_straight.yaml"
    robot.waypoints = robot.read_waypoints(WAYPOINT_YAML_PATH)

    robot.drive_for(
        speed=1.5,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "frontright", "min_dist": 0, "max_dist": 5.0},
    )

    robot.stop(duration=5.0)

    robot.drive_for(
        speed=1.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 2.1},
        # duration=10,
    )

    robot.stop(duration=5.0)

    robot.print_title("Test Completed")

    time.sleep(20)
    robot.destroy_node()  # DESTROY EVERYTHING!!!!!


# Main Executer (No need to change) -----------------------------------------------------------------------------------
def main(args=None):  # <<< ROS entry point
    try:
        rclpy.init(args=args)
        script()
        # rclpy.shutdown()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()

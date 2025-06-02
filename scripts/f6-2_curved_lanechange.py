#!/usr/bin/env python3


# Test FIV.3. Parking. Parallel
# 1. Test Goal
# This test is intended to evaluate if a vehicle is able to parallel park into the representative parking space. The
# direction of parallel parking (to the right or to the left) is selected by the judges. The same direction is repeated for
# all 3 attempts.
# 2. Test Setup
# The following items shall be placed on the road:
# - Barrel 1 to indicate starting point at which vehicle is stationary
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle backs off from full stop at Barrel 1
# 4. Vehicle slowly pulls into the parking spot
# 5. Vehicle reaches full stop. It should be fully in the box without crossing any lines.
# 6. End test run
# 4. Evaluation
# Fail Criteria – vehicle crosses solid white line


#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus()
    robot.print_title("Curved Lane Change")

    # Follow turn waypoints
    # WAYPOINT_YAML_PATH = "/home/dev/waypoints/pothole.yaml"
    # robot.waypoints = robot.read_waypoints(WAYPOINT_YAML_PATH)
    # end_waypoint = robot.waypoints[-1]

    robot.drive_for(
        speed=1.0,
        angle=robot.lane_center,
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 4.0},
        # duration=10,
    )

    robot.lane_change_left()

    # WAYPOINT_YAML_PATH = "/home/dev/waypoints/pothole_barrel.yaml"
    # robot.waypoints = robot.read_waypoints(WAYPOINT_YAML_PATH)

    robot.drive_for(
        speed=1.0,
        angle=robot.lane_center,
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 2.1},
        # duration=10,
    )

    robot.stop_rotate()

    robot.print_title("Test Completed")

    time.sleep(10)
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

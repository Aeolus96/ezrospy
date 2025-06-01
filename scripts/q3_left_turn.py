#!/usr/bin/env python3

# Test Q.3 Left Turn
# 1. Test Goal
# This test is intended to evaluate if a vehicle is able to make a left turn across the traffic, merge into
# expected lane and drive within this lane until an obstacle is detected.
# 2. Test setup
# The following items shall be placed on the road:
# - Barrel 1 to indicate a starting point at which vehicle is stationary. The Barrel 1 could be placed
# near the stop bar, or several feet away from the stop bar per judges’ decision.
# - Barrel 2 to indicate an ending point. The barrel is placed about 30 ft away from the stop bar
# in the right lane
# 3.Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle maintains the target speed (between 3 – 5 mph)
# 5. Vehicle turns left across the traffic and merges into correct lane
# 6. Vehicle maintains the target speed (between 3 – 5 mph)
# 7. Vehicle reaches full stop within 5 ft from the Barrel 2
# 8. End test run
# 4. Evaluation
# Pass Criteria - vehicle is able to turn left, merge into correct lane and stop without hitting abarrel
# or crossing boundaries


#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus()
    robot.print_title("Test Left Turn")

    # Follow turn waypoints
    WAYPOINT_YAML_PATH = "/home/dev/waypoints/SE_FS_left_turn.yaml"
    robot.waypoints = robot.read_waypoints(WAYPOINT_YAML_PATH)
    end_waypoint = robot.waypoints[-1]

    robot.drive_for(
        speed=1.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": end_waypoint, "radius": 1.5},
        # duration=10,
    )

    # Follow Lane to Barrel
    robot.drive_for(
        speed=1.0,
        angle=0.05,
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 2.1},
    )

    robot.stop()

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

#!/usr/bin/env python3


# Test FIV.1 Parking. Pull Out
# 1. Test Goal
# This test is intended to evaluate if a vehicle is able to reverse out (or pull out) of the representative parking space.
# The direction of pull out (right-turn-pull-out or left-turn-pull-out) is selected by the judges. The same direction is
# repeated for all 3 attempts.
# 2. Test Setup
# The following items shall be placed on the road:
# - Barrel 1 to indicate a starting point at which vehicle is stationary
# - Barrel 2 to indicate an ending point
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle slowly pulls out from the parking spot
# 5. Vehicle reaches full stop within 3 ft from the Barrel 2
# 6. End test run
# 4. Evaluation
# Fail Criteria – vehicle crosses solid white lines
# Penalties – hits barrel at the end of the run (25 points), stops further than 3 ft from the barrel (10 points)

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
    # WAYPOINT_YAML_PATH = "/home/dev/waypoints/parking_pull_out.yaml"
    # robot.waypoints = robot.read_waypoints(WAYPOINT_YAML_PATH)
    # end_waypoint = robot.waypoints[-1]

    # robot.drive_for(
    #     speed=1.0,
    #     angle=robot.follow_waypoints,
    #     angle_kwargs={"radius": 1.5},
    #     end_function=robot.waypoint_in_range,
    #     end_function_kwargs={"goal_waypoint": end_waypoint, "radius": 1.5},
    #     # duration=10,
    # )

    robot.drive_for(speed=1.0, duration=5.5)

    robot.stop(duration=0.5)

    robot.drive_mode(mode="rotate")

    robot.drive_for(speed=0.0, angle=-0.1, duration=1.0)
    robot.drive_for(speed=0.0, angle=-0.6, duration=1.8)

    robot.stop(duration=0.5)

    robot.drive_mode()

    robot.drive_for(
        speed=1.0,
        angle=0.05,
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 2.1},
    )

    robot.stop(duration=0.5)

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

#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import HeadingEstimator, Schoolbus

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
WAYPOINT_YAML_PATH = "/home/dev/waypoints/merging.yaml"


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus(verbose=True)
    robot.print_title("Test Heading")
    robot.waypoints = robot.read_waypoints(WAYPOINT_YAML_PATH)
    # robot.drive_for(speed=1, angle=robot.lane_center, duration=30)

    # print(robot.waypoint)
    # print(robot.waypoints[-1])
    end_waypoint = robot.waypoints[-1]

    robot.drive_for(
        speed=1.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": end_waypoint, "radius": 1.5},
        # duration=10,
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
        exit(0)

        # rclpy.shutdown()

    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()

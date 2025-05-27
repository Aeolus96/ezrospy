#!/usr/bin/env python3


# Test FVI.2 Curved Road Evaluation. Lane Changing
# 1. Test Goal
    # This test is intended to evaluate if a vehicle is able to perform a lane change on the curved road if
    # obstacles are detected. This test consists of 4 possible case scenarios: changing right lane on the left
    # curve, changing left lane on the left curve, changing right lane on the right curve and changing left lane
    # on the right curve. Any of above scenarios could be chosen as this year’s test.
# 2. Test Setup
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate a starting point at which vehicle is
    # stationary 
    # - Barrel 2 to indicate an obstacle in current lane 
    # - Barrel 3 to indicate an ending point
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 3 – 5 mph)
    # 5. Vehicle detects obstacle (Barrel 2), and safely moves into the next lane
    # 6. Vehicle maintains the target speed in the new lane (between 3 – 5 mph)
    # 7. Vehicle reaches full stop within 3 ft from the obstacle (Barrel 3)
    # 8. End test run
# 4. Evaluation
    # Fail Criteria – crosses white solid line, hits Barrel 2
    # Penalties - hits Barrel 3 at the end of the run (25 points), stops further or closer than 3 ft to the Barrel
    # 2 (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.waypoints = actor.read_waypoints(file_path="/home/dev/curved_waypoint.yaml")


def case_1():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    actor.drive_for(speed=3, angle=35, speed_distance=3)

    actor.drive_for(speed=3, angle=0, speed_distance=2)

    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=3.15)


def case_2():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )
    actor.drive_for(speed=3, angle=-30, speed_distance=3)
    actor.drive_for(speed=3, angle=40, speed_distance=7)
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )
    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=4.0)


def case_3():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    actor.drive_for(speed=3, angle=20, speed_distance=2.25)

    actor.drive_for(speed=3, angle=-40, speed_distance=7)

    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.9)


def case_4():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    actor.drive_for(speed=3, angle=-35, speed_distance=4)

    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.5)


actor.print_title("Q3 - Lane Keeping")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until barrel is detected")

case_2()

actor.print_highlights("Q3 - Lane Keeping Complete!")

# ---------------------------------------------------------------------------------------------------------------------

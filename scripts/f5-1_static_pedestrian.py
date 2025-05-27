#!/usr/bin/env python3


# Test FV.1 Unobstructed STATIC pedestrian detection
# 1. Test Goal
    # This test evaluates ability of Ego vehicle to stop if a pedestrian is detected within boundaries of a current
    # lane.
# 2. Test Setup
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate a starting point at which vehicle is
    # stationary 
    # - Mannequin
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 4 – 5 mph)
    # 5. Vehicle reaches full stop within 5 ft from the Mannequin
    # 6. End test run
# 4. Evaluation
    # Fail Criteria – fails to stop 5 ft from the mannequin, or hits mannequin
    # Penalties – hits barrel at the end of the run (25 points), stops closer than 5 ft from the Mannequin
    # (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here

actor.print_title("F5.1 Static Pedestrian")

actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

# actor.drive_for(speed=1, angle=actor.lane_center, end_function=actor.yolo_look_for("person", 100))

actor.drive_for(
    speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 7.0}
)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1)

actor.print_highlights("Static Pedestrian Complete!")

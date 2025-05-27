#!/usr/bin/env python3


# Test FV.2 Obstructed DYNAMIC pedestrian detection
# 1. Test Goal
    # This test evaluates ability of Ego vehicle to stop if an obstructed by barrel pedestrian (mannequin) suddenly
    # starts crossing an Ego’s vehicle lane.
# 2. Test Setup
    # - Barrel 1 to indicate a starting point at which vehicle is
    # stationary 
    # - Barrel 2 placed in adjacent lane, with Mannequin
    # behind it 
    # - Barrel 3 to indicate an ending point 
    # - Mannequin
# 3. Test Script
    # 1. Begin test run
    # 2. Judge 1 pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 3 – 5 mph)
    # 5. Judge 2 rolls out Mannequin from behind Barrel 2 and stops Mannequin in Ego’s vehicle lane
    # 6. Vehicle reaches full stop within 5 ft from the Mannequin
    # 7. Judge 2 pulls back Mannequin behind Barrel 2
    # 8. Vehicle takes off from the full stop
    # 9. Vehicle maintains the target speed (between 3 – 5 mph)
    # 10. Vehicle reaches full stop within 3 ft from the Barrel 2
    # 11. End test run
# 4. Evaluation
    # Fail Criteria – fails to stop 5 ft from the mannequin, or hits mannequin
    # Penalties – hits barrel at the end of the run (25 points), stops closer than 5 ft from the Mannequin (10
    # points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here

actor.print_title("F5.2 Dynamic Obstructed Pedestrian")

actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.drive_for(
    speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 6.0}
)

while actor.lidar_3d(max_distance=7):
    print("waiting")
    actor.stop_vehicle(duration=2.0, using_brakes=True, softness=0.1, brake_distance=5)


actor.drive_for(
    speed=2.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.5}
)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.95)

actor.print_highlights("Dynamic Pedestrian Complete!")

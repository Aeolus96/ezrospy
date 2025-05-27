#!/usr/bin/env python3


# Test FV.3 STATIC Pedestrian Detection. Lane Changing
# 1. Test Goal
    # This test imitates a situation of a broken vehicle in a current lane with STATIC pedestrian standing in
    # FRONT of barrel(s) in the same lane as Ego vehicle. Ego vehicle must slow down, and safely change into
    # an adjacent lane.
# 2. Test Setup
    # There will be a distance of approximately 85 ft between the mannequin/barrel when mannequin will start
    # crossing the road.
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate starting point at which vehicle is
    # stationary 
    # - Mannequin to indicate obstacle
    # - Barrels 1 and 2 to indicate a broken vehicle in a current
    # lane 
    # - Barrel 3 to indicate end of a run
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 3 -5 mph)
    # 5. Vehicle detects Mannequin
    # 6. Vehicle performs safe transition into the next lane 10 ft away from the Mannequin
    # 7. Vehicle maintains the target speed in the new lane (between 3-5 mph)
    # 8. Vehicle reaches full stop within 3 ft from the obstacle (Barrel 3)
    # 9. End test run
# 4. Evaluation
    # Fail Criteria –hits mannequin, crosses white solid line
    # Penalties – hits barrel at the end of the run (25 points), lane change completed closer than 10 feet from the
    # obstacle (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here

actor.print_title("F5.3 Static Pedestrian Lane Change")

actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.drive_for(
    speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 10.0}
)

actor.drive_for(speed=3, angle=25, speed_distance=2)

actor.drive_for(speed=3, angle=0, speed_distance=3.5)

actor.drive_for(speed=3, angle=-25, speed_distance=2)

actor.drive_for(
    speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.5}
)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.95)

actor.print_highlights("Pedestrian Lane Change Complete!")

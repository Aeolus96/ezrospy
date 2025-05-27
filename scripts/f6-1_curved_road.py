#!/usr/bin/env python3


# Test FVI.1 Curved Road Evaluation. Lane Keeping
# 1. Test Goal
    # This test is intended to evaluate Ego vehicle’s ability to stay in the lane on a curved road, and be able to stop at
    # the obstacle within a current lane. This test consists of 4 possible case scenarios: driving in right lane on the left
    # curve, driving in left lane on the left curve, driving in right lane on the right curve and driving in left lane on the right
    # curve. Any of above scenarios could be chosen at judges’ discretion.
# 2. Test Setup
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate a starting point at which vehicle is
    # stationary 
    # - Barrel 2 to indicate an ending point
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 3 – 5 mph)
    # 5. Vehicle reaches full stop within 3 ft from the Barrel 2
    # 6. End test run
# 4. Evaluation
    # Fail Criteria – crosses white solid line
    # Penalties – hits barrel at the end of the run (25 points), stops further or closer than 3 ft to the Barrel
    # 2 (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("Q3 - Lane Keeping")

# estop.reset()  # Reset E-Stop if needed - Preferably this should done manually via the GUI
estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until barrel is detected")

actor.waypoints = actor.read_waypoints(file_path="/home/dev/curved_waypoint.yaml")

actor.drive_for(
    speed=3.0,
    angle=actor.lane_center,
    end_function=actor.waypoint_in_range,
    end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
)

actor.drive_for(
    speed=3.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 3.5}
)

# actor.drive_for(speed-4.0, angle=actor.lane_center, speed_distance=10.0)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=3.25)

actor.print_highlights("Q3 - Lane Keeping Complete!")

# ---------------------------------------------------------------------------------------------------------------------

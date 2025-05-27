#!/usr/bin/env python3


# Test FIV.2. Parking. Pull In
# 1. Test Goal
  # This test is intended to evaluate if a vehicle is able to pull into a representative parking space. The direction of pull
  # in (right-turn-pull-in or left-turn-pull-in) is selected by the judges. The same direction is repeated for all 3 attempts.
# 2. Test Setup
  # The following items shall be placed on the road:
  # - Barrel 1 to indicate starting point at which vehicle is stationary
# 3. Test Script
  # 1. Begin test run
  # 2. Judge pushes 'start' button
  # 3. Vehicle takes off from full stop at Barrel 1
  # 4. Vehicle slowly pulls into the parking spot
  # 5. Vehicle reaches full stop. It should be fully in the box without crossing any lines
  # 6. End test run
# 4. Evaluation
  # Fail Criteria – vehicle crosses solid white lines


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("F4.2 Parking Pull In")

#actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

# # Pull Out To Right
actor.print_title("F4.1 Parking Pull Out Right")

actor.drive_for(speed=1.5, angle=0.0, speed_distance=4.6)

actor.drive_for(speed=1.5, angle=-30.0, speed_distance=6.0)

actor.drive_for(speed=1.5, angle=0.0, speed_distance=0.5)

# actor.drive_for(
#     speed=1.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 3.0}
# )

# actor.stop_vehicle(duration=5.0, using_brakes=True)

# Pull Out To Right
# actor.print_title("F4.1 Parking PullIn Left")

# actor.drive_for(speed=1.5, angle=0.0, speed_distance=3.5)

# actor.drive_for(speed=1.5, angle=28.0, speed_distance=7.2)

# actor.drive_for(speed=1.5, angle=0.0, speed_distance=0.6)

# actor.drive_for(
#     speed=1.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 3.0}
# )

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1)
actor.print_highlights("Parking Pull In Complete!")

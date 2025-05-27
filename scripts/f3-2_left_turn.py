#!/usr/bin/env python3


# 1. Test Goal
    # This test is intended to evaluate if a vehicle is able to stop at the 'Stop' traffic sign, make a left turn across
    # the traffic, merge into expected lane and drive within this lane until an obstacle is detected.
# 2. Test Setup
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate a starting point at which vehicle is stationary o 'Stop' sign o 'One Way' sign
    # - Barrel 2 to indicate an ending point
    # - Duct tape’s dashed line to indicate 30 cm from the perpendicular line
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 4-5 mph)
    # 5. Vehicle reaches full stop within 30 cm from perpendicular white line next to the "Stop" sign. A
    # vehicle’s bumper should be within two lines at the time when a vehicle reaches full stop.
    # 6. Vehicle takes off from full stop
    # 7. Vehicle turns left across the traffic and merges into correct lane
    # 8. Vehicle maintains the target speed (between 4 – 5 mph)
    # 9. Vehicle reaches full stop within 3 ft from the Barrel 2
    # 10. End test run
# 4. Evaluation
    # Fail Criteria – crosses white parallel lines, crosses perpendicular white line, makes a wrong turn, stops
    # further than 30 cm from a perpendicular line
    # Penalties – hits barrel at the end of the run (25 points), stops further than 3 ft from the barrel (10 points)


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------------------------------------------------------------------------
actor.print_title("F3.2 - Left Turn Intersection")

# estop.reset()  # Reset E-Stop if needed - Preferably this should done manually via the GUI
actor.print_title("F3.1 - Lane Keeping")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until stop sign is detected")

# actor.drive_for(
#     speed=3.0,
#     angle=actor.lane_center,
#     end_function=actor.yolo_look_for,
#     end_function_kwargs={"stop_sign": True, "size": 60},
# )

# actor.drive_for(speed=0.5, angle=actor.lane_center, end_function=white_line)

actor.drive_for(
    speed=4,
    angle=actor.lane_center,
    end_function=actor.lidar_3d,
    end_function_kwargs={"lidar_zone": "right", "max_distance": 4.30},
)

actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, sign_distance=1.75)

actor.print_highlights("Lane keeping until barrel is detected")

actor.drive_for(speed=3.0, angle=0.0, speed_distance=3.5)

actor.drive_for(speed=3.0, angle=25.0, speed_distance=7.5)

actor.drive_for(
    speed=3.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.6}
)

actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, brake_distance=2.95)
actor.print_highlights("F3.2 - Left Turn Complete!")


# Ex: if needed disable or estop can be triggered anywhere

# estop.disable_dbw()  # Disable vehicle control via ROS - one time message
# # NOTE: ^ This is not an E-Stop. It just disables vehicle control
# # OR
# estop.trigger_e_stop()
# # OR
# estop()  # same as above

# ---------------------------------------------------------------------------------------------------------------------

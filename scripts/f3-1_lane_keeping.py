# #!/usr/bin/env python3


# Test FIII.1. Lane Keeping
# 1. Test Goal
    # This test is intended to evaluate if the vehicle is able maneuver within lane boundaries, without wheels
    # crossing the line or driving on the line. Additionally, this test evaluates if the vehicle stops at the “Stop”
    # sign at the intersection, goes straight through intersection, and stops before an obstacle placed on the
    # road.
# 2. Test Setup
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate a starting point at which vehicle is stationary o 'Stop' sign
    # - Barrel 2 to indicate an ending point
    # - Duct tape’s dashed line to indicate 30 cm from the perpendicular line
# 3.Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. Vehicle takes off from full stop at Barrel 1
    # 4. Vehicle maintains the target speed (between 4 – 5 mph)
    # 5. Vehicle reaches full stop within 30 cm from perpendicular white line next to the "Stop" sign. A
    # vehicle’s bumper should be within two lines at the time when a vehicle reaches full stop.
    # 6. Vehicle takes off from full stop
    # 7. Vehicle maintains the target speed (between 4 – 5 mph)
    # 8. Vehicle reaches full stop within 3 ft the Barrel 2
    # 9. End test run
# 4. Evaluation
    # Fail Criteria – crosses white parallel lines, crosses perpendicular white line, stops further than 30 cm
    # from a perpendicular line
    # Penalties – hits barrel at the end of the run (25 points), stops further than 3 ft from the barrel (10 points


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------


# def white_line():
#     return actor.msg_bumper_camera_bumper_line_detected.data


# previous_size = 0


# def slow_to_sign(speed_max: float = 4.0, distance: int = 70, gain: float = 31):
#     # min(brake_target, (1 / max(self.lidar_2d() - brake_distance, 0.1)) / 10)
#     speed = speed_max / max((actor.msg_region_right_closest - distance / gain), 1)
#     return speed


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

actor.drive_for(speed=3.0, angle=0.0, speed_distance=10.0)

actor.drive_for(
    speed=3.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.75}
)

actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, brake_distance=2.95)

# actor.print_highlights("F3.1 - Lane Keeping Complete!")

# ---------------------------------------------------------------------------------------------------------------------

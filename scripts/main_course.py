#!/usr/bin/env python3
import time  # noqa: F401

import rclpy  # type: ignore  # noqa: F401
from rclpy.executors import ExternalShutdownException  # type: ignore  # noqa: F401

from modules.ezros_robot import Schoolbus, Waypoint


# Main Script ---------------------------------------------------------------------------------------------------------
def script():
    robot = Schoolbus()
    robot.print_title("Main Course")

    robot.print_title("Right Turn and Lane Change at Barrel 1")  #############################################

    robot.load_new_waypoints("main_N_right_turn_E")
    robot.drive_for(
        speed=1.5,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 6.0},
    )
    # robot.lane_change_left()
    robot.stop(duration=1.0)
    robot.drive_for(speed=0.0, angle=0.75, duration=2.0)
    robot.drive_for(speed=1.0, angle=0.75, duration=2.0)
    robot.stop(duration=1.0)
    robot.drive_for(speed=1.0, angle=0.0, duration=3.0)
    robot.stop(duration=1.0)
    robot.drive_for(speed=0.0, angle=-0.75, duration=1.5)
    robot.drive_for(speed=1.0, angle=-0.75, duration=1.5)
    robot.stop(duration=1.0)

    robot.drive_for(speed=1.0, angle=0.0, duration=2.0)

    robot.print_title("Lane Change Barrel 2")  #####################################

    robot.load_new_waypoints("main_barrel_2")
    robot.drive_for(
        speed=2.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 7.5},
    )
    # robot.lane_change_right()
    robot.stop(duration=1.0)
    robot.drive_for(speed=0.0, angle=-0.75, duration=2.0)
    robot.drive_for(speed=1.0, angle=-0.75, duration=2.0)
    robot.stop(duration=1.0)
    robot.drive_for(speed=1.0, angle=0.0, duration=5.0)
    robot.stop(duration=1.0)
    robot.drive_for(speed=0.0, angle=0.75, duration=1.5)
    robot.drive_for(speed=1.0, angle=0.75, duration=1.5)
    robot.stop(duration=1.0)

    robot.print_title("Stop at Stop Sign")  #######################################

    robot.load_new_waypoints("main_S_right_turn_W")
    robot.drive_for(
        speed=2.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.2},
        # end_function=robot.object_in_zone,
        # end_function_kwargs={"zone": "frontright", "min_dist": 0, "max_dist": 5.0},
        duration=6,
    )
    
    robot.drive_for(
        speed=0.85,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.2},
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "frontright", "min_dist": 0, "max_dist": 5.0},
    )
    robot.stop(duration=3.0)

    robot.print_title("Right Turn Dynamic Pedestrian")  ######################################

    robot.drive_for(
        speed=1.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.object_in_zone,
        end_function_kwargs={"zone": "front", "min_dist": 0, "max_dist": 6.0},
    )

    robot.stop_rotate()
    robot.stop(duration=5.0)

    for i in range(10):
        robot.stop(
            duration=robot.object_not_in_zone,
            duration_kwargs={"zone": "front", "min_dist": 0, "max_dist": 10.0},
        )

    robot.print_highlights("CLEAR!")

    robot.drive_mode()
    # robot.stop(duration=1.0)

    robot.print_title("Go Through Intersection and Turn Right")  ##############################

    robot.load_new_waypoints("main_SW_right_turn_N")
    end_waypoint = robot.waypoints[-1]
    robot.drive_for(
        speed=2.2,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.2},
        end_function=robot.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": end_waypoint, "radius": 1.5},
    )

    robot.print_title("Lane Change Pothole")  #############################

    robot.load_new_waypoints("main_pothole")
    robot.drive_for(
        speed=1.2,
        angle=robot.lane_center,
        # angle_kwargs={"radius": 1.5},
        end_function=robot.detect_pothole,
        end_function_kwargs={"size": 4.0},
    )

    robot.lane_change_left()

    robot.print_title("Lane Change Tire")  ################################

    robot.load_new_waypoints("main_tire")
    robot.drive_for(
        speed=1.2,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.2},
        end_function=robot.detect_tire,
        end_function_kwargs={"size": 0.4},
    )

    robot.lane_change_right()

    robot.print_title("Right Turn and Stop IF Stop Sign and then Park")  ##########################################

    robot.found_sign = False
    robot.end_condition = False
    end_waypoint = Waypoint(latitude=42.6683714, longitude=-83.2166566, heading=67.40945)

    robot.load_new_waypoints("main_last_turn_right_and_left")
    robot.drive_for(
        speed=2.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.5},
        end_function=robot.check_fake_sign,
        end_function_kwargs={"goal_waypoint": end_waypoint},
    )

    if robot.found_sign:
        print("STOP!")
        robot.stop(duration=3.0)

    end_waypoint = robot.waypoints[-1]
    robot.drive_for(
        speed=2.0,
        angle=robot.follow_waypoints,
        angle_kwargs={"radius": 1.2},
        end_function=robot.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": end_waypoint, "radius": 1.5},
    )

    robot.stop(duration=2.0)

    robot.print_title("Complete")

    # End of Script - - - - -
    time.sleep(10)
    robot.destroy_node()  # DESTROY EVERYTHING!!!!!


# Main Executer (No need to change) -----------------------------------------------------------------------------------
def main(args=None):  # <<< ROS entry point
    try:
        rclpy.init(args=args)
        script()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()

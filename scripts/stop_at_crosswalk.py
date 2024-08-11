#!/usr/bin/env python3

from ezrospy import ezros_tools
from ezrospy.ezros_robot import EzRobot

import time

# Get the robot config file path using helper function
robot1_config = ezros_tools.package_path() + "/cfg/robot1.yaml"

# Initialize the EzRobot using the config file
robot1 = EzRobot("robot1", robot1_config)

# Use simple drive for X condition built-in function
robot1.drive_for(speed=1.0, duration=1.0)  # Drive at speed 1.0m/s for 1 seconds

# Stop the robot instantly with built-in stop function
robot1.stop()


# Build behaviors to "extend" inputs or conditions using various functions:


def stop_at_crosswalk() -> bool:
    """Returns True if the robot identifies a crosswalk"""

    # This feature need not be implemented in this very script.
    # An external ROS node can simply keep checking for the presence of a crosswalk independently
    # When found, this independent node publishes True

    robot1.print_highlights(f"Crosswalk --> {robot1.msg_detected_crosswalk}")
    return True if robot1.msg_detected_crosswalk else False


# The end condition can be set to call a function
# NOTE: provide function name only, arguments can be passed as a dictionary to end_function_kwargs
robot1.drive_for(speed=0.5, end_function=stop_at_crosswalk)


def crosswalk_is_clear(wait_for=10.0) -> bool:
    """Returns True if the crosswalk is clear drive over"""

    # Some function to check if the crosswalk is clear. Time used as an example
    if time.time() - initial_time > wait_for:
        robot1.print_highlights("Crosswalk is clear!")
        return True
    else:
        return False


robot1.print_highlights("Checking if crosswalk is clear...")
initial_time = time.time()

# The duration can be set to call a function with arguments
robot1.stop(duration=crosswalk_is_clear, duration_kwargs={"wait_for": 3.0})

robot1.drive_for(speed=0.5, speed_distance=2.0)  # Start driving again for X "speed derived" meters

robot1.stop(duration=2.0)  # Send zeros to robot for X seconds. Real robots take time to come to a complete stop!

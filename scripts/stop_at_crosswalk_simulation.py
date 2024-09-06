#!/usr/bin/env python3

import time

from ezrospy import ezros_tools
from ezrospy.ezros_robot import EzRobot

# Use a helper function to get full path of this ROS package and add the config file path to it
robot1_config = ezros_tools.package_path() + "/cfg/robot1.yaml"
robot1 = EzRobot("robot1", robot1_config)  # Initialize the EzRobot using the config file
# This file contains definitions of the robot's namespace, publishers, subscribers, etc. used for initialization


# Now we can use the simple built-in function to drive the robot using X parameters for/until Y condition is met
robot1.drive_for(speed=1.0, duration=1.0)  # Drive at speed 1.0m/s for 1 seconds
robot1.stop()  # Stop the robot instantly with the built-in stop function


# Now we can build "behaviors" to "extend" input parameters or end conditions using customizable functions that can
# be passed to drive_for() or stop() methods:
def stop_at_crosswalk(target_percent: int = 50) -> bool:
    """Returns True if the robot identifies a crosswalk"""

    # The processing for this feature need not be implemented in this very script or within the robot class.
    # An external ROS node, like the white_pixel_percent node used for this demo, can keep checking for the presence
    # of a crosswalk concurrently with other nodes. When a crosswalk is found, it publishes True to it's output topic.
    # # We've added this output topic in the robot's config file as one of the subscribers and can be accessed as such:

    # NOTE: The ROS messages are retrieved as is. Use proper attribute access (msg.data or msg.twist.linear.x)
    return True if robot1.msg_white_percent.data > target_percent else False


# Now we can use the function to drive the robot until the crosswalk is detected
robot1.print_highlights("Driving until crosswalk is detected...")  # Print a "checkpoint" message to help with debugging
robot1.drive_for(speed=0.15, end_function=stop_at_crosswalk, end_function_kwargs={"target_percent": 40})
# NOTE: provide function name only, arguments can be passed as a dictionary to end_function_kwargs


# Once the crosswalk is detected, we can stop the robot until the crosswalk is clear to drive over
robot1.print_highlights("Stopping until crosswalk is clear...")  # Print a "checkpoint" message to help with debugging


# For demo purposes, we will stop the robot for X seconds and then start driving again
def crosswalk_is_clear(wait_for=10.0) -> bool:
    """Returns True if the crosswalk is clear drive over"""

    if time.time() - initial_time > wait_for:
        robot1.print_highlights("Crosswalk is clear!")
        return True
    else:
        return False


initial_time = time.time()
robot1.stop(duration=crosswalk_is_clear)
robot1.drive_for(speed=0.5, speed_distance=1.0)  # Start driving again for X "speed derived" meters
robot1.stop(duration=3.0)  # Another method of stopping the robot by using the built-in time duration function.
# Beacause real robots take time to come to a complete stop from different speeds or on a slope, you are able to
# create your own stopping behaviors. This dynamic behavior flexibility is very useful in many scripted simulations
# and control applications. Overall, this makes using scripted meta-behaviors intuitive and easy to develop.

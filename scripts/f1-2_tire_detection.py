#!/usr/bin/env python3

# Test FI.2 Tire Detection
# 1. Test Goal
    # This test is intended to evaluate detection of a small item present in a current lane using traditional
    # Machine Vision algorithms. A GUI interface with extracted shape of a tire MUST be present
    # during a run. There are NO PENALTIES for crossing or moving over a line.
# 2. Test Setup
    # The following items shall be placed on the road:
    # o Barrel 1 to indicate starting point at which vehicle is stationary
# 3.Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. The extracted tire is present on the screen
    # 4. End test run
# 4. Evaluation
    # Fail Criteria – no GUI interface is present during the run, incorrect identification of the tire
    # Penalties – no penalties for crossing or moving over the lines, if vehicle is moving during the test

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("F1.2 Tire Detection")

actor.print_highlights("Detecting Pedestrians")

count = 0

while count < 300:
    actor.yolo_look_for(tire=True, size=100)
    count += 1
    rospy.sleep(0.1)

actor.print_highlights("Tire Detection Complete!")

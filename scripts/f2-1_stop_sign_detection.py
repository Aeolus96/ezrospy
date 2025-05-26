#!/usr/bin/env python3

# Test FII.1 Stop Sign Detection
# 1. Test Goal
    # This test is intended to evaluate Stop Sign classification detection and accuracy. Any type of
    # algorithm could be used for this test. Before test, a RANDOM picture might be put on top of a
    # STOP sign. A forgery sign could be red in color with random letters, be a different color with same
    # letters, or be a different picture. Examples used in the previous years: “Soup” and “IGVC” signs. A
    # GUI interface shell display a relevant classification as “Stop Sign” or “Unknown”. There are NO
    # PENALTIES for crossing or moving over a lane.
# 2. Test Setup
    # o Barrel 1 to indicate starting point at which vehicle is
    # stationary o 3 different “Stop” signs are being tested
    # randomly
# 3. Test Script
    # 1. Begin test run
    # 2. The 1 st judge inside of the vehicle pushes a 'start' button
    # 3. The extracted sign is shown on the screen with a correct identification
    # 4. The 2nd judge removes a current sign, and puts a new “stop” sign. It could be a fake or a real sign.
    # 5. The extracted sign is shown on the screen with a correct identification
    # 6. The 2nd judge removes a current sign, and puts a new “stop” sign. It could be a fake or a real sign.
    # 7. End test run
# 4. Evaluation
    # Fail Criteria – no GUI interface is present during the run, incorrect identification of any of 3 signs,
    # keyboard touching between the sign changes. To pass the test, all 3 signs must be correctly
    # identified.
    # Penalties – no penalties for crossing or moving over the lines, if vehicle is moving during the test

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("F2.1 Stop Sign Detection")

actor.print_highlights("Detecting Stop Signs")

count = 0

while count < 300:
    actor.yolo_look_for(stop_sign=True, size=100)
    count += 1
    rospy.sleep(0.1)

actor.print_highlights("Stop Sign Detection Complete!")

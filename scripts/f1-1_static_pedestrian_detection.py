# #!/usr/bin/env python3

# Test FI.1 Static Pedestrian Detection
# 1. Test Goal
    # This test is intended to evaluate detection of a mannequin using traditional Machine Vision
    # algorithms. A mannequin wears ORANGE construction vest. A GUI interface with extracted
    # orange blob MUST be present during a run. There are NO PENALTIES for crossing or moving
    # over a line.
# 2. Test Setup
    # The following items shall be placed on the road:
    # - Barrel 1 to indicate starting point at which vehicle is stationary
# 3. Test Script
    # 1. Begin test run
    # 2. Judge pushes 'start' button
    # 3. The extracted orange blob is present on the screen.
    # 4. End test run
# 4. Evaluation
    # Fail Criteria – no GUI interface is present during the run, incorrect identification of the shape/object
    # Penalties – no penalties for crossing or moving over the lines, in case if vehicle is moving during the test

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("F1.1 Static Pedestrian Detection")

actor.print_highlights("Detecting Pedestrians")

count = 0

while count < 1000:
    actor.yolo_look_for(person=True, size=100)
    count += 1
    rospy.sleep(0.1)

actor.print_highlights("Static Pedestrian Detection Complete!")

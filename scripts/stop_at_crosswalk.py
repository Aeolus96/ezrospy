#!/usr/bin/env python3

from ezrospy import ezros_tools
from ezros_robot import EzRobot
import time

robot1_config = ezros_tools.package_path() + "/cfg/robot1.yaml"

robot1 = EzRobot("robot1", robot1_config)

robot1.drive_for(speed=1.0, duration=2.0)

robot1.stop(duration=2.0)

time.sleep(10)

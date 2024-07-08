#!/usr/bin/env python3

from ezrospy import ezros_tools
import time

test_config = ezros_tools.package_path() + "/cfg/test.yaml"

test = ezros_tools.EzRosNode(name="ScriptTest", config_file_path=test_config)

time.sleep(10)

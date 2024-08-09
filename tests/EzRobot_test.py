#!/usr/bin/env python3

import os
import sys
import time

import yaml

# Adjust the path to include the module folder
sys.path.insert(0, sys.path[0] + "/../src/")
print("new path to get to ezrospy modules: ", sys.path[0])

from ezrospy.ezros_robot import EzRobot  # noqa: E402

def create_test_yaml(file_path):
    """Create a sample YAML configuration file for testing."""
    sample_data = {
        "namespace": "test_namespace",
        "publishers": [{"name": "test_pub", "topic": "test_topic", "msg_file": "std_msgs.msg", "msg_type": "String"}],
        "subscribers": [{"topic": "test_topic", "msg_file": "std_msgs.msg", "msg_type": "String"}],
    }
    with open(file_path, "w") as file:
        yaml.safe_dump(sample_data, file)
    print(f"YAML: Created test configuration at {file_path}")

# Create test YAML file
test_config = os.path.join(os.path.dirname(__file__), "test.yaml")
create_test_yaml(test_config)

test_robot = EzRobot(name="Test_Robot", config_file_path=test_config, verbose=True)

time.sleep(2)

# Remove test YAML file
os.remove(test_config)

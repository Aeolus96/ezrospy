#!/usr/bin/env python3

import os
import sys
import time

import yaml
from std_msgs.msg import String

# Adjust the path to include the module folder
sys.path.insert(0, sys.path[0] + "/../src/")
print("new path to get to ezrospy modules: ", sys.path[0])

from ezrospy.ezros_tools import EzRosNode  # noqa: E402


# Test Functions ------------------------------------------------------------------------------------------------------
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


def test_ezrosnode():
    """Test the EzRosNode class."""

    # Create test YAML file
    test_config = os.path.join(os.path.dirname(__file__), "test.yaml")
    create_test_yaml(test_config)

    # Create an instance of EzRosNode
    test_node = EzRosNode(name="Test", config_file_path=test_config, verbose=True)
    time.sleep(1)  # Allow some time for ROS node to start and register publishers and subscribers with Core

    # Check if namespace is correctly set
    assert test_node.namespace == "test_namespace", "Namespace not set correctly"

    # Check if publishers and subscribers are initialized
    assert hasattr(test_node, "test_pub"), "Publisher 'test_pub' not initialized"
    assert hasattr(test_node, "msg_test_topic"), "Subscriber 'test_topic' not initialized"

    # Check if publishers and subscribers are working
    test_msg = String()
    test_msg.data = "Aloha!"
    test_node.test_pub.publish(test_msg)
    print(f"Publisher sent message: {test_msg.data}")
    time.sleep(0.1)  # Allow some time for subscriber to receive message
    assert test_node.msg_test_topic.data == test_msg.data, "Subscriber did not receive message"
    print(f"Subscriber received message: {test_node.msg_test_topic.data}")

    print(f"{test_node.name}: All tests passed!")

    # Remove test YAML file
    os.remove(test_config)


if __name__ == "__main__":
    try:
        test_ezrosnode()

    except KeyboardInterrupt:
        print("EzRosNode: Test interrupted by user")

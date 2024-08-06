#!/usr/bin/env python3

import os
import time

import rospy
import yaml
from munch import Munch, munchify, unmunchify
from std_msgs.msg import String


class YAMLReader(Munch):
    """Read YAML files and store the data in an attribute style access object."""

    def __init__(self, file_path):
        """Initialize the YAML Reader"""
        super().__init__()
        self.read(file_path)  # Automatically read YAML data on initialization

    def read(self, file_path=None):
        """Read YAML data from file and munchify it"""
        if file_path is None:
            raise ValueError("YAML: No file path provided")
        try:
            with open(file_path, "r") as file:
                print(f"YAML: Loading file '{file_path}'")
                self.update(munchify(yaml.safe_load(file)))
        except Exception as e:
            print(f"YAML: Failed to load file '{file_path}': {e}")

    def write(self, file_path=None):
        """Write the YAML data to a file"""
        if file_path is None:
            raise ValueError("YAML: No file path provided")
        try:
            with open(file_path, "w") as file:
                yaml.safe_dump(unmunchify(self), file)
                print(f"YAML: File successfully written to '{file_path}'")
        except Exception as e:
            print(f"YAML: Failed to write file '{file_path}': {e}")

    # End of class ----------------------------------------------------------------------------------------------------


class EzRosNode:
    """Custom ROS Tooling that simplifies setup and scripting for ROS nodes.\n
    NOTICE: Only one instance of this class should be created per python script."""

    def __init__(self, name: str = "EzRosNode", config_file_path: str = None, verbose: bool = False):
        """Initializes ROS node, makes publishers and subscribers as described in YAML file @ config_file_path.\n
        IMPORTANT: Full path from root is required!\n
        NOTICE: Only one instance of this class should be created per python script"""

        if config_file_path is None:
            raise ValueError(f"{name}: No config file provided")

        self.name = name
        self.config_file_path = config_file_path
        self.verbose = verbose
        if self._load_config():
            self._run_node()

    def _load_config(self) -> bool:
        """Loads configuration from YAML file"""

        try:
            config_parameters = YAMLReader(file_path=self.config_file_path)
            for key, value in config_parameters.items():
                setattr(self, key, value)
            # Default namespace is relative unless specified
            self.namespace = getattr(config_parameters, "namespace", "")
            if self.verbose:
                print(f"{self.name}: Loaded config file '{self.config_file_path}'")
                print(f"{self.name}: Namespace is '{self.namespace}'")
            return True
        except Exception as e:
            print(f"{self.name}: {e}")
            return False

    def _initialize_publishers(self) -> None:
        """Initializes publishers as described in YAML file @ config_file_path"""

        for publisher in self.publishers:
            exec(f"from {publisher.msg_file} import {publisher.msg_type}")
            topic = str(publisher.topic)
            if not topic.startswith("/"):  # Add namespace if topic is not absolute
                topic = (self.namespace if self.namespace.endswith("/") else self.namespace + "/") + topic
            msg = eval(publisher.msg_type)
            temp_publisher = rospy.Publisher(topic, msg, queue_size=1)
            setattr(self, publisher.name, temp_publisher)  # publisher.name is defined in the YAML file
            if self.verbose:
                print(f"{self.name}: Initialized publisher '{publisher.name}' on topic '{topic}'")

    def _initialize_subscribers(self) -> None:
        """Initializes subscribers as described in YAML file @ config_file_path"""

        for subscriber in self.subscribers:
            exec(f"from {subscriber.msg_file} import {subscriber.msg_type}")
            topic = str(subscriber.topic)
            if topic.startswith("/"):  # Add namespace if topic is not absolute
                name = f"msg_{subscriber.topic.replace('/', '_')}"  # Absolute topic name
            else:
                name = f"msg_{subscriber.topic.replace('/', '_')}"  # Relative topic name
                topic = (self.namespace if self.namespace.endswith("/") else self.namespace + "/") + topic
            msg = eval(subscriber.msg_type)
            msg_instance = msg()
            setattr(self, name, msg_instance)  # Initialize the instance attributes with default message objects
            rospy.Subscriber(topic, msg, callback=self._any_callback, callback_args=name, queue_size=1)
            if self.verbose:
                print(f"{self.name}: Initialized subscriber '{name}' on topic '{topic}'")

    def _any_callback(self, msg, name) -> None:
        """With the power of interpreted types, retrieve "Any" type of ROS messages from this callback function"""

        setattr(self, name, msg)

    def _shutdown_hook(self):
        """Shutdown hook for ROS node"""

        self.print_title(f"{self.name} is shutting down")

    def _run_node(self) -> None:
        """Starts ROS Node and initializes publishers and subscribers"""

        node_name = self.name if self.name == "EzRosNode" else "EzRosNode_" + self.name
        rospy.init_node(node_name, anonymous=True)
        rospy.on_shutdown(self._shutdown_hook)
        self._initialize_publishers()
        self._initialize_subscribers()
        self.print_title("Node Initialized")

    def print_highlights(self, text: str) -> None:
        """Prints text to stdout in a centered "highlight" style format"""

        text = f"{self.name}: {text}"
        width = 80
        padding = max(((width - len(text)) // 2) - 2, 3)
        print(f"{'-' * padding}| {text} |{'-' * padding}")

    def print_title(self, title: str) -> None:
        """Prints text to stdout in a centered "title" style format"""

        title = f"{self.name}: {title}"
        width = 80
        padding = max((width - len(title)) // 2, 3)
        print("=" * width)
        print(f"{' ' * padding}{title.upper()}{' ' * padding}")
        print("=" * width)

    # End of Class ----------------------------------------------------------------------------------------------------


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


if __name__ == "__main__":
    try:
        test_ezrosnode()

    except KeyboardInterrupt:
        print("EzRosNode: Test interrupted by user")

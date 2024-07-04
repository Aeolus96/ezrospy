#!/usr/bin/env python3

"""--------------------------------------------------------------------------------------------------------------------
EzRosPy Tooling:
    - Python Script Player
    - YAML I/O
    - GPS Waypoint Utilities
    - ROS Node Lifecycle Manager

--------------------------------------------------------------------------------------------------------------------"""

import os
import signal
import socket
import subprocess
import time
from math import asin, atan2, cos, degrees, radians, sin, sqrt
from threading import Thread

import rospkg
import rospy
import yaml
from munch import Munch, munchify, unmunchify

# End of Imports ------------------------------------------------------------------------------------------------------


def package_path(package_name: str = "ezrospy") -> str:
    """Get a ROS package's full path"""

    return rospkg.RosPack().get_path(package_name)


def get_local_ip():
    """Uses socket to get the machine ip on network"""

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        local_ip = s.getsockname()[0]
    except Exception as e:
        print(e)
        local_ip = "127.0.0.1"
    finally:
        s.close()
    return local_ip


class ScriptPlayer:
    """Executes Python scripts as a subprocess."""

    def __init__(self, directory: str) -> None:
        """Initialize the Script Player"""

        self.active_directory = directory  # Full path to the directory
        self.file_list = ""  # List of files in the active directory
        self.file_selected = ""  # File name with extension
        self.process = None  # Process object
        self.process_is_running = False  # Flag to indicate if a script is running
        self.process_output_text = []  # List to store the output text
        self.process_return_code = ""  # Return code of the process

    def __del__(self) -> None:
        """Cleanup method"""

        self.stop_script()

    def load_files(self) -> str:
        """Make a list of files in the active directory"""

        # Makes a list of Python scripts in the specified directory
        self.file_list = [file for file in os.listdir(self.active_directory) if file.endswith(".py")]
        if "gui.py" in self.file_list:  # Remove the GUI script if it exists in the same directory
            self.file_list.remove("gui.py")
        self.file_list.sort()  # Sort the list alphabetically
        self.file_selected = ""  # Clear the selected file so that the user can select a new one
        return "Directory loaded"

    def execute(self) -> str:
        """Execute the selected file in a separate process"""

        if self.process_is_running:
            return "A script is already running. Please stop it first."
        if self.file_selected == "":
            return "Please select a script to execute"
        # Set the process flag to True and clear the output text
        self.process_is_running = True
        self.process_output_text = [f"INITIALIZING __{self.file_selected}__ ..."]
        try:
            self.process = subprocess.Popen(
                [f"python3 -u {self.active_directory}{self.file_selected}"],  # NOTE:
                shell=True,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                text=True,
            )
            # NOTE: ------------------------------
            # python3 -u forces unbuffered output
            # shell=True executes in a shell subprocess which inherits the current environment
            # preexec_fn=os.setsid sets a known process group id. IMPORTANT for killing all child processes
            # stdout=subprocess.PIPE captures print() output
            # stdout is not logged by default. ROS logging should be used to maintain logs
            # stderr=subprocess.STDOUT captures Raised Errors and Exceptions
            # universal_newlines=True ensures that every new output is its own new string
            # text=True ensures that the output is string and not bytes

            # Use multithreading to monitor the process state and read stdout and stderr streams
            Thread(target=self.monitor_process, daemon=True).start()
            Thread(target=self.read_output, daemon=True).start()
            self.process_output_text.append(f"SPAWNED __{self.file_selected}__ @ {os.getpgid(self.process.pid)} ...")
            return "Script is running"
        except Exception as e:
            self.process_is_running = False
            self.process = None
            return f"---------------- Error in script ----------------\n{e}"

    def monitor_process(self) -> None:
        """Monitor the process and gets the return code"""

        try:
            self.process_return_code = self.process.wait()  # Wait until the process finishes to get the return code
            self.process_is_running = False
            self.process_output_text.append("SCRIPT FINISHED ...")
            self.process_output_text.append(f"RETURN CODE: {self.process_return_code}")
        except Exception as e:
            print(f"---------------- Exception in monitor_process ----------------\n{e}")
            self.process_is_running = False

    def read_output(self) -> None:
        """Read output stream and add lines into process_output_text stream"""

        try:
            while self.process.poll() is None:
                line = self.process.stdout.readline()
                if line:
                    print(line, end="")  # Display the line in the shell
                    self.process_output_text.append(line.rstrip("\n"))  # Remove the newline character and append
            # When EOF is reached (process is terminated), set the running flag to False just in case
            self.process_is_running = False
        except Exception as e:
            print(f"---------------- Exception in read_output ----------------\n{e}")
            self.process_is_running = False

    def stop_script(self, timeout: float = 0.25) -> str:
        """Stop the currently running script"""

        terminate_text = """
        --------------------------------------------------------
                           Terminating script...
        --------------------------------------------------------
        """
        self.process_output_text.append(terminate_text)
        print(terminate_text)
        try:
            if self.process is None:  # Process has already been successfully terminated
                self.process_is_running = False
                return "Script successfully stopped"
            if self.process and self.process_is_running:
                gpid = os.getpgid(self.process.pid)  # Get the process group ID. Includes shell and its child processes
                os.killpg(gpid, signal.SIGTERM)  # SIGTERM shell and its child processes
                time.sleep(timeout)  # Wait for the process to terminate
                if self.process.poll() is None:  # If process is still running
                    os.killpg(gpid, signal.SIGKILL)  # SIGKILL shell and its child processes
                self.process_is_running = False
                self.process = None
                return "Script successfully stopped"
            else:
                return "No script is currently running"
        except Exception as e:
            self.process_is_running = False
            self.process = None
            return f"---------------- Error/Exception in script termination ----------------\n{e}"

    # End of class ----------------------------------------------------------------------------------------------------


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


class Waypoint:
    """Class for waypoint and related operations"""

    def __init__(self, latitude: float = None, longitude: float = None, heading: float = None) -> None:
        """Sets waypoint latitude and longitude (decimal degrees) and current heading (degrees)"""

        if latitude is None or longitude is None:
            print("Waypoint: Please specify latitude and longitude for the waypoint")
            return

        self.latitude = latitude
        self.radian_latitude = radians(self.latitude)
        self.longitude = longitude
        self.radian_longitude = radians(self.longitude)
        self.heading = heading

    def __str__(self) -> str:
        """Returns a string representation of the waypoint"""
        return f"Waypoint: {self.latitude:.6f}, {self.longitude:.6f}, {self.heading:.3f}"

    def update(self, latitude: float, longitude: float, heading: float = None) -> None:
        """Updates waypoint latitude, longitude, and heading (decimal degrees)"""

        self.latitude = latitude
        self.radian_latitude = radians(self.latitude)
        self.longitude = longitude
        self.radian_longitude = radians(self.longitude)
        self.heading = heading

    def distance_to(self, goal: "Waypoint") -> float:
        """Returns Haversine distance between two waypoints (meters)"""

        radius_earth = 6371000  # meters
        phi_1 = self.radian_latitude
        lambda_1 = self.radian_longitude
        phi_2 = goal.radian_latitude
        lambda_2 = goal.radian_longitude
        delta_lambda = lambda_2 - lambda_1
        delta_phi = phi_2 - phi_1
        a = sin(delta_phi / 2) ** 2 + cos(phi_1) * cos(phi_2) * sin(delta_lambda / 2) ** 2
        return 2 * radius_earth * asin(sqrt(a))  # distance in meters

    def absolute_bearing_with(self, goal: "Waypoint") -> float:
        """Returns absolute bearing between two waypoints (degrees)"""

        phi_1 = self.radian_latitude
        lambda_1 = self.radian_longitude
        phi_2 = goal.radian_latitude
        lambda_2 = goal.radian_longitude
        delta_lambda = lambda_2 - lambda_1
        x = sin(delta_lambda) * cos(phi_2)
        y = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(delta_lambda)
        return (degrees(atan2(x, y)) + 360) % 360  # Normalized to 0-360

    def relative_bearing_with(self, goal: "Waypoint") -> float:
        """Returns the relative bearing from the current heading to the goal waypoint in degrees"""

        if self.heading is None:
            print("Waypoint: Please specify current heading to calculate relative bearing")
            return 0
        elif goal.heading is None:
            print("Waypoint: Please specify goal heading to calculate relative bearing")
            return 0
        absolute_bearing = self.absolute_bearing_with(goal)
        relative_bearing = self.heading - absolute_bearing
        if relative_bearing < -180:  # Normalize to -180 or 180
            relative_bearing += 360
        elif relative_bearing > 180:
            relative_bearing -= 360
        return relative_bearing

    # End of Class ----------------------------------------------------------------------------------------------------


class EzRosNode:
    """Custom ROS Tooling that simplifies setup and scripting for ROS nodes"""

    def __init__(self, name: str = "EzRosNode", config_file_path: str = None):
        """Initializes ROS node, makes publishers and subscribers as described in YAML file @ config_file_path"""

        if config_file_path is None:
            raise ValueError(f"{name}: No config file provided")

        self.name = name
        self.config_file_path = config_file_path
        self.load_config()
        self.thread = Thread(target=self.run_node)  # Start ROS Node in a separate thread
        self.thread.start()

    def __del__(self):
        """Destructor for ROS node"""

        rospy.signal_shutdown(f"{self.name} destructor called")

    def shutdown_hook(self):
        """Shutdown hook for ROS node"""

        rospy.loginfo(f"{self.name} is shutting down")

    def load_config(self) -> None:
        """Loads configuration from YAML file"""

        config_parameters = YAMLReader(self.config_file_path)
        for key, value in config_parameters.items():
            setattr(self, key, value)
        self.namespace = getattr(config_parameters, "namespace", "")  # Default namespace is relative unless specified

    def initialize_publishers(self) -> None:
        """Initializes publishers as described in YAML file @ config_file_path"""

        for publisher in self.publishers:
            exec(f"from {publisher.msg_file} import {publisher.msg_type}")
            topic = str(publisher.topic)
            if not topic.startswith("/"):  # Add namespace if topic is not absolute
                topic = self.namespace + topic
            msg = eval(publisher.msg_type)
            temp_publisher = rospy.Publisher(topic, msg, queue_size=1)
            setattr(self, publisher.name, temp_publisher)  # publisher.name is defined in the YAML file

    def initialize_subscribers(self) -> None:
        """Initializes subscribers as described in YAML file @ config_file_path"""

        for subscriber in self.subscribers:
            exec(f"from {subscriber.msg_file} import {subscriber.msg_type}")
            topic = str(subscriber.topic)
            if topic.startswith("/"):  # Add namespace if topic is not absolute
                name = f"msg{subscriber.topic.replace('/', '_')}"  # Absolute topic name
            else:
                name = f"msg{subscriber.topic.replace('/', '_')}"  # Relative topic name
                topic = self.namespace + topic
            msg = eval(subscriber.msg_type)
            msg_instance = msg()
            setattr(self, name, msg_instance)  # Initialize the instance attributes with default message objects
            rospy.Subscriber(topic, msg, callback=self.any_callback, callback_args=name, queue_size=1)

    def any_callback(self, msg, name) -> None:
        """With the power of Python, retrieve "Any" type of ROS messages from this callback function"""

        setattr(self, name, msg)

    def run_node(self) -> None:
        """Starts ROS Node and initializes publishers and subscribers"""

        rospy.init_node(self.name, anonymous=True, disable_signals=True)
        rospy.on_shutdown(self.shutdown_hook)
        self.initialize_publishers()
        self.initialize_subscribers()
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

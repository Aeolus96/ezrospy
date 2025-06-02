#!/usr/bin/env python3

"""--------------------------------------------------------------------------------------------------------------------
Defined Robot Types and Related Interfaces

--------------------------------------------------------------------------------------------------------------------"""

import time
from copy import deepcopy
from math import asin, atan2, cos, degrees, radians, sin, sqrt

import rclpy  # type: ignore  # noqa: F401
from std_msgs.msg import String

from modules.ezros_tools import EzRosNode, package_path

# End of Imports ------------------------------------------------------------------------------------------------------


class EzRobot(EzRosNode):
    """Class for a simplistic robot and related operations"""

    def __init__(self, name: str = "EzRobot", config_file_path: str = None, verbose: bool = False) -> None:
        """Initializes robot, publishers and subscribers"""

        super().__init__(name, config_file_path, verbose)

        # Robot States and Properties
        self.speed = 0.0  # m/s
        self.create_timer(0.01, self.update_speed)  # 100 Hz
        self.waypoint = Waypoint(0.0, 0.0)  # Default Waypoint is kept at (0,0) for simplicity
        self.heading = 0.0  # Default heading is kept at 0 for simplicity
        self.heading_estimator = HeadingEstimator(verbose=verbose)
        self.create_timer(0.1, self.update_gps)  # 10 Hz

    def update_speed(self) -> None:
        """Updates current speed (m/s) using subscribed Odom message"""

        self.speed = self.msg_odom.twist.twist.linear.x

    def update_gps(self) -> None:
        """Updates current GPS latitude and longitude (decimal degrees) using subscribed NavSatFix message"""

        latitude = self.msg_gps.latitude
        longitude = self.msg_gps.longitude
        self.waypoint.update(latitude, longitude)  # Update self waypoint
        self.heading_estimator.add_waypoint(self.waypoint)  # Add to heading_estimator to estimate heading
        if self.heading_estimator.estimated_heading is not None:
            self.heading = self.heading_estimator.get_heading()  # Update self heading if available

    def drive(self, speed=0.0, speed_kwargs: dict = {}, angle=0.0, angle_kwargs: dict = {}) -> None:
        """Publishes twist message to drive the robot\n
        It allows optional function-based speed and angle control.\n
        Example: (speed=control_func, speed_kwargs={'min_speed': 0.0, 'max_speed': 3.0})"""

        if callable(speed):  # Use function-based speed control
            speed = speed(**speed_kwargs)
        if callable(angle):  # Use function-based angle control
            angle = angle(**angle_kwargs)

        from geometry_msgs.msg import Twist  # ROS Message Type

        msg = Twist()  # Create message and publish
        msg.linear.x = float(speed)
        msg.angular.z = float(angle)
        self.pub_twist.publish(msg)

    def drive_for(
        self,
        speed=0.0,
        speed_kwargs: dict = {},
        angle=0.0,
        angle_kwargs: dict = {},
        speed_distance: float = None,  # meters
        duration: float = None,  # seconds
        end_function=None,
        end_function_kwargs: dict = {},
    ) -> None:
        """Creates a loop around the drive function and ends when end_function returns True\n
        Offers some built-in end functions: speed interpolated distance and time duration\n"""

        distance_traveled = 0.0  # meters
        rate = self.create_rate(50)  # 50Hz

        if speed_distance is not None:  # Use speed-interpolated distance calculations
            self.print_highlights(f"Driving for {round(speed_distance, 2)}meters...")
            initial_time = self.get_clock().now()
            while rclpy.ok() and distance_traveled < speed_distance:
                # Calculate distance based on measured current speed (m/s) and time interval (dt)
                distance_traveled += (self.speed) * ((self.get_clock().now() - initial_time).nanoseconds / 1e9)
                initial_time = self.get_clock().now()  # Reset initial time for next iteration
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif duration is not None:  # Use time-based end condition
            self.print_highlights(f"Driving for {round(duration, 2)}seconds...")
            initial_time = self.get_clock().now()
            while rclpy.ok() and ((self.get_clock().now() - initial_time).nanoseconds / 1e9 < duration):
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif callable(end_function):  # Use function-based end condition
            while rclpy.ok() and not end_function(**end_function_kwargs):
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

    def stop(
        self,
        duration=None,
        duration_kwargs: dict = {},
    ) -> None:
        """Stops the robot, provides built-in time duration and custom duration function capabilities\n
        Example: (duration=wait_for_traffic_light, duration_kwargs={'check_for_pedestrians': True})"""

        rate = self.create_rate(50)  # 50Hz

        if callable(duration):  # Use function-based end condition
            while rclpy.ok() and not duration(**duration_kwargs):
                self.drive(0.0, 0.0)
                rate.sleep()

        elif duration is not None:  # Use time-based end condition
            self.print_highlights(f"Stopping for {round(duration, 2)}s...")

            initial_time = self.get_clock().now()
            while rclpy.ok() and ((self.get_clock().now() - initial_time).nanoseconds / 1e9 < duration):
                self.drive(0.0, 0.0)
                rate.sleep()

        else:  # Send a single stop command
            self.print_highlights("Stopped...")
            self.drive(0.0)

    # End of Class ----------------------------------------------------------------------------------------------------


class Waypoint:
    """Class for waypoint and related operations\n
    WARNING: Values are not validated and should be checked before inputting"""

    def __init__(self, latitude: float, longitude: float, heading: float = None) -> None:
        """Sets waypoint latitude and longitude (decimal degrees) and current heading (degrees)"""

        self.update(latitude, longitude, heading)

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
        # print(f"Absolute: {(degrees(atan2(x, y)) + 360) % 360}")
        return (degrees(atan2(x, y)) + 360) % 360  # Normalized to 0-360

    def __str__(self) -> str:
        """Returns a string representation of the waypoint"""

        tmp_heading = self.heading if self.heading is not None else 0
        return f"Waypoint: {self.latitude:.6f}, {self.longitude:.6f}, {tmp_heading:.3f}"

    # End of Class ----------------------------------------------------------------------------------------------------


class HeadingEstimator:
    """Class to calculate heading based on recent waypoints"""

    def __init__(self, max_history=5, min_distance=0.1, max_distance=100, verbose=False) -> None:
        """Initialize with a maximum history size for waypoints\n
        Minimum distance between waypoints in meters"""

        self.max_history = max_history
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.verbose = verbose
        self.waypoints: list[Waypoint] = []
        self.estimated_heading = None
        self.too_far_count = 0

    def add_waypoint(self, waypoint: Waypoint):
        """Add a waypoint to the history and remove old waypoints if necessary"""

        if len(self.waypoints) == 0:
            self.waypoints.append(deepcopy(waypoint))
            if self.verbose:
                print("HeadingEstimator: First waypoint added")
            return
        elif self.waypoints[-1].distance_to(waypoint) < self.min_distance:
            if self.verbose:
                print("HeadingEstimator: Waypoint too close to estimate heading.")
            return
        elif self.waypoints[-1].distance_to(waypoint) > self.max_distance:
            self.too_far_count += 1
            if self.verbose:
                print("HeadingEstimator: Waypoint too far from previous waypoint.")
            if self.too_far_count > self.max_history:  # Reset if too many waypoints too far
                self.reset_history()
            return

        self.waypoints.append(deepcopy(waypoint))

        if len(self.waypoints) > self.max_history:
            self.waypoints.pop(0)

        if len(self.waypoints) >= 2:
            self._calculate_heading()

    def _calculate_heading(self) -> float:
        """Calculate and return a smoothed heading"""

        headings = []
        for i in range(len(self.waypoints) - 1):
            heading = self.waypoints[i].absolute_bearing_with(self.waypoints[i + 1])
            headings.append(heading)

        # Calculate moving average of headings
        self.estimated_heading = sum(headings) / len(headings)
        if self.verbose:
            print(f"HeadingEstimator: estimated heading = {self.estimated_heading:.3f}")
        return self.estimated_heading

    def get_heading(self) -> float:
        """Get the estimated heading"""

        if self.estimated_heading is None:
            raise ValueError("HeadingEstimator: No heading available")

        return self.estimated_heading

    def reset_history(self):
        """Reset the waypoint history"""

        self.waypoints.clear()
        self.estimated_heading = None
        self.too_far_count = 0

        if self.verbose:
            print("HeadingEstimator: History reset")

    # End of Class ----------------------------------------------------------------------------------------------------


class Schoolbus(EzRobot):
    """Class for the Schoolbus robot"""

    def __init__(
        self,
        name: str = "Schoolbus",
        config_file_path: str = package_path("ezrospy") + "/config/schoolbus.yaml",
        verbose: bool = False,
    ):
        super().__init__(name, config_file_path, verbose)
        self.yolo_count = 0
        self.yolo_size = 0
        self.waypoints = None
        time.sleep(1.0)
        self.update_gps()

    def update_gps(self) -> None:
        """Updates current GPS latitude and longitude (decimal degrees) using subscribed NavSatFix message"""

        latitude = self.msg_gps.latitude
        longitude = self.msg_gps.longitude
        self.waypoint.update(latitude, longitude)  # Update self waypoint
        # self.heading = (((self.msg_imu.orientation.z * -180) + 180) + 330) % 360  # Heading from IMU in degrees
        # -1 to 1 > 180 to -180 > 0 to 360 > offset to North

        # RelPosNED heading
        self.heading = self.msg_navrelposned.rel_pos_heading * 1e-5

    def lane_center(self, gain: float = 1.0):
        return self.msg_blob_cmd.angular.z * gain * -10.0

    def yolo_look_for(self, target: str = "person") -> None:
        """Calls the yolo service to look for a specific target class.
        Check values of yolo_count and yolo_size to see if a target was found"""
        # NOTE: throttle the yolo service calls to avoid overloading threads

        self.srv_yolo_req.target = target
        # Call the yolo service and tie it to a callback
        future = self.srv_yolo.call_async(self.srv_yolo_req)
        future.add_done_callback(self.yolo_callback)

    def yolo_callback(self, future):
        """Callback function for the yolo service"""

        try:
            response = future.result()
            if response:  # update the yolo_count and yolo_size directly in the class
                self.yolo_count = response.count
                self.yolo_size = response.size
                if self.verbose:
                    print(f"Found {self.yolo_count}x {self.srv_yolo_req.target}, {self.yolo_size}% of image")
        except Exception as e:
            print(f"- ! - ! - ! - !- Exception in yolo_callback - ! - ! - ! - !-\n{e}")

    def object_in_zone(self, zone: str, min_dist: float = 0.0, max_dist: float = 5.0) -> bool:
        """Returns True if object is in zone, False otherwise"""

        # within_zone = eval(f"self.msg_{zone}.data > {min_dist} and self.msg_{zone}.data < {max_dist}")
        distance = eval(f"self.msg_{zone}.data")
        within_zone = distance < max_dist and distance > min_dist
        # print(f"Object in {zone}:: {distance} ::{within_zone}")
        return within_zone

    def object_not_in_zone(self, zone: str, min_dist: float = 0.0, max_dist: float = 5.0) -> bool:
        return not self.object_in_zone(zone=zone, min_dist=min_dist, max_dist=max_dist)

    def update_current_waypoint(self) -> None:
        """Updates self Waypoint instance from the current vehicle status"""

        self.update_gps()

        # self.waypoint.update(self.msg_gps.latitude, self.msg_gps.longitude, self.msg_navheading.orientation.z)

    def read_waypoints(self, file_path: str = None, verbose: bool = False) -> list:
        """Reads waypoints from saved YAML file"""
        from modules.ezros_tools import YAMLReader

        file = YAMLReader(file_path=file_path)
        file.read(file_path=file_path)

        waypoint_list = []
        for i in range(len(file)):
            waypoint = eval(f"file.waypoint{i}")
            waypoint_list.append(
                Waypoint(latitude=waypoint[0].lat, longitude=waypoint[1].long, heading=waypoint[2].heading)
            )

        if verbose:
            print(waypoint_list)

        return waypoint_list

    def relative_bearing_with(self, goal: "Waypoint") -> float:
        """Returns the relative bearing from the current heading to the goal waypoint in degrees"""

        relative_bearing = self.heading - self.waypoint.absolute_bearing_with(goal)  # degrees

        # Normalize to 0 - 360 only when over -180 or 180
        if relative_bearing < -180:
            relative_bearing += 360
        elif relative_bearing > 180:
            relative_bearing -= 360

        # print(f"Relative: {relative_bearing}")
        return relative_bearing

    def waypoint_in_range(self, goal_waypoint: "Waypoint" = None, radius: float = 3.0) -> bool:
        """Returns True if GPS coordinates are within the specified radius (meters)"""

        if goal_waypoint is None:
            print("Please specify a waypoint")
            return False

        self.update_current_waypoint()  # Update current waypoint position
        distance = self.waypoint.distance_to(goal_waypoint)
        # print(distance)

        return distance < radius  # or distance is within radius

    def follow_waypoints(self, radius: float = 1.5, gain: float = 1.0, verbose: bool = False) -> float:
        """Returns the angle needed to follow the waypoint trajectory using a list of Waypoints(class). Make sure the list is a defined object and ordered correctly"""

        if self.waypoints is None:
            print("Please specify a list of waypoints @ self.waypoints")
            return 0

        num_waypoints = len(self.waypoints)
        # print(f"Waypoints Remaining: {num_waypoints}")
        # print(f"Heading: {self.heading}")

        if num_waypoints > 0:  # If there are waypoints available
            self.update_current_waypoint()  # Update current waypoint position

            # Calculate target angle based on the average relative angle of the first n waypoints
            n = min(10, num_waypoints)
            target_angle_sum = 0
            for i in range(n):
                target_angle_sum += self.relative_bearing_with(self.waypoints[i]) * gain

            target_angle = target_angle_sum / n  # Average target angle

            # Check if waypoint is within specified radius
            if self.waypoint_in_range(goal_waypoint=self.waypoints[0], radius=radius):
                if verbose:
                    print("Reached ", self.waypoints[0])
                self.waypoints.pop(0)  # remove waypoint because it has been sufficiently reached

            return radians(target_angle)

        else:  # If there are no more waypoints in the list
            if verbose:
                print("--final waypoint reached--")
            return 0

    def drive_mode(self, mode: str = ""):
        msg = String()
        print(f"Switching to {mode} mode")

        if mode == "heading":
            msg.data = "heading"
        elif mode == "rotate":
            msg.data = "rotate"
        else:
            print("Switching to ackermann mode")
            msg.data = "ackermann"

        for i in range(20):
            self.pub_drive_mode.publish(msg)

    def detect_pothole(self, size: float = 5.0):
        # print(self.msg_pothole.data)
        if self.msg_pothole.data > size:
            print("Pothole detected")
            return True
        return False

    def lane_change_left(self, heading: bool = False):
        if heading:
            self.stop(duration=1.5)
            self.drive_mode(mode="heading")
            self.drive_for(speed=0.1, angle=1.0, duration=1.0)
            self.drive_for(speed=0.75, angle=1.0, duration=6.5)
            self.drive_mode()
            self.stop(duration=1.0)
        else:
            self.drive_for(speed=1.0, angle=0.6, duration=1.2)
            self.drive_for(speed=1.0, angle=0.0, duration=5.0)
            self.drive_for(speed=1.0, angle=-0.6, duration=1.0)

    def lane_change_right(self, heading: bool = False):
        if heading:
            self.stop(duration=1.5)
            self.drive_mode(mode="heading")
            self.drive_for(speed=0.1, angle=-1.0, duration=1.0)
            self.drive_for(speed=0.75, angle=-1.0, duration=6.5)
            self.drive_mode()
            self.stop(duration=1.0)
        else:
            self.drive_for(speed=1.0, angle=-0.6, duration=0.9)
            self.drive_for(speed=1.0, angle=0.0, duration=4.5)
            self.drive_for(speed=1.0, angle=0.6, duration=1.0)

    def stop_rotate(self):
        self.stop(duration=1.0)
        self.drive_mode(mode="rotate")
        self.drive_for(speed=0.01, angle=0.001, duration=10.0)
        self.drive_mode()

    def load_new_waypoints(self, file_name):
        self.waypoints = self.read_waypoints(f"/home/dev/waypoints/{file_name}.yaml")

    def detect_tire(self, size: float = 0.5):
        self.yolo_look_for(target="tire")
        print(f"looking for tire, {self.yolo_size}")
        time.sleep(0.2)
        return self.yolo_size > size

    def check_fake_sign(
        self,
        goal_waypoint: "Waypoint" = None,
        zone: str = "frontright",
        min_dist: float = 0.0,
        max_dist: float = 5.0,
    ) -> bool:
        """Returns True if object is in zone, False otherwise"""

        # within_zone = eval(f"self.msg_{zone}.data > {min_dist} and self.msg_{zone}.data < {max_dist}")
        distance = eval(f"self.msg_{zone}.data")
        within_zone = distance < max_dist and distance > min_dist
        # print(f"Object in {zone}:: {distance} ::{within_zone}")

        self.yolo_look_for("stop")

        if not self.found_sign:
            self.found_sign = self.yolo_count > 0

        if self.found_sign and within_zone:
            return True

        if self.waypoint_in_range(goal_waypoint=goal_waypoint):
            self.end_condition = True
            return True

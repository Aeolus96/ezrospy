#!/usr/bin/env python3

"""--------------------------------------------------------------------------------------------------------------------
Defined Robot Types and Related Interfaces

--------------------------------------------------------------------------------------------------------------------"""

from math import asin, atan2, cos, degrees, radians, sin, sqrt

import rospy

from ezrospy.ezros_tools import EzRosNode

# End of Imports ------------------------------------------------------------------------------------------------------


class EzRobot(EzRosNode):
    """Class for a simplistic robot and related operations"""

    def __init__(self, name: str = "EzRobot", config_file_path: str = None, verbose: bool = False) -> None:
        """Initializes robot, publishers and subscribers"""

        super().__init__(name, config_file_path, verbose)

        # Robot States and Properties
        self.speed = 0.0  # m/s
        rospy.Timer(rospy.Duration(0.01), self.update_speed)  # 100 Hz
        self.waypoint = Waypoint(0.0, 0.0)  # Default Waypoint is kept at (0,0) for simplicity
        self.heading = 0.0  # Default heading is kept at 0 for simplicity
        self.heading_estimator = HeadingEstimator()
        rospy.Timer(rospy.Duration(0.1), self.update_gps)  # 10 Hz

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
        msg.linear.x = speed
        msg.angular.z = angle
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
        rate = rospy.Rate(20)  # Hz

        if speed_distance is not None:  # Use speed-interpolated distance calculations
            self.print_highlights(f"Driving for {round(speed_distance, 2)}meters...")
            initial_time = rospy.Time.now()
            while not rospy.is_shutdown() and distance_traveled < speed_distance:
                # Calculate distance based on measured current speed (m/s) and time interval (dt)
                distance_traveled += (self.speed) * (rospy.Time.now() - initial_time).to_sec()
                initial_time = rospy.Time.now()  # Reset initial time for next iteration
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif duration is not None:  # Use time-based end condition
            self.print_highlights(f"Driving for {round(duration, 2)}seconds...")
            initial_time = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - initial_time < rospy.Duration(duration)):
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif callable(end_function):  # Use function-based end condition
            while not rospy.is_shutdown() and not end_function(**end_function_kwargs):
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

    def stop(
        self,
        duration=None,
        duration_kwargs: dict = {},
    ) -> None:
        """Stops the robot, provides built-in time duration and custom duration function capabilities\n
        Example: (duration=wait_for_traffic_light, duration_kwargs={'check_for_pedestrians': True})"""

        rate = rospy.Rate(20)  # Hz

        if callable(duration):  # Use function-based end condition
            while not rospy.is_shutdown() and not duration(**duration_kwargs):
                self.drive(0.0, 0.0)
                rate.sleep()

        elif duration is not None:  # Use time-based end condition
            self.print_highlights(f"Stopping for {round(duration, 2)}s...")

            initial_time = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - initial_time < rospy.Duration(duration)):
                self.drive(0.0, 0.0)
                rate.sleep()

        else:  # Send a single stop command
            self.drive(0.0)

    def update_speed(self, TimerEvent) -> None:
        """Updates current speed (m/s) using subscribed Odom message"""

        self.speed = self.msg_odom.twist.twist.linear.x

    def update_gps(self, TimerEvent) -> None:
        """Updates current GPS latitude and longitude (decimal degrees) using subscribed NavSatFix message"""

        latitude = self.msg_gps.latitude
        longitude = self.msg_gps.longitude
        self.waypoint.update(latitude, longitude)  # Update self waypoint
        self.heading_estimator.add_waypoint(self.waypoint)  # Add to heading_estimator to estimate heading
        if self.heading_estimator.estimated_heading is not None:
            self.heading = self.heading_estimator.get_heading()  # Update self heading if available

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
        return (degrees(atan2(x, y)) + 360) % 360  # Normalized to 0-360

    # TODO: Implement relative bearing inside robot NOT in waypoint class
    # def relative_bearing_with(self, goal: "Waypoint") -> float:
    #     """Returns the relative bearing from the current heading to the goal waypoint in degrees\n
    #     WARNING: Only valid if current heading is known"""

    #     if self.heading is None:
    #         print("Waypoint: Please specify current heading to calculate relative bearing")
    #         return 0
    #     elif goal.heading is None:
    #         print("Waypoint: Please specify goal heading to calculate relative bearing")
    #         return 0
    #     absolute_bearing = self.absolute_bearing_with(goal)
    #     relative_bearing = self.heading - absolute_bearing
    #     if relative_bearing < -180:  # Normalize to -180 or 180
    #         relative_bearing += 360
    #     elif relative_bearing > 180:
    #         relative_bearing -= 360
    #     return relative_bearing

    def __str__(self) -> str:
        """Returns a string representation of the waypoint"""
        return f"Waypoint: {self.latitude:.6f}, {self.longitude:.6f}, {self.heading:.3f}"

    # End of Class ----------------------------------------------------------------------------------------------------


class HeadingEstimator:
    """Class to calculate heading based on recent waypoints"""

    def __init__(self, max_history=5, min_distance=0.5, max_distance=100, verbose=False) -> None:
        """Initialize with a maximum history size for waypoints\n
        Minimum distance between waypoints in meters"""

        self.max_history = max_history
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.verbose = verbose
        self.waypoints = []
        self.estimated_heading = None
        self.too_far_count = 0

    def add_waypoint(self, waypoint: Waypoint):
        """Add a waypoint to the history and remove old waypoints if necessary"""

        if len(self.waypoints) == 0:
            self.waypoints.append(waypoint)
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

        self.waypoints.append(waypoint)

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

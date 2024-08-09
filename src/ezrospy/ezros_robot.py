#!/usr/bin/env python3

"""--------------------------------------------------------------------------------------------------------------------
Defined Robot Types and Related Interfaces

--------------------------------------------------------------------------------------------------------------------"""

import time

import rospkg
import rospy

from ezrospy import ezros_tools as ezros_tools

# End of Imports ------------------------------------------------------------------------------------------------------


class EzRobot(ezros_tools.EzRosNode):
    """Class for a simplistic robot and related operations"""

    def __init__(self, name: str = "EzRobot", config_file_path: str = None) -> None:
        """Initializes robot, publishers and subscribers"""

        super().__init__(name, config_file_path)

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

    # End of Class ----------------------------------------------------------------------------------------------------

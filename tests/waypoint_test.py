import sys

# Adjust the path to include the module folder
sys.path.insert(0, sys.path[0] + "/../src/")
print("new path to get to ezrospy modules: ", sys.path[0])

from ezrospy.ezros_robot import HeadingEstimator, Waypoint  # noqa: E402

# Initialize the HeadingEstimator
heading_estimator = HeadingEstimator(max_history=5, min_distance=1, max_distance=100, verbose=True)

# Test 1: Add a single waypoint and attempt to get heading
heading_estimator.reset_history()
waypoint1 = Waypoint(42.3314, -83.0458)
heading_estimator.add_waypoint(waypoint1)
assert len(heading_estimator.waypoints) == 1, "Test 1 Failed: Single waypoint not added correctly"

try:
    heading = heading_estimator.get_heading()
    assert False, "Test 1 Failed: Heading should not be available with only one waypoint"
except ValueError:
    print("Test 1 Passed: No heading available with only one waypoint")

# Test 2: Add a second waypoint and calculate heading
waypoint2 = Waypoint(42.3316, -83.0460)
heading_estimator.add_waypoint(waypoint2)
assert len(heading_estimator.waypoints) == 2, "Test 2 Failed: Second waypoint not added correctly"
assert heading_estimator.get_heading() is not None, "Test 2 Failed: Heading not calculated with two waypoints"
print(f"Test 2 Passed: Heading calculated: {heading_estimator.get_heading():.2f} degrees")

# Test 3: Add a third waypoint and check heading update
waypoint3 = Waypoint(42.33181, -83.04621)
heading_estimator.add_waypoint(waypoint3)
heading_after_third = heading_estimator.get_heading()
assert heading_after_third is not None, "Test 3 Failed: Heading not updated with third waypoint"
print(f"Test 3 Passed: Heading updated: {heading_after_third:.2f} degrees")

# Test 4: Add a waypoint that is too close and verify it's not added
waypoint_too_close = Waypoint(42.331812, -83.046212)  # Very close to waypoint3
heading_estimator.add_waypoint(waypoint_too_close)
assert len(heading_estimator.waypoints) == 3, "Test 4 Failed: Waypoint too close should not be added"
print("Test 4 Passed: Waypoint too close not added")

# Test 5: Add a waypoint that is too far and verify it's not added
waypoint_too_far = Waypoint(43.0000, -83.0000)
heading_estimator.add_waypoint(waypoint_too_far)
assert len(heading_estimator.waypoints) == 3, "Test 5 Failed: Waypoint too far should not be added"
print("Test 5 Passed: Waypoint too far not added")

# Test 6: Add a fourth valid waypoint and verify heading calculation
waypoint4 = Waypoint(42.3320, -83.0464)
heading_estimator.add_waypoint(waypoint4)
heading_after_fourth = heading_estimator.get_heading()
assert heading_after_fourth is not None, "Test 6 Failed: Heading not updated with fourth waypoint"
print(f"Test 6 Passed: Heading updated: {heading_after_fourth:.2f} degrees")

# Test 7: Check behavior when multiple waypoints are too far apart and trigger history reset
waypoint_far = Waypoint(43.0000, -83.0000)

# Add multiple far waypoints, expect the history to reset
heading_estimator.add_waypoint(waypoint_far)
heading_estimator.add_waypoint(waypoint_far)
heading_estimator.add_waypoint(waypoint_far)
heading_estimator.add_waypoint(waypoint_far)
heading_estimator.add_waypoint(waypoint_far)

# After the loop, the history should have been reset, so the waypoint list should be empty
assert len(heading_estimator.waypoints) == 0, "Test 7 Failed: Waypoints too far apart did not trigger history reset"
print("Test 7 Passed: Multiple far waypoints triggered history reset")

#!/usr/bin/env python3

import os
import shutil
import sys
import time

# Adjust the path to include the module folder
sys.path.insert(0, sys.path[0] + "/../src/")
print("new path to get to ezrospy modules: ", sys.path[0])

from ezrospy.ezros_tools import ScriptPlayer  # noqa: E402

# Create a temporary directory for testing
test_directory = "test_scripts/"
os.makedirs(test_directory, exist_ok=True)

# Create a sample Python script to execute
sample_script_path = os.path.join(test_directory, "sample_script.py")
with open(sample_script_path, "w") as f:
    f.write("""
import time
print('Hello from sample_script')
for i in range(10):
    print(f'Running... {i+1}')
    time.sleep(1)
print('Sample script completed')
    """)

try:
    # Initialize ScriptPlayer with the test directory
    player = ScriptPlayer(test_directory)
    print("ScriptPlayer initialization passed")

    # Load files in the directory
    load_result = player.load_files()
    print(load_result)
    assert "sample_script.py" in player.file_list, f"Expected 'sample_script.py' in {player.file_list}"
    print("load_files method passed")

    # Select the script to execute
    player.file_selected = "sample_script.py"

    # Execute the script
    execute_result = player.execute()
    print(execute_result)
    assert execute_result == "Script is running", f"Expected 'Script is running', but got '{execute_result}'"
    print("execute method passed")

    # Wait a few seconds to simulate monitoring and reading output
    time.sleep(5)
    assert player.process_is_running, "Expected process to be running, but it's not."
    assert (
        "INITIALIZING __sample_script.py__ ..." in player.process_output_text
    ), "Expected initialization message not found in output."
    assert any(
        "SPAWNED __sample_script.py__" in line for line in player.process_output_text
    ), "Expected spawned message not found in output."
    print("monitor_process and read_output methods passed")

    # Stop the script
    stop_result = player.stop_script()
    print(stop_result)
    assert (
        stop_result == "Script successfully stopped"
    ), f"Expected 'Script successfully stopped', but got '{stop_result}'"
    assert not player.process_is_running, "Expected process to be stopped, but it's still running."
    print("stop_script method passed")

    # Wait for any remaining output to be read
    time.sleep(1)
    assert any(
        "Running..." in line for line in player.process_output_text
    ), "Expected 'Running...' messages not found in output."
    assert any(
        "Terminating script..." in line for line in player.process_output_text
    ), "Expected 'Terminating script...' message not found in output."
    print("All tests passed successfully!")

except AssertionError as e:
    print(f"AssertionError: {e}")
except Exception as e:
    print(f"Error occurred during testing: {e}")
finally:
    # Clean up the test directory
    shutil.rmtree(test_directory)

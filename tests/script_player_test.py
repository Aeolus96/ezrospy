#!/usr/bin/env python3

import os
import shutil
import subprocess
import signal
import time
from threading import Thread


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
    assert "INITIALIZING __sample_script.py__ ..." in player.process_output_text, "Expected initialization message not found in output."
    assert any("SPAWNED __sample_script.py__" in line for line in player.process_output_text), "Expected spawned message not found in output."
    print("monitor_process and read_output methods passed")

    # Stop the script
    stop_result = player.stop_script()
    print(stop_result)
    assert stop_result == "Script successfully stopped", f"Expected 'Script successfully stopped', but got '{stop_result}'"
    assert not player.process_is_running, "Expected process to be stopped, but it's still running."
    print("stop_script method passed")

    # Wait for any remaining output to be read
    time.sleep(1)
    assert any("Running..." in line for line in player.process_output_text), "Expected 'Running...' messages not found in output."
    assert any("Terminating script..." in line for line in player.process_output_text), "Expected 'Terminating script...' message not found in output."
    print("All tests passed successfully!")

except AssertionError as e:
    print(f"AssertionError: {e}")
except Exception as e:
    print(f"Error occurred during testing: {e}")
finally:
    # Clean up the test directory
    shutil.rmtree(test_directory)
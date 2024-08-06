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

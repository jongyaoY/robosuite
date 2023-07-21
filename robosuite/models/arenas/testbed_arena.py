import numpy as np
from scipy.spatial.transform import Rotation as R

import robosuite.utils.transform_utils as T
from robosuite.models.arenas import Arena
from robosuite.utils.mjcf_utils import array_to_string, string_to_array, xml_path_completion


class TestBedArena(Arena):
    """
    Workspace that contains an empty table.


    Args:
        table_full_size (3-tuple): (L,W,H) full dimensions of the table
        table_friction (3-tuple): (sliding, torsional, rolling) friction parameters of the table
        table_offset (3-tuple): (x,y,z) offset from center of arena when placing table.
            Note that the z value sets the upper limit of the table
        has_legs (bool): whether the table has legs or not
        xml (str): xml file to load arena
    """

    def __init__(
        self,
        xml="arenas/testbed_arena.xml",
    ):

        super().__init__(xml_path_completion(xml))
        self.table_full_size = np.array([1.215, 0.81, 0.05])  # the testbed size is fixed
        self.table_half_size = self.table_full_size / 2

        self.table_offset = np.array([0, 0, 0.4068 * 2])  # height of the table is also fixed
        self.center_pos = self.bottom_pos + np.array([0, 0, -self.table_half_size[2]]) + self.table_offset

        self.table_body = self.worldbody.find("./body[@name='table']")
        self.table_collision = self.table_body.find("./geom[@name='table_collision']")
        self.table_top = self.table_body.find("./site[@name='table_top']")

        self.configure_location()

    def configure_location(self):
        """Configures correct locations for this arena"""
        self.floor.set("pos", array_to_string(self.bottom_pos))

        self.table_body.set("pos", array_to_string(self.center_pos))
        self.table_collision.set("size", array_to_string(self.table_half_size))

        self.table_top.set("pos", array_to_string(np.array([0, 0, self.table_half_size[2]])))

    @property
    def table_top_abs(self):
        """
        Grabs the absolute position of table top

        Returns:
            np.array: (x,y,z) table position
        """
        return string_to_array(self.floor.get("pos")) + self.table_offset

    @property
    def table_mount_pos(self):
        """
        Mounting position for the robot

        Returns:
            np.array: (x,y,z) mount position
        """
        table_top_pos = string_to_array(self.floor.get("pos")) + self.table_offset
        offset = -self.table_half_size
        offset[2] = 0
        return table_top_pos + offset

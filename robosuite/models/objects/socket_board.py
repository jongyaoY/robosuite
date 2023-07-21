from scipy.spatial.transform import Rotation as R

from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import OBJECT_COLLISION_COLOR, array_to_string, xml_path_completion
from robosuite.utils.transform_utils import convert_quat


class SocketBoard(MujocoXMLObject):
    """
    Square plate with a hole in the center (used in PegInHole)
    """

    def __init__(self, name, inner_size, outter_size, duplicate_collision_geoms=True):
        super().__init__(
            xml_path_completion("objects/plate-with-hole.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=False,
        )
        # Sizes for left-right geom
        size_x_1 = (outter_size[0] - inner_size[0]) / 2.0
        size_y_1 = outter_size[1]
        size_x_2 = inner_size[0]
        size_y_2 = (outter_size[1] - inner_size[1]) / 2.0
        size_z = 0.001
        geom_attrib = {
            "geom_left": {
                "pos": [-(outter_size[0] - size_x_1) / 2, 0, size_z / 2],
                "size": [size_x_1 / 2, size_y_1 / 2, size_z / 2],
            },
            "geom_right": {
                "pos": [(outter_size[0] - size_x_1) / 2, 0, size_z / 2],
                "size": [size_x_1 / 2, size_y_1 / 2, size_z / 2],
            },
            "geom_up": {
                "pos": [0, (outter_size[1] - size_y_2) / 2.0, size_z / 2],
                "size": [size_x_2 / 2, size_y_2 / 2, size_z / 2],
            },
            "geom_down": {
                "pos": [0, -(outter_size[1] - size_y_2) / 2.0, size_z / 2],
                "size": [size_x_2 / 2, size_y_2 / 2, size_z / 2],
            },
        }
        for geom, attrib in zip(list(self._obj), geom_attrib.values()):
            geom.set("pos", array_to_string(attrib["pos"]))
            geom.set("size", array_to_string(attrib["size"]))
            if duplicate_collision_geoms:
                self._obj.append(self._duplicate_visual_from_collision(geom))
                geom.set("rgba", array_to_string(OBJECT_COLLISION_COLOR))
                if geom.get("material") is not None:
                    del geom.attrib["material"]

    def set_pose(self, pos, rot=None):
        self._obj.set("pos", array_to_string(pos))
        self.init_pos = pos
        if rot is not None:
            quat = R.from_rotvec(rot).as_quat()
            self.init_quat = quat
            self._obj.set("quat", array_to_string(convert_quat(quat, to="wxyz")))

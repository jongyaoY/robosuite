import numpy as np
from scipy.spatial.transform import Rotation as R

from robosuite.models.base import MujocoXML
from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import array_to_string, xml_path_completion
from robosuite.utils.transform_utils import convert_quat


class EVChagerSocket(MujocoXMLObject):
    """
    EVChagerSocket object (used in peg-in-hole task)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/socket.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=False,
        )
        collision_model = MujocoXML(xml_path_completion("objects/meshes/socket_col_32/socket_col_outter.xml"))
        self.merge_assets(collision_model)
        collision = collision_model.worldbody.find("./body[@name='collision']")
        for geom in list(collision):
            self._obj.append(geom)
        front_site = self.worldbody.find("./body/site[@name='{}front_site']".format(self.naming_prefix))
        bottom_site = self.worldbody.find("./body/site[@name='{}end_site']".format(self.naming_prefix))
        self._obj.append(front_site)
        self._obj.append(bottom_site)
        self.init_pos = np.zeros(3)
        self.init_quat = np.zeros(4)

    def set_pose(self, pos, rot=None):
        self._obj.set("pos", array_to_string(pos))
        self.init_pos = pos
        if rot is not None:
            quat = R.from_rotvec(rot).as_quat()
            self.init_quat = quat
            self._obj.set("quat", array_to_string(convert_quat(quat, to="wxyz")))

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of door handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"target": self.naming_prefix + "end_site"})
        return dic

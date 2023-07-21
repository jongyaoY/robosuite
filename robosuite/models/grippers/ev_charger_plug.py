from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class EVChargerPlug(GripperModel):
    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/ev_charger_plug.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return None

    @property
    def _important_geoms(self):
        return {
            "eef": ["grip_site"],
        }

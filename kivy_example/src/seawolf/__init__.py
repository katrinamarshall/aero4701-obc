import rospkg

rospack = rospkg.RosPack()
__seawolf_path = rospack.get_path("seawolf")


def get_kivy_config_path():
    return __seawolf_path + "/kivy/"


def get_assets_path():
    return __seawolf_path + "/assets/"


def form_kivy_config_path(kivy_file):
    return get_kivy_config_path() + kivy_file


def form_assets_path(asset_file):
    return get_assets_path() + asset_file


from .utils.helper_functions import *
from .utils.ros_helper import *

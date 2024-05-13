import json
import os
import subprocess

import dynamic_reconfigure.client
import kivy
import requests
import rospy
import yaml
from dynamic_reconfigure import (
    DynamicReconfigureCallbackException,
    DynamicReconfigureParameterException,
)
from dynamic_reconfigure.encoding import Config

import seawolf
from seawolf.utils.ThreadingSupport import threaded

kivy.require("1.2.0")

from dismissable_popup import show_dismissable_popup
from kivy.base import runTouchApp
from kivy.lang import Builder
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.popup import Popup
from kivy.uix.relativelayout import RelativeLayout
from rospy.service import ServiceException

DATABASE_ID = "87c34cdb1a5448afa190097acd9124c5"
NOTION_URL = "https://api.notion.com/v1/databases/"
INTEGRATION_TOKEN = "secret_bUmgKQsJSzpvBOGU9GbbmZEl36csQboJiL7EaONaX8x"

HEADERS = {
    "Authorization": "Bearer " + INTEGRATION_TOKEN,
    "Content-Type": "application/json",
    "Notion-Version": "2021-08-16",
}


class NotionSync:
    """
    Class to fetch data from a Notion database:
    Inspired by:
        https://prettystatic.com/notion-api-python/
        https://towardsdatascience.com/productivity-tracking-with-the-notion-api-and-python-f5f866fe11d8
    """

    def __init__(self, titles):
        self.titles_of_interest = titles

    def query_database(self):
        """
        Sends a post request to fetch database
        """
        database_url = NOTION_URL + DATABASE_ID + "/query"
        response = requests.post(database_url, headers=HEADERS)
        if response.status_code != 200:
            rospy.loginfo("Database response Status: {}".format(response.status_code))
        else:
            return response.json()

    def get_database_titles(self, data_json):
        """
        Gets column titles of database
        """
        return list(data_json["results"][0]["properties"].keys())

    def check_database_titles(self, titles):
        """
        Checks that column titles of interest are in the database data
        """
        for title in self.titles_of_interest:
            if title not in titles:
                return False
        return True

    def get_database_data(self, data_json, titles, valid_configurations):
        """
        Gets the 'Utilisation' and 'MAC Address' column data
        and returns as two lists of strings. If a cell is empty, places an empty string.
        """
        bot_configurations = []
        mac_addresses = []

        for t in titles:
            changed_json = False
            for i in range(len(data_json["results"])):
                try:  # If accessing the json results in error, this means the database json structure has changed
                    if self.titles_of_interest[0] == t:
                        if len(data_json["results"][i]["properties"][t]["multi_select"]) == 0:
                            bot_configurations.append("")
                        else:
                            bot_config = ""
                            for data in data_json["results"][i]["properties"][t]["multi_select"]:
                                if data["name"] in valid_configurations:
                                    bot_config = data["name"]
                                    break
                            bot_configurations.append(bot_config)

                    elif self.titles_of_interest[1] == t:
                        if len(data_json["results"][i]["properties"][t]["rich_text"]) == 0:
                            mac_addresses.append("")
                        else:
                            mac_addresses.append(
                                data_json["results"][i]["properties"][t]["rich_text"][0][
                                    "plain_text"
                                ]
                            )
                except KeyError:
                    changed_json = True
                    rospy.logerr("Key error when accessing column: {}".format(t))
                    break
                except TypeError:
                    changed_json = True
                    rospy.logerr("Type error when accessing column: {}".format(t))
                    break
            if changed_json:
                bot_configurations = []
                mac_addresses = []
                break
        return bot_configurations, mac_addresses


Builder.load_file(seawolf.form_kivy_config_path("botconfigurationpane.kv"))


def config_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    assert "state" in mapping
    return Config(mapping.get("dictitems", {}))


yaml.add_constructor(
    "tag:yaml.org,2002:python/object/new:dynamic_reconfigure.encoding.Config",
    config_constructor,
    Loader=yaml.SafeLoader,
)


class BotConfigurationPane(RelativeLayout):
    def __init__(self, **kwargs):
        super(BotConfigurationPane, self).__init__(**kwargs)
        self.titles_of_interest = ("Utilisation", "MAC Address")
        self.nsync = NotionSync(self.titles_of_interest)
        self.motion_controller_position_client = None
        self.motion_controller_orientation_client = None
        self.configuration_files = "/home/user/catkin_ws/src/seawolf/configs"
        self.configuration_success = False
        self.local_database_path = self.configuration_files + "/robot_config_database.json"
        self.BOT_CONFIGURATIONS = {
            "Hullcleaningbot": {
                "background_color": [0, 0, 1, 0.9],
                "position_config": "hullbot_6dof_lqi_position_gains.yaml",
                "orientation_config": "hullbot_6dof_lqi_orientation_gains.yaml",
            },
            "Displaybot": {
                "background_color": [0.72, 0.33, 0.83, 1],
                "position_config": "hullbot_6dof_lqi_position_gains.yaml",
                "orientation_config": "hullbot_6dof_lqi_orientation_gains.yaml",
            },
            "Cullbot": {
                "background_color": [1, 0, 0, 0.9],
                "position_config": "surveybot_6dof_lqi_position_gains.yaml",
                "orientation_config": "surveybot_6dof_lqi_orientation_gains.yaml",
            },
            "Surveybot": {
                "background_color": [0, 1, 0, 0.9],
                "position_config": "surveybot_6dof_lqi_position_gains.yaml",
                "orientation_config": "surveybot_6dof_lqi_orientation_gains.yaml",
            },
        }

        # Generate dropdown menu
        self.dropdown = DropDown()
        for config in self.BOT_CONFIGURATIONS.keys():
            btn = Button(
                text=config,
                background_color=self.BOT_CONFIGURATIONS[config]["background_color"],
                size_hint_y=None,
                height=40,
            )
            btn.bind(on_release=lambda btn: self.dropdown.select(btn.text))
            self.dropdown.add_widget(btn)
        self.ids.configuration_selector.bind(on_release=self.dropdown.open)
        self.dropdown.bind(on_select=self.update_bot_configuration)

        # Continually try to fetch configuration until botside is launched
        self.auto_configuration_worker()

    @threaded
    def auto_configuration_worker(self):
        """
        Attempts to initialise dynamic reconfigure client every 5 seconds when seawolf launches.
        Launching botside will initiate the entire process.
        """
        rospy.loginfo("Finding 'motion_controller_node' client")
        r = rospy.Rate(0.2)
        while (
            not rospy.is_shutdown()
            and self.motion_controller_position_client is None
            and self.motion_controller_orientation_client is None
        ):
            self.initialise_motion_controller_node_client()
            r.sleep()

    @threaded
    def update_bot_configuration(self, obj, config):
        """
        Update the bot configuration by reading the config files, updating the dynamic reconfigure servers
        and changing the apperance of configuration button
        """
        # Initialise dynamic reconfigure client if not already intialised
        if (
            self.motion_controller_position_client is None
            and self.motion_controller_orientation_client is None
        ):
            self.initialise_motion_controller_node_client()

        # Update bot configuration
        if (
            self.motion_controller_position_client is not None
            and self.motion_controller_orientation_client is not None
        ):

            position_path = os.path.join(
                self.configuration_files, self.BOT_CONFIGURATIONS[config]["position_config"]
            )
            orientation_path = os.path.join(
                self.configuration_files, self.BOT_CONFIGURATIONS[config]["orientation_config"]
            )
            position_configuration = self.read_config_yaml(position_path)
            orientation_configuration = self.read_config_yaml(orientation_path)

            try:
                self.motion_controller_position_client.update_configuration(position_configuration)
                self.motion_controller_orientation_client.update_configuration(
                    orientation_configuration
                )
                rospy.loginfo("Updated bot configuration to: {}".format(config))
            except ServiceException as e:
                rospy.logerr(
                    "Call for reconfiguration wasn't successful" " because: {}".format(e.message)
                )
            except DynamicReconfigureParameterException as e:
                rospy.logerr("Reconfiguration wasn't successful" " because: {}".format(e.message))
            except DynamicReconfigureCallbackException as e:
                rospy.logerr("Reconfiguration wasn't successful" " because: {}".format(e.message))
            setattr(self.ids.configuration_selector, "text", config)
            setattr(
                self.ids.configuration_selector,
                "background_color",
                self.BOT_CONFIGURATIONS[config]["background_color"],
            )
        else:
            show_dismissable_popup(
                title="motion_controller_node client not running",
                text="Cannot update bot configuration as motion_controller_node client is not running",
                size_x=700,
                size_y=400,
            )

    @threaded
    def initialise_motion_controller_node_client(self):
        """
        Initialise the dynamic_reconfigure client for motion_controller_node (orientation & position).
        Once initilised, attempt to update bot configuration remotely then locally
        """
        # If motion_controller_node cannot be found, we get the following excpetion:
        # ROSException: timeout exceeded while waiting for service /motion_controller_node/set_parameters etc
        try:
            self.motion_controller_position_client = dynamic_reconfigure.client.Client(
                "/motion_controller_node/lqi_position_controller_params", timeout=5
            )
            self.motion_controller_orientation_client = dynamic_reconfigure.client.Client(
                "/motion_controller_node/lqi_orientation_controller_params", timeout=5
            )
            rospy.loginfo("'motion_controller_node' client found")

            self.bot_mac_address = self.get_bot_mac_address()
            if self.bot_mac_address != "MACnomacaddress":
                self.fetch_bot_configuration("remote")
                if self.configuration_success == False:
                    self.fetch_bot_configuration("local")
            else:
                rospy.logwarn("MAC address for this bot cannot be found!")
        except rospy.ROSException:
            pass

    def get_bot_mac_address(self):
        """
        Gets the MAC address (unique identifier for each bot)
        """
        try:
            robot_ip = "192.168.2.2"
            robot_mac = subprocess.check_output(
                "ping -c 1 -W 2 {} > /dev/null 2>&1 && echo $(arp -n | grep '{}' | awk '{{print $3}}')".format(
                    robot_ip, robot_ip
                ),
                shell=True,
            )
            if len(robot_mac.strip()) > 0:
                stripped_robot_mac = robot_mac.strip()
                rospy.loginfo("Unique bot MAC address: '{}'".format(stripped_robot_mac))
                splitted_robot_mac = stripped_robot_mac.split()
                if len(splitted_robot_mac) > 1:
                    rospy.loginfo(
                        "Seems we got more than one MAC for the robot ip {}, using first one".format(
                            robot_ip
                        )
                    )
                    stripped_robot_mac = splitted_robot_mac[0]
                robot_mac = stripped_robot_mac
                # Just in case, make sure no spaces are left
                robot_mac = robot_mac.replace(" ", "")
            else:
                rospy.loginfo(
                    "Could not find the robot's MAC address (checking IP {}), rosbag name won't include it".format(
                        robot_ip
                    )
                )
                robot_mac = "MACnomacaddress"
        except Exception as e:
            robot_mac = "MACnomacaddress"
            rospy.loginfo(
                "Could not get MAC address of bot at IP {} (Exception: {})".format(robot_ip, e)
            )
        return robot_mac

    def read_config_yaml(self, file_path):
        """
        Read the configuration yaml and return as dict
        """
        with open(file_path, "r") as f:
            configuration = {}
            for doc in yaml.safe_load_all(f.read()):
                configuration.update(doc)
        return configuration

    def fetch_bot_configuration(self, configuration_type):
        """
        Fetches the bot configuration from the 'Robot Status' database on Notion
        and updates the current bot configuration. This step can fail at numerous steps,
        however we dont want to break seawolf but just notify the user that automatic bot configuration has failed.
        """
        data = None
        if configuration_type == "remote":
            rospy.loginfo("Fetching bot configuration from Notion")
            configuration_text = "Configuration (Remote)"
            try:
                data = self.nsync.query_database()  # Fetch database
            except requests.ConnectionError:
                rospy.logerr("No internet connectivity detected")

        elif configuration_type == "local":
            rospy.loginfo(
                "Unable to update from Notion. Loading local data from last successful fetch"
            )
            configuration_text = "Configuration (Local)"
            try:
                with open(self.local_database_path, "r") as f:
                    data = json.load(f)
            except IOError as e:
                rospy.loginfo(
                    "Local data does not exist yet. Need to successfully fetch remotely once. Bot configuration can be set manually. \n{}".format(
                        e
                    )
                )

        if data != None:

            titles = self.nsync.get_database_titles(data)  # Grab database titles
            if self.nsync.check_database_titles(titles):

                bot_configurations, mac_addresses = self.nsync.get_database_data(
                    data, titles, self.BOT_CONFIGURATIONS.keys()
                )  # Get relevant data
                if bot_configurations and mac_addresses:

                    try:

                        index = mac_addresses.index(self.bot_mac_address)
                        if bot_configurations[index] != "":

                            self.update_bot_configuration(
                                None, bot_configurations[index]
                            )  # Update bot configuration
                            with open(self.local_database_path, "w") as f:  # Save database locally
                                json.dump(data, f, encoding="utf8", indent=4)

                            setattr(self.ids.configuration_button, "text", configuration_text)
                            self.configuration_success = True

                        else:
                            rospy.logerr(
                                "Current bot configuration has not been updated on Notion \nAutomatic bot configuration has failed"
                            )

                    except ValueError:
                        rospy.logerr(
                            "Current bot MAC address has not been updated on Notion \nAutomatic bot configuration has failed"
                        )
                else:
                    rospy.logerr(
                        "Database JSON Structure has changed \nAutomatic bot configuration has failed"
                    )
            else:
                rospy.logerr(
                    "'{}' or '{}' title are not in the Notion database \nAutomatic bot configuration has failed".format(
                        self.titles_of_interest[0], self.titles_of_interest[1]
                    )
                )

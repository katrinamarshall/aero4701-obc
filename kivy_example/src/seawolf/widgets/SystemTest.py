import kivy

kivy.require("1.2.0")
from dismissable_popup import show_dismissable_popup
from kivy.lang import Builder
from kivy.properties import ListProperty, ObjectProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.progressbar import ProgressBar

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("systemTest.kv"))

import actionlib
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from hb_common_msgs.msg import (
    SystemTestAction,
    SystemTestFeedback,
    SystemTestGoal,
    SystemTestResult,
)
from std_srvs.srv import Trigger

# From actionlib
status_to_str = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}


class SystemTest(object):
    """This class manages the interface for calling the system test action.
    It prompts the user to confirm if it is safe (ConfirmationPopup),
        shows progress of the system test action (ProgressPopup),
        and displays the resuls (ResultsPopup)

    """

    def __init__(self):
        self.systest_action_client = actionlib.SimpleActionClient("~system_test", SystemTestAction)
        self.systest_needed_client = rospy.ServiceProxy("~systest_needed", Trigger)

        self.progress_popup = ProgressPopup(self.cancel_action)
        self.confirmation_popup = ConfirmationPopup()

    def wait_and_check_needed(self, timeout=5):
        try:
            rospy.wait_for_service("~systest_needed", timeout=timeout)
        except rospy.ROSException as err:
            rospy.loginfo(err)
            return None, str(err)

        return self.check_needed()

    def check_needed(self):

        check = None
        message = None
        try:
            r = self.systest_needed_client()
        except rospy.ServiceException as err:
            rospy.loginfo("systest_needed service call returned with error {}".format(err))
            message = str(err)
        else:
            check = r.success
            message = r.message

        return check, message

    def prompt_system_test(self):
        self.confirmation_popup.open(self.run)

    def run(self):
        rospy.loginfo("Run system test")
        if not self.systest_action_client.wait_for_server(timeout=rospy.Duration(5)):
            show_dismissable_popup(
                title="Error",
                text="System test action server could not be contacted.\nIs mission manager running?",
            )
            return False

        self.systest_action_client.send_goal(
            SystemTestGoal(), done_cb=self.done_cb, feedback_cb=self.feedback_cb
        )
        # Reset the current feedback before opening
        self.progress_popup.update(SystemTestFeedback())
        self.progress_popup.open()
        return True

    def cancel_action(self):
        self.systest_action_client.cancel_goal()

    def done_cb(self, state, result):
        self.progress_popup.dismiss()
        text = self.systest_action_client.get_goal_status_text()
        if not text:
            text = "Status: {}".format(status_to_str[state])
        results_popup = ResultsPopup(result, text)
        results_popup.open()

    def feedback_cb(self, feedback):
        self.progress_popup.update(feedback)


class ConfirmationPopup(Popup):
    """
    Popup to prompt the user Yes/No whether the system test is safe to run
    """

    def __init__(self, **kwargs):
        super(ConfirmationPopup, self).__init__(**kwargs)

    def open(self, on_confirm_func):
        self.on_confirm_func = on_confirm_func
        super(ConfirmationPopup, self).open()

    def on_confirm(self):
        self.dismiss()
        self.on_confirm_func()


class ProgressPopup(Popup):
    """
    Popup to show the progress of the system test action, and to let it be cancelled
    """

    progress_bar = ObjectProperty(None)
    progress_text = ObjectProperty(None)

    def __init__(self, cancel_func, **kwargs):
        super(ProgressPopup, self).__init__(**kwargs)
        self.cancel_func = cancel_func

    def cancel(self):
        self.dismiss()
        self.cancel_func()

    def update(self, feedback):
        self.progress_bar.value = feedback.percent_complete
        self.progress_text.text = str(int(feedback.percent_complete * 100)) + "%"


class ResultsPopup(Popup):
    """
    Show a SystemTestResult

    Args:
        Popup (_type_): _description_
    """

    scroll_content = ObjectProperty(None)
    result = SystemTestResult()

    def __init__(self, results, text, **kwargs):
        super(ResultsPopup, self).__init__(**kwargs)

        # If we get text back from the action, display it first
        if text:
            self.scroll_content.add_widget(
                Label(text=text, halign="center", height=40, size_hint_y=None)
            )

        # Put the errors first
        sorted_statuses = sorted(results.result.status, key=lambda s: s.level, reverse=True)
        for status in sorted_statuses:
            self.scroll_content.add_widget(StatusLabel(status))

        # Reset the height of the box to fit all the children
        self.scroll_content.height = sum([x.height for x in self.scroll_content.children])


class StatusLabel(GridLayout):
    """
    Displays a DiagnosticStatus as a single row, with status name on left, key value pairs ending with "_message" on the right and coloured according to level
    """

    name_label = ObjectProperty(None)
    messages_box = ObjectProperty(None)

    level_color = {
        DiagnosticStatus.ERROR: [1, 0, 0, 0.2],
        DiagnosticStatus.WARN: [1, 0.6, 0, 0.2],
        DiagnosticStatus.OK: [0, 1, 0, 0.2],
    }

    def __init__(self, status, **kwargs):
        self.color = self.level_color[status.level]
        super(StatusLabel, self).__init__(**kwargs)
        self.name_label.text = status.name.replace("System test:", "")

        messages = [m.value for m in status.values if m.key.endswith("_message")]

        # Add an empty placeholder if there are no messages
        if not messages and not status.message:
            self.messages_box.add_widget(
                Label(text="", halign="center", valign="center", height=30, size_hint_y=None)
            )

        # Add the status message at the top if it exists
        if status.message:
            self.messages_box.add_widget(
                Label(
                    text=status.message,
                    halign="center",
                    valign="center",
                    height=30,
                    size_hint_y=None,
                )
            )
        # Add the KeyValue messages
        for message in messages:
            self.messages_box.add_widget(
                Label(text=message, halign="center", valign="center", height=30, size_hint_y=None)
            )

        # Calculate the height based on the number of messages
        self.height = sum([x.height for x in self.messages_box.children])

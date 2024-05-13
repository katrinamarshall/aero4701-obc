import os

## Kivy imports are placed after standard library imports
import kivy

kivy.require("1.2.0")
from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ObjectProperty, StringProperty
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.popup import Popup

import seawolf

Builder.load_file(seawolf.form_kivy_config_path("stateQueueEditor.kv"))


class StateQueueEditor(FloatLayout):

    text_input = ObjectProperty(None)

    def __init__(self, **kwargs):
        super(StateQueueEditor, self).__init__(**kwargs)

    def dismiss_popup(self):
        self._popup.dismiss()

    def show_load(self):
        """
        opens file structure popup for loading file to edit
        """
        content = EditDialog(load=self.load, cancel=self.dismiss_popup)
        self._popup = Popup(title="Load file", content=content, size_hint=(0.9, 0.9))
        self._popup.open()  # launch popup window for loading file

    def show_save(self):
        """
        opens file structure popup for selecting file/ entering filename to save edited text as
        """
        content = SaveDialog(save=self.save, cancel=self.dismiss_popup)
        self._popup = Popup(title="Save file", content=content, size_hint=(0.9, 0.9))
        self._popup.open()  # launch popup window for editing file

    def load(self, filename):
        """
        opens file and reads text from file, fills widget text field with loaded text
        """
        with open(filename[0]) as stream:
            self.text_input.text = stream.read()

        self.dismiss_popup()  # closes popup once done

    def save(self, path, filename):
        """
        writes text from widget editing window to selected path + filename
        """
        with open(os.path.join(path, filename), "w") as stream:
            stream.write(self.text_input.text)

        self.dismiss_popup()  # closes popup once done


class EditDialog(FloatLayout):
    """
    Container widget for FileChooserListView widget and Edit and Cancel buttons
    """

    load = ObjectProperty(None)
    cancel = ObjectProperty(None)
    auto_scripts_path = StringProperty(
        "/home/user/catkin_ws/src/seawolf/auto_scripts"
    )  # absolute path to auto_scripts folder

    def __init__(self, **kwargs):
        super(EditDialog, self).__init__(**kwargs)


class SaveDialog(FloatLayout):
    """
    Container widget for FileChooserListView widget and Save and Cancel buttons
    """

    save = ObjectProperty(None)
    text_input = ObjectProperty(None)
    cancel = ObjectProperty(None)
    auto_scripts_path = StringProperty(
        "/home/user/catkin_ws/src/seawolf/auto_scripts"
    )  # absolute path to auto_scripts folder


if __name__ == "__main__":

    class WidgetApp(App):
        def build(self):
            return StateQueueEditor()

    WidgetApp().run()

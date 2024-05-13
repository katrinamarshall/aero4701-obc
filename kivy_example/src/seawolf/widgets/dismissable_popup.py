import kivy

kivy.require("1.2.0")
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.popup import Popup


def show_dismissable_popup(
    title, text, size_x=700, size_y=400, ok_button_fn=None, ok_button_text="Ok"
):
    """
    Show a popup that can be dismissed (there must be a window already present).
    Ok button can be used to call some function.  The popup must be closed from the function called
    Ie. add 'self.popup.dismiss()' to start of function bound to 'Ok' button
    """
    ok_button = Button(
        size_hint=(None, None),
        size=(80, 40),
        text=ok_button_text,
    )
    cancel_button = Button(
        size_hint=(None, None),
        size=(80, 40),
        text="Close",
    )
    popup_content = BoxLayout(orientation="vertical")
    popup_content.add_widget(Label(text=text, markup=True))
    button_content = BoxLayout(orientation="horizontal", size_hint=(0.1, 0.1))
    if ok_button_fn:
        button_content.add_widget(ok_button)
    button_content.add_widget(cancel_button)
    popup_content.add_widget(button_content)

    popup = Popup(
        title=title, content=popup_content, size_hint=(None, None), size=(size_x, size_y)
    )

    cancel_button.bind(on_release=popup.dismiss)
    if ok_button_fn:
        ok_button.bind(on_release=ok_button_fn)

    popup.open()
    return popup

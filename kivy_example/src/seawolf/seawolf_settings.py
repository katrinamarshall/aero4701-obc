import json

seawolf_settings_list = [
    {"type": "title", "title": "View"},
    {
        "type": "bool",
        "title": "Demo View Toggle",
        "desc": "ON: Demo view, OFF: Pilot view",
        "section": "Seawolf",
        "key": "demo_view",
    },
    {
        "type": "bool",
        "title": "Terminal",
        "desc": "Terminal message display",
        "section": "Seawolf",
        "key": "terminal_switch",
    },
    {"type": "title", "title": "Press F2 for keyboard help"},
    {"type": "title", "title": "Some examples of other types of settings"},
    {
        "type": "numeric",
        "title": "A numeric setting",
        "desc": "Numeric description text",
        "section": "Seawolf",
        "key": "numericexample",
    },
    {
        "type": "options",
        "title": "An options setting",
        "desc": "Options description text",
        "section": "Seawolf",
        "key": "optionsexample",
        "options": ["option1", "option2", "option3"],
    },
    {
        "type": "string",
        "title": "A string setting",
        "desc": "String description text",
        "section": "Seawolf",
        "key": "stringexample",
    },
    {
        "type": "path",
        "title": "A path setting",
        "desc": "Path description text",
        "section": "Seawolf",
        "key": "pathexample",
    },
]
seawolf_settings_json = json.dumps(seawolf_settings_list)

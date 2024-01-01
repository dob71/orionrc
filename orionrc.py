#!/usr/bin/env python

import wx
import os
import configparser
from mctrl import *

# Get the Windows user home directory
USER_HOME = os.path.expanduser("~")

# Global configuration file names
WINDOW_CONFIG_FILE = os.path.join(USER_HOME, "orionrc_window_config.ini")
APP_CONFIG_FILE = os.path.join(USER_HOME, "orionrc_app_config.ini")

# Global timer firing rate in milliseconds
TIMER_INTERVAL_MS = 100

# Defaults for settings not found in the configuration file
DEFAULT_SETTINGS = {
    "com_port": "COM1",
    "zoom_id": "1",
    "focuser_id": "2",
    "focus_step_speed": "500"
}

# Zoom min and max values for zoom control
ZOOM_MIN_VAL = 8
ZOOM_MAX_VAL = 24

# Controls if the application needs to be restarted after closing
global NEED_RESTART
NEED_RESTART = False

class MyApp(wx.App):
    def OnInit(self):
        frame = MyFrame(None, title="OrionRc")
        self.SetTopWindow(frame)
        frame.Show(True)
        return True

class MyFrame(wx.Frame):
    def __init__(self, *args, **kw):
        super(MyFrame, self).__init__(*args, **kw)

        # Create a panel to hold the widgets
        panel = wx.Panel(self)

        # Create a notebook (tabbed interface)
        notebook = wx.Notebook(panel)

        # Add the tabs to the notebook
        self.setup_zoom_tab(notebook)
        self.setup_focus_tab(notebook)
        self.setup_settings_tab(notebook)

        # Create a box sizer for the overall layout
        vbox = wx.BoxSizer(wx.VERTICAL)

        # Add the notebook to the main sizer
        vbox.Add(notebook, proportion=1, flag=wx.EXPAND|wx.ALL, border=10)

        # Set the panel's sizer
        panel.SetSizer(vbox)

        # Adjust the size of the frame to fit the content
        self.Fit()

        # Load the initial window position and size from the configuration file
        self.load_window_position_and_size()

        # Load settings from the configuration file, prep MCTRLSettings instance
        self.settings = MCTRLSettings()
        self.load_settings()

        # Bind the EVT_CLOSE event to the OnClose method
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # Create and start the wx Timer
        self.timer = wx.Timer(self)
        self.timer.Start(TIMER_INTERVAL_MS)  # Set the timer firing rate in milliseconds
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)  # Bind the timer event to the on_timer method

        # Initialize the motor controller
        try:
            MCTRL.initialize(self.settings)
        except Exception as e:
            wx.MessageBox(f"Error initializing the motor controller: {str(e)}", "Error", wx.OK | wx.ICON_ERROR)

        # Set the initial slider position and "Current Zoom Level" on the zoom tab
        # If not enabled or not working remove the tab
        if not self.set_zoom_current_values():
            self.remove_tab(notebook, "Zoom")

        # Set the "Current Steps" and "Current Motor Angle" fields on the focus tab
        # If not enabled or not working remove the tab
        if not self.set_focus_current_values():
            self.remove_tab(notebook, "Focus")

    def setup_zoom_tab(self, notebook):
        zoom_tab = wx.Panel(notebook)
        hbox_zoom = wx.BoxSizer(wx.HORIZONTAL)  # Use horizontal box sizer

        target_zoom_label = wx.StaticText(zoom_tab, label="Target Zoom Level:")
        self.target_zoom_input = wx.TextCtrl(zoom_tab, style=wx.TE_PROCESS_ENTER)  # Add TE_PROCESS_ENTER style
        hbox_zoom.Add(target_zoom_label, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)  # Center the label vertically
        hbox_zoom.Add(self.target_zoom_input, flag=wx.ALL|wx.EXPAND, border=10)  # Expand the input box
        # Bind EVT_TEXT_ENTER event to on_zoom_target_enter
        self.target_zoom_input.Bind(wx.EVT_TEXT_ENTER, self.on_zoom_target_enter)

        # Add a stretch spacer to align items to the right
        hbox_zoom.AddStretchSpacer()

        current_zoom_label = wx.StaticText(zoom_tab, label="Current Zoom Level:")
        hbox_zoom.Add(current_zoom_label, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)  # Center the label vertically

        # Add the text field (readonly) for displaying the current zoom level
        self.current_zoom_value = wx.TextCtrl(zoom_tab, style=wx.TE_READONLY|wx.ALIGN_RIGHT)
        hbox_zoom.Add(self.current_zoom_value, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)  # Center the text field vertically

        vbox_zoom = wx.BoxSizer(wx.VERTICAL)
        vbox_zoom.Add(hbox_zoom, flag=wx.EXPAND|wx.ALL, border=10)  # Add the horizontal box sizer to a vertical sizer

        self.slider = wx.Slider(zoom_tab, value=8, minValue=ZOOM_MIN_VAL, maxValue=ZOOM_MAX_VAL, style=wx.SL_HORIZONTAL|wx.SL_LABELS)
        self.slider.SetTickFreq(1)  # Show ticks at intervals of 1
        self.slider.Bind(wx.EVT_SLIDER, self.on_slider_change)  # Bind slider change event
        vbox_zoom.Add(self.slider, flag=wx.EXPAND|wx.ALL, border=10)

        zoom_tab.SetSizer(vbox_zoom)

        notebook.AddPage(zoom_tab, "Zoom")

    def setup_focus_tab(self, notebook):
        focus_tab = wx.Panel(notebook)
        vbox_focus = wx.BoxSizer(wx.VERTICAL)

        hbox_current_steps = wx.BoxSizer(wx.HORIZONTAL)
        current_steps_label = wx.StaticText(focus_tab, label="Current Steps:")
        hbox_current_steps.Add(current_steps_label, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)

        self.current_steps_value = wx.TextCtrl(focus_tab, style=wx.TE_READONLY|wx.ALIGN_RIGHT)
        hbox_current_steps.Add(self.current_steps_value, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)

        # Add the Reset button
        reset_button = wx.Button(focus_tab, label="Reset")
        hbox_current_steps.Add(reset_button, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)
        reset_button.Bind(wx.EVT_BUTTON, self.on_focus_reset_steps)

        vbox_focus.Add(hbox_current_steps, flag=wx.EXPAND|wx.ALL, border=10)

        hbox_current_angle = wx.BoxSizer(wx.HORIZONTAL)
        current_angle_label = wx.StaticText(focus_tab, label="Current Motor Angle:")
        hbox_current_angle.Add(current_angle_label, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)

        self.current_angle_value = wx.TextCtrl(focus_tab, style=wx.TE_READONLY|wx.ALIGN_RIGHT)
        hbox_current_angle.Add(self.current_angle_value, flag=wx.ALL|wx.ALIGN_CENTER_VERTICAL, border=10)

        vbox_focus.Add(hbox_current_angle, flag=wx.EXPAND|wx.ALL, border=10)

        hbox_buttons = wx.BoxSizer(wx.HORIZONTAL)
        vbox_up = wx.BoxSizer(wx.VERTICAL)
        vbox_down = wx.BoxSizer(wx.VERTICAL)

        up_label = wx.StaticText(focus_tab, label="Up:")
        vbox_up.Add(up_label, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)

        self.up_button_values = [1, 10, 100, 1000]
        for value in self.up_button_values:
            button = wx.Button(focus_tab, label=str(value))
            vbox_up.Add(button, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=5)
            button.Bind(wx.EVT_BUTTON, lambda event, value=value: self.on_focus_button(event, value))

        down_label = wx.StaticText(focus_tab, label="Down:")
        vbox_down.Add(down_label, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)

        self.down_button_values = [-1, -10, -100, -1000]
        for value in self.down_button_values:
            button = wx.Button(focus_tab, label=str(abs(value)))
            vbox_down.Add(button, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=5)
            button.Bind(wx.EVT_BUTTON, lambda event, value=value: self.on_focus_button(event, value))

        hbox_buttons.Add(vbox_up, flag=wx.EXPAND|wx.ALL, border=10)
        hbox_buttons.Add(vbox_down, flag=wx.EXPAND|wx.ALL, border=10)

        vbox_focus.Add(hbox_buttons, flag=wx.EXPAND|wx.ALL, border=10)

        focus_tab.SetSizer(vbox_focus)

        notebook.AddPage(focus_tab, "Focus")

    def setup_settings_tab(self, notebook):
        settings_tab = wx.Panel(notebook)
        vbox_settings = wx.BoxSizer(wx.VERTICAL)

        # Create a grid sizer for the labels and input fields
        grid_sizer = wx.FlexGridSizer(rows=6, cols=2, vgap=10, hgap=5)

        com_port_label = wx.StaticText(settings_tab, label=f"COM Port [{DEFAULT_SETTINGS['com_port']}]:")
        self.com_port_input = wx.TextCtrl(settings_tab)
        grid_sizer.Add(com_port_label, flag=wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=10)
        grid_sizer.Add(self.com_port_input, flag=wx.EXPAND|wx.ALL, border=10)

        zoom_id_label = wx.StaticText(settings_tab, label=f"Zoom Motor ID [{DEFAULT_SETTINGS['zoom_id']}]:")
        self.zoom_id_input = wx.TextCtrl(settings_tab)
        grid_sizer.Add(zoom_id_label, flag=wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=10)
        grid_sizer.Add(self.zoom_id_input, flag=wx.EXPAND|wx.ALL, border=10)

        focuser_id_label = wx.StaticText(settings_tab, label=f"Focuser Motor ID [{DEFAULT_SETTINGS['focuser_id']}]:")
        self.focuser_id_input = wx.TextCtrl(settings_tab)
        grid_sizer.Add(focuser_id_label, flag=wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=10)
        grid_sizer.Add(self.focuser_id_input, flag=wx.EXPAND|wx.ALL, border=10)

        focus_step_speed_label = wx.StaticText(settings_tab, label=f"Focus Step Speed [{DEFAULT_SETTINGS['focus_step_speed']}]:")
        self.focus_step_speed_input = wx.TextCtrl(settings_tab)
        grid_sizer.Add(focus_step_speed_label, flag=wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=10)
        grid_sizer.Add(self.focus_step_speed_input, flag=wx.EXPAND|wx.ALL, border=10)

        vbox_settings.Add(grid_sizer, flag=wx.EXPAND|wx.ALL, border=10)

        save_settings_button = wx.Button(settings_tab, label="Save")
        vbox_settings.Add(save_settings_button, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)
        save_settings_button.Bind(wx.EVT_BUTTON, self.on_save_settings)

        settings_tab.SetSizer(vbox_settings)

        notebook.AddPage(settings_tab, "Settings")

    def load_window_position_and_size(self):
        config = configparser.ConfigParser()
        if os.path.exists(WINDOW_CONFIG_FILE):
            try:
                config.read(WINDOW_CONFIG_FILE)
                x = config.getint("Window", "x")
                y = config.getint("Window", "y")
                width = config.getint("Window", "width")
                height = config.getint("Window", "height")
                self.SetPosition((x, y))
                self.SetSize((width, height))
            except (configparser.NoSectionError, configparser.NoOptionError) as e:
                wx.MessageBox(f"Error loading window position and size: {str(e)}", "Error", wx.OK | wx.ICON_ERROR)

    def on_save_settings(self, event):
        config = configparser.ConfigParser()
        config["Settings"] = {
            "com_port": self.com_port_input.GetValue(),
            "zoom_id": self.zoom_id_input.GetValue(),
            "focuser_id": self.focuser_id_input.GetValue(),
            #"focus_step_period": self.focus_step_period_input.GetValue(),
            #"focus_step_pause": self.focus_step_pause_input.GetValue(),
            "focus_step_speed": self.focus_step_speed_input.GetValue()
        }

        try:
            with open(APP_CONFIG_FILE, "w") as configfile:
                config.write(configfile)

            dlg = wx.MessageDialog(self, "Settings saved successfully. Do you want to restart the application?", "Restart Application", wx.YES_NO | wx.ICON_QUESTION)
            response = dlg.ShowModal()
            dlg.Destroy()

            if response == wx.ID_YES:
                self.restart_application()

        except Exception as e:
            wx.MessageBox(f"Error saving settings: {str(e)}", "Error", wx.OK | wx.ICON_ERROR)

    def load_settings(self):
        config = configparser.ConfigParser()
        if os.path.exists(APP_CONFIG_FILE):
            try:
                config.read(APP_CONFIG_FILE)
            except (configparser.NoSectionError, configparser.NoOptionError) as e:
                wx.MessageBox(f"Error loading settings: {str(e)}", "Error", wx.OK | wx.ICON_ERROR)

        if not config.has_section("Settings"):
            config.add_section("Settings")

        for option, default_value in DEFAULT_SETTINGS.items():
            if not config.has_option("Settings", option) or config.get("Settings", option) == "":
                config.set("Settings", option, default_value)

        self.com_port_input.SetValue(config.get("Settings", "com_port"))
        self.settings.com_port = config.get("Settings", "com_port")
        self.zoom_id_input.SetValue(config.get("Settings", "zoom_id"))
        self.settings.zoom_motor_id = int(config.get("Settings", "zoom_id"))
        self.focuser_id_input.SetValue(config.get("Settings", "focuser_id"))
        self.settings.focus_motor_id = int(config.get("Settings", "focuser_id"))
        self.focus_step_speed_input.SetValue(config.get("Settings", "focus_step_speed"))
        self.settings.focus_step_speed = int(config.get("Settings", "focus_step_speed"))
        #self.focus_step_period_input.SetValue(config.get("Settings", "focus_step_period"))
        #self.settings.focus_step_period = float(config.get("Settings", "focus_step_period"))
        #self.focus_step_pause_input.SetValue(config.get("Settings", "focus_step_pause"))
        #self.settings.focus_step_pause = float(config.get("Settings", "focus_step_pause"))

        self.settings.timer_interval = float(TIMER_INTERVAL_MS) / 1000.0

    def save_window_position_and_size(self):
        # Save the current window position and size to the configuration file
        x, y = self.GetPosition()
        width, height = self.GetSize()

        config = configparser.ConfigParser()
        config["Window"] = {
            "x": str(x),
            "y": str(y),
            "width": str(width),
            "height": str(height)
        }

        try:
            with open(WINDOW_CONFIG_FILE, "w") as configfile:
                config.write(configfile)
        except Exception as e:
            wx.MessageBox(f"Error saving window position and size: {str(e)}", "Error", wx.OK | wx.ICON_ERROR)

    def OnClose(self, event):
        # Call save_window_position_and_size when the window is closed
        self.save_window_position_and_size()
        try:
            MCTRL.shutdown()
        except:
            pass
        event.Skip()

    def restart_application(self):
        global NEED_RESTART
        NEED_RESTART = True
        self.Close()

    def on_timer(self, event):
        # Process the timer firing event here
        # This method will be called every 100 milliseconds (as set in TIMER_INTERVAL_MS)
        # It returns a list of string representing MCTRL events (see mctrl.py for details)
        ret = MCTRL.timer_tick()
        for val in ret:
            if val == "ZOOM_DONE":
                self.set_zoom_current_values()
            elif val == "FOCUS_DONE":
                self.set_focus_current_values()
            elif val == "FOCUS_MOVE":
                if self.last_moving == None:
                    self.last_moving = time.time()
                elif time.time() - self.last_moving > 1.0:
                    self.last_moving = time.time()
                    self.set_focus_current_values()

    def remove_tab(self, notebook, tab_name):
        # Find the tab page index by comparing the text, then remove the page
        for ii in range(notebook.GetPageCount()):
            if notebook.GetPageText(ii) == tab_name:
                # Hide the content of the tab
                notebook.RemovePage(ii)
                return True
        return False

    # ========= Zoom handlers =========

    def get_zoom_current_level(self):
        val = None
        try:
            val = ZOOM_MIN_VAL + int(round(MCTRL.get_zoom_current_level() * float(ZOOM_MAX_VAL - ZOOM_MIN_VAL)))
        except Exception as e:
            print(f"Error getting zoom position: {e}")
            pass
        if val == None:
            val = ZOOM_MIN_VAL
        return val

    def set_zoom_current_values(self):
        # do nothing if not enabled
        if not MCTRL.is_zoom_enabled():
            return False

        # Set the initial slider position and "Current Zoom Level" value
        initial_zoom_level = self.get_zoom_current_level()
        self.slider.SetValue(initial_zoom_level)
        self.current_zoom_value.SetValue(str(initial_zoom_level))

        return True

    def on_slider_change(self, event):
        # Get the slider value and set it as the "Target Zoom Level" value
        slider_value = self.slider.GetValue()
        self.target_zoom_input.SetValue(str(slider_value))
        self.on_zoom_change(slider_value)

    def on_zoom_target_enter(self, event):
        # Call the on_zoom_change subroutine with the value entered in the "Target Zoom Level" field
        target_zoom_value = self.target_zoom_input.GetValue()
        if target_zoom_value.isdigit():  # Check if the entered value is a valid integer
            self.on_zoom_change(int(target_zoom_value))

    def on_zoom_change(self, zoom_level):
        # Handle the event when the "Target Zoom Level" value is entered or the slider changes
        if zoom_level > ZOOM_MAX_VAL:
            zoom_level = ZOOM_MAX_VAL
        if zoom_level < ZOOM_MIN_VAL:
            zoom_level = ZOOM_MIN_VAL
        val = float(zoom_level - ZOOM_MIN_VAL) / float(ZOOM_MAX_VAL - ZOOM_MIN_VAL)
        MCTRL.set_zoom_current_level(val)

    # ========= Focus handlers =========

    # Get focuser current position in steps
    def get_focus_current_steps(self):
        val = MCTRL.get_focus_current_steps()
        if val == None:
            val = "Unknown"
        return val

    # Get focuser motor angle
    def get_focus_current_angle(self):
        val = MCTRL.get_focus_current_angle()
        if val == None:
            val = "Unknown"
        return val

    # Update the UI with the focuser position and angle
    def set_focus_current_values(self):
        # do nothing if not enabled
        if not MCTRL.is_focus_enabled():
            return False

        # Get the current steps and angle values
        current_steps = self.get_focus_current_steps()
        current_angle = self.get_focus_current_angle()
        # Update the "Current Steps" and "Current Motor Angle" fields
        self.current_steps_value.SetValue(str(current_steps))
        self.current_angle_value.SetValue(str(current_angle))

        return True

    # Reset the focuser position "steps" counter
    def on_focus_reset_steps(self, event):
        MCTRL.clear_focus_current_steps()
        self.current_steps_value.SetValue('0')

    # Move the focuser up or down to the requested number of "steps"
    def on_focus_button(self, event, value):
        # Handle the event when an "Up:" or "Down:" button is clicked
        # The 'value' parameter will be the positive or negative integer passed by the button
        MCTRL.move_focus(value)

if __name__ == "__main__":
    while True:
        app = MyApp(False)
        app.MainLoop()
        if NEED_RESTART:
            NEED_RESTART = False
            del app
            continue
        else:
            break

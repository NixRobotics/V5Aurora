from vex import *
import json

class RobotConfiguration:
    def __init__(self, brain: Brain, controller: Controller):
        self.enable_heading_hold = False
        self.enable_field_orient = False
        self.enable_auto_claw_down = False
        self.enable_auto_claw_mid1 = False
        self.enable_slow_ramp = False
        self.enable_motion_model = False

        self.brain = brain
        self.controller = controller

    def save_settings(self):
        print("saving settings")
        settings = {
            "heading_hold": self.enable_heading_hold,
            "field_orient": self.enable_field_orient,
            "slow_ramp": self.enable_slow_ramp,
            "auto_claw_down": self.enable_auto_claw_down,
            "auto_claw_mid1": self.enable_auto_claw_mid1,
            "motion_model": self.enable_motion_model
        }
        print("settings to save:", settings)
        with open("settings.json", "w") as f:
            json.dump(settings, f)

    def load_settings(self):
        print("loading settings")
        try:
            with open("settings.json", "r") as f:
                settings = json.load(f)
                print("settings loaded:", settings)
                self.enable_heading_hold = settings.get("heading_hold", self.enable_heading_hold)
                self.enable_field_orient = settings.get("field_orient", self.enable_field_orient)
                self.enable_slow_ramp = settings.get("slow_ramp", self.enable_slow_ramp)
                self.enable_auto_claw_down = settings.get("auto_claw_down", self.enable_auto_claw_down)
                self.enable_auto_claw_mid1 = settings.get("auto_claw_mid1", self.enable_auto_claw_mid1)
                self.enable_motion_model = settings.get("motion_model", self.enable_motion_model)
        except:
            print("settings file not found, saving default settings")
            self.save_settings()
        print("heading_hold:", self.enable_heading_hold)
        print("field_orient:", self.enable_field_orient)
        print("slow_ramp:", self.enable_slow_ramp)
        print("auto_claw_down:", self.enable_auto_claw_down)
        print("auto_claw_mid1:", self.enable_auto_claw_mid1)
        print("motion_model:", self.enable_motion_model)

    def configuration_UI(self):
         # Use up and down arrows to select different menu items on screen
        # Left and right arrows to change the value of the selected menu item
        # Pressing A confirms the selected option
        # Changes saved to SDCard and read upon next initialization
        # Current options are:
        # - Enable / Disable Heading Hold
        # - Enable / Disable Field Orient
        # - Enable / Disable Auto Claw Down
        # - Enable / Disable Auto Claw Mid1

        self.brain.screen.clear_screen()
        self.brain.screen.set_cursor(1, 1)
        self.brain.screen.print("Robot Configuration")
        self.brain.screen.new_line()
        self.brain.screen.print("Use arrows to navigate")
        self.brain.screen.new_line()
        self.brain.screen.print("Hold A to save and exit")
        self.brain.screen.new_line()
        self.brain.screen.print("Press B to discard changesand exit")
        self.brain.screen.new_line()
        self.brain.screen.new_line()

        menu_data = [
            {"name": "Heading Hold", "enabled": self.enable_heading_hold},
            {"name": "Field Orient", "enabled": self.enable_field_orient},
            {"name": "Slow Ramp", "enabled": self.enable_slow_ramp},
            {"name": "Auto Claw Down", "enabled": self.enable_auto_claw_down},
            {"name": "Auto Claw Mid1", "enabled": self.enable_auto_claw_mid1},
            {"name": "Motion Model", "enabled": self.enable_motion_model}
        ]

        menu_selection = 0
        self.brain.screen.print("[{}] {}".format("X" if menu_data[menu_selection]["enabled"] else " ", menu_data[menu_selection]["name"]))

        # Wait for user input to navigate the menu
        pressing_timer = 0
        options_changed = False
        while True:
            if self.controller.buttonA.pressing():
                if pressing_timer > 2000:
                    self.brain.screen.clear_screen()
                    self.brain.screen.set_cursor(1, 1)
                    self.brain.screen.print("Saving settings...")
                    break
                else:
                    pressing_timer += 20
            else:
                pressing_timer = 0

            if self.controller.buttonB.pressing():
                # Discard changes and exit
                self.brain.screen.clear_screen()
                self.brain.screen.set_cursor(1, 1)
                self.brain.screen.print("Discarding changes...")
                return
            if self.controller.buttonDown.pressing():
                menu_selection = (menu_selection + 1) % len(menu_data)
                wait(200, MSEC)  # Debounce delay
                options_changed = True
            if self.controller.buttonRight.pressing():
                menu_data[menu_selection]["enabled"] = not menu_data[menu_selection]["enabled"]
                options_changed = True
                wait(200, MSEC)  # Debounce delay
            if options_changed:
                options_changed = False
                self.brain.screen.set_cursor(6, 1)
                self.brain.screen.print("[{}] {}          ".format("X" if menu_data[menu_selection]["enabled"] else " ", menu_data[menu_selection]["name"]))
            wait(20, MSEC)

        self.enable_heading_hold = menu_data[0]["enabled"]
        self.enable_field_orient = menu_data[1]["enabled"]
        self.enable_slow_ramp = menu_data[2]["enabled"]
        self.enable_auto_claw_down = menu_data[3]["enabled"]
        self.enable_auto_claw_mid1 = menu_data[4]["enabled"]
        self.enable_motion_model = menu_data[5]["enabled"]

        self.brain.screen.new_line()
        self.save_settings()
        self.brain.screen.print("Settings saved!")
        wait(1000, MSEC)


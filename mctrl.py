from math import sin, cos, copysign
from lx225 import *
import time
import atexit

# The zoom control is simple mappping of the servo angle to the zoom wheel
# position on the Baader Hyperion Zoom eyepiece. The mechanics are that
# full 270 degrees of motion correspond to the full zoom range of the eyepiece.

# The focuser is currently operating in motor mode making timed motor moves
# that are translated into "steps" value based on the period of time the motor
# is left moving. This is not pecise and serves only for the remote control.
# The newer servo with 360 deg magnetic position encoder should allow to change
# that in the future.

class MCTRLSettings:
    def __init__(self) -> None:
        self.com_port : str = "COM1"           # serial port to use
        self.zoom_motor_id : int = 1           # ID of the zoom motor
        self.focus_motor_id : int = 2          # ID of the focuser motor
        self.timer_interval : float = 0.1      # The timer_tick() call interval
        self.focus_step_speed : int = 500      # focuser minimal step motor speed (in LX225 protocol units)
        self.serial_timeout : float = 0.2      # serial port communication timeout
        self.zoom_min_angle : float = 270.0    # zoom min angle position
        self.zoom_max_angle : float = 0.0      # zoom max angle position
        self.zoom_in_to_max_time : float = 5.0 # seconds to change zoom from its min to max or back (determines zoom motor speed)

class MCTRL:
    zoom_motor_state = "UNINITIALIZED"
    focus_motor_state = "UNINITIALIZED"
    focus_position = 0 # curret position (in steps, whatever they are)
    focus_steps = 0    # steps to make (up or down)
    s = MCTRLSettings()

    # Call to init motor controller
    # _s : motor controller settings
    @staticmethod
    def initialize(_s : MCTRLSettings) -> None:
        MCTRL.s = _s
        if MCTRL.s.zoom_motor_id == MCTRL.s.focus_motor_id:
            raise ServoArgumentError(
                "Servo IDs for zoom and focus should be different"
            )
        LX225.initialize(MCTRL.s.com_port, MCTRL.s.serial_timeout)
        MCTRL.zoom_motor = LX225(MCTRL.s.zoom_motor_id)
        MCTRL.zoom_motor.servo_mode()
        MCTRL.zoom_motor.set_angle_limits(0, 270)
        MCTRL.zoom_motor.disable_torque()
        MCTRL.zoom_motor_state = "READY"
        MCTRL.focus_motor = LX225(MCTRL.s.focus_motor_id)
        MCTRL.focus_motor.motor_mode(0)
        MCTRL.focus_motor.disable_torque()
        MCTRL.focus_position = 0
        MCTRL.focus_steps = 0
        MCTRL.focus_motor_state = "READY"
        time.sleep(1.0) # Without this the initial position read times out

    # Call to shut down motor controller
    @staticmethod
    def shutdown() -> None:
        if MCTRL.zoom_motor_state != "UNINITIALIZED":
            MCTRL.zoom_motor.disable_torque()
            MCTRL.zoom_motor_state = "UNINITIALIZED"
        if MCTRL.focus_motor_state != "UNINITIALIZED":
            MCTRL.focus_motor.disable_torque()
            MCTRL.zoom_motor_state = "UNINITIALIZED"
        del MCTRL.zoom_motor
        del MCTRL.focus_motor
        LX255.close()

    # Timer tick (called every 0.1 sec)
    # Returns a list of strings for the caller to act upon
    @staticmethod
    def timer_tick():
        ret = []
        if MCTRL.zoom_motor_state == "BUSY":
            if MCTRL.zoom_motor_busy_until_time < time.time():
                MCTRL.zoom_motor.disable_torque()
                MCTRL.zoom_motor_state = "READY"
                ret.append("ZOOM_DONE")
            else:
                ret.append("ZOOM_MOVING")

        if MCTRL.focus_motor_state == "BUSY":
            if MCTRL.focus_steps == 0:
                MCTRL.focus_motor.motor_mode(0);
                MCTRL.focus_motor.disable_torque()
                MCTRL.focus_motor_state = "READY"
                ret.append("FOCUS_DONE")
            else:
                MCTRL.focus_do_steps()
                ret.append("FOCUS_MOVING")

        return ret
            
    # Returns normalized zoom level as a float between 0.0 and 1.0
    @staticmethod
    def get_zoom_current_level():
        val = None
        if MCTRL.zoom_motor_state != "UNINITIALIZED":
            angle = MCTRL.zoom_motor.get_physical_angle()
            val = (angle - MCTRL.s.zoom_min_angle) / (MCTRL.s.zoom_max_angle - MCTRL.s.zoom_min_angle)
            if val < 0.0:
                return 0.0
            if val > 1.0:
                return 1.0
        return val

    # Sets zoom level
    # val : target zoom value as a normalized float between 0.0 and 1.0
    @staticmethod
    def set_zoom_current_level(val : float) -> None:
        if val < 0.0 or val > 1.0:
            raise ServoArgumentError(
                f"Trget zoom value {val} must be normalized to the range from 0.0 to 1.0"
            )
        if MCTRL.zoom_motor_state == "UNINITIALIZED":
            raise ServoLogicalError(
                f"Zoom motor is not initialized!"
            )
        elif MCTRL.zoom_motor_state == "BUSY":
            MCTRL.zoom_motor.disable_torque()
            MCTRL.zoom_motor_state = "READY"
        elif MCTRL.zoom_motor_state == "READY":
            pass
        else:
            raise ServoLogicalError(
                f"Unknown zoom motor state {MCTRL.zoom_motor_state}"
            )
        old_angle_norm = MCTRL.get_zoom_current_level()
        new_angle_real = MCTRL.s.zoom_min_angle + val * (MCTRL.s.zoom_max_angle - MCTRL.s.zoom_min_angle)
        wait_time = MCTRL.s.zoom_in_to_max_time * abs(val - old_angle_norm)
        time_ms = int(round(wait_time * 1000)) # the motor takes time in milliseconds (accepted range 0 - 30000)
        if time_ms > 30000:
            time_ms = 30000
        if MCTRL.zoom_motor_state == "READY":
            MCTRL.zoom_motor.enable_torque()
            MCTRL.zoom_motor.move(new_angle_real, time_ms)
            MCTRL.zoom_motor_busy_until_time = time.time() + wait_time + 0.1 + (wait_time * 0.1) # give it 0.1sec + 10% extra time
            MCTRL.zoom_motor_state = "BUSY"


    # Returns the focuser position in "steps" made (can be negative)
    @staticmethod
    def get_focus_current_steps():
        return MCTRL.focus_position

    # Returns the focuser motor angle (or None if motor is not initialized)
    @staticmethod
    def get_focus_current_angle():
        val = None
        if MCTRL.focus_motor_state != "UNINITIALIZED":
            val = MCTRL.focus_motor.get_physical_angle()
        return val

    # Reset the focuser position "steps" counter
    @staticmethod
    def clear_focus_current_steps():
        MCTRL.focus_position = 0

    # Inititate the focuser move up or down (or update if already moving) to the requested number of "steps"
    @staticmethod
    def move_focus(value) -> None:
        if MCTRL.focus_motor_state == "UNINITIALIZED":
            raise ServoLogicalError(
                f"Focuser motor is not initialized!"
            )
        elif MCTRL.focus_motor_state == "READY":
            MCTRL.focus_motor.enable_torque()
            MCTRL.focus_steps = value
            MCTRL.focus_motor_state = "BUSY"
        elif MCTRL.focus_motor_state == "BUSY":
            MCTRL.focus_steps += value
        else:
            raise ServoLogicalError(
                f"Unknown focuser motor state {MCTRL.focus_motor_state}"
            )

    # Called by timer function when focuser has steps to do
    # Performs one step (blocking app)
    @staticmethod
    def focus_do_steps() -> None:
        MCTRL.focus_motor.motor_mode(int(copysign(MCTRL.s.focus_step_speed, MCTRL.focus_steps)));
        if abs(MCTRL.focus_steps) < 10: # for small steps
            time.sleep(MCTRL.s.timer_interval / 10.0)
            MCTRL.focus_motor.motor_mode(0);
            MCTRL.focus_position += int(copysign(1, MCTRL.focus_steps))
            MCTRL.focus_steps -= int(copysign(1, MCTRL.focus_steps))
        else:
            MCTRL.focus_position += int(copysign(10, MCTRL.focus_steps))
            MCTRL.focus_steps -= int(copysign(10, MCTRL.focus_steps))
        

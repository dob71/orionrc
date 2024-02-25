from math import sin, cos, copysign
from hx35 import *
import time
import atexit

# The zoom control is simple mappping of the servo angle to the zoom wheel
# position of the Baader Hyperion Zoom eyepiece. The mechanics are that
# full 270 degrees of motion correspond to the full zoom range of the eyepiece.

# The focuser is currently operating in motor mode making timed moves to achieve
# the target position in "steps", where each step is equal to 1 degree of the focuser
# wheel turn. The logic allows to make more than 360 steps (multiple full wheel turns).
# The HX35 motor encoder precision is 360/1500 (0.24) of a degree.
# In order to position the focuser to a specific distance one might have to movet it
# all the way down to reach the lower limit where wheel spins without any more movement
# and reset the position steps value to 0 there. After that it can be moved up to the
# desired position, and as long as the limits are no longer hit and no slipping occurs
# the same positions in steps should correspond to the same focuser tube positions.

class MCTRLSettings:
    def __init__(self) -> None:
        self.com_port : str = "COM1"           # serial port to use
        self.zoom_motor_id : int = 1           # ID of the zoom motor
        self.focus_motor_id : int = 2          # ID of the focuser motor
        self.timer_interval : float = 0.1      # The timer_tick() call interval
        self.focus_step_speed : int = 1000     # focuser motor speed (in the motor protocol units)
        self.focus_pos_precision : float = 0.3 # target precision for positioning the focuser wheel (in degrees)
        self.serial_timeout : float = 0.2      # serial port communication timeout
        self.zoom_min_angle : float = 0.0      # zoom min angle position
        self.zoom_max_angle : float = 270.0    # zoom max angle position
        self.zoom_in_to_max_time : float = 5.0 # seconds to change zoom from its min to max or back (determines zoom motor speed)

class MCTRL:
    zoom_motor_state = "UNINITIALIZED"
    focus_motor_state = "UNINITIALIZED"
    focus_position = 0        # curret position (in steps)
    focus_target_position = 0 # target position (like the above)
    train_state = 0           # 0 - off, 1 - on
    last_known_time = 0.0     # monotonic time of the last timer tick is stored here
    focus_motor_last_run_init_speed = 0.0 # for training
    focus_motor_last_run_time = 0.0       # for training
    focus_motor_last_run_direction = 0.0  # for training
    s = MCTRLSettings()

    # Call to init motor controller
    # _s : motor controller settings
    @staticmethod
    def initialize(_s : MCTRLSettings) -> None:
        MCTRL.s = _s
        if MCTRL.s.zoom_motor_id == MCTRL.s.focus_motor_id and MCTRL.s.zoom_motor_id > 0:
            raise ServoArgumentError(
                "Servo IDs for zoom and focus should be different"
            )
        HX35.initialize(MCTRL.s.com_port, MCTRL.s.serial_timeout)
        if MCTRL.s.zoom_motor_id != 0:
            MCTRL.zoom_motor = HX35(MCTRL.s.zoom_motor_id)
            MCTRL.zoom_motor.servo_mode()
            MCTRL.zoom_motor.set_angle_limits(0, 270.0)
            MCTRL.zoom_motor.disable_torque()
            MCTRL.zoom_motor.set_temp_limit(100)
        if MCTRL.s.focus_motor_id != 0:
            MCTRL.focus_motor = HX35(MCTRL.s.focus_motor_id)
            MCTRL.focus_motor.motor_mode(0)
            MCTRL.focus_motor.disable_torque()
            MCTRL.focus_motor.set_temp_limit(100)
            MCTRL.focus_position = 0.0
            MCTRL.focus_target_position = 0.0
        time.sleep(1.0) # Without this the initial position read times out
        if MCTRL.s.zoom_motor_id != 0:
            MCTRL.set_zoom_motor_state("READY")
        if MCTRL.s.focus_motor_id != 0:
            MCTRL.focus_motor_last_known_angle = MCTRL.focus_motor.get_physical_angle()
            MCTRL.focus_temp = MCTRL.focus_motor.get_temp() # temperature
            MCTRL.focus_volt = MCTRL.focus_motor.get_vin()  # voltage
            MCTRL.focus_temp_volt_next_check = time.monotonic() + 3.0
            MCTRL.set_focus_motor_state("READY")

    # set zoom motor state
    @staticmethod
    def set_zoom_motor_state(s) -> None:
        #print(f"zoom: {MCTRL.zoom_motor_state} -> {s}")
        MCTRL.zoom_motor_state = s

    # Check if Zoom is enabled
    # Returns true if the Zoom motor is initialized
    @staticmethod
    def is_zoom_enabled():
        return MCTRL.zoom_motor_state != "UNINITIALIZED"

    # set focus motor state
    @staticmethod
    def set_focus_motor_state(s) -> None:
        #print(f"zoom: {MCTRL.focus_motor_state} -> {s}")
        MCTRL.focus_motor_state = s

    # Call to shut down motor controller
    @staticmethod
    def shutdown() -> None:
        if MCTRL.zoom_motor_state != "UNINITIALIZED":
            MCTRL.zoom_motor.disable_torque()
            MCTRL.set_zoom_motor_state("UNINITIALIZED")
        if MCTRL.focus_motor_state != "UNINITIALIZED":
            MCTRL.focus_motor.disable_torque()
            MCTRL.set_focus_motor_state("UNINITIALIZED")
        del MCTRL.zoom_motor
        del MCTRL.focus_motor
        HX35.close()

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
            MCTRL.set_zoom_motor_state("READY")
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
            MCTRL.zoom_motor_busy_until_time = time.monotonic() + (wait_time * 1.1) + 0.1 # give it 0.1sec + 10% extra time
            MCTRL.set_zoom_motor_state("BUSY")

    # Check if Focus motor is enabled
    # Returns true if the Focuser motor is initialized
    @staticmethod
    def is_focus_enabled():
        return MCTRL.focus_motor_state != "UNINITIALIZED"

    # Returns the focuser position in "steps" made (can be negative)
    @staticmethod
    def get_focus_current_position():
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
    def reset_focus_current_position(value = 0.0):
        MCTRL.focus_position = value
        MCTRL.focus_target_position = MCTRL.focus_position
        MCTRL.focus_motor_last_known_angle = MCTRL.focus_motor.get_physical_angle()

    # Start/stop or add a row of data for training
    # negative angle means retrieve the value from motor
    @staticmethod
    def train_add_row(angle_change : float, run_time : float, direction : float, initial_speed : float):
        if MCTRL.train_state == 0:
            return
        if run_time == 0.0 or direction == 0.0:
            return
        temp = MCTRL.focus_temp
        volt = MCTRL.focus_volt
        set_speed = MCTRL.s.focus_step_speed
        MCTRL.train_set.append([round(angle_change,3), round(run_time, 3), direction, round(initial_speed, 3), temp, volt, set_speed/1000.0])
        if len(MCTRL.train_set) % 32 == 0:
          print(f"rec[{len(MCTRL.train_set)}] = {MCTRL.train_set[-1]}")

    # Called on timer tick. Updates the focuser postion (in steps) basing on the
    # degrees of turn during the tick time.
    @staticmethod
    def update_focus_position(time_passed):
        last_known_angle = MCTRL.focus_motor_last_known_angle
        # the position readings often appear to be delayed or incorrect, try to mitigate
        new_angle_test = MCTRL.focus_motor.get_physical_angle()
        for ii in range(5):
            new_angle = MCTRL.focus_motor.get_physical_angle()
            if(abs(new_angle - new_angle_test) <= MCTRL.s.focus_pos_precision):
                break
            new_angle_test = new_angle
        MCTRL.focus_motor_last_known_angle = new_angle
        if new_angle > last_known_angle:
            if  new_angle - last_known_angle <= 180.0:
                angle_change = new_angle - last_known_angle
            else:
                angle_change = (new_angle - last_known_angle) - 360.0
        else:
            if last_known_angle - new_angle <= 180.0:
                angle_change = new_angle - last_known_angle
            else:
                angle_change = 360.0 - (last_known_angle - new_angle)
        MCTRL.focus_position += angle_change
        # for training
        if MCTRL.focus_motor_last_run_time == -1.0:  # use passed time if the motor stayed on full interval
            MCTRL.focus_motor_last_run_time = time_passed
        MCTRL.train_add_row(angle_change, MCTRL.focus_motor_last_run_time, MCTRL.focus_motor_last_run_direction, MCTRL.focus_motor_last_run_init_speed)
        MCTRL.focus_motor_last_run_init_speed = angle_change / time_passed
        MCTRL.focus_motor_last_run_time = 0.0
        MCTRL.focus_motor_last_run_direction = 0.0
        return

    @staticmethod
    def focus_estimate_steps_time(steps_to_make: float) -> float:
        # Normalize
        ispeed = (MCTRL.focus_motor_last_run_init_speed * copysign(1.0, steps_to_make)) / 400.0
        temp = (MCTRL.focus_temp + 100.0) / 200.0
        volt = MCTRL.focus_volt / 20.0
        power = MCTRL.s.focus_step_speed / 1000.0
        steps = abs(steps_to_make) / 40.0
        # Feed into the model
        run_time = (1.4064 + (3.4872 * steps) - (0.1769 * ispeed) - (0.9400 * temp) - (0.9994 * volt) - (0.7981 * power)) / 10.0
        return run_time if run_time >= 0.001 else 0.001
        
    # Called by timer function when focuser has to adjust the position.
    # The timer function updates the current position before the call.
    # Note: It seems like they use DC motors and PWM to control their
    #       speed. If requesting motor to move at the low speeds there is no
    #       torque. The focus_step_speed setting is chosen to provide enough
    #       torque while keeping the motor reasonably slow when constantly
    #       spinning across multiple timer_interval calls, and also allow
    #       timer_interval/10 long or even shorter time moves when the desired
    #       target position is approached. The time for the moves
    #       is estimated using the motor model built using the empirical data.
    #       It's calculated basing on the desired position change, 
    #       angular speed (degrees change during the last timer_interval),
    #       temperature and voltage. It has to be re-trained for a different
    #       type of a motor.
    @staticmethod
    def focus_do_steps(steps_to_make: float) -> None:
        MCTRL.focus_motor.motor_mode(int(copysign(MCTRL.s.focus_step_speed, steps_to_make)))
        steps_time = MCTRL.focus_estimate_steps_time(steps_to_make)
        #print(f"steps_to_make:{steps_to_make} steps_time:{steps_time}")
        MCTRL.focus_motor_last_run_direction = copysign(1.0, steps_to_make) # for training
        # Every 1 sec the motor jerks at 2x of the set speed (measured over 0.1sec),
        # and I'm not getting very consistent results in general.
        if steps_time > MCTRL.s.timer_interval * 4.0:
            MCTRL.focus_motor_last_run_time = -1.0 # for training, special value for using real time measurement
            return
        elif steps_time > (MCTRL.s.timer_interval / 10.0) * 2.0:
            time.sleep(MCTRL.s.timer_interval / 10.0)
            MCTRL.focus_motor_last_run_time = MCTRL.s.timer_interval / 10.0  # for training
        else:
            time.sleep(steps_time / 2.0)
            MCTRL.focus_motor_last_run_time = steps_time # for training
        MCTRL.focus_motor.motor_mode(0)

    # Timer tick (called every 0.1 sec)
    # Returns a list of strings for the caller to act upon
    @staticmethod
    def timer_tick():
        ret = []
        time_now = time.monotonic()

        # time passed since the previous call
        time_passed = time_now - MCTRL.last_known_time
        MCTRL.last_known_time = time_now
        if time_passed == 0.0:
            return ret

        if MCTRL.zoom_motor_state == "BUSY":
            if MCTRL.zoom_motor_busy_until_time < time_now:
                MCTRL.zoom_motor.disable_torque()
                MCTRL.set_zoom_motor_state("READY")
                ret.append("ZOOM_DONE")
            else:
                ret.append("ZOOM_MOVING")

        if MCTRL.focus_motor_state != "UNINITIALIZED":
            if MCTRL.focus_temp_volt_next_check < time_now:
                MCTRL.focus_temp = MCTRL.focus_motor.get_temp() # temperature
                MCTRL.focus_volt = MCTRL.focus_motor.get_vin()  # voltage
                MCTRL.focus_temp_volt_next_check = time_now + 3.0
            MCTRL.update_focus_position(time_passed)
        if MCTRL.focus_motor_state == "BUSY":
            # Note: the encoder resolution for HX35 motor is 0.24 deg
            steps_to_make = MCTRL.focus_target_position - MCTRL.focus_position
            if abs(steps_to_make) < MCTRL.s.focus_pos_precision:
                MCTRL.focus_motor.motor_mode(0)
                MCTRL.focus_motor.disable_torque()
                MCTRL.set_focus_motor_state("READY")
                ret.append("FOCUS_DONE")
            else:
                MCTRL.focus_do_steps(steps_to_make)
                ret.append("FOCUS_MOVING")

        return ret
            
    # Inititate the focuser move up or down (or update if already moving) to the requested number of "steps"
    @staticmethod
    def move_focus(value) -> None:
        if MCTRL.focus_motor_state == "UNINITIALIZED":
            raise ServoLogicalError(
                f"Focuser motor is not initialized!"
            )
        #MCTRL.update_focus_position()
        if MCTRL.focus_motor_state == "READY":
            MCTRL.focus_motor.enable_torque()
            MCTRL.focus_target_position = MCTRL.focus_position + value
            MCTRL.set_focus_motor_state("BUSY")
        elif MCTRL.focus_motor_state == "BUSY":
            MCTRL.focus_target_position = MCTRL.focus_position + value
        else:
            raise ServoLogicalError(
                f"Unknown focuser motor state {MCTRL.focus_motor_state}"
            )

    # Turn train state on in_val 1, turn it off 0
    @staticmethod
    def test_send_command(in_val):
        if MCTRL.focus_motor_state == "READY":
            if MCTRL.train_state == 0 and in_val == "start":
                MCTRL.train_set = []
                MCTRL.focus_motor_last_run_time = 0.0
                MCTRL.focus_motor_last_run_direction = 0.0
                MCTRL.train_state = 1
                return "recording started"
            elif MCTRL.train_state != 0 and in_val == "drop":
                MCTRL.train_set = []
                MCTRL.train_state = 0
                return "data dropped"
            elif MCTRL.train_state != 0 and in_val == "stop":
                # Dump the MCTRL.train_set array to a file
                from datetime import datetime
                import csv
                timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
                file_name = f"train_set_dump_{timestamp}.txt"
                with open(file_name, 'w') as file:
                    headers = ["angle", "rtime", "dir", "ispeed", "temp", "volt", "power"]
                    writer = csv.writer(file)    
                    writer.writerow(headers)
                    for item in MCTRL.train_set:
                        writer.writerow(item)
                MCTRL.train_set = []
                MCTRL.train_state = 0
                return file_name
        return "start | stop | drop"
        

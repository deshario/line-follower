from enum import Enum
from controller import Robot

# epuck proximity sensors
#
#             front
#        ps7        ps0
#          \        /
#     ps6 \          / ps1
#          \        /
#   ps5 ---- [epuck] ---- ps2
#          /       \
#     ps4 /         \ ps3
#            back
#
# epuck floor sensors
#
#   gs0      gs1      gs2
#  (left)  (center)  (right)

LEFT_FRONT_SENSORS = (6, 7)
RIGHT_FRONT_SENSORS = (0, 1)

TURN_LEFT = -1
TURN_RIGHT = 1

# Line following
BASE_SPEED = 2.2 # driving speed
LINE_P_GAIN = 0.9 # higher = stronger steer
MAX_SLOWDOWN = 0.5 # higher = slows down more in sharp turns

# Obstacle bypass
BYPASS_SPEED = 1.2 # speed when going around obstacles
WALL_P_GAIN = 0.005

class LogLevel(Enum):
    INFO = 'ℹ️'
    WARNING = '⚠️'

ENABLED_LOG_LEVELS = (LogLevel.INFO, LogLevel.WARNING)

class Mode(Enum):
    FOLLOW = 1
    RECOVERING = 2
    TURN_AWAY = 3
    FOLLOW_WALL = 4
    DRIVE_FORWARD = 5
    ALIGN_WITH_LINE = 6

BYPASS_STEPS = (
    Mode.TURN_AWAY,
    Mode.FOLLOW_WALL,
    Mode.DRIVE_FORWARD,
    Mode.ALIGN_WITH_LINE,
)

def clamp(value, low, high):
    return max(low, min(high, value))

class LineFollower:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        # ---------- sensors ----------
        self.floor_sensors = [self.robot.getDevice(f"gs{i}") for i in range(3)]
        self.obstacle_sensors = [self.robot.getDevice(f"ps{i}") for i in range(8)]
        for sensor in self.floor_sensors + self.obstacle_sensors:
            sensor.enable(self.timestep)

        # -------- current robot state --------
        self.mode = Mode.FOLLOW
        self.time = 0.0

        # ------- latest sensor readings -------
        self.floor_values = [0.0] * 3
        self.obstacle_values = [0.0] * 8
        self.sees_black = [False] * 3

        # ---- line following and recovery ----
        self.last_line_position = 0.0 # line position: -1 = line far left, 0 = centered, +1 = line far right
        self.smoothed_steering = 0.0
        self.last_line_seen_time = 0.0

        # Lost line
        self.line_lost_time = 0.0
        self.gap_steering = None    # steering to cross a line gap, None = spin to search

        # Bypass
        self.bypass_start = 0.0
        self.step_start = 0.0
        self.bypass_turn_side = TURN_LEFT
        self.has_left_line = False

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.update()

    def update(self):
        self.read_sensors()

        if not self.is_bypassing() and self.obstacle_ahead():
            self.start_bypass()

        if self.is_bypassing():
            self.update_bypass()
            if self.is_bypassing():
                return # bypass is still driving the robot

        if self.mode == Mode.RECOVERING:
            self.update_recovery()
        else:
            self.follow_line()

    def log(self, level, message):
        if level in ENABLED_LOG_LEVELS:
            print(f"{level.value} {message}")

    # ---------- sensors ----------
    def read_sensors(self):
        self.time = self.robot.getTime()
        self.floor_values = [s.getValue() for s in self.floor_sensors]
        self.obstacle_values = [s.getValue() for s in self.obstacle_sensors]

        self.sees_black = [v < 550 for v in self.floor_values]  # raise 550 if the line is missed
        if any(self.sees_black):
            self.last_line_seen_time = self.time
    
    def no_floor_line(self):
        return not any(self.sees_black)

    def max_reading(self, ids):
        return max(self.obstacle_values[i] for i in ids)

    def obstacle_ahead(self):
        OBSTACLE_THRESHOLD = 100 # idle reading is about 65, higher = closer
        front_sensors = LEFT_FRONT_SENSORS + RIGHT_FRONT_SENSORS

        return self.max_reading(front_sensors) > OBSTACLE_THRESHOLD

    def wall_sensor_reading(self):
        sensor = 2 if self.bypass_turn_side < 0 else 5
        return self.obstacle_values[sensor]

    # ----------------- motor control -----------------
    def set_wheels(self, left, right):
        MAX_WHEEL_SPEED = 6.28 # epuck max speed
        self.left_motor.setVelocity(clamp(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        self.right_motor.setVelocity(clamp(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))

    def drive_with_steering(self, speed, steering):
        # Positive steer = turn right | negative steer = turn left | zero = straight
        # (3, 1) --> left=4, right=2 (right)
        # (3, -1) --> left=2, right=4 (left)
        # (3, 0)  --> left=3, right=3 (straight)
        
        left_speed = speed + steering
        right_speed = speed - steering
        self.set_wheels(left_speed, right_speed)

    def spin(self, side, speed=1.0):
        self.set_wheels(side * speed, -side * speed)
 
    # ----------------- line following -----------------
    def measure_line_position(self):
        """Returns the line position: -1 = far left, 0 = center, +1 = far right."""

        WHITE_VALUE = 765.0 # floor sensor reading on white
        BLACK_VALUE = 300.0 # floor sensor reading on black

        # convert each raw reading to darkness (0 = white, 1 = black)
        darkness = []
        darkness_range = WHITE_VALUE - BLACK_VALUE
        for raw_value in self.floor_values:
            distance_from_white = WHITE_VALUE - raw_value
            sensor_darkness = distance_from_white / darkness_range
            darkness.append(clamp(sensor_darkness, 0.0, 1.0))

        left_darkness, _, right_darkness = darkness
        darkness_difference = right_darkness - left_darkness

        # dividing by the total keeps the result between -1 and +1
        return darkness_difference / sum(darkness)

    def follow_line(self):
        if self.no_floor_line():
            self.handle_line_loss()
            return
    
        STEERING_SMOOTHING = 0.04   # how slowly smoothed_steering follows steering

        line_position = self.measure_line_position()

        # keep the line at the center
        steering = LINE_P_GAIN * line_position
        self.smoothed_steering += STEERING_SMOOTHING * (steering - self.smoothed_steering)

        # full speed on straights and slower in sharp turns
        speed = BASE_SPEED * (1.0 - MAX_SLOWDOWN * abs(line_position))
        self.drive_with_steering(speed, steering)
        self.last_line_position = line_position

    # ----------------- handle line lost -----------------
    def handle_line_loss(self):
        if self.is_brief_line_loss():
            return  # keep current wheel speeds, wait a bit longer

        self.enter_recovery()
        self.drive_while_lost()

    def is_brief_line_loss(self):
        LINE_LOSS_DELAY = 0.05
        return self.time - self.last_line_seen_time < LINE_LOSS_DELAY

    def enter_recovery(self):
        self.mode = Mode.RECOVERING
        self.line_lost_time = self.time

        # last line position: middle = gap, side = corner (an L shows up on a side sensor first)
        lost_at_corner = abs(self.last_line_position) >= 0.1
        if lost_at_corner:
            self.gap_steering = None # spin to find the line
        else:
            # keep driving the same way to cross the gap
            self.gap_steering = clamp(self.smoothed_steering, -0.6, 0.6)
            self.log(LogLevel.INFO, f"Gap recovery steering: {self.gap_steering:.3f}")

    def update_recovery(self):
        MIN_RECOVERY_TIME = 0.15  # stay in recovery at least this long
        SLOW_RECOVERY_TIME = 0.5  # only log recoveries slower than this

        lost_duration = self.time - self.line_lost_time
        recovered_too_soon = lost_duration < MIN_RECOVERY_TIME
        still_recovering = self.no_floor_line() or recovered_too_soon

        if still_recovering:
            self.drive_while_lost()
            return

        if lost_duration > SLOW_RECOVERY_TIME:
            self.log(LogLevel.INFO, f"Line recovered after {lost_duration:.1f}s")
        
        self.mode = Mode.FOLLOW
        self.follow_line()
        
    def drive_while_lost(self):
        GAP_TIMEOUT = 2.6

        gap_duration = self.time - self.line_lost_time

        trying_to_cross_gap = self.gap_steering is not None
        if trying_to_cross_gap and gap_duration > GAP_TIMEOUT:
            self.log(LogLevel.WARNING, "Gap recovery timed out... searching for line")
            self.gap_steering = None
            trying_to_cross_gap = False

        if trying_to_cross_gap: # keep trying to cross the gap.
            self.drive_with_steering(1.8, self.gap_steering)
        else:
            search_side = TURN_LEFT if self.smoothed_steering < 0 else TURN_RIGHT
            self.spin(search_side, 0.8) # spin to find the line

    # ----------------- obstacle bypass -----------------
    def is_bypassing(self):
        return self.mode in BYPASS_STEPS

    def switch_step(self, step):
        self.mode, self.step_start = step, self.time

    def start_bypass(self):
        right_reading = self.max_reading(RIGHT_FRONT_SENSORS)
        left_reading = self.max_reading(LEFT_FRONT_SENSORS)

        # turn away from the side with the stronger reading (the closer obstacle)
        if left_reading > right_reading + 20:
            self.bypass_turn_side = TURN_RIGHT
        else:
            self.bypass_turn_side = TURN_LEFT
    
        self.has_left_line = False
        self.bypass_start = self.time
        self.switch_step(Mode.TURN_AWAY)

        self.log(LogLevel.INFO, "Obstacle bypass started")

    def update_bypass(self):
        if self.time - self.bypass_start > 40.0:  # stuck too long, give up
            self.log(LogLevel.WARNING, "Obstacle bypass timed out")
            self.mode = Mode.FOLLOW
            return

        # each step switches to the next one when it's done
        match self.mode:
            case Mode.TURN_AWAY:
                self.turn_away()
            case Mode.FOLLOW_WALL:
                self.follow_wall()
            case Mode.DRIVE_FORWARD:
                self.bypass_drive_forward()
            case Mode.ALIGN_WITH_LINE:
                self.align_with_line()

    def turn_away(self):
        self.spin(self.bypass_turn_side)
        obstacle_cleared = (
            not self.obstacle_ahead()
            and self.wall_sensor_reading() > 90.0
        )
        timeout = self.time - self.step_start > 3.5
        if obstacle_cleared or timeout:
            self.switch_step(Mode.FOLLOW_WALL)

    def follow_wall(self):
        WALL_LOST_READING = 85.0
        WALL_SEARCH_SLOW_WHEEL = 0.55
        WALL_TARGET_READING = 120 # higher = closer to the wall

        line_seen = any(self.sees_black)
        self.has_left_line = self.has_left_line or not line_seen
        wall_reading = self.wall_sensor_reading()

        if self.has_left_line and line_seen:
            self.switch_step(Mode.DRIVE_FORWARD) # found the line again on the far side

        elif self.obstacle_ahead():
            self.spin(self.bypass_turn_side) # something right ahead, turn away

        # lost the wall (ex: at the obstacle corner): curve back toward it.
        elif wall_reading < WALL_LOST_READING:
            if self.bypass_turn_side < 0: # turned left, so the wall is on the right
                self.set_wheels(BYPASS_SPEED, WALL_SEARCH_SLOW_WHEEL)
            else:
                self.set_wheels(WALL_SEARCH_SLOW_WHEEL, BYPASS_SPEED)
        else:
            # p-control: keep the wall at the target distance
            wall_error = wall_reading - WALL_TARGET_READING
            steering = self.bypass_turn_side * WALL_P_GAIN * wall_error

            self.drive_with_steering(BYPASS_SPEED, steering)
            
    def bypass_drive_forward(self):
        self.drive_with_steering(BYPASS_SPEED, 0.0)
        elapsed_time = self.time - self.step_start

        # drive straight so the robot body clears the obstacle corner
        if elapsed_time > 0.8:
            self.switch_step(Mode.ALIGN_WITH_LINE)

    def align_with_line(self):
        # spin until only the center sensor sees the line
        self.spin(self.bypass_turn_side)
        left, center, right = self.sees_black

        if center and not left and not right:
            self.log(LogLevel.INFO, f"Obstacle bypass completed in {self.time - self.bypass_start:.1f}s")
            self.mode = Mode.FOLLOW
            self.last_line_position = 0.0

LineFollower().run()
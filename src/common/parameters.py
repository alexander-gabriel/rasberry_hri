from math import pi

import rospy

ROBOT_NS = rospy.names.get_namespace()
NS = "{}hri".format(ROBOT_NS)
ME = ROBOT_NS.strip("/")

# the navigation system doesn't stop when requested, add some buffer space
ROBOT_REACTION_SAFETY_MARGIN = 0.5

def define(key, default):
    key_path = "{}/{}".format(NS, key)
    # rospy.set_param(key_path, default)
    if rospy.has_param(key_path):
        return rospy.get_param(key_path, default)
    else:
        rospy.set_param(key_path, default)
        return default

# Do Not Change Anything Above This Line
# --------------------------------------------------------------


VERBOSE = False

EXPERIMENT_LABEL = define("experiment_label", "NO ID SET")
EXPERIMENT_ID = define("experiment_id", "NO ID SET")
RUN_ID = define("run_id", "NO ID SET")

CONFIG_DIRECTORY = define("config_folder", "/home/rasberry/hri_config")
ACTIVE_DIRECTORY = define("active_folder", "active")
LOG_DIRECTORY = define("log_folder", "log")
STATE_DIRECTORY = define("state_folder", "state")

USE_ACTION_RECOGNITION = define("use_action_recognition", True)

# DISABLE ABILITIES

GESTURE_PERCEPTION = define("gesture_perception", True)
BEHAVIOR_PERCEPTION = define("behavior_perception", True)
DIRECTION_PERCEPTION = define("direction_perception", True)

ROBOT_CRATE_POSSESSION_PERCEPTION = define(
    "robot_crate_possession_perception", True)
PICKER_CRATE_POSSESSION_PERCEPTION = define(
    "picker_crate_possession_perception", True)
PICKER_CRATE_FILL_PERCEPTION = define("picker_crate_fill_perception", True)

BERRY_POSITION_PERCEPTION = define("berry_position_perception", True)


# PERCEPTION NOISE

# rotation -> rotation
MOVEMENT_NOISE_ALPHA = define("movement_noise_alpha", 0)
# translation -> rotation
MOVEMENT_NOISE_BETA = define("movement_noise_beta", 0)
# translation -> translation
MOVEMENT_NOISE_GAMMA = define("movement_noise_gamma", 0)
# rotation -> translation
MOVEMENT_NOISE_DELTA = define("movement_noise_delta", 0)

# TODO: movement noise addition?


# REASONING

REASONING_LOOP_FREQUENCY = 20  # Hz
ITERATIONS = define("iterations", "30")
COMPLEXITY_PENALTY = define("complexity_penalty", 0.1)
TRUTH_THRESHOLD = 0.5

INF = float("inf")
TARGET = "target"


# BDI

MAX_COST = define("max_cost", 240)
MIN_GAIN = define("min_gain", 0)

# ACTION RECOGNITION

CAMERA_TOPIC = define("camera", "/camera/color/image_raw")
CLASSIFICATION_TOLERANCE = define("classification_tolerance", 20)
ANGLE_WEIGHT = define("angle_weight", 1.0)
DETECTION_COUNT = define("detection_count", 10)
COOLDOWN = define("cooldown", 0)


# PICKER SETUP

PICKERS = define("pickers", [
    "picker01", "picker02", "picker03", "picker04", "picker05",
    "picker06", "picker07", "picker08", "picker09", "picker10"])

PICKER_LENGTH = 0.4
PICKER_WIDTH = 0.5

PICKER_SPEED = define("picker_speed", 0.7)  # m/s

PICKER_UPDATE_FREQUENCY = define("picker_sim_frequency", 30)  # Hz

PICKER_DISTANCE_PREFERENCE = define("picker_distance", 0.3)


# ROBOT SETUP

ROBOT_LENGTH = 1.5
ROBOT_WIDTH = 1.35584
ROBOT_SPEED = define("robot_speed", 0.7)
ROBOT_POSITION = define("robot_position", [5.53, 4.62])
MINIMUM_DISTANCE = define("minimum_distance", 0.1)  # m
CRATE_CAPACITY = define("crate_capacity", 2)
ROBOT_MODES = define("robot_modes", ["standing", "moving"])

# EXPERIMENT CONFIGURATION

INITIAL_WAYPOINT = define("initial_robot_waypoint", "WayPoint104")
FULL_CRATE_COUNT = define("full_crate_count", 0)
EMPTY_CRATE_COUNT = define("empty_crate_count", 2)
TARGET_PICKER = define("target_picker", "picker01")

INITIAL_PICKER_POSITION = define("picker_position",
                                 [17.561, 4.609])  # [19.997, 4.568])
INITIAL_PICKER_ORIENTATION = define("picker_orientation", 2*pi)
CALLED_ROBOT = define("called_robot", False)
DISMISSED_ROBOT = define("dismissed_robot", False)
SEEN_PICKING = define("seen_picking", False)
HAS_CRATE = define("has_crate", False)
CRATE_FULL = define("crate_full", False)

NO_BERRY_PLACES = define("no_berry_places", [])


# WORLD SETUP

MEAN_WAYPOINT_DISTANCE = define("mean_waypoint_distance", 2.95)  # m

DEPOT = define("depot_waypoint", "WayPoint71")
READY_POINT = define("ready_waypoint", "WayPoint133")


# ROBOT ACTION PARAMETERS

GIVE_GAIN = define("give_gain", 240)
GIVE_COST = define("give_cost", 3.5)  # s

EXCHANGE_GAIN = define("exchange_gain", 240)
EXCHANGE_COST = define("exchange_cost", 5)  # s

DEPOSIT_GAIN = define("deposit_gain", 480)
DEPOSIT_COST = define("deposit_cost", 2.5)  # s

EVADE_GAIN = define("evade_gain", 200)

LEAVE_GAIN = define("leave_gain", 240)

WAIT_GAIN = define("wait_gain", 100)
WAIT_TIME = define("wait_time", 0.5)  # s

APPROACH_GAIN = define("move_gain", 180)

MOVE_GAIN = define("move_gain", 0)

MOVETO_GAIN = define("moveto_gain", 0)

TIMEOUT_LENGTH = 60  # s

# PICKER BEHAVIOURS

BEHAVIOURS = {
    "deliver standard": [
        ["call robot", 3],
        ["wait for robot to move", 0],
        ["approach without crate", 0],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "deliver no call": [
        ["expect service", 0],
        ["approach without crate", 0],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "deliver expect robot to come": [
        ["call robot", 3],
        ["wait for robot to arrive", 0],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "deliver expect mind read": [
        ["expect service", 0],
        ["wait for robot to arrive", 0],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "exchange standard": [
        ["pick berries", 4],
        ["call robot", 3],
        ["wait for robot to move", 0],
        ["approach with crate", 0],
        ["return crate", 4],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "exchange no call": [
        ["pick berries", 4],
        ["expect service", 0],
        ["approach with crate", 0],
        ["return crate", 4],
        ["get crate", 3],
        ["leave with crate", 0],
        ["pick berries", 4]
    ],
    "exchange expect robot to come": [
        ["pick berries", 4],
        ["call robot", 3],
        ["wait for robot to arrive", 0],
        ["return crate", 2.5],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "exchange expect mind read": [
        ["pick berries", 4],
        ["expect service", 0],
        ["wait for robot to arrive", 0],
        ["return crate", 4],
        ["get crate", 3],
        ["pick berries", 4]
    ],
    "call": [
        ["call robot", 3],
        ["expect service", 0]
    ],
    "do nothing": [["expect service", 0]],
    "wait": [["wait for robot to arrive", 10]],
    "picking berries": [["pick berries", 3]],
    "evade standard": [["pick berries", 3],
                   ["pass with crate", 0],
                   ["pick berries", 3]]}

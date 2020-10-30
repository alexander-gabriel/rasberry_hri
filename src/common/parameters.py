from math import pi

import rospy

VERBOSE = False

REASONING_LOOP_FREQUENCY = 20  # Hz
ME = rospy.names.get_namespace()
NS = "{}hri".format(ME)
ME = ME.strip("/")

EXPERIMENT_ID = rospy.get_param("{}/experiment_id".format(NS), "NO ID SET")

ITERATIONS = rospy.get_param("{}/iterations".format(NS), "30")
COMPLEXITY_PENALTY = rospy.get_param("{}/complexity_penalty".format(NS), 0.1)
MAX_COST = rospy.get_param("{}/max_cost", 240)
MIN_GAIN = rospy.get_param("{}/min_gain", 0)
TRUTH_THRESHOLD = 0.5
TARGET = "target"
INF = float("inf")
NO_BERRY_PLACES = rospy.get_param("{}/no_berry_places".format(NS), [])
ROBOT_SPEED = rospy.get_param("{}/robot_speed".format(NS), 0.8)
PICKER_SPEED = rospy.get_param("{}/picker_speed".format(NS), 0.8)
MINIMUM_DISTANCE = rospy.get_param("{}/minimum_distance".format(NS), 0.1)  # m
ROBOT_LENGTH = 1.5
ROBOT_WIDTH = 1.35584
PICKER_LENGTH = 0.4
PICKER_WIDTH = 0.5
PICKERS = rospy.get_param("{}/pickers".format(NS), [
    "picker01", "picker02", "picker02", "picker03", "picker04",
    "picker05", "picker06", "picker07", "picker08", "picker09",
    "picker10"])
PICKER_MODE = rospy.get_param("{}/picker_mode".format(NS), "standing")
PICKER_SPEED = rospy.get_param("{}/picker_speed".format(NS), 0.7)  # m/s
PICKER_UPDATE_FREQUENCY = rospy.get_param("{}/picker_sim_frequency".format(NS), 30)  # Hz
TARGET_PICKER = rospy.get_param(
    "{}/target_picker".format(NS), "TARGET_PICKER-IS-NOT-SET"
)
INITIAL_PICKER_POSITION = rospy.get_param("{}/picker_position".format(NS),
                                          [17.561, 4.609])
                                          # [19.997, 4.568])
INITIAL_PICKER_ORIENTATION = rospy.get_param("{}/picker_orientation"
                                             .format(NS), 2*pi)
PICKER_DISTANCE_PREFERENCE = rospy.get_param("{}/picker_distance"
                                             .format(NS), 0.3)
CALLED_ROBOT = rospy.get_param("{}/called_robot".format(NS), False)
DISMISSED_ROBOT = rospy.get_param("{}/dismissed_robot".format(NS), False)
SEEN_PICKING = rospy.get_param("{}/seen_picking".format(NS), False)
HAS_CRATE = rospy.get_param("{}/has_crate".format(NS), False)
CRATE_FULL = rospy.get_param("{}/crate_full".format(NS), False)
INITIAL_WAYPOINT = rospy.get_param(
    "{}/initial_robot_waypoint".format(NS), "WayPoint104"
)
MEAN_WAYPOINT_DISTANCE = rospy.get_param(
    "{}/mean_waypoint_distance".format(NS), 2.95
)  # m

DEPOT = rospy.get_param("{}/depot_waypoint".format(NS), "WayPoint71")
READY_POINT = rospy.get_param("{}/ready_waypoint".format(NS), "WayPoint133")

DETECTION_COUNT = rospy.get_param("{}/detection_count".format(NS), 10)
COOLDOWN = rospy.get_param("{}/cooldown".format(NS), 0)
CAMERA_TOPIC = rospy.get_param(
    "{}/camera".format(NS), "/camera/color/image_raw"
)

CLASSIFICATION_TOLERANCE = rospy.get_param(
    "{}/classification_tolerance".format(NS), 20
)
ANGLE_WEIGHT = rospy.get_param("{}/angle_weight".format(NS), 1.0)

CRATE_CAPACITY = rospy.get_param("{}/crate_capacity".format(NS), 2)
FULL_CRATE_COUNT = rospy.get_param("{}/full_crate_count".format(NS), 0)
EMPTY_CRATE_COUNT = rospy.get_param("{}/empty_crate_count".format(NS), 2)


GIVE_GAIN = rospy.get_param("{}/give_gain".format(NS), 240)
GIVE_COST = rospy.get_param("{}/give_cost".format(NS), 3.5)  # s

EXCHANGE_GAIN = rospy.get_param("{}/exchange_gain".format(NS), 240)
EXCHANGE_COST = rospy.get_param("{}/exchange_cost".format(NS), 5)  # s

DEPOSIT_GAIN = rospy.get_param("{}/deposit_gain".format(NS), 480)
DEPOSIT_COST = rospy.get_param("{}/deposit_cost".format(NS), 2.5)  # s

EVADE_GAIN = rospy.get_param("{}/evade_gain".format(NS), 200)

LEAVE_GAIN = rospy.get_param("{}/leave_gain".format(NS), 240)

WAIT_GAIN = rospy.get_param("{}/wait_gain".format(NS), 100)
WAIT_TIME = rospy.get_param("{}/wait_time".format(NS), 0.5)  # s

APPROACH_GAIN = rospy.get_param("{}/move_gain".format(NS), 180)

MOVE_GAIN = rospy.get_param("{}/move_gain".format(NS), 0)

MOVETO_GAIN = rospy.get_param("{}/moveto_gain".format(NS), 0)

TIMEOUT_LENGTH = 60  # s

# rotation -> rotation
MOVEMENT_NOISE_ALPHA = rospy.get_param("{}/movement_noise_alpha".format(NS), 1)
# translation -> rotation
MOVEMENT_NOISE_BETA = rospy.get_param("{}/movement_noise_beta".format(NS), 0)
# translation -> translation
MOVEMENT_NOISE_GAMMA = rospy.get_param("{}/movement_noise_gamma".format(NS), 1)
# rotation -> translation
MOVEMENT_NOISE_DELTA = rospy.get_param("{}/movement_noise_delta".format(NS), 0)

BEHAVIOURS = {"deliver wait after call": [["wait for robot", 2],
                                          ["call robot", 0],
                                          ["wait for robot to move", 0],
                                          ["approach without crate", 0],
                                          ["wait for robot", 2.5],
                                          ["leave with crate", 0],
                                          ["pick berries", 10]],
              "deliver no call": [["wait for robot", 2],
                                  ["wait for robot to move", 0],
                                  ["approach without crate", 0],
                                  ["wait for robot", 2.5],
                                  ["leave with crate", 0],
                                  ["pick berries", 10]],
              "deliver no wait": [["wait for robot", 2],
                                  ["call robot", 0],
                                  ["approach without crate", 0],
                                  ["wait for robot", 2.5],
                                  ["leave with crate", 0],
                                  ["pick berries", 10]],
              "exchange wait after call": [["pick berries", 2],
                                           ["call robot", 0],
                                           ["wait for robot to move", 0],
                                           ["approach with crate", 0],
                                           ["wait for robot", 5.0],
                                           ["leave with crate", 0],
                                           ["pick berries", 10]],
              "exchange no call": [["pick berries", 2],
                                   ["wait for robot to move", 0],
                                   ["approach with crate", 0],
                                   ["wait for robot", 5.0],
                                   ["leave with crate", 0],
                                   ["pick berries", 10]],
              "exchange no wait": [["pick berries", 2],
                                   ["call robot", 0],
                                   ["approach with crate", 0],
                                   ["wait for robot", 5],
                                   ["leave with crate", 0],
                                   ["pick berries", 10]],
              "wait": [["wait for robot", 10]],
              "picking berries": [["pick berries", 10]],
              "pass robot": [["pick berries", 2],
                             ["pass with crate", 0],
                             ["pick berries", 10]]}

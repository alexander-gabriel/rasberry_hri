
import rospy
VERBOSE = False

FREQUENCY = 5 # Hz
NS = "{}/hri".format(rospy.names.get_namespace())
ITERATIONS = rospy.get_param("{}/iterations".format(NS), "30")
COMPLEXITY_PENALTY = rospy.get_param("{}/complexity_penalty".format(NS), 0.1)
MAX_COST = rospy.get_param("{}/max_cost", 240)
MIN_GAIN = rospy.get_param("{}/min_gain", 100)
TRUTH_THRESHOLD = 0.5
TARGET = "target"
ME = "me"
INF = float('inf')
ROBOT_SPEED = rospy.get_param("{}/robot_speed".format(NS), 0.8)
PICKER_SPEED = rospy.get_param("{}/picker_speed".format(NS), 0.8)
MINIMUM_DISTANCE = rospy.get_param("{}/minimum_distance".format(NS), 0.5) # m
ROBOT_LENGTH = 1.5
ROBOT_WIDTH = 1.35584
PICKER_LENGTH = 0.4
PICKER_WIDTH = 0.6
TARGET_PICKER = rospy.get_param("{}/target_picker".format(NS), "Picker02")
CALLED_ROBOT = rospy.get_param("{}/called_robot".format(NS), False)
SEEN_PICKING = rospy.get_param("{}/seen_picking".format(NS), False)
INITIAL_WAYPOINT = rospy.get_param("{}/initial_robot_waypoint".format(NS), "WayPoint104")

DETECTION_COUNT = rospy.get_param("{}/detection_count".format(NS), 10)
COOLDOWN = rospy.get_param("{}/cooldown".format(NS), 0)
CAMERA_TOPIC = rospy.get_param("{}/camera".format(NS), "/camera/color/image_raw")

CLASSIFICATION_TOLERANCE = rospy.get_param("{}/classification_tolerance".format(NS), 20)
ANGLE_WEIGHT = rospy.get_param("{}/angle_weight".format(NS), 1.0)


EVADE_GAIN = rospy.get_param("{}/evade_gain".format(NS), 200)

GIVE_GAIN = rospy.get_param("{}/give_gain".format(NS), 240)
GIVE_COST = rospy.get_param("{}/give_cost".format(NS), 2.5)

EXCHANGE_GAIN = rospy.get_param("{}/exchange_gain".format(NS), 240)
EXCHANGE_COST = rospy.get_param("{}/exchange_cost".format(NS), 5)

MEAN_WAYPOINT_DISTANCE = rospy.get_param("{}/mean_waypoint_distance".format(NS), 2.95) # m


MOVE_GAIN = rospy.get_param("{}/move_gain".format(NS), 0)

MOVETO_GAIN = rospy.get_param("{}/moveto_gain".format(NS), 0)

import time
import rospy
import pickle

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

from topological_navigation.tmap_utils import get_distance

from rasberry_people_perception.topological_localiser import TopologicalNavLoc
from topological_navigation.route_search import TopologicalRouteSearch
from utils import OrderedConsistentSet, suppress, atomspace, not_has_crate, has_crate, not_seen_picking, seen_picking, standing, leaving, approaching
from opencog.type_constructors import *
from world_state import WorldState, TRUE, FALSE
from goals import ExchangeGoal, DeliverGoal, EvadeGoal, WrongParameterException
from robot_control import RobotControl

MAX_COST = 10
MIN_GAIN = 10
TRUTH_THRESHOLD = 0.5
TARGET = "target"
ME = "me"
INF = float('inf')
SPEED = 0.55
MINIMUM_DISTANCE = 0.5 # m

# TODO test conditions
# is_at(Thorvald,Waypoint4)
# is_at(Picker,Waypoint3)
# is_a(Picker,Human)
# is_a(Thorvald,Robot)
# seen_picking(Picker)
# has_crate(Picker)
#
# leads_to(Waypoint1,Waypoint2)
# leads_to(Waypoint2,Waypoint3)
# leads_to(Waypoint3,Waypoint4)
# leads_to(Waypoint4,Waypoint5)




class BDISystem:


    def __init__(self, me):
        rospy.loginfo("BDI: Initializing BDI System")
        self.me = me
        self.robco = RobotControl(self.me)
        self.world_state = WorldState(self.me)
        rospy.loginfo("BDI: Initialized World State")
        self.robot_track = []
        self.people_tracks = {}
        self.directions = {}
        self.goals = [EvadeGoal] #[DeliverGoal, ExchangeGoal, EvadeGoal]
        self.intentions = []
        self.latest_robot_msg = None
        self.latest_people_msgs = {}
        self.latest_people_nodes = {}
        self.qsrlib = QSRlib()
        self.locator = TopologicalNavLoc()
        self.links = {}
        self.node_positions = {}
        self.route_search = TopologicalRouteSearch(self.locator.tmap)
        # rospy.loginfo("BDI: Adding Example Waypoints")
        # for waypoint in [("WayPoint133", "WayPoint102"), ("WayPoint102", "WayPoint103"), ("WayPoint103", "WayPoint104"), ("WayPoint104", "WayPoint105"), ("WayPoint105", "WayPoint106")]:
        #     self.world_state.add_thing(waypoint[0], "place")
        #     self.world_state.add_thing(waypoint[1], "place")
        #     self.world_state.add_place_link(waypoint[0], waypoint[1])
        rospy.loginfo("BDI: Adding Waypoints")
        for node in self.locator.tmap.nodes:
            self.world_state.add_thing(node.name, "place")
            self.node_positions[node.name] = node.pose
            for edge in node.edges:
                self.links["_".join([node.name, edge.node])] = 0
                self.world_state.add_place_link(node.name, edge.node)
        for link in self.links.keys():
            nodes = link.split("_")
            self.links[link] = get_distance(self.node_positions[nodes[0]], self.node_positions[nodes[1]])
        rospy.loginfo("BDI: Adding Pickers")
        for picker in ["Picker01", "Picker02"]:
            self.world_state.add_thing(picker, "human")
        rospy.loginfo("BDI: Initialized BDI System")
        self.setup_experiment()

    def setup_experiment(self):
        picker = rospy.get_param("target_picker", "Picker02")
        if rospy.get_param("has_crate", True):
            has_crate(ConceptNode(picker)).tv = TRUE
        else:
            not_has_crate(ConceptNode(picker)).tv = TRUE
        if rospy.get_param("seen_picking", False):
            seen_picking(ConceptNode(picker)).tv = TRUE
        else:
            not_seen_picking(ConceptNode(picker)).tv = TRUE
        self.robco.move_to(rospy.get_param("initial_robot_pose", "WayPoint106"))


    def generate_options(self):
        rospy.logdebug("BDI: Generating Behaviour Options")
        desires = []
        for goal in self.goals:
            args_list = goal.find_instances(self.world_state)
            for args in args_list:
                if len(args) > 0:
                    try:
                        desires.append(goal(self.world_state, self.robco, args))
                    except WrongParameterException:
                        pass
        rospy.logdebug("BDI: Desires: {}".format(desires))
        rospy.logdebug("BDI: Intentions: {}".format(self.intentions))

        for intention in self.intentions:
            if intention in desires and intention.is_achieved(self.world_state):
                desires.remove(intention)
        return desires


    def filter(self, desires):
        rospy.logdebug("BDI: Filtering Desires")
        intentions = self.intentions or OrderedConsistentSet()
        for desire in desires:
            gain = desire.get_gain()
            cost = desire.get_cost()
            if gain > MIN_GAIN and cost < MAX_COST:
                intentions.append(desire)
        return intentions


    def perform_action(self):
        rospy.logdebug("BDI: Choosing Next Action")
        chosen_intention = None
        min_cost = 999999
        for intention in self.intentions:
            action = intention.get_next_action()
            with suppress(AttributeError):
                cost = action.get_cost()
                rospy.loginfo("BDI: Action option {:} with cost {:.2f}".format(action, cost))
                if cost < min_cost:
                    chosen_intention = intention
                    min_cost  = cost
        if not chosen_intention is None:
            rospy.loginfo("BDI: Following: {:}".format(chosen_intention))
            chosen_intention.perform_action()


    def loop(self):
        rospy.logdebug("----- Started Loop -----")
        self.update_beliefs()
        desires = self.generate_options()
        self.intentions = self.filter(desires)
        self.perform_action()
        self.clean_intentions()
        time.sleep(0.1)
        rospy.logdebug("----- Ended Loop -----")


    def clean_intentions(self):
        index = 0
        while index < len(self.intentions):
            if self.intentions[index].is_achieved(self.world_state):
                del self.intentions[index]
            else:
                index += 1


    def update_beliefs(self):
        rospy.logdebug("BDI: Updating Beliefs")
        timestamp = rospy.get_time()
        world = World_Trace()
        pairs = []
        added_picker = False
        for person, msg in self.latest_people_msgs.items():
            pairs.append((self.me.capitalize(), person))
            position = Object_State(name=person, timestamp=timestamp, x=msg.pose.position.x, y=msg.pose.position.y, xsize=0.6, ysize=0.4, object_type="Person")
            (current_node, closest_node) = self.locator.localise_pose(msg)
            latest_node = None
            with suppress(KeyError):
                latest_node = self.latest_people_nodes[person][1]
                if current_node != "none":
                    pass # maybe delete the latest picker position from the kb as it's no longer correct
            if current_node != "none":
                self.latest_people_nodes[person] = ("is_at", current_node)
                if current_node != latest_node:
                    self.world_state.update_position(person, current_node)
            elif closest_node is None:
                rospy.logwarn("BDI: We have no idea where {} is currently".format(person))
            #     self.latest_people_nodes[person] = ("is_near", closest_node)
            #     self.world_state.add_belief("is_near({:},{:})".format(person, closest_node))
            #     rospy.logdebug("added is_near({:},{:})".format(person, closest_node))
            if not person in self.people_tracks:
                self.people_tracks[person] = [position]
            else:
                self.people_tracks[person].append(position)
            world.add_object_state_series(self.people_tracks[person])
            added_picker = True
        if added_picker and not self.latest_robot_msg is None:
            self.robot_track.append(Object_State(name=self.me.capitalize(), timestamp=timestamp, x=self.latest_robot_msg.position.x, y=self.latest_robot_msg.position.y, xsize=1.35584, ysize=1.5))
            world.add_object_state_series(self.robot_track)

            dynamic_args = {"qtcbs": {"quantisation_factor": 0.1,
    							  "validate": True,
    							  "no_collapse": True,
    							  "qsrs_for": pairs}}
            qsrlib_request_message = QSRlib_Request_Message(which_qsr="qtcbs", input_data=world, dynamic_args=dynamic_args)
            try:
                qsrlib_response_message = self.qsrlib.request_qsrs(qsrlib_request_message)
                t = qsrlib_response_message.qsrs.get_sorted_timestamps()[-1]
                for k, v in qsrlib_response_message.qsrs.trace[t].qsrs.items():
                    entity = k.split(",")[1]
                    direction = v.qsr.get("qtcbs").split(",")[1]
                    self.directions[entity] = direction
            except (ValueError, KeyError, AttributeError, IndexError) as err:
                rospy.logwarn("BDI: {}".format(err))
            for picker, direction in self.directions.items():
                if direction == "+":
                    leaving(ConceptNode(picker)).tv = TRUE
                elif direction == "-":
                    approaching(ConceptNode(picker)).tv = TRUE
                else:
                    standing(ConceptNode(picker)).tv = TRUE
                # stop if we're to close to a picker
                distance =  get_distance(self.latest_robot_msg, self.latest_people_msgs[picker].pose)
                if distance < MINIMUM_DISTANCE:
                    pass
                    # rospy.loginfo("BDI: Robot too close to picker")
                    # self.robco.cancel_movement()


    def write(self):
        self.world_state.write()


    def save(self):
        self.world_state.save()


    def add_goal(self, goal):
        self.goals.append(goal)

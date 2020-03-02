import time
import rospy
import pickle

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

from std_msgs.msg import String
from std_srvs.srv import Empty
from topological_navigation.tmap_utils import get_distance
from topological_navigation.route_search import TopologicalRouteSearch

from rasberry_people_perception.topological_localiser import TopologicalNavLoc

from parameters import *
from utils import OrderedConsistentSet, suppress
from robot_control import RobotControl
from world_state import WorldState
from goals import ExchangeGoal, DeliverGoal, EvadeGoal, WrongParameterException


class BDISystem:

    def __init__(self, me, kb):
        rospy.loginfo("BDI: Initializing BDI System")
        self.too_close = False
        self.durations = []
        self.kb = kb
        with self.kb.lock:
            self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            self.picker_pose_publisher = rospy.Publisher('/picker_mover', String, queue_size=10)
            # self.unpause()
            self.me = me
            self.robco = RobotControl(self.me)
            self.world_state = WorldState(self.me, self.kb)
            rospy.loginfo("BDI: Initialized World State")
            self.last_behaviours = {}
            self.robot_track = []
            self.people_tracks = {}
            self.directions = {}
            self.latest_directions = {}
            self.goals = [DeliverGoal, ExchangeGoal, EvadeGoal]
            # self.goals = [DeliverGoal, ExchangeGoal]
            # self.goals = [DeliverGoal]
            # self.goals = [ExchangeGoal]
            # self.goals = [EvadeGoal]
            # self.goals = []
            self.intentions = []
            self.latest_robot_msg = None
            self.latest_people_msgs = {}
            self.latest_people_nodes = {}
            self.qsrlib = QSRlib()
            self.locator = TopologicalNavLoc()
            self.links = {}
            self.node_positions = {}
            self.route_search = TopologicalRouteSearch(self.locator.tmap)
            rospy.loginfo("BDI: Adding Waypoints")
            for node in self.locator.tmap.nodes:
                # if node in ["WayPoint103", "WayPoint104", "WayPoint105", "WayPoint106", "WayPoint107", "WayPoint108", "WayPoint109"]:
                self.world_state.add_thing(node.name, "place")
                self.node_positions[node.name] = node.pose
                for edge in node.edges:
                    if node.name != edge.node:
                        self.links["_".join([node.name, edge.node])] = 0
                        self.world_state.add_place_link(node.name, edge.node)
            for link in self.links.keys():
                nodes = link.split("_")
                self.links[link] = get_distance(self.node_positions[nodes[0]], self.node_positions[nodes[1]])
            self.setup_experiment()
        rospy.loginfo("BDI: Initialized BDI System")


    def setup_experiment(self):
        rospy.loginfo("BDI: Adding Me")
        self.world_state.add_thing(self.me.capitalize(), "robot")
        rospy.loginfo("BDI: Adding Pickers")
        for picker in ["Picker01", "Picker02"]:
            self.world_state.add_thing(picker, "human")
        rospy.loginfo("BDI: Setting Target Picker State")
        picker = TARGET_PICKER
        if CALLED_ROBOT:
            self.world_state.called_robot(self.kb.concept(picker)).tv = self.kb.TRUE
        else:
            self.world_state.not_called_robot(self.kb.concept(picker)).tv = self.kb.TRUE
        if SEEN_PICKING:
            self.world_state.seen_picking(self.kb.concept(picker)).tv = self.kb.TRUE
        else:
            self.world_state.not_seen_picking(self.kb.concept(picker)).tv = self.kb.TRUE
        # self.robco.move_to(INITIAL_WAYPOINT)


    def generate_options(self):
        rospy.logdebug("BDI: Generating Behaviour Options")
        desires = []
        for goal in self.goals:
            args_list = goal.find_instances(self.world_state)
            for args in args_list:
                if len(args) > 0:
                    try:
                        desires.append(goal(self.world_state, self.robco, args))
                    except WrongParameterException as err:
                        rospy.logwarn("BDI: {}".format(err))
                        # pass
        for intention in self.intentions:
            if intention in desires and intention.is_achieved(self.world_state):
                desires.remove(intention)
        if len(desires) > 0:
            rospy.logdebug("BDI: Desires: {}".format(desires))
        return desires


    def filter(self, desires):
        rospy.logdebug("BDI: Filtering Desires")
        intentions = self.intentions or OrderedConsistentSet()
        for desire in desires:
            gain = desire.get_gain()
            cost = desire.get_cost()
            if gain > MIN_GAIN and cost < MAX_COST:
                intentions.append(desire)
        if len(self.intentions) > 0:
            rospy.logdebug("BDI: Intentions: {}".format(self.intentions))
        return intentions


    def perform_action(self):
        rospy.logdebug("BDI: Choosing Next Action")
        chosen_intention = None
        min_cost = 999999
        for intention in self.intentions:
            action = intention.get_next_action()
            with suppress(AttributeError):
                cost = action.get_cost()
                rospy.logdebug("BDI: Action option {:} with cost {:.2f}".format(action, cost))
                if cost < min_cost:
                    chosen_intention = intention
                    min_cost  = cost
        if not chosen_intention is None:
            rospy.loginfo("BDI: Following {:}".format(chosen_intention))
            chosen_intention.perform_action()


    def loop(self):
        # self.pause()
        # rospy.logwarn("BDI: ----- Started Loop -----")
        start_time = time.time()
        loop_start = start_time
        self.update_beliefs()
        rospy.logdebug("BDI: --  updated beliefs   -- {:.4f}".format(time.time() - start_time))
        start_time = time.time()
        desires = self.generate_options()
        rospy.logdebug("BDI: -- generated desires  -- {:.4f}".format(time.time() - start_time))
        start_time = time.time()
        self.intentions = self.filter(desires)
        rospy.logdebug("BDI: -- filtered desires   -- {:.4f}".format(time.time() - start_time))
        start_time = time.time()
        self.perform_action()
        rospy.logdebug("BDI: -- performed action   -- {:.4f}".format(time.time() - start_time))
        start_time = time.time()
        self.clean_intentions()
        rospy.logdebug("BDI: -- cleaned intentions -- {:.4f}".format(time.time() - start_time))
        duration = time.time() - loop_start
        # if duration > 0.01:
        # rospy.loginfo("BDI: -----     Loop     ----- {:.4f}".format(duration))
        # self.unpause()


    def clean_intentions(self):
        index = 0
        while index < len(self.intentions):
            if self.intentions[index].is_achieved(self.world_state):
                rospy.loginfo("BDI: Finished following {}".format(self.intentions[index]))
                # self.robco.move_to(INITIAL_WAYPOINT)
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
            position = Object_State(name=person, timestamp=timestamp, x=msg.pose.position.x, y=msg.pose.position.y, xsize=PICKER_WIDTH, ysize=PICKER_LENGTH, object_type="Person")
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
            try:
                self.people_tracks[person].append(position)
            except:
                self.people_tracks[person] = [position]

            world.add_object_state_series(self.people_tracks[person])
            added_picker = True
        if added_picker and not self.latest_robot_msg is None:
            self.robot_track.append(Object_State(name=self.me.capitalize(), timestamp=timestamp, x=self.latest_robot_msg.position.x, y=self.latest_robot_msg.position.y, xsize=ROBOT_WIDTH, ysize=ROBOT_LENGTH))
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
                    picker = k.split(",")[1]
                    direction = v.qsr.get("qtcbs").split(",")[1]
                    self.directions[picker] = direction
                    update_direction = False
                    try:
                        update_direction = self.latest_directions[picker] != direction
                    except:
                        update_direction = True
                        self.latest_directions[picker] = direction
                    if update_direction:
                        if direction == "+":
                            self.world_state.leaving(self.kb.concept(picker)).tv = self.kb.TRUE
                            rospy.logwarn("BDI: {} is leaving".format(picker))
                        elif direction == "-":
                            self.world_state.approaching(self.kb.concept(picker)).tv = self.kb.TRUE
                            rospy.logwarn("BDI: {} is approaching".format(picker))
                        else:
                            self.world_state.standing(self.kb.concept(picker)).tv = self.kb.TRUE
                            rospy.logwarn("BDI: {} is standing".format(picker))
                        self.latest_directions[picker] = direction
                    distance = get_distance(self.latest_robot_msg, self.latest_people_msgs[picker].pose)
                    # stop if we're to close to a picker
                    rospy.logwarn("here")
                    if distance < MINIMUM_DISTANCE + 0.5 * (ROBOT_LENGTH + PICKER_LENGTH):
                        if not self.too_close:
                            self.too_close = True
                            rospy.loginfo("BDI: Robot has met picker. Distance: {}".format(distance - 0.5 * (ROBOT_LENGTH + PICKER_LENGTH)))
                            self.picker_pose_publisher.publish("at robot")
                            self.robco.cancel_movement()
                        else:
                            rospy.logwarn("collision")
                    elif self.too_close:
                        rospy.loginfo("BDI: Robot has left picker. Distance: {}".format(distance - 0.5 * (ROBOT_LENGTH + PICKER_LENGTH)))
                        self.too_close = False
            except (KeyError, AttributeError) as err:
                rospy.logwarn("BDI: {}".format(err))
            except (ValueError, IndexError) as err:
                # pass
                rospy.logwarn("BDI: {}".format(err))


    def write(self):
        self.world_state.write()


    def save(self):
        self.world_state.save()


    def add_goal(self, goal):
        self.goals.append(goal)

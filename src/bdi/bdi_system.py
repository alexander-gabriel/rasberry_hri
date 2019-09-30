import time
import rospy

from pyparsing import ParseException
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State,World_Trace

from topological_navigation.tmap_utils import get_distance

from rasberry_people_perception.topological_localiser import TopologicalNavLoc
from topological_navigation.route_search import TopologicalRouteSearch
from utils import OrderedConsistentSet, suppress
from world_state import WorldState
from goals import MoveGoal, GiveCrateGoal, DeliverGoal

MAX_COST = 1.0
MIN_GAIN = 1.0
TRUTH_THRESHOLD = 0.5
TARGET = "?target"
ME = "?me"
INF = float('inf')
SPEED = 0.55



class BDISystem:


    def __init__(self, me):
        rospy.loginfo("BDI: Initializing BDI System")
        self.me = me
        self.world_state = WorldState()
        self.robot_track = []
        self.people_tracks = {}
        self.goals = [DeliverGoal]
        self.intentions = []
        self.robot_position = None
        self.latest_people_msgs = {}
        self.latest_people_nodes = {}
        self.qsrlib = QSRlib()
        self.options = sorted(self.qsrlib.qsrs_registry.keys())
        self.which_qsr = "rcc8"#"tpcc"
        self.locator = TopologicalNavLoc()
        time.sleep(1)
        self.links = {}
        self.node_positions = {}
        self.route_search = TopologicalRouteSearch(self.locator.tmap)
        for node in self.locator.tmap.nodes:
            self.world_state.add_belief("is_a({:},Place)".format(node.name))
            self.node_positions[node.name] = node.pose
            for edge in node.edges:
                self.links["_".join([node.name, edge.node])] = 0
                self.world_state.add_belief("leads_to({:},{:})".format(node.name, edge.node))
        for link in self.links.keys():
            nodes = link.split("_")
            self.links[link] = get_distance(self.node_positions[nodes[0]], self.node_positions[nodes[1]])
        self.world_state.add_belief("is_a(picker01,Human)")
        self.world_state.add_belief("has_role(picker01,Picker)")
        self.world_state.add_belief("has_requested_crate(picker01)")
        self.world_state.add_belief("has_crate({:})".format(self.me))
        rospy.loginfo("BDI: Initialized BDI System")
        time.sleep(3)


    def generate_options(self):
        rospy.loginfo("Generating Options")
        desires = []
        for goal in self.goals:
            args_list = goal.find_instances(self.world_state)
            rospy.loginfo(args_list)
            for args in args_list:
                rospy.loginfo(args)
                desires.append(goal.instantiate(world_state, args))
        for intention in self.intentions:
            if intention in desires and intention.is_achieved():
                desires.remove(intention)
        return desires


    def filter(self, desires):
        rospy.loginfo("BDI: Filtering desires")
        intentions = []
        for desire in desires:
            gain = desire.get_gain(desire[1])
            cost = desire.get_cost(desire[1])
            if gain > MIN_GAIN and cost < MAX_COST:
                intentions.append(desire)
        return intentions


    def perform_action(self):
        rospy.loginfo("BDI: Acting")
        chosen_intention = None
        min_cost = 999999
        for intention in self.intentions:
            action = intention.get_next_action()
            cost = action.get_cost()
            if cost < min_cost:
                chosen_intention = intention
                min_cost  = cost
        if not chosen_intention is None:
            chosen_intention.perform_action(self.world_state)


    def loop(self):
        self.update_beliefs()
        desires = self.generate_options()
        self.intentions = self.filter(desires)
        self.perform_action()


    def update_beliefs(self):
        rospy.loginfo("BDI: Updating beliefs")
        world = World_Trace()
        if not self.robot_position is None:
            self.robot_track.append(self.robot_position)
            world.add_object_state_series(self.robot_track)
        for person, msg in self.latest_people_msgs.items():
            position = Object_State(name=person, timestamp=msg.header.stamp.to_sec(), x=msg.pose.position.x, y=msg.pose.position.y, width=0.6, length=0.4)
            (current_node, closest_node) = self.locator.localise_pose(msg)
            with suppress(KeyError):
                latest_node = self.latest_people_nodes[person]
                try:
                    self.world_state.abandon_belief("{:}({:},{:})".format(latest_node[0], person, latest_node[1]))
                    rospy.logdebug("retracted: {:}({:},{:})".format(latest_node[0], person, latest_node[1]))
                except ParseException:
                    rospy.loginfo("error!: {:}({:},{:})".format(latest_node[0], person, latest_node[1]))

            if current_node != "none":
                self.latest_people_nodes[person] = ("is_at", current_node)
                self.world_state.add_belief("is_at({:},{:})".format(person, current_node))
                rospy.logdebug("added is_at({:},{:})".format(person, current_node))
            else:
                self.latest_people_nodes[person] = ("is_near", closest_node)
                self.world_state.add_belief("is_near({:},{:})".format(person, closest_node))
                rospy.logdebug("added is_near({:},{:})".format(person, closest_node))
            if not person in self.people_tracks:
                self.people_tracks[person] = [position]
            else:
                self.people_tracks[person].append(position)
            world.add_object_state_series(self.people_tracks[person])
        qsrlib_request_message = QSRlib_Request_Message(which_qsr="tpcc", input_data=world)
        # request your QSRs
        qsrlib_response_message = self.qsrlib.request_qsrs(req_msg=qsrlib_request_message)
        for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
            foo = str(t) + ": "
            for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                            qsrlib_response_message.qsrs.trace[t].qsrs.values()):
                foo += str(k) + ":" + str(v.qsr) + "; "
            rospy.loginfo(foo)


        # pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)
        try:
            t = qsrlib_response_message.qsrs.get_sorted_timestamps()[-1]
            for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(), qsrlib_response_message.qsrs.trace[t].qsrs.values()):
                print(k)
                print(v)
        except IndexError:
            pass
        #TODO: parse result and update beliefs


    def write(self):
        self.world_state.write()


    def save(self):
        self.world_state.save()


    def add_goal(self, goal):
        self.goals.append(goal)

import time
import rospy

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

from topological_navigation.tmap_utils import get_distance

from rasberry_people_perception.topological_localiser import TopologicalNavLoc
from topological_navigation.route_search import TopologicalRouteSearch
from utils import OrderedConsistentSet, suppress
from world_state import WorldState
from goals import ExchangeGoal, DeliverGoal, EvadeGoal

MAX_COST = 10
MIN_GAIN = 10
TRUTH_THRESHOLD = 0.5
TARGET = "target"
ME = "me"
INF = float('inf')
SPEED = 0.55

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


    def __init__(self, me, robco):
        rospy.loginfo("BDI: Initializing BDI System")
        self.me = me
        self.robco = robco
        self.world_state = WorldState(self.me)
        rospy.loginfo("BDI: Initialized World State")
        self.robot_track = []
        self.people_tracks = {}
        self.goals = [ExchangeGoal, DeliverGoal, EvadeGoal]
        self.intentions = []
        self.robot_position = None
        self.latest_people_msgs = {}
        self.latest_people_nodes = {}
        self.qsrlib = QSRlib()
        self.options = sorted(self.qsrlib.qsrs_registry.keys())
        self.which_qsr = "tpcc"#"tpcc"
        # self.locator = TopologicalNavLoc()
        self.links = {}
        self.node_positions = {}
        # self.route_search = TopologicalRouteSearch(self.locator.tmap)
        rospy.loginfo("BDI: Adding Example Waypoints")
        for waypoint in [("WayPoint133", "WayPoint102"), ("WayPoint102", "WayPoint103"), ("WayPoint103", "WayPoint104"), ("WayPoint104", "WayPoint105"), ("WayPoint105", "WayPoint106")]:
            self.world_state.add_thing(waypoint[0], "place")
            self.world_state.add_thing(waypoint[1], "place")
            self.world_state.add_place_link(waypoint[0], waypoint[1])
        rospy.loginfo("BDI: Adding Real Waypoints")
        # for node in self.locator.tmap.nodes:
        #     self.world_state.add_thing(node.name, "place")
        #     self.node_positions[node.name] = node.pose
        #     for edge in node.edges:
        #         self.links["_".join([node.name, edge.node])] = 0
        #         self.world_state.add_place_link(node.name, edge.node, TRUE)
        # for link in self.links.keys():
        #     nodes = link.split("_")
        #     self.links[link] = get_distance(self.node_positions[nodes[0]], self.node_positions[nodes[1]])
        rospy.loginfo("BDI: Adding Pickers")
        for picker in ["Picker01", "Picker02"]:
            self.world_state.add_thing(picker, "human")
        rospy.loginfo("BDI: Initialized BDI System")


    def generate_options(self):
        rospy.logdebug("Generating Behaviour Options")
        desires = []
        for goal in self.goals:
            args_list = goal.find_instances(self.world_state)
            for args in args_list:
                desires.append(goal(self.world_state, self.robco, args))
        for intention in self.intentions:
            if intention in desires and intention.is_achieved():
                desires.remove(intention)
        return desires


    def filter(self, desires):
        rospy.logdebug("BDI: Filtering Desires")
        intentions = self.intentions or []
        for desire in desires:
            gain = desire.get_gain()
            cost = desire.get_cost()
            rospy.logdebug("BDI: Got desire {:} with gain: {:d} and cost {:d}".format(desire.__class__.__name__, gain, cost))
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
                if cost < min_cost:
                    chosen_intention = intention
                    min_cost  = cost
        if not chosen_intention is None:
            rospy.loginfo("BDI: Following goal: {:}".format(chosen_intention.__class__.__name__))
            chosen_intention.perform_action()


    def loop(self):
        rospy.logdebug("----- Started Loop -----")
        self.update_beliefs()
        desires = self.generate_options()
        self.intentions = self.filter(desires)
        self.perform_action()
        rospy.logdebug("----- Ended Loop -----")


    def update_beliefs(self):
        rospy.logdebug("BDI: Updating Beliefs")
        world = World_Trace()
        if not self.robot_position is None:
            self.robot_track.append(self.robot_position)
            world.add_object_state_series(self.robot_track)
        for person, msg in self.latest_people_msgs.items():
            position = Object_State(name=person, timestamp=msg.header.stamp.to_sec(), x=msg.pose.position.x, y=msg.pose.position.y, width=0.6, length=0.4)
            (current_node, closest_node) = self.locator.localise_pose(msg)
            with suppress(KeyError):
                latest_node = self.latest_people_nodes[person]
                if current_node != "none":
                    pass
                    # self.world_state.abandon_belief("{:}({:},{:})".format(latest_node[0], person, latest_node[1]))
            if current_node != "none":
                self.latest_people_nodes[person] = ("is_at", current_node)
                self.world_state.update_position(person, current_node)

            # else:
            #     self.latest_people_nodes[person] = ("is_near", closest_node)
            #     self.world_state.add_belief("is_near({:},{:})".format(person, closest_node))
            #     rospy.logdebug("added is_near({:},{:})".format(person, closest_node))
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


    def write(self):
        self.world_state.write()


    def save(self):
        self.world_state.save()


    def add_goal(self, goal):
        self.goals.append(goal)

import time
import rospy

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

from opencog.utilities import initialize_opencog
from opencog.type_constructors import (
    set_type_ctor_atomspace,
    FloatValue,
    ConceptNode,
)

from std_msgs.msg import String
from std_srvs.srv import Empty
from topological_navigation.route_search import TopologicalRouteSearch

from rasberry_people_perception.topological_localiser import TopologicalNavLoc

from parameters import (
    NO_BERRY_PLACES,
    ROBOT_WIDTH,
    ROBOT_LENGTH,
    PICKER_WIDTH,
    PICKER_LENGTH,
    TARGET_PICKER,
    CALLED_ROBOT,
    SEEN_PICKING,
    HAS_CRATE,
    CRATE_FULL,
    MIN_GAIN,
    MAX_COST,
    MINIMUM_DISTANCE,
    FULL_CRATE_COUNT,
    EMPTY_CRATE_COUNT,
    DISMISSED_ROBOT,
)

from utils import OrderedConsistentSet, suppress
from bdi.goals import (
    ExchangeGoal,
    DeliverGoal,
    EvadeGoal,
    DepositGoal,
    WrongParameterException,
    ApproachGoal,
    CloseApproachGoal,
    LeaveGoal,
    WaitForGoal,
    WaitGoal,
)
from bdi.robot_control import RobotControl
from bdi.world_state import WorldState
from bdi.intention_recognition import IntentionRecognition


class BDISystem:
    def __init__(self, me, kb):
        rospy.loginfo("BDI: Initializing BDI System")
        self.durations = []
        self.kb = kb
        initialize_opencog(self.kb.atomspace)
        set_type_ctor_atomspace(self.kb.atomspace)
        self.last_intention = None
        self.last_distance = 0
        with self.kb.lock:
            self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            self.picker_pose_publisher = rospy.Publisher(
                "/picker_mover", String, queue_size=10
            )
            # self.unpause()
            self.robco = RobotControl(me)
            self.world_state = WorldState(self.kb, me)
            self.intention_recognition = IntentionRecognition(self.world_state)
            rospy.loginfo("BDI: Initialized World State")
            self.last_behaviours = {}
            self.robot_track = []
            self.people_tracks = {}
            self.directions = {}
            self.latest_directions = {}
            self.desires = []
            self.desires.append(WaitGoal)
            self.desires.append(LeaveGoal)
            self.desires.append(ApproachGoal)
            self.desires.append(CloseApproachGoal)
            self.desires.append(DeliverGoal)
            self.desires.append(ExchangeGoal)
            self.desires.append(EvadeGoal)
            self.desires.append(DepositGoal)
            self.intentions = []
            self.latest_robot_msg = None
            self.latest_people_msgs = {}
            self.latest_people_nodes = {}
            self.latest_distances = {}
            self.qsrlib = QSRlib()
            self.locator = TopologicalNavLoc()
            # self.links = {}
            # self.node_positions = {}
            self.route_search = TopologicalRouteSearch(self.locator.tmap)
            rospy.loginfo("BDI: Adding Waypoints")
            for node in self.locator.tmap.nodes:
                # if node in ["WayPoint103", "WayPoint104", "WayPoint105",
                #             "WayPoint106", "WayPoint107", "WayPoint108",
                #             "WayPoint109"]:
                place = self.world_state.add_place(
                    node.name, node.pose.position.x, node.pose.position.y
                )
                if place.name in NO_BERRY_PLACES:
                    self.world_state.set_berry_state(place, 0)
                    rospy.loginfo("BDI: No Berries at {}".format(place.name))
                else:
                    self.world_state.set_berry_state(place, 1)
                # self.node_positions[node.name] = node.pose
                for edge in node.edges:
                    if node.name != edge.node:
                        # self.links["_".join([node.name, edge.node])] = 0
                        self.world_state.add_place_link(node.name, edge.node)
            # for link in self.links.keys():
            #     nodes = link.split("_")
            # self.links[link] = self.world_state.get_distance(nodes[0],
            #                                                  nodes[1])
            self._setup_experiment(me)
        rospy.loginfo("BDI: Initialized BDI System")

    def _setup_experiment(self, me):
        rospy.loginfo("BDI: Adding Me")
        self.me = self.world_state.add_thing(me.capitalize(), "robot")
        self.world_state.set_size(self.me, ROBOT_WIDTH, ROBOT_LENGTH)
        rospy.loginfo("BDI: Adding Pickers")
        picker = self.world_state.add_thing(TARGET_PICKER, "human")
        picker.set_value(self.world_state._timeout, FloatValue(-1))
        self.world_state.set_size(picker, PICKER_WIDTH, PICKER_LENGTH)
        rospy.loginfo("BDI: Setting Target Picker State")
        if CALLED_ROBOT:
            self.world_state.called_robot(picker).tv = self.kb.TRUE
        else:
            self.world_state.not_called_robot(picker).tv = self.kb.TRUE
        if DISMISSED_ROBOT:
            self.world_state.dismissed_robot(picker).tv = self.kb.TRUE
        else:
            self.world_state.not_dismissed_robot(picker).tv = self.kb.TRUE
        if SEEN_PICKING:
            self.world_state.seen_picking(picker).tv = self.kb.TRUE
        else:
            self.world_state.not_seen_picking(picker).tv = self.kb.TRUE
        if HAS_CRATE:
            self.world_state.has_crate(picker).tv = self.kb.TRUE
        else:
            self.world_state.not_has_crate(picker).tv = self.kb.TRUE
        if CRATE_FULL:
            self.world_state.crate_full(picker).tv = self.kb.TRUE
        else:
            self.world_state.not_crate_full(picker).tv = self.kb.TRUE
        self.me.set_value(
            self.world_state._empty_crate_count, FloatValue(EMPTY_CRATE_COUNT),
        )
        # rospy.logerr("{:} has {:} empty crates".format(
        #     self.me.name,
        #     self.me.get_value(self.world_state._empty_crate_count)))
        self.me.set_value(
            self.world_state._full_crate_count, FloatValue(FULL_CRATE_COUNT),
        )
        # self.robco.move_to(INITIAL_WAYPOINT)

    def _generate_intention_candidates(self):
        rospy.logdebug("BDI: Generating Behaviour Options")
        intention_candidates = []
        for goal in self.desires:
            if goal not in self.intentions:
                args_list = goal.find_instances(self.world_state)
                for args in args_list:
                    if len(args) > 0:
                        try:
                            intention_candidates.append(
                                goal(self.world_state, self.robco, args)
                            )
                        except WrongParameterException as err:
                            # rospy.logerr(
                            #     "BDI: Couldn't add intention {:} because of error {:}".format(
                            #         goal, err
                            #     )
                            # )
                            pass
        for intention in self.intentions:
            if intention in intention_candidates and intention.is_achieved(
                self.world_state
            ):
                intention_candidates.remove(intention)
        if len(intention_candidates) > 0:
            rospy.logdebug("BDI: Desires: {}".format(intention_candidates))
        return intention_candidates

    def _filter_intention_candidates(self, intention_candidates):
        rospy.logdebug("BDI: Filtering Desires")
        intentions = self.intentions or OrderedConsistentSet()
        for desire in intention_candidates:
            gain = desire.get_gain()
            cost = desire.get_cost()
            if gain >= MIN_GAIN and cost <= MAX_COST:
                intentions.append(desire)
            else:
                rospy.logwarn(
                    (
                        "BDI: 149 - Desire {} is outside bounds "
                        "of gain >{} and cost <{}"
                    ).format(desire, MIN_GAIN, MAX_COST)
                )
        if len(self.intentions) > 0:
            rospy.logdebug("BDI: Intentions: {}".format(self.intentions))
        return intentions

    def _perform_action(self):
        rospy.logdebug("BDI: Choosing Next Action")
        chosen_intention = None
        min_cost = 999999
        for intention in self.intentions:
            action = intention.get_next_action()
            with suppress(AttributeError):
                cost = action.get_cost()
                rospy.logdebug(
                    ("BDI: Action option {:} " "with cost {:.2f}").format(
                        action, cost
                    )
                )
                if cost < min_cost:
                    chosen_intention = intention
                    min_cost = cost
        if chosen_intention is not None:
            if chosen_intention != self.last_intention:
                rospy.loginfo("BDI: Following {:}".format(chosen_intention))
                self.last_intention = chosen_intention
            chosen_intention.perform_action()

    def loop(self):
        # self.pause()
        # rospy.logwarn("BDI: ----- Started Loop -----")
        start_time = time.time()
        # loop_start = start_time
        self._update_beliefs()
        rospy.logdebug(
            "BDI: --  updated beliefs   -- {:.4f}".format(
                time.time() - start_time
            )
        )
        start_time = time.time()
        intention_candidates = self._generate_intention_candidates()
        rospy.logdebug(
            "BDI: -- generated desires  -- {:.4f}".format(
                time.time() - start_time
            )
        )
        start_time = time.time()
        self.intentions = self._filter_intention_candidates(
            intention_candidates
        )
        rospy.logdebug(
            "BDI: -- filtered desires   -- {:.4f}".format(
                time.time() - start_time
            )
        )
        start_time = time.time()
        self._perform_action()
        rospy.logdebug(
            "BDI: -- performed action   -- {:.4f}".format(
                time.time() - start_time
            )
        )
        start_time = time.time()
        self._clean_intentions()
        rospy.logdebug(
            "BDI: -- cleaned intentions -- {:.4f}".format(
                time.time() - start_time
            )
        )
        # duration = time.time() - loop_start
        # if duration > 0.01:
        # rospy.loginfo("BDI: -----     Loop     ----- {:.4f}"
        #               .format(duration))
        # self.unpause()

    def _clean_intentions(self):
        index = 0
        while index < len(self.intentions):
            if self.intentions[index].is_achieved(self.world_state):
                rospy.loginfo(
                    "BDI: Finished following {}".format(self.intentions[index])
                )
                # self.robco.move_to(INITIAL_WAYPOINT)
                del self.intentions[index]
            else:
                index += 1

    def _calculate_directions(self, world, pairs):
        dynamic_args = {
            "qtcbs": {
                "quantisation_factor": 0.1,
                "validate": True,
                "no_collapse": True,
                "qsrs_for": pairs,
            }
        }
        qsrlib_request_message = QSRlib_Request_Message(
            which_qsr="qtcbs", input_data=world, dynamic_args=dynamic_args
        )
        try:
            qsrlib_response_message = self.qsrlib.request_qsrs(
                qsrlib_request_message
            )
            t = qsrlib_response_message.qsrs.get_sorted_timestamps()[-1]
            for k, v in qsrlib_response_message.qsrs.trace[t].qsrs.items():
                picker = ConceptNode(k.split(",")[1])
                direction = v.qsr.get("qtcbs").split(",")[1]
                self.directions[picker.name] = direction
                update_direction = False
                try:
                    update_direction = (
                        self.latest_directions[picker.name] != direction
                    )
                except Exception:
                    update_direction = True
                    self.latest_directions[picker.name] = direction
                if update_direction:
                    if direction == "+":
                        self.world_state.leaving(picker).tv = self.kb.TRUE
                        rospy.logwarn("BDI: {} is leaving".format(picker.name))
                    elif direction == "-":
                        self.world_state.approaching(picker).tv = self.kb.TRUE
                        rospy.logwarn(
                            "BDI: {} is approaching".format(picker.name)
                        )
                    else:
                        if (
                            self.world_state.approaching(picker).tv
                            == self.kb.TRUE
                        ):
                            self.world_state.set_latest_distance(
                                picker, self.latest_distances[picker.name]
                            )
                        self.world_state.standing(picker).tv = self.kb.TRUE
                        rospy.logwarn(
                            "BDI: {} is standing".format(picker.name)
                        )
                    self.latest_directions[picker.name] = direction
        # except (KeyError, AttributeError) as err:
        # rospy.logwarn("BDI: 242 - Key or Attribute error: {}"
        #               .format(err))
        except (ValueError, IndexError) as err:
            rospy.logwarn("BDI: 244 - Value or Index error: {}".format(err))

    # TODO: re-integrate direction indicator

    def _react_to_distance_events(self, person):
        try:
            distance = self.world_state.get_distance(self.me, person)
            self.latest_distances[person.name] = distance
            minimum_distance = max(
                MINIMUM_DISTANCE, self.world_state.get_optimum_distance(person)
            )

            if distance < minimum_distance:
                if not self.world_state.too_close:
                    rospy.logwarn(
                        ("BDI: Robot has met picker. " "Distance: {}").format(
                            distance
                        )
                    )
                    self.world_state.too_close = True
                    self.picker_pose_publisher.publish("at robot")
                    self.robco.cancel_movement()
                elif distance < 0.2:
                    rospy.logwarn(
                        ("BDI: Picker is too close. " "Distance: {}").format(
                            distance
                        )
                    )
                # elif abs(distance - self.last_distance) < 0.04:
            elif self.world_state.too_close:
                rospy.loginfo(
                    ("BDI: Robot has left picker. " "Distance: {}").format(
                        distance
                    )
                )
                self.world_state.too_close = False
            # self.last_distance = distance
        except Exception as err:
            rospy.logerr(
                "BDI: 268 - Couldn't react to distance events. Error: {:}".format(
                    err
                )
            )
            raise err

    def _update_picker_node(self, person, msg):
        try:
            (current_node, closest_node) = self.locator.localise_pose(msg)
            # rospy.logwarn("\ncurrent: {:}\nclosest: {:}".format(current_node, closest_node))
            if current_node == "WayPoint104":
                self.kb.debug += 1
            if current_node != "none":
                latest_node = None
                with suppress(KeyError):
                    latest_node = self.latest_people_nodes[person.name][1]
                self.latest_people_nodes[person.name] = ("is_at", current_node)
                if current_node != latest_node:
                    self.world_state.update_position(
                        person, ConceptNode(current_node)
                    )
            elif closest_node is None:
                rospy.logwarn(
                    ("BDI: We have no idea " "where {} is currently").format(
                        person.name
                    )
                )
            else:  # we know the closest but not the current node
                pass
                # self.latest_people_nodes[person] = ("is_near", closest_node)
                # self.world_state.add_belief("is_near({:},{:})"
                #                             .format(person, closest_node))
                # rospy.logdebug("added is_near({:},{:})"
                #                .format(person, closest_node))
        except TypeError as err:
            rospy.logerr(
                ("BDI - Couldn't update picker node. " "Error: {:}").format(
                    err
                )
            )

    def _handle_position_msgs(self):
        """Abstracts received position messages to the Knowledge Base"""
        timestamp = rospy.get_time()
        world = World_Trace()
        pairs = []
        # robot positions
        if self.latest_robot_msg is not None:
            self.world_state.set_position(
                self.me,
                self.latest_robot_msg.position.x,
                self.latest_robot_msg.position.y,
                timestamp,
            )
        # people positions
        for person, msg in self.latest_people_msgs.items():
            person = ConceptNode(person)
            self.world_state.set_position(
                person, msg.pose.position.x, msg.pose.position.y, timestamp
            )
            self._react_to_distance_events(person)
            self._update_picker_node(person, msg)

            self.world_state.set_position(
                person, msg.pose.position.x, msg.pose.position.y, timestamp
            )
            pairs.append((self.me.name, person.name))
            position = Object_State(
                name=person.name,
                timestamp=timestamp,
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                xsize=PICKER_WIDTH,
                ysize=PICKER_LENGTH,
                object_type="Person",
            )
            try:
                self.people_tracks[person.name].append(position)
            except Exception:
                self.people_tracks[person.name] = [position]
            world.add_object_state_series(self.people_tracks[person.name])

        if pairs and self.latest_robot_msg is not None:
            self.robot_track.append(
                Object_State(
                    name=self.me.name,
                    timestamp=timestamp,
                    x=self.latest_robot_msg.position.x,
                    y=self.latest_robot_msg.position.y,
                    xsize=ROBOT_WIDTH,
                    ysize=ROBOT_LENGTH,
                )
            )
            world.add_object_state_series(self.robot_track)
            self._calculate_directions(world, pairs)

    def _update_beliefs(self):
        rospy.logdebug("BDI: Updating Beliefs")
        self._handle_position_msgs()
        self.intention_recognition.run_untargeted()

        # distance = self.ws.get_distance(
        #     self.latest_robot_msg,
        #     (self.latest_people_msgs[picker].pose)
        #     - 0.5 * (ROBOT_LENGTH + PICKER_LENGTH))
        # # stop if we're to close to a picker
        # minimum_distance = MINIMUM_DISTANCE
        # # if self.world_state.moving:
        # #     minimum_distance +=
        # if distance < minimum_distance:
        #     if not self.too_close:
        #         rospy.logwarn("BDI: Robot has met picker. Distance: {}"
        #                       .format(distance))
        #         self.too_close = True
        #         self.picker_pose_publisher.publish("at robot")
        #         self.robco.cancel_movement()
        #     elif distance < 0.2:
        #         rospy.logwarn("BDI: Picker is too close. Distance: {}"
        #                       .format(distance))
        #     # elif abs(distance - self.last_distance) < 0.04:
        #
        # elif self.too_close:
        #     rospy.loginfo("BDI: Robot has left picker. Distance: {}"
        #                   .format(distance))
        #     self.too_close = False
        # self.last_distance = distance

    def add_goal(self, goal):
        """Adds a goal to the BDI system"""
        self.desires.append(goal)

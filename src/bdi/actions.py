from time import time

import rospy

from opencog.type_constructors import (
    ConceptNode,
    TruthValue,
    FloatValue,
    NumberNode,
)

from common.parameters import (
    ME,
    MOVE_GAIN,
    MEAN_WAYPOINT_DISTANCE,
    ROBOT_SPEED,
    MOVETO_GAIN,
    EVADE_GAIN,
    GIVE_GAIN,
    GIVE_COST,
    CRATE_CAPACITY,
    EXCHANGE_GAIN,
    EXCHANGE_COST,
    DEPOSIT_GAIN,
    DEPOSIT_COST,
    DEPOT,
    WAIT_GAIN,
    WAIT_TIME,
    TIMEOUT_LENGTH,
    STANDBY_GAIN,
    APPROACH_GAIN,
    READY_POINT,
    MAX_COST,
    MIN_GAIN
)

from common.utils import VariableCondition as V
from common.utils import ConceptCondition as C
from common.utils import PredicateCondition as P
from common.utils import NumberCondition as N
from common.utils import db
from world_state import WorldState as ws


class Action(object):

    placeholders = []
    gain = 0

    def __init__(self, world_state, args):
        self.kb = world_state.kb
        self.ws = world_state
        self.instances = args
        rospy.logdebug(
            "{:} has args: {}".format(self.__class__.__name__, args)
        )
        self.conditions = []
        self.consequences = []
        self.cost = 0
        self.first_try = True
        self.start_time = None

        replacement_table = args
        # {key: value for key, value in zip(self.placeholders, self.instances)}

        for condition in self.condition_templates:
            new_variables = list(
                map(
                    lambda variable: variable.replace(replacement_table),
                    condition[1],
                )
            )
            self.conditions.append([condition[0], new_variables])
        for consequence in self.consequence_templates:
            new_variables = list(
                map(
                    lambda variable: variable.replace(replacement_table),
                    consequence[1],
                )
            )
            self.consequences.append([consequence[0], new_variables])

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return (
                self.instances == other.instances
                and self.conditions == other.conditions
                and self.consequences == other.consequences
            )
        return False

    def __neq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return not (
                self.instances == other.instances
                and self.conditions == other.conditions
                and self.consequences == other.consequences
            )
        return True

    def __repr__(self):
        return self.__class__.__name__

    def perform(self):
        if self.first_try:
            self.start_time = rospy.get_time()
            if type(self) != WaitAction:
                rospy.loginfo("ACT: Performing {:}".format(self))
            self.first_try = False
        # for condition in self.conditions:
        #     if not condition in self.consequences:
        #         self.ws.abandon_belief(condition)
        # for consequence in self.consequences:
        #     self.ws.add_belief(consequence)

    def get_cost(self):
        return self.cost

    def get_gain(self):
        return self.gain


class MoveAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.not_same, [V("my_position", ConceptNode),
                       V("my_destination", ConceptNode)]],
        [ws.is_a, [V("my_destination", ConceptNode), C("place")]],
        # [ws.is_target, [V("my_destination", ConceptNode)]],  # variance experiment
        # [ws.linked, [V("my_position", ConceptNode),
        #                V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]]
    ]
    placeholders = [ME, "my_position", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(MoveAction, self).__init__(world_state, args)
        self.robco = robco
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length  = self.robco.get_path_length(self.position, self.destination)
        self.gain = MOVE_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(MoveAction, self).perform()
        if not self.sent_movement_request:
            self.startint_too_close = self.ws.too_close
            # timestamp, typ, distance, x, y
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                self.__class__.__name__, "{} - {}"
                                .format(self.position, self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
            return False
        try:
            if self.robco.get_result() or (self.ws.too_close and not self.startint_too_close):
                time = rospy.get_time()
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                self.ws.moving = False
                rospy.loginfo("ACT: Reached my destination.")
                db.update_action_entry(self.start_time, float(x), float(y),
                                            time - self.start_time)
                return True
            else:
                return False
        except AttributeError:
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)


class MoveToAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        # [ws.linked, [V("my_position", ConceptNode),
        #              V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]]
    ]
    placeholders = [ME, "picker", "my_position", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(MoveToAction, self).__init__(world_state, args)
        self.robco = robco
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length  = self.robco.get_path_length(self.position, self.destination)
        self.gain = MOVETO_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(MoveToAction, self).perform()
        if self.position == self.destination:
            time = rospy.get_time()
            rospy.loginfo(
                "ACT: Reached my destination: {}".format(self.destination)
            )
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.update_action_entry(self.start_time, float(x), float(y),
                                        time - self.start_time)
            return True
        else:
            if not self.sent_movement_request:
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(x), float(y),
                                         self.__class__.__name__,
                                         "{} - {}".format(
                                            self.position,
                                            self.destination))
                self.robco.move_to(self.destination)
                self.sent_movement_request = True
                self.ws.moving = True
            try:
                result = self.robco.get_result()
                if result or self.ws.too_close:
                    self.ws.moving = False

                    rospy.loginfo(
                        "ACT: Reached my destination"
                    )
                    return True
            except Exception as err:
                rospy.logwarn("ACT: {}".format(err))
                return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:MoveTo {:}>".format(self.destination)


class EvadeAction(Action):

    condition_templates = [
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.wants_to_pass, [V("picker", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.query_not_at,
            [V("anything", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [
            ws.leads_to,
            [V("my_position", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [
            ws.leads_to,
            [V("picker_position", ConceptNode), V("my_position", ConceptNode)],
        ],
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
        [ws.wants_nothing, [V("picker", ConceptNode)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(EvadeAction, self).__init__(world_state, args)
        self.robco = robco
        self.position = args["my_position"]
        self.picker = args["picker"]
        self.destination = args["my_destination"]
        self.path_length  = self.robco.get_path_length(self.position, self.destination)
        self.gain = EVADE_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(EvadeAction, self).perform()
        if not self.sent_movement_request:
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                     self.__class__.__name__,
                                     "{} - {}".format(
                                        self.position,
                                        self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
            return False
        try:
            if self.robco.get_result():
                self.ws.moving = False
                time = rospy.get_time()
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.update_action_entry(self.start_time, float(x), float(y),
                                            time - self.start_time)
                return True
            else:
                return False
        except Exception:
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Evade {:} by moving to {:}>".format(
            self.picker, self.destination
        )


class GiveCrateAction(Action):

    # NOTE: working version with implicit intention recognition
    # condition_templates = [
    # [ws.is_a, [V("picker", ConceptNode), C("human")]],
    # [ws.is_at, [V("picker", ConceptNode), V("my_destination", ConceptNode)]],
    # [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
    # [ws.not_seen_picking, [V("picker", ConceptNode)]],
    # [ws.called_robot, [V("picker", ConceptNode)]],
    # ]

    condition_templates = [
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.wants_to_get_crate, [V("picker", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        [ws.wants_nothing, [V("picker", ConceptNode)]],
        [ws.has_crate, [V("picker", ConceptNode)]],
        [ws.dismissed_robot, [V("picker", ConceptNode)]],
        [ws.robot_remove_crate2, [C(ME), P(ws._empty_crate_count.name)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(GiveCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = GIVE_GAIN
        self.cost = GIVE_COST

    def get_cost(self):
        return self.cost

    def perform(self):
        super(GiveCrateAction, self).perform()
        rospy.sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg)
            consequence = fun(self.ws, *new_args)
            # rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
        rospy.loginfo("ACT: {} received a crate".format(self.picker))
        rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))
        me = ConceptNode(ME)
        full_count = self.ws.robot_get_crate_count(
            me, self.ws._full_crate_count)
        empty_count = self.ws.robot_get_crate_count(
            me, self.ws._empty_crate_count)
        rospy.loginfo("ACT: New robot crate state: Full/Empty {}/{}".format(full_count, empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:Give crate to {:}>".format(self.picker)


class ExchangeCrateAction(Action):

    # NOTE: working version with implicit intention recognition
    # condition_templates = [
    # [ws.is_a, [V("picker", ConceptNode), C("human")]],
    # [ws.is_at, [V("picker", ConceptNode), V("my_destination", ConceptNode)]],
    # [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
    # [ws.seen_picking, [V("picker", ConceptNode)]],
    # [ws.called_robot, [V("picker", ConceptNode)]],
    # ]

    condition_templates = [
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.robot_has_crate_capacity, [C(ME), P(ws._full_crate_count.name)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.wants_to_exchange_their_crate, [V("picker", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("my_destination", ConceptNode)],
        ],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.not_crate_full, [V("picker", ConceptNode)]],
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        [ws.wants_nothing, [V("picker", ConceptNode)]],
        [ws.robot_add_crate2, [C(ME), P(ws._full_crate_count.name)]],
        [ws.robot_remove_crate2, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.dismissed_robot, [V("picker", ConceptNode)]],
    ]
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        # [me, picker, my_destination]
        super(ExchangeCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = EXCHANGE_GAIN
        self.cost = EXCHANGE_COST

    def get_cost(self):
        return self.cost

    def perform(self):
        super(ExchangeCrateAction, self).perform()
        rospy.sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg)
            consequence = fun(self.ws, *new_args)
            # rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
        rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))
        me = ConceptNode(ME)
        full_count = self.ws.robot_get_crate_count(me, self.ws._full_crate_count)
        empty_count = self.ws.robot_get_crate_count(me, self.ws._empty_crate_count)
        rospy.loginfo("ACT: New robot crate state: Full/Empty {}/{}".format(full_count, empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:Exchange crate with {:}>".format(self.picker)


class DepositCrateAction(Action):

    condition_templates = [
        # [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        [ws.robot_not_has_crate_capacity, [C(ME), P(ws._full_crate_count.name)]],
        [ws.robot_not_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.same, [V("my_destination", ConceptNode), C(DEPOT)]],
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
    ]
    consequence_templates = [
        [
            ws.robot_set_crate_count2,
            [C(ME), P(ws._full_crate_count.name), N(0)],
        ],
        [
            ws.robot_set_crate_count2,
            [C(ME), P(ws._empty_crate_count.name), N(CRATE_CAPACITY)],
        ],
    ]
    placeholders = [ME, DEPOT, "my_destination"]

    def __init__(self, world_state, robco, args):
        super(DepositCrateAction, self).__init__(world_state, args)
        self.robco = robco
        me = ConceptNode(ME)
        self.position = args["my_destination"]
        self.gain = DEPOSIT_GAIN
        self.full_crate_count = self.ws.robot_get_crate_count(
            me, self.ws._full_crate_count)
        self.empty_crate_count = self.ws.robot_get_crate_count(
            me, self.ws._empty_crate_count)
        if self.full_crate_count == "unknown" or self.empty_crate_count == "unknown":
            raise Exception()


    def get_cost(self):
        try:
            return DEPOSIT_COST * (self.full_crate_count + CRATE_CAPACITY - self.empty_crate_count)
        except:
            return DEPOSIT_COST * CRATE_CAPACITY

    def get_gain(self):
        try:
            gain = DEPOSIT_GAIN * (
                max(self.full_crate_count, CRATE_CAPACITY - self.empty_crate_count)
                / CRATE_CAPACITY
            )
        except:
            return DEPOSIT_GAIN / 2

    def perform(self):
        super(DepositCrateAction, self).perform()
        rospy.sleep(self.get_cost())
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg)
            consequence = fun(self.ws, *new_args)
            # rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
        me = ConceptNode(ME)
        full_count = self.ws.robot_get_crate_count(
            me, self.ws._full_crate_count)
        empty_count = self.ws.robot_get_crate_count(
            me, self.ws._empty_crate_count)
        rospy.loginfo("ACT: New robot crate state: Full/Empty {}/{}".format(full_count, empty_count))
        time = rospy.get_time()
        x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
        db.add_action_entry(self.start_time, float(x), float(y),
                            self.__class__.__name__,
                            "{}".format(self.position))
        db.update_action_entry(self.start_time, float(x), float(y),
                               time - self.start_time)
        return True

    def __repr__(self):
        return "<Action:Deposit crates at {:}>".format(DEPOT)


class ApproachAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.needs_help_soon, [V("picker", ConceptNode)]],
        # [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        [ws.robot_has_crate, [C(ME), P(ws._empty_crate_count.name)]],
        [ws.robot_has_crate_capacity, [C(ME), P(ws._full_crate_count.name)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.leads_to,
            [V("my_destination", ConceptNode), V("midway_point", ConceptNode)],
        ],
        [
            ws.leads_to,
            [
                V("midway_point", ConceptNode),
                V("picker_position", ConceptNode),
            ],
        ],
        [
            ws.query_not_at,
            [V("anything", ConceptNode), V("midway_point", ConceptNode)],
        ],
        [
            ws.not_same,
            [
                V("my_destination", ConceptNode),
                V("picker_position", ConceptNode),
            ],
        ],
        [
            ws.not_same,
            [V("my_destination", ConceptNode), V("midway_point", ConceptNode)],
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("my_destination", ConceptNode)],
        ],
        # [ws.is_closer, [V("my_position", ConceptNode),
        #              V("my_destination", ConceptNode),
        #              V("picker_position", ConceptNode)]]
    ]
    consequence_templates = []
    placeholders = [
        ME,
        "my_position",
        "my_destination",
        "picker",
        "picker_position",
    ]

    def __init__(self, world_state, robco, args):
        super(ApproachAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length  = self.robco.get_path_length(self.position, self.destination)
        self.gain = APPROACH_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(ApproachAction, self).perform()
        if self.position == self.destination:
            rospy.loginfo(
                "ACT: Reached my destination: {}".format(self.destination)
            )
            return True
        else:
            if not self.sent_movement_request:
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(x), float(y),
                                    self.__class__.__name__,
                                    "{} - {}".format(self.position,
                                                     self.destination))
                self.robco.move_to(self.destination)
                self.sent_movement_request = True
                self.ws.moving = True
            try:
                # this next line will
                result = self.robco.get_result()
                if result:
                    time = rospy.get_time()
                    self.ws.moving = False
                    rospy.loginfo(
                        "ACT: Reached my destination: {}".format(
                            result.success
                        )
                    )
                    x, y, _ = self.ws.get_position(
                        ConceptNode(ME))[-1].to_list()
                    db.update_action_entry(self.start_time, float(x), float(y),
                                           time - self.start_time)
                    return True
            except Exception as err:
                # rospy.logwarn(err)
                return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Approach {:} at {:}>".format(
            self.picker, self.destination
        )


class CloseApproachAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.wants_something, [V("picker", ConceptNode)]],
        [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.leads_to,
            [V("my_destination", ConceptNode),
             V("picker_position", ConceptNode)],
        ],
        [
            ws.not_same,
            [
                V("my_destination", ConceptNode),
                V("picker_position", ConceptNode),
            ],
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("my_destination", ConceptNode)],
        ],
        # [ws.is_closer, [V("my_position", ConceptNode),
        #              V("my_destination", ConceptNode),
        #              V("picker_position", ConceptNode)]]
    ]
    consequence_templates = []
    placeholders = [
        ME,
        "my_position",
        "my_destination",
        "picker",
        "picker_position",
    ]

    def __init__(self, world_state, robco, args):
        super(CloseApproachAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = args["my_destination"]
        self.path_length  = self.robco.get_path_length(self.position, self.destination)
        self.gain = APPROACH_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(CloseApproachAction, self).perform()
        if self.position == self.destination:
            rospy.loginfo(
                "ACT: Reached my destination: {}".format(self.destination)
            )
            return True
        else:
            if not self.sent_movement_request:
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(x), float(y),
                                    self.__class__.__name__,
                                    "{} - {}".format(self.position,
                                                     self.destination))
                self.robco.move_to(self.destination)
                self.sent_movement_request = True
                self.ws.moving = True
            try:
                # this next line will
                result = self.robco.get_result()
                if result:
                    time = rospy.get_time()
                    self.ws.moving = False
                    rospy.loginfo(
                        "ACT: Reached my destination: {}".format(
                            result.success
                        )
                    )
                    x, y, _ = self.ws.get_position(
                        ConceptNode(ME))[-1].to_list()
                    db.update_action_entry(self.start_time, float(x), float(y),
                                           time - self.start_time)
                    return True
            except Exception as err:
                # rospy.logwarn(err)
                return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:CloseApproach {:} at {:}>".format(
            self.picker, self.destination
        )


class WaitAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.unknown_called_robot, [V("picker", ConceptNode)]],
        [ws.unknown_dismissed_robot, [V("picker", ConceptNode)]],
        # [ws.timeout_not_reached, [V("picker", ConceptNode)]],
        [ws.needs_help_soon, [V("picker", ConceptNode)]],
        [
            ws.is_at,
            [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.leads_to,
            [V("my_position", ConceptNode), V("midway_point", ConceptNode)],
        ],
        [
            ws.leads_to,
            [
                V("midway_point", ConceptNode),
                V("picker_position", ConceptNode),
            ],
        ],
        [
            ws.query_not_at,
            [V("anything", ConceptNode), V("midway_point", ConceptNode)],
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("picker_position", ConceptNode)],
        ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), V("midway_point", ConceptNode)],
        ],
    ]
    consequence_templates = []
    placeholders = [ME, "picker", "my_position"]

    def __init__(self, world_state, robco, args):
        super(WaitAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.gain = WAIT_GAIN
        self.timeout = -1

    def perform(self):
        super(WaitAction, self).perform()
        picker = ConceptNode(self.picker)
        current_time = time()
        if self.timeout == -1:
            self.x, self.y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            # db.add_action_entry(self.start_time, float(x), float(y),
            #                     self.__class__.__name__,
            #                     "{}".format(self.position))
            self.timeout = current_time + TIMEOUT_LENGTH
        else:
            if current_time > self.timeout:
                self.timeout = -1
                self.ws.dismissed_robot(picker).tv = self.kb.TRUE
                rospy.loginfo("ACT: {} dismissed {}".format(self.picker, ME))
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.add_action_entry(self.start_time, float(self.x), float(self.y),
                                    self.__class__.__name__,
                                    "{}".format(self.position))
                db.update_action_entry(self.start_time, float(x), float(y),
                                       current_time - self.start_time)
        return True

    def get_cost(self):
        return 0

    def __repr__(self):
        return "<Action:Wait for {:} at {:}>".format(
            self.picker, self.position
        )


class StandByAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.dismissed_robot, [V("picker", ConceptNode)]],
        # [
        #     ws.is_at,
        #     [V("picker", ConceptNode), V("picker_position", ConceptNode)],
        # ],
        # [
        #     ws.leads_to,
        #     [V("my_position", ConceptNode), V("picker_position", ConceptNode)],
        # ],
        [
            ws.not_same,
            [V("my_position", ConceptNode), C(READY_POINT)],
        ],
    ]
    consequence_templates = [[ws.unknown_dismissed_robot,
                              [V("picker", ConceptNode)]]]
    placeholders = [ME, "picker", "my_position"]

    def __init__(self, world_state, robco, args):
        super(StandByAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.position = args["my_position"]
        self.destination = READY_POINT
        self.path_length  = self.robco.get_path_length(self.position, self.destination)
        self.gain = STANDBY_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(StandByAction, self).perform()
        if not self.sent_movement_request:
            x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
            db.add_action_entry(self.start_time, float(x), float(y),
                                self.__class__.__name__, "{} - {}"
                                .format(self.position, self.destination))
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True

        try:
            result = self.robco.get_result()
            if result:
                time = rospy.get_time()
                self.ws.moving = False
                rospy.loginfo("ACT: Reached my destination: {}"
                              .format(result.success))
                x, y, _ = self.ws.get_position(ConceptNode(ME))[-1].to_list()
                db.update_action_entry(self.start_time, float(x), float(y),
                                       time - self.start_time)
                picker = ConceptNode(self.picker)
                self.ws.dismissed_robot(picker).tv = self.kb.FALSE
                rospy.loginfo("ACT: removed {}'s dismissal note for {}"
                              .format(self.picker, ME))
                return True
        except Exception as err:
            rospy.logwarn(err)
            return False

    def get_cost(self):
        return self.path_length * MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:StandBy {:} to {:}>".format(
            self.picker, self.destination
        )

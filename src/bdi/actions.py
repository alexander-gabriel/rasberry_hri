from time import sleep

import rospy

from opencog.type_constructors import ConceptNode, TruthValue, NumberNode

from parameters import ME, MOVE_GAIN, MEAN_WAYPOINT_DISTANCE, ROBOT_SPEED, \
                       MOVETO_GAIN, EVADE_GAIN, GIVE_GAIN, GIVE_COST, \
                       CRATE_CAPACITY, EXCHANGE_GAIN, EXCHANGE_COST, \
                       DEPOSIT_GAIN, DEPOSIT_COST, DEPOT
from utils import VariableCondition as V
from utils import ConceptCondition as C
from utils import PredicateCondition as P
from utils import NumberCondition as N
from world_state import WorldState as ws


class Action(object):

    placeholders = []
    gain = 0

    def __init__(self, world_state, args):
        self.kb = world_state.kb
        self.ws = world_state
        self.instances = args
        rospy.logdebug("{:} has args: {}"
                       .format(self.__class__.__name__, args))
        self.conditions = []
        self.consequences = []
        self.cost = 0
        self.first_try = True

        replacement_table = args
        # {key: value for key, value
                             # in zip(self.placeholders, self.instances)}

        for condition in self.condition_templates:
            new_variables = list(map(lambda variable:
                                     variable.replace(replacement_table),
                                     condition[1]))
            self.conditions.append([condition[0], new_variables])
        for consequence in self.consequence_templates:
            new_variables = list(map(lambda variable:
                                     variable.replace(replacement_table),
                                     consequence[1]))
            self.consequences.append([consequence[0], new_variables])

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return (self.instances == other.instances
                    and self.conditions == other.conditions
                    and self.consequences == other.consequences)
        return False

    def __neq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return not (self.instances == other.instances
                        and self.conditions == other.conditions
                        and self.consequences == other.consequences)
        return True

    def __repr__(self):
        return self.__class__.__name__

    def perform(self):
        if self.first_try:
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
        [ws.linked, [V("my_position", ConceptNode),
                       V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]]
    ]
    # placeholders must be in same order
    # as variables in "args" in the constructor
    placeholders = [ME, "my_position", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(MoveAction, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args["my_destination"]
        self.gain = MOVE_GAIN
        self.sent_movement_request = False
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?my_position", my_position).replace(C(ME), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?my_destination", my_destination).replace(C(ME), me))

    def perform(self):
        super(MoveAction, self).perform()
        if not self.sent_movement_request:
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
        try:
            success = self.robco.get_result().success
            self.ws.moving = False
            rospy.logwarn("ACT: Reached final my_destination: {}"
                          .format(success))
            return True
        except AttributeError:
            return False

    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)


class MoveToAction(Action):

    condition_templates = [
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.is_at, [V("picker", ConceptNode),
                    V("my_destination", ConceptNode)]],
        [ws.linked, [V("my_position", ConceptNode),
                     V("my_destination", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]]
    ]
    # placeholders must be in same order
    # as variables in "args" in the constructor
    placeholders = [ME, "picker", "my_position", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(MoveToAction, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args["my_destination"]
        self.gain = MOVETO_GAIN
        self.sent_movement_request = False
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?my_position", my_position).replace(C(ME), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?my_destination", my_destination).replace(C(ME), me))

    def perform(self):
        super(MoveToAction, self).perform()
        if not self.sent_movement_request:
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
        try:
            success = self.robco.get_result().success
            self.ws.moving = False
            rospy.logwarn("ACT: Reached final my_destination: {}"
                          .format(success))
            return True
        except Exception as err:
            rospy.logwarn(err)
            return False

    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)


class EvadeAction(Action):

    condition_templates = [
        [ws.is_a, [V("picker", ConceptNode), C("human")]],
        [ws.wants_to_pass, [V("picker", ConceptNode)]],
        [ws.is_at, [V("picker", ConceptNode),
                    V("picker_position", ConceptNode)]],
        [ws.query_not_at, [V("anything", ConceptNode),
                           V("my_destination", ConceptNode)]],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]],
        [ws.leads_to, [V("my_position", ConceptNode),
                       V("my_destination", ConceptNode)]],
        [ws.leads_to, [V("picker_position", ConceptNode),
                       V("my_position", ConceptNode)]],
    ]
    consequence_templates = [
        [ws.is_at, [C(ME), V("my_destination", ConceptNode)]],
        [ws.wants_nothing, [V("picker", ConceptNode)]]
    ]
    # placeholders must be in same order
    # as variables in "args" in the constructor
    placeholders = [ME, "picker", "my_destination", "my_position"]

    def __init__(self, world_state, robco, args):
        super(EvadeAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.destination = args["my_destination"]
        self.gain = EVADE_GAIN
        self.sent_movement_request = False

    def perform(self):
        super(EvadeAction, self).perform()
        if not self.sent_movement_request:
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
        try:
            success = self.robco.get_result().success
            self.ws.moving = False
            rospy.logwarn("movement success is: {}".format(success))
            return True
        except Exception:
            return False

    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED

    def __repr__(self):
        return "<Action:Evade {:} by moving to {:}>".format(self.picker,
                                                            self.destination)


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
        [ws.is_at, [V("picker", ConceptNode),
                    V("my_destination", ConceptNode)]],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        [ws.wants_nothing, [V("picker", ConceptNode)]],
        [ws.has_crate, [V("picker", ConceptNode)]],
        [ws.robot_remove_crate, [C(ME), P(ws._empty_crate_count.name)]]
    ]
    # placeholders must be in same order
    # as variables in "args" in the constructor
    placeholders = [ME, "picker", "my_destination"]

    def __init__(self, world_state, robco, args):
        super(GiveCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.gain = GIVE_GAIN
        self.cost = GIVE_COST
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(C(ME), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(C(ME), me))

    def get_cost(self):
        return self.cost

    def perform(self):
        super(GiveCrateAction, self).perform()
        sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg)
            consequence = fun(self.ws, *new_args)
            rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
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
        [ws.is_at, [V("picker", ConceptNode),
                    V("my_destination", ConceptNode)]],
        [ws.is_at, [C(ME), V("my_position", ConceptNode)]]
    ]
    consequence_templates = [
        [ws.not_crate_full, [V("picker", ConceptNode)]],
        [ws.not_called_robot, [V("picker", ConceptNode)]],
        [ws.wants_nothing, [V("picker", ConceptNode)]],
        [ws.robot_add_crate, [C(ME), P(ws._full_crate_count.name)]],
        [ws.robot_remove_crate, [C(ME), P(ws._empty_crate_count.name)]]
    ]
    # placeholders must be in same order
    # as variables in "args" in the constructor
    placeholders = [ME, "picker", "my_destination"]

    def __init__(self, world_state, robco, args):
        # [me, picker, my_destination]
        super(ExchangeCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args["picker"]
        self.gain = EXCHANGE_GAIN
        self.cost = EXCHANGE_COST

    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(C(ME), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(C(ME), me))

    def get_cost(self):
        return self.cost

    def perform(self):
        super(ExchangeCrateAction, self).perform()
        sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg)
            consequence = fun(self.ws, *new_args)
            rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
        return True

    def __repr__(self):
        return "<Action:Exchange crate with {:}>".format(self.picker)


class DepositCrateAction(Action):

    condition_templates = [
        [ws.robot_has_crate, [C(ME), P(ws._full_crate_count.name)]],
        [ws.robot_get_crate_count, [C(ME),
                                    P(ws._full_crate_count.name),
                                    V("full crates", NumberNode)]],
        [ws.robot_get_crate_count, [C(ME),
                                    P(ws._empty_crate_count.name),
                                    V("empty crates", NumberNode)]],
        [ws.is_at, [C(ME), C(DEPOT)]]
    ]
    consequence_templates = [
        [ws.robot_set_crate_count, [C(ME),
                                    P(ws._full_crate_count.name),
                                    N("0")]],
        [ws.robot_set_crate_count, [C(ME),
                                    P(ws._empty_crate_count.name),
                                    N(str(CRATE_CAPACITY))]]
    ]
    # placeholders must be in same order
    # as variables in "args" in the constructor
    placeholders = [ME, DEPOT, "full crates", "empty crates"]

    def __init__(self, world_state, robco, args):
        super(DepositCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.full_crate_count = int(args["full crates"])
        self.empty_crate_count = int(args["empty crates"])
        self.gain = DEPOSIT_GAIN

    def get_cost(self):
        return (self.full_crate_count
                + self.empty_crate_count) * DEPOSIT_COST

    def get_gain(self):
        return (self.full_crate_count + CRATE_CAPACITY
                - self.empty_crate_count) / 2 * DEPOSIT_GAIN

    def perform(self):
        super(DepositCrateAction, self).perform()
        sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg.typ(arg.node))
            consequence = fun(self.ws, *new_args)
            rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
        return True

    def __repr__(self):
        return "<Action:Deposit crates at {:}>".format(DEPOT)

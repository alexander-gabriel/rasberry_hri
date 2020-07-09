from time import sleep

import rospy

from opencog.type_constructors import *

from parameters import *
from utils import atomspace
from utils import Variable as V
from world_state import WorldState as ws



class Action(object):

    placeholders = []
    gain = 0

    def __init__(self, world_state, args):
        self.kb = world_state.kb
        self.ws = world_state
        self.instances = args
        rospy.logdebug("{:} has args: {}".format(self.__class__.__name__, args))
        self.conditions = []
        self.consequences = []
        self.cost = 0
        self.first_try = True
        # rospy.loginfo("--")
        # rospy.loginfo(self.__class__.__name__)
        # rospy.loginfo(self.placeholders)
        # rospy.loginfo(self.instances)
        placeholder_instances = zip(self.placeholders, self.instances)
        # rospy.loginfo(placeholder_instances)
        # rospy.loginfo("--")
        for condition in self.condition_templates:
            new_variables = []
            for variable in condition[1]:
                new_variable = variable
                for placeholder, instance in placeholder_instances:
                    new_variable = new_variable.replace(placeholder, instance)
                new_variables.append(new_variable)
            self.conditions.append([condition[0], new_variables])
        for consequence in self.consequence_templates:
            new_variables = []
            for variable in consequence[1]:
                new_variable = variable
                for placeholder, instance in placeholder_instances:
                    new_variable = new_variable.replace(placeholder, instance)
                new_variables.append(new_variable)
            self.consequences.append([consequence[0], new_variables])



        # for index in range(len(args)):
        #     for condition in self.condition_templates:
        #         self.conditions.append(condition.replace("?{:}".format(self.placeholders[index]), args[index]))
        #     for consequence in self.consequence_templates:
        #         self.consequences.append(consequence.replace("?{:}".format(self.placeholders[index]), args[index]))

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            # for instance in self.instances:
            #     if not instance in other.instances:
            #         return False
            # for instance in other.instances:
            #     if not instance in self.instances:
            #         return False
            # for condition in self.conditions:
            #     if not condition in other.conditions:
            #         return False
            # for condition in other.conditions:
            #     if not condition in self.conditions:
            #         return False
            # for consequence in self.consequences:
            #     if not consequence in other.consequences:
            #         return False
            # for consequence in other.consequences:
            #     if not consequence in self.consequences:
            #         return False
            return self.instances == other.instances and self.conditions == other.conditions and self.consequences == other.consequences
        return False


    def __neq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            # for instance in self.instances:
            #     if not instance in other.instances:
            #         return True
            # for instance in other.instances:
            #     if not instance in self.instances:
            #         return True
            # for condition in self.conditions:
            #     if not condition in other.conditions:
            #         return True
            # for condition in other.conditions:
            #     if not condition in self.conditions:
            #         return True
            # for consequence in self.consequences:
            #     if not consequence in other.consequences:
            #         return True
            # for consequence in other.consequences:
            #     if not consequence in self.consequences:
            #         return True
            return not (self.instances == other.instances and self.conditions == other.conditions and self.consequences == other.consequences)
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


    @classmethod
    def get_gain(cls):
        return cls.gain



class MoveAction(Action):

    condition_templates = [
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    # [ws.linked, [, V("destination", VariableNode)]]
    ]

    consequence_templates = [[ws.is_at, [V(ME, ConceptNode), V("destination", VariableNode)]]]
    placeholders = [V(ME, ConceptNode), V("origin", VariableNode), V("destination", VariableNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, origin, destination]
        super(MoveAction, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args[2]
        self.gain = MOVE_GAIN
        self.sent_movement_request = False
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(V(ME, ConceptNode), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(V(ME, ConceptNode), me))


    def perform(self):
        super(MoveAction, self).perform()
        if not self.sent_movement_request:
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
        try:
            success = self.robco.get_result().success
            self.ws.moving = False
            rospy.logwarn("ACT: Reached final destination: {}".format(success))
            return True
        except AttributeError:
            return False


    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED


    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)



class MoveToAction(Action):

    condition_templates = [
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    [ws.is_at, [V("picker", VariableNode), V("destination", VariableNode)]],
    [ws.linked, [V("origin", VariableNode), V("destination", VariableNode)]]
    ]

    consequence_templates = [[ws.is_at, [V(ME, ConceptNode), V("destination", VariableNode)]]]

    placeholders = [V(ME, ConceptNode), V("picker", VariableNode), V("origin", VariableNode), V("destination", VariableNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(MoveToAction, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args[3]
        self.gain = MOVETO_GAIN
        self.sent_movement_request = False
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(V(ME, ConceptNode), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(V(ME, ConceptNode), me))


    def perform(self):
        super(MoveToAction, self).perform()
        if not self.sent_movement_request:
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
        try:
            success = self.robco.get_result().success
            self.ws.moving = False
            rospy.logwarn("ACT: Reached final destination: {}".format(success))
            return True
        except Exception as err:
            rospy.logwarn(err)
            return False


    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED


    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)



class BerryEvadeAction(Action):

    # NOTE: working version with implicit intention recognition
    # condition_templates = [
    # [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    # [ws.approaching, [V("picker", VariableNode)]],
    # [ws.seen_picking, [V("picker", VariableNode)]],
    # [ws.not_called_robot, [V("picker", VariableNode)]],
    # [ws.is_at, [V("picker", VariableNode), V("place1", VariableNode)]],
    # [ws.not_has_berries, [V("place1", VariableNode)]],
    # [ws.has_berries, [V("origin", VariableNode)]],
    # [ws.query_not_at, [V("anyhing", VariableNode), V("destination", VariableNode)]],
    # [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    # [ws.leads_to, [V("origin", VariableNode), V("destination", VariableNode)]],
    # [ws.leads_to, [V("place1", VariableNode), V("origin", VariableNode)]],
    # ]

    condition_templates = [
    [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    [ws.wants_to_pass, [V("picker", VariableNode)]],  # <- see the intention :)
    [ws.is_at, [V("picker", VariableNode), V("place1", VariableNode)]],
    # [ws.not_has_berries, [V("place1", VariableNode)]],
    [ws.has_berries, [V("origin", VariableNode)]],
    [ws.query_not_at, [V("anyhing", VariableNode), V("destination", VariableNode)]],
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    [ws.leads_to, [V("origin", VariableNode), V("destination", VariableNode)]],
    [ws.leads_to, [V("place1", VariableNode), V("origin", VariableNode)]],
    ]

    consequence_templates = [[ws.is_at, [V(ME, ConceptNode), V("destination", VariableNode)]], [ws.wants_nothing, [V("picker", VariableNode)]]]

    placeholders = [V(ME, ConceptNode), V("picker", VariableNode), V("origin", VariableNode), V("destination", VariableNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(BerryEvadeAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args[1]
        self.destination = args[3]
        self.gain = EVADE_GAIN
        self.sent_movement_request = False


    def perform(self):
        super(BerryEvadeAction, self).perform()
        if not self.sent_movement_request:
            self.robco.move_to(self.destination)
            self.sent_movement_request = True
            self.ws.moving = True
        try:
            success = self.robco.get_result().success
            self.ws.moving = False
            rospy.logwarn("movement success is: {}".format(success))
            return True
        except:
            return False


    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED


    def __repr__(self):
        return "<Action:BerryEvade {:} by moving to {:}>".format(self.picker, self.destination)



class EvadeAction(Action):

    # NOTE: working version with implicit intention recognition
    # condition_templates = [
    # [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    # [ws.approaching, [V("picker", VariableNode)]],
    # [ws.seen_picking, [V("picker", VariableNode)]],
    # [ws.not_called_robot, [V("picker", VariableNode)]],
    # [ws.is_at, [V("picker", VariableNode), V("place1", VariableNode)]],
    # [ws.query_not_at, [V("anyhing", VariableNode), V("destination", VariableNode)]],
    # [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    # [ws.leads_to, [V("origin", VariableNode), V("destination", VariableNode)]],
    # [ws.leads_to, [V("place1", VariableNode), V("origin", VariableNode)]],
    # ]

    condition_templates = [
    [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    [ws.wants_to_pass, [V("picker", VariableNode)]],  # <- see the intention :)
    [ws.is_at, [V("picker", VariableNode), V("place1", VariableNode)]],
    [ws.query_not_at, [V("anyhing", VariableNode), V("destination", VariableNode)]],
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    [ws.leads_to, [V("origin", VariableNode), V("destination", VariableNode)]],
    [ws.leads_to, [V("place1", VariableNode), V("origin", VariableNode)]],
    ]

    consequence_templates = [[ws.is_at, [V(ME, ConceptNode), V("destination", VariableNode)]], [ws.wants_nothing, [V("picker", VariableNode)]]]

    placeholders = [V(ME, ConceptNode), V("picker", VariableNode), V("origin", VariableNode), V("destination", VariableNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(EvadeAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args[1]
        self.destination = args[3]
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
        except:
            return False


    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / ROBOT_SPEED


    def __repr__(self):
        return "<Action:Evade {:} by moving to {:}>".format(self.picker, self.destination)



class GiveCrateAction(Action):

    # NOTE: working version with implicit intention recognition
    # condition_templates = [
    # [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    # [ws.is_at, [V("picker", VariableNode), V("destination", VariableNode)]],
    # [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    # [ws.not_seen_picking, [V("picker", VariableNode)]],
    # [ws.called_robot, [V("picker", VariableNode)]],
    # ]

    condition_templates = [
    [ws.approaching [V("picker", VariableNode)]],
    [ws.robot_has_crate, [V(ME, ConceptNode), V("empty crate count", PredicateNode)]],
    [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    [ws.wants_to_get_crate, [V("picker", VariableNode)]],  # <- see the intention :)
    [ws.is_at, [V("picker", VariableNode), V("destination", VariableNode)]],
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]]
    ]

    consequence_templates = [[ws.not_called_robot, [V("picker", VariableNode)]], [ws.wants_nothing, [V("picker", VariableNode)]], [ws.robot_remove_crate, [V(ME, ConceptNode), V("empty crate count", PredicateNode)]]]

    placeholders = [V(ME, ConceptNode), V("picker", VariableNode), V("destination", VariableNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, picker]
        super(GiveCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args[1]
        self.gain = GIVE_GAIN
        self.cost = GIVE_COST
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(V(ME, ConceptNode), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(V(ME, ConceptNode), me))


    def get_cost(self):
        return self.cost


    def perform(self):
        super(GiveCrateAction, self).perform()
        sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(self.ws.kb.concept(arg))
            consequence = fun(self.ws, *new_args)
            rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.tv = TruthValue(1, 1)
        return True


    def __repr__(self):
        return "<Action:Give crate to {:}>".format(self.picker)



class ExchangeCrateAction(Action):

    # NOTE: working version with implicit intention recognition
    # condition_templates = [
    # [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    # [ws.is_at, [V("picker", VariableNode), V("destination", VariableNode)]],
    # [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]],
    # [ws.seen_picking, [V("picker", VariableNode)]],
    # [ws.called_robot, [V("picker", VariableNode)]],
    # ]

    condition_templates = [
    [ws.approaching [V("picker", VariableNode)]],
    [ws.robot_has_crate, [V(ME, ConceptNode), V("empty crate count", PredicateNode)]],
    [ws.robot_has_crate_capacity, [V(ME, ConceptNode), V("full crate count", PredicateNode)]],
    [ws.is_a, [V("picker", VariableNode), V("human", ConceptNode)]],
    [ws.wants_to_exchange_their_crate, [V("picker", VariableNode)]],  # <- see the intention :)
    [ws.is_at, [V("picker", VariableNode), V("destination", VariableNode)]],
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]]
    ]

    consequence_templates = [[ws.not_seen_picking, [V("picker", VariableNode)]], [ws.not_called_robot, [V("picker", VariableNode)]], [ws.wants_nothing, [V("picker", VariableNode)]], [ws.robot_add_crate, [V(ME, ConceptNode), V("full crate count", PredicateNode)]], [ws.robot_remove_crate, [V(ME, ConceptNode), V("empty crate count", PredicateNode)]]]

    placeholders = [V(ME, ConceptNode), V("picker", VariableNode), V("destination", VariableNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, picker, destination]
        super(ExchangeCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args[1]
        self.gain = EXCHANGE_GAIN
        self.cost = EXCHANGE_COST

    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(V(ME, ConceptNode), me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(V(ME, ConceptNode), me))


    def get_cost(self):
        return self.cost


    def perform(self):
        super(ExchangeCrateAction, self).perform()
        sleep(self.cost)
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(arg.typ(arg.label))
            consequence = fun(self.ws, *new_args)
            consequence.tv = TruthValue(1, 1)
        return True


    def __repr__(self):
        return "<Action:Exchange crate with {:}>".format(self.picker)



class DepositCrateAction(Action):

    condition_templates = [
    [ws.robot_has_crate, [V(ME, ConceptNode), V("full crate count", PredicateNode)]],
    [ws.robot_get_crate_count, [V(ME, ConceptNode), V("full crate count", PredicateNode)]],
    [ws.is_at, [V("depot", VariableNode), V("destination", VariableNode)]],
    [ws.is_at, [V(ME, ConceptNode), V("origin", VariableNode)]]
    ]

    consequence_templates = [[ws.robot_set_crate_count, [V(ME, ConceptNode), V("full crate count", PredicateNode), V("0", NumberNode)]], [ws.robot_set_crate_count, [V(ME, ConceptNode), V("empty crate count", PredicateNode), V(str(CRATE_CAPACITY), NumberNode)]]]

    placeholders = [V(ME, ConceptNode), V("depot", VariableNode), V("destination", VariableNode), V("depot", PredicateNode)] # in same order as constructor arguments


    def __init__(self, world_state, robco, args):
        # [me, picker]
        super(DepositCrateAction, self).__init__(world_state, args)
        self.robco = robco
        self.depot = args[1]
        self.crate_count = args[3]
        self.gain = DEPOSIT_GAIN
        # self.cost = int(args[3]) * DEPOSIT_COST


    def get_cost(self):
        result = self.ws.kb.execute(self.ws.robot_get_crate_count(V(ME, ConceptNode), V("empty crate count", PredicateNode)))
        empties = result.get_out()[0].to_list()[0]
        result = self.ws.kb.execute(self.ws.robot_get_crate_count(V(ME, ConceptNode), self.V("full crate count", PredicateNode)))
        fulls = result.get_out()[0].to_list()[0]
        return (fulls + empties) * DEPOSIT_COST
        return self.cost


    @classmethod
    def get_gain(cls):
        result = self.ws.kb.execute(self.ws.robot_get_crate_count(V(ME, ConceptNode), V("empty crate count", PredicateNode)))
        empties = result.get_out()[0].to_list()[0]
        result = self.ws.kb.execute(self.ws.robot_get_crate_count(V(ME, ConceptNode), self.V("full crate count", PredicateNode)))
        fulls = result.get_out()[0].to_list()[0]
        return (fulls + CRATE_CAPACITY - empties)/2 * DEPOSIT_GAIN


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
        return "<Action:Deposit crates at {:}>".format(self.depot)

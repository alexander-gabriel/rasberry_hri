import rospy

from actions import MoveAction, MoveToAction, GiveCrateAction, ExchangeCrateAction, EvadeAction
from utils import OrderedConsistentSet, suppress
from opencog.type_constructors import *

from time import sleep

class Goal(object):

    action_template = None
    subgoal_templates = []

    def __init__(self, world_state, robco, args):
        # list of subgoals (only if there is no action to be performed)
        self.subgoals = []
        self.world_state = world_state
        # action to be performed (only if there are no subgoals)
        self.action = self.instantiate_action_template(world_state, robco, args)


    @classmethod
    def instantiate_action_template(cls, world_state, robco, args):
        try:
            return cls.action_template(world_state, robco, args)
        except TypeError:
            return None


    @classmethod
    def get_condition_templates(cls):
        try:
            cls.guide = []
            # cls.guide = cls.action_template.get_instance_guide()
            # rospy.loginfo(cls.condition_templates,cls.guide)
            return cls.action_template.condition_templates
        except AttributeError:
            try:
                return cls.condition_templates
            except AttributeError:
                cls.condition_templates = OrderedConsistentSet()
                consequences = OrderedConsistentSet()
                guide = OrderedConsistentSet()
                for subgoal in cls.subgoal_templates:
                    new_conditions = subgoal.get_condition_templates()
                    # rospy.loginfo("new conditions:")
                    # rospy.loginfo(new_conditions)
                    # rospy.loginfo("new consequences:")
                    # rospy.loginfo(consequences)
                    for condition in new_conditions:
                        if not condition in consequences:
                            cls.condition_templates.append(condition)
                    consequences += subgoal.get_consequence_templates()
                return cls.condition_templates


    @classmethod
    def get_consequence_templates(cls):
        try:
            return cls.action_template.consequence_templates
        except AttributeError:
            try:
                return cls.consequences
            except AttributeError:
                cls.consequences = OrderedConsistentSet()
                for subgoal in cls.subgoal_templates:
                    cls.consequences += subgoal.get_consequence_templates()
                return cls.consequences


    def get_conditions(self):
        try:
            return self.action.conditions
        except AttributeError:
            try:
                return self.conditions
            except AttributeError:
                self.conditions = OrderedConsistentSet()
                consequences = OrderedConsistentSet()
                for subgoal in self.subgoals:
                    new_conditions = subgoal.get_conditions()
                    for condition in new_conditions:
                        if not condition in consequences:
                            self.conditions.append(condition)
                    consequences += subgoal.get_consequences()
                return self.conditions


    def get_consequences(self):
        try:
            return self.action.consequences
        except AttributeError:
            try:
                return self.consequences
            except AttributeError:
                self.consequences = OrderedConsistentSet()
                for subgoal in self.subgoals:
                    self.consequences += subgoal.get_consequences()
                return self.consequences


    @classmethod
    def find_instances(cls, world_state):
        templates = cls.get_condition_templates()
        conditions = []
        variables = OrderedConsistentSet()
        for fun, args in templates:
            args = [w.replace('me', world_state.me.capitalize()) for w in args]
            vars, condition = fun(*args)
            variables+=vars
            conditions.append(condition)
        query = GetLink(VariableList(*variables.items), AndLink(*conditions))
        prob, formula = world_state.check(query)
        if prob < 0.75:
            return []
        targets = []
        for atom in formula.getGroundAtoms():
            targets.append(atom.params)
        return [targets]


    def is_achieved(self):
        for consequence in self.get_consequences():
            #TODO: fix achievement checking
            truth = self.world_state.truth(consequence.replace(TARGET, target))
            if consequence.startswith("!"):
                if truth is None or truth > TRUTH_THRESHOLD:
                    return False
            else:
                if truth is None or truth <= TRUTH_THRESHOLD:
                    return False
        return True #TODO cache result


    def get_action_queue(self):
        try:
            return self.action_queue
        except AttributeError:
            if not self.action is None:
                self.action_queue = [self.action]
            else:
                self.action_queue = []
                for subgoal in self.subgoals:
                    self.action_queue += subgoal.get_action_queue()
        return self.action_queue


    def get_next_action(self):
        with suppress(IndexError):
            return self.get_action_queue()[0]
        return None


    def perform_action(self):
        action_queue = self.get_action_queue()
        action = action_queue.pop(0)
        tries = 5
        succeeded = False
        while tries > 0 and not succeeded:
            succeeded = action.perform()
            tries -= 1
            if not succeeded:
                rospy.loginfo("GOL: Tried to peform action; result: {}, {:d} tries remaining".format(succeeded, tries))
                sleep(2)
        if not succeeded:
            action_queue.insert(0,action)



    def get_cost(self):
        try:
            return self.cost
        except AttributeError:
            try:
                self.cost = self.action.get_cost()
            except AttributeError:
                self.cost = 0
                for subgoal in self.subgoals:
                    self.cost += subgoal.get_cost()
        return self.cost


    def get_gain(self):
        try:
            return self.gain
        except AttributeError:
            try:
                self.gain = self.action.gain
            except AttributeError:
                self.gain = 0
                for subgoal in self.subgoals:
                    self.gain += subgoal.get_gain()
        return self.gain



class MoveGoal(Goal):

    action_template = MoveAction

    def __init__(self, world_state, robco, args):
        # me, origin, destination
        super(MoveGoal, self).__init__(world_state, robco, args)



class MoveTo(Goal):

    action_template = MoveToAction

    def __init__(self, world_state, robco, args):
        # me, picker, origin, destination = None
        super(MoveToGoal, self).__init__(world_state, robco, args)



class GiveCrateGoal(Goal):

    action_template = GiveCrateAction

    def __init__(self, world_state, robco, args):
        # me, picker, destination
        super(GiveCrateGoal, self).__init__(world_state, robco, args)



class ExchangeCrateGoal(Goal):

    action_template = ExchangeCrateAction

    def __init__(self, world_state, robco, args):
        # rospy.loginfo(args)
        # me, picker, destination = None
        super(ExchangeCrateGoal, self).__init__(world_state, robco, args)



class EvadeGoal(Goal):

    action_template = EvadeAction

    def __init__(self, world_state, robco, args):
        me = args[1][0]
        origin = args[1][1]
        picker = args[0][0]
        destination = args[3][0]
        super(EvadeGoal, self).__init__(world_state, robco, [me, picker, origin, destination])



class DeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,GiveCrateGoal]

    def __init__(self, world_state, robco, args):
        rospy.loginfo(args)
        me = args[0][0]
        origin = args[0][1]
        picker = args[3][0]
        destination = args[3][1]
        super(DeliverGoal, self).__init__(world_state, robco, args)
        self.subgoals.append(MoveGoal(world_state, robco, [me, origin, destination]))
        self.subgoals.append(GiveCrateGoal(world_state, robco, [me, picker]))
        self.subgoals.append(MoveGoal(world_state, robco, [me, destination, origin]))



class ExchangeGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,ExchangeCrateGoal,MoveGoal]

    def __init__(self, world_state, robco, args):
        me = args[0][0]
        origin = args[0][1]
        picker = args[3][0]
        destination = args[3][1]
        super(ExchangeGoal, self).__init__(world_state, robco, [])
        self.subgoals.append(MoveGoal(world_state, robco, [me, origin, destination]))
        self.subgoals.append(ExchangeCrateGoal(world_state, robco, [me, picker, destination]))
        self.subgoals.append(MoveGoal(world_state, robco, [me, destination, origin]))

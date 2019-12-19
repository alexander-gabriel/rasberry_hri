import rospy

from actions import MoveAction, MoveToAction, GiveCrateAction, ExchangeCrateAction, EvadeAction
from utils import OrderedConsistentSet, suppress, atomspace
from opencog.type_constructors import *
from opencog.bindlink import execute_atom, evaluate_atom

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


    def __eq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            # for subgoal in self.subgoal_templates:
            #     if not subgoal in other.subgoal_templates:
            #         return False
            # for subgoal in other.subgoal_templates:
            #     if not subgoal in self.subgoal_templates:
            #         return False
            return self.action_template == other.action_template and self.subgoal_templates == other.subgoal_templates
        return False


    def __neq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            # for subgoal in self.subgoal_templates:
            #     if not subgoal in other.subgoal_templates:
            #         return True
            # for subgoal in other.subgoal_templates:
            #     if not subgoal in self.subgoal_templates:
            #         return True
            return not (self.action_template == other.action_template and self.subgoal_templates == other.subgoal_templates)
        return True


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
        clauses = []
        variables = OrderedConsistentSet()
        for fun, args in templates:
            new_args = []
            for arg in args:
                if arg == "me":
                    new_args.append(ConceptNode(world_state.me.capitalize()))
                elif fun.__name__ == "is_a" and args.index(arg) == 1:
                    new_args.append(ConceptNode(arg))
                else:
                    variable = VariableNode(arg)
                    new_args.append(variable)
                    variables.append(variable)
            condition = fun(*new_args)
            if condition.type_name == "EvaluationLink" or condition.type_name == "InheritanceLink":
                conditions.append(condition)
            else:
                clauses.append(condition)
        conditions.append(PresentLink(*clauses))
        query = GetLink(VariableList(*variables.items), AndLink(*conditions))
        results = world_state.check(query)
        targets = cls.get_targets(results)
        return targets


    def is_achieved(self, world_state):
        consequences = []
        clauses = []
        for fun,args in self.get_consequences():
            candidate = fun(*[ConceptNode(arg) for arg in args])
            if candidate.type_name == "EvaluationLink" or candidate.type_name == "InheritanceLink":
                consequences.append(candidate)
            else:
                clauses.append(candidate)
        consequences.append(PresentLink(*clauses))
        query = SatisfactionLink(AndLink(*consequences))
        # query = GetLink(AndLink(*consequences))
        results = world_state.check(query)
        for result in results.get_out():
            rospy.loginfo("------ is achieved? ------")
            rospy.loginfo(query)
            rospy.loginfo(results)
            rospy.loginfo(result)
            rospy.loginfo(result.tv)
            rospy.loginfo("--------------------------")
            return True


            #TODO: fix achievement checking
            # truth = self.world_state.truth(consequence.replace(TARGET, target))
            # if consequence.startswith("!"):
            #     if truth is None or truth > TRUTH_THRESHOLD:
            #         return False
            # else:
            #     if truth is None or truth <= TRUTH_THRESHOLD:
            #         return False
        return False #TODO cache result


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
        else:
            rospy.loginfo("GOL: Performed action {}, action queue: {} ".format(action.__class__.__name__, action_queue))


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
        me = world_state.me.capitalize()
        origin = args[1][1]
        picker = args[0][0]
        destination = args[3][0]
        super(EvadeGoal, self).__init__(world_state, robco, [me, picker, origin, destination])


    @classmethod
    def get_targets(cls, results):
        targets = []
        # try:
        #     for result in results.get_out():
        #         rospy.loginfo(result)
        #         if result.is_link():
        #             rospy.loginfo("Link result")
        #             for element in result.get_out():
        #                 rospy.loginfo(dir(element))
        #         else:
        #             rospy.loginfo("Single Element result")
        #             rospy.loginfo(dir(result))
        #         # rospy.loginfo(result.get_out()[0].name)
        #         # raise Exception()
        #     # rospy.loginfo("GOL: Result is: {:}".format(results))
        # except IndexError:
        #     rospy.logdebug("GOL: Empty Result")
        #
        # for atom in formula.getGroundAtoms():
        #     targets.append(atom.params)
        return targets



class DeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,GiveCrateGoal]

    def __init__(self, world_state, robco, args):
        # rospy.loginfo(args)
        me = world_state.me.capitalize()
        origin = args[0][0]
        picker = args[0][1]
        destination = args[0][2]
        super(DeliverGoal, self).__init__(world_state, robco, args)
        self.subgoals.append(MoveGoal(world_state, robco, [me, origin, destination]))
        self.subgoals.append(GiveCrateGoal(world_state, robco, [me, picker, destination]))
        self.subgoals.append(MoveGoal(world_state, robco, [me, destination, origin]))


    @classmethod
    def get_targets(cls, results):
        # rospy.loginfo(results)
        targets = []
        rospy.loginfo(results)
        for result in results.get_out():
            subtarget = []
            for list_link in result.get_out():
                nodes = map(lambda x: x.name, list_link.get_out())
                subtarget.append(nodes)
            targets.append(subtarget)
        return targets



class ExchangeGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,ExchangeCrateGoal,MoveGoal]

    def __init__(self, world_state, robco, args):
        me = world_state.me.capitalize()
        origin = args[0]
        picker = args[1]
        destination = args[2]
        super(ExchangeGoal, self).__init__(world_state, robco, [])
        self.subgoals.append(MoveGoal(world_state, robco, [me, origin, destination]))
        self.subgoals.append(ExchangeCrateGoal(world_state, robco, [me, picker, destination]))
        self.subgoals.append(MoveGoal(world_state, robco, [me, destination, origin]))


    @classmethod
    def get_targets(cls, results):
        rospy.loginfo(results)
        targets = []
        for result in results.get_out():
            for list_link in result.get_out():
                nodes = map(lambda x: x.name, list_link.get_out())
                targets.append(nodes)
        return targets

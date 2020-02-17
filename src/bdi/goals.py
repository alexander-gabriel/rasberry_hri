import rospy
import sys
import time

from actions import MoveAction, MoveToAction, GiveCrateAction, ExchangeCrateAction, EvadeAction
from utils import OrderedConsistentSet, suppress, atomspace, TRUE
from opencog.type_constructors import *
from opencog.bindlink import execute_atom, evaluate_atom

from time import sleep

class WrongParameterException(Exception):

    def __init__(self):
        super(WrongParameterException, self).__init__()


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
            rospy.logdebug("{:}.action_template.condition_templates: {}".format(cls.__name__, cls.action_template.condition_templates))
            return cls.action_template.condition_templates
        except AttributeError:
            try:
                rospy.logdebug("{:}.condition_templates: {}".format(cls.__name__, cls.condition_templates))
                return cls.condition_templates
            except AttributeError:
                cls.condition_templates = OrderedConsistentSet()
                consequences = OrderedConsistentSet()
                guide = OrderedConsistentSet()
                for subgoal in cls.subgoal_templates:
                    new_conditions = subgoal.get_condition_templates()
                    rospy.logdebug("{:}'s subgoal {:} condition_templates: {}".format(cls.__name__, subgoal.__name__, new_conditions))
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
                return cls.consequences_templates
            except AttributeError:
                cls.consequences_templates = OrderedConsistentSet()
                for subgoal in cls.subgoal_templates:
                    cls.consequences_templates += subgoal.get_consequence_templates()
                return cls.consequences_templates


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
        rospy.logdebug("GOL: Getting consequences for: {}".format(self))
        try:
            rospy.logdebug("GOL: self.action.consequences: {}".format(self.action.consequences))
            return self.action.consequences
        except AttributeError:
            try:
                rospy.logdebug("GOL: self.consequences: {}".format(self.consequences))
                return self.consequences
            except AttributeError:
                self.consequences = OrderedConsistentSet()
                for subgoal in self.subgoals:
                    self.consequences += subgoal.get_consequences()
                return self.consequences


    @classmethod
    def get_targets(cls, query, results):
        targets = {}
        query_out = query.get_out()
        results_out = results.get_out()
        for index in range(len(query_out)):
            if query_out[index].type_name == "TypedVariableLink":
                for atom in query_out[index].get_out():
                    if atom.type_name == "VariableNode":
                        targets[atom.name] = results_out[index].name
            elif query_out[index].type_name == "VariableNode":
                targets[query_out[index].name] = results_out[index].name
            else:
                targets.update(cls.get_targets(query_out[index], results_out[index]))
        rospy.logdebug("GOL: Found targets: {:} for goal: {:}".format(targets, cls.__name__))
        return targets


    @classmethod
    def find_instances(cls, world_state):
        start_time = time.time()
        templates = cls.get_condition_templates()
        rospy.logdebug("GOL: Templates: {}".format(templates))
        conditions = []

        clauses = []
        variables = OrderedConsistentSet()
        for fun, args in templates:
            new_args = []
            # variables2 = []
            for arg in args:
                if arg == "me":
                    new_args.append(ConceptNode(world_state.me.capitalize()))
                elif fun.__name__ == "is_a" and args.index(arg) == 1:
                    new_args.append(ConceptNode(arg))
                else:
                    variable = VariableNode(arg)
                    # variables2.append(TypedVariableLink(variable, TypeNode("ConceptNode")))
                    new_args.append(variable)
                    variables.append(TypedVariableLink(variable, TypeNode("ConceptNode")))
            condition = fun(*new_args)
            # rospy.loginfo("------------------------------------")
            # if len(variables2) > 0:
            #     r = world_state.check(GetLink(VariableList(*variables2), condition))
            # else:
            #     r = world_state.check(GetLink(VariableList(), condition))
            # rospy.logwarn("condition: {}".format(condition))
            # rospy.logerr("result: {}".format(r))
            # results = world_state.reason(condition, VariableList())
            # rospy.loginfo("GOL: Condition: {:}".format(condition))
            # # rospy.loginfo("GOL: Condition result: {:}".format(results))
            conditions.append(condition)

        query = AndLink(*conditions)
        variables = VariableList(*variables.items)
        rospy.logdebug("GOL: Reason: {:}".format(query))
        world_state.reason(query, variables)
        results = world_state.check(GetLink(variables, query))

        targets = []
        if results.type_name == "SetLink":
            for result in results.get_out():
                rospy.logdebug("GOL: Result: {:}".format(result))
                targets.append(cls.get_targets(variables, result))
        else:
            rospy.logwarn("Strange result: {:}".format(results))
        rospy.logdebug("GOL: Checked goal {:} for targets -- {:.4f}".format(cls.__name__, time.time() - start_time))
        return targets


    def is_achieved(self, world_state):
        start_time = time.time()
        success = False
        if not self.get_action_queue():
            consequences = []
            for fun, args in self.get_consequences():
                rospy.logdebug("{}: {}".format(fun, args))
                new_args = []
                for arg in args:
                    if arg == "me":
                        new_args.append(ConceptNode(world_state.me.capitalize()))
                    else:
                        new_args.append(ConceptNode(arg))
                candidate = fun(*new_args)
                consequences.append(candidate)
            # query = AndLink(*consequences)
            query = consequences[0]
            # query = GetLink(AndLink(*consequences))
            rospy.logdebug("GOL: Check: {:}".format(query))
            results = world_state.check(query)
            rospy.logdebug("------ is achieved? ------")
            rospy.logdebug(results)
            if (results.tv == TRUE):
                rospy.logdebug("pleasure achieved")
                success = True
            else:
                rospy.logdebug("no bueno")
                success = False
        rospy.logdebug("GOL: Checked if goal {:} was achieved -- {:.4f}".format(self, time.time() - start_time))
        return success
        # rospy.logdebug("--------------------------")
        # return True

            #TODO: fix achievement checking
            # truth = self.world_state.truth(consequence.replace(TARGET, target))
            # if consequence.startswith("!"):
            #     if truth is None or truth > TRUTH_THRESHOLD:
            #         return False
            # else:
            #     if truth is None or truth <= TRUTH_THRESHOLD:
            #         return False
        # return False #TODO cache result


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
                rospy.logwarn("GOL: Tried to peform action; result: {}, {:d} tries remaining".format(succeeded, tries))
                sleep(2)
        if not succeeded:
            action_queue.insert(0,action)
        else:
            rospy.loginfo("GOL: Performed action {}; Action queue is: {} ".format(action, action_queue))


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


    def __repr__(self):
        return "<Goal:Move>"



class MoveTo(Goal):

    action_template = MoveToAction

    def __init__(self, world_state, robco, args):
        # me, picker, origin, destination
        super(MoveToGoal, self).__init__(world_state, robco, args)


    def __repr__(self):
        return "<Goal:Move to>"



class GiveCrateGoal(Goal):

    action_template = GiveCrateAction

    def __init__(self, world_state, robco, args):
        # me, picker, destination
        super(GiveCrateGoal, self).__init__(world_state, robco, args)


    def __repr__(self):
        return "<Goal:Give crate>"



class ExchangeCrateGoal(Goal):

    action_template = ExchangeCrateAction

    def __init__(self, world_state, robco, args):
        # me, picker, destination
        super(ExchangeCrateGoal, self).__init__(world_state, robco, args)


    def __repr__(self):
        return "<Goal:Exchange crate>"



class EvadeGoal(Goal):

    action_template = EvadeAction

    def __init__(self, world_state, robco, args):
        me = world_state.me.capitalize()
        origin = args["origin"]
        self.picker = args["picker"]
        self.destination = args["destination"]
        if self.destination == args["place1"]:
            raise WrongParameterException()
        super(EvadeGoal, self).__init__(world_state, robco, [me, self.picker, origin, self.destination])


    def __repr__(self):
        return "<Goal:Evade {:} by moving to {:} ({}, {})>".format(self.picker, self.destination, self.get_gain(), self.get_cost())




class DeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,GiveCrateGoal]

    def __init__(self, world_state, robco, args):
        me = world_state.me.capitalize()
        origin = args["origin"]
        self.picker = args["picker"]
        self.destination = args["destination"]
        super(DeliverGoal, self).__init__(world_state, robco, args)
        self.subgoals.append(MoveGoal(world_state, robco, [me, origin, self.destination]))
        self.subgoals.append(GiveCrateGoal(world_state, robco, [me, self.picker, self.destination]))
        self.subgoals.append(MoveGoal(world_state, robco, [me, self.destination, origin]))


    def __repr__(self):
        return "<Goal:Deliver crate to {:} at {:} ({}, {})>".format(self.picker, self.destination, self.get_gain(), self.get_cost())


class ExchangeGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,ExchangeCrateGoal,MoveGoal]

    def __init__(self, world_state, robco, args):
        me = world_state.me.capitalize()
        origin = args["origin"]
        self.picker = args["picker"]
        self.destination = args["destination"]
        super(ExchangeGoal, self).__init__(world_state, robco, [])
        self.subgoals.append(MoveGoal(world_state, robco, [me, origin, self.destination]))
        self.subgoals.append(ExchangeCrateGoal(world_state, robco, [me, self.picker, self.destination]))
        self.subgoals.append(MoveGoal(world_state, robco, [me, self.destination, origin]))

    def __repr__(self):
        return "<Goal:Exchange crate with {:} at {:} ({}, {})>".format(self.picker, self.destination, self.get_gain(), self.get_cost())

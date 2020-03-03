import rospy
import sys
import time
import inspect
from copy import copy

# from opencog.bindlink import execute_atom, evaluate_atom

from parameters import *
from utils import OrderedConsistentSet, suppress
from actions import MoveAction, MoveToAction, GiveCrateAction, ExchangeCrateAction, EvadeAction
from opencog.atomspace import types




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
        if inspect.isclass(other):
            return other.__name__ == self.__class__.__name__
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
    def get_targets(cls, targets, variables, results):
        # variables = variables.get_out()
        if results.is_a(types.ListLink):
            results_out = results.get_out()
        else:
            results_out = [results]
        for index in range(len(variables)):
            if variables[index].type_name == "TypedVariableLink":
                variable = variables[index].get_out()[0]
                # rospy.logwarn("GOL: Adding: {}: {}".format(variable.name, results_out[index].name))
                targets[variable.name] = results_out[index].name
            elif variables[index].type_name == "VariableNode":
                targets[variables[index].name] = results_out[index].name
                # rospy.logwarn("GOL: Adding: {}: {}".format(variables[index].name, results_out[index].name))
        rospy.logdebug("GOL: Found targets: {:} for goal: {:}".format(targets, cls.__name__))
        return targets


    @classmethod
    def build_query_fancy(cls, world_state, templates, picker, picker_location, me, my_location):
        conditions = []
        full_conditions = []
        picker_location_var = None
        my_location_var = None
        clauses = []
        variables = OrderedConsistentSet()
        target = {}
        for fun, args in templates:
            if fun.__name__ == "is_at" and args[0] == "picker":
                picker_location_var = args[1]
            elif fun.__name__ == "is_at" and args[0] == ME:
                my_location_var = args[1]
        for fun, args in templates:
            new_args = []
            full_args = []
            skip = False
            if fun.__name__ == "is_at" and args[0] == "picker":
                target["picker"] = picker.name
                skip = True
            elif fun.__name__ == "is_at" and args[0] == ME:
                target[ME] = me.name
                skip = True
            elif fun.__name__ == "is_a" and (args[0] == "picker"):
                target["picker"] = picker.name
                skip = True
            for arg in args:
                if arg == ME:
                    full_args.append(me)
                    if not skip:
                        new_args.append(me)
                        target[ME] = me.name
                elif arg == "picker":
                    full_args.append(picker)
                    if not skip:
                        new_args.append(picker)
                        target["picker"] = picker.name
                elif arg == picker_location_var:
                    full_args.append(picker_location)
                    if not skip:
                        new_args.append(picker_location)
                        target[picker_location_var] = picker_location.name
                elif arg == my_location_var:
                    full_args.append(my_location)
                    if not skip:
                        new_args.append(my_location)
                        target[my_location_var] = my_location.name
                elif fun.__name__ == "is_a" and args.index(arg) == 1:
                    full_args.append(world_state.kb.concept(arg))
                    if not skip:
                        new_args.append(world_state.kb.concept(arg))
                else:
                    variable = world_state.kb.variable(arg)
                    full_args.append(variable)
                    if not skip:
                        new_args.append(variable)
                        variables.append(world_state.kb.typed_variable(variable, world_state.kb.type("ConceptNode")))
            if not skip:
                conditions.append(fun(world_state, *new_args))
            full_conditions.append(fun(world_state, *full_args))
        query = world_state.kb.And(*conditions)
        # rospy.logwarn("Built query:\n{:}".format(query))
        # rospy.logwarn("Built variables:\n{:}".format(variables))
        return query, variables, target

    @classmethod
    def find_instances_fancy(cls, world_state):
        rospy.loginfo("GOL: Checking goal {:} for targets --".format(cls.__name__))
        targets = []
        start_time = time.time()
        templates = cls.get_condition_templates()
        picker_locations = world_state.get_picker_locations()
        me = world_state.kb.concept(world_state.me.capitalize())
        my_location = world_state.get_location(me)
        if my_location is None:
            return targets
        for picker, picker_location in picker_locations:
            query, variables, target = cls.build_query(world_state, templates, picker, picker_location, me, my_location)
            if len(variables) > 0:
                # rospy.loginfo("GOL: Reason (has args):\n{:}".format(query))
                vars = world_state.kb.variable_list(*variables.items)
                # rospy.loginfo("GOL: Reason Variables:\n{:}".format(vars))
                results = world_state.kb.reason(query, vars)
                # rospy.logwarn("GOL: incl. variable results:\n{}".format(results))
                for result in results.get_out():
                    if result.tv == world_state.kb.TRUE:
                        rospy.logwarn("GOL: Found a target for goal: {}".format(cls.__name__))
                        this_target = copy(target)
                        world_state.kb.recursive_query_matcher(query, result, this_target)
                        # rospy.logwarn(result)
                        # rospy.logwarn(this_target)
                        targets.append(this_target)
            else:
                # rospy.loginfo("GOL: Reason (no args):\n{:}".format(query))
                results = world_state.kb.reason(query, world_state.kb.variable_list())
                # rospy.logwarn("GOL: excl. variable results:\n{}".format(results))
                for result in results.get_out():
                    if result.tv == world_state.kb.TRUE:
                        rospy.logwarn("GOL: Found a target for goal: {}".format(cls.__name__))
                        this_target = copy(target)
                        # rospy.logwarn("WE'VE GOT RESULTS - NO ARGS")
                        # rospy.logwarn(result)
                        # rospy.logwarn(this_target)
                        targets.append(this_target)
        duration = time.time() - start_time
        # if duration > 0.01:
        rospy.loginfo("GOL: Checked goal {:} for targets -- {:.4f}".format(cls.__name__, duration))
        return targets


    @classmethod
    def build_query(cls, world_state, templates, me):
        full_conditions = []
        clauses = []
        variables = OrderedConsistentSet()
        target = {}
        for fun, args in templates:
            full_args = []
            skip = False
            if fun.__name__ == "is_at" and args[0] == ME:
                target[ME] = me.name
            for arg in args:
                if arg == ME:
                    full_args.append(me)
                else:
                    variable = world_state.kb.variable(arg)
                    full_args.append(variable)
                    variables.append(world_state.kb.typed_variable(variable, world_state.kb.type("ConceptNode")))
            full_conditions.append(fun(world_state, *full_args))
        query = world_state.kb.And(*full_conditions)
        # rospy.logwarn("Built query:\n{:}".format(query))
        # rospy.logwarn("Built variables:\n{:}".format(variables))
        return query, variables, target


    @classmethod
    def find_instances(cls, world_state):
        # rospy.loginfo("GOL: Checking goal {:} for targets --".format(cls.__name__))
        targets = []
        start_time = time.time()
        templates = cls.get_condition_templates()
        me = world_state.kb.concept(world_state.me.capitalize())
        query, variables, target = cls.build_query(world_state, templates, me)
        # rospy.loginfo("GOL: Reason (has args):\n{:}".format(query))
        vars = world_state.kb.variable_list(*variables.items)
        # rospy.loginfo("GOL: Reason Variables:\n{:}".format(vars))
        results = world_state.kb.reason(world_state.kb.get(vars,query), vars)
        # rospy.logwarn("GOL: classic reasoning results:\n{}".format(results))
        for setlink in results.get_out():
            for listlink in setlink.get_out():
                # if result.tv == world_state.kb.TRUE:
                # (SetLink (ListLink (ConceptNode "Picker02") (ConceptNode "human") (ConceptNode "WayPoint106") (ConceptNode "WayPoint104")))
                this_target = copy(target)
                # rospy.logwarn(listlink)
                cls.get_targets(this_target, variables, listlink)
                rospy.logwarn("GOL: Found a target for goal: {}".format(cls.__name__))
                # rospy.logwarn(this_target)
                # world_state.kb.recursive_query_matcher(query, result, this_target)
                targets.append(this_target)
        duration = time.time() - start_time
        # if duration > 0.01:
        # rospy.loginfo("GOL: Checked goal {:} for targets -- {:.4f}".format(cls.__name__, duration))
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
                    if arg == ME:
                        new_args.append(world_state.kb.concept(world_state.me.capitalize()))
                    else:
                        new_args.append(world_state.kb.concept(arg))
                candidate = fun(world_state, *new_args)
                consequences.append(candidate)
            # rospy.logwarn("Consequences from is_achieved:\n{}".format(consequences))
            # query = world_state.kb.And(*consequences)
            query = consequences[0]
            consequence_count = len(consequences)
            if consequence_count > 1:
                query = world_state.kb.And(*consequences)
            elif consequence_count == 1:
                query = consequences[0]
            else:
                return True
            rospy.logdebug("GOL: Check:\n{:}".format(query))
            results = world_state.kb.reason(query, world_state.kb.variable_list())
            rospy.logdebug("------ is achieved? ------")
            rospy.logdebug(results)
            for listlink in results.get_out():
                if listlink.tv == world_state.kb.TRUE:
                    rospy.logwarn("pleasure achieved")
                    success = True
                else:
                    rospy.logdebug("no bueno")
                    success = False
            # if (results.tv == world_state.kb.TRUE):
            #     rospy.logdebug("pleasure achieved")
            #     success = True
            # else:
            #     rospy.logdebug("no bueno")
            #     success = False
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
        succeeded = action.perform()
        if not succeeded:
            # rospy.logwarn("GOL: Peforming action {:}; result: {}".format(action, succeeded))
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

import rospy

from actions import MoveAction, MoveToAction, GiveCrateAction
from utils import OrderedConsistentSet, suppress

class Goal(object):

    action_template = None
    subgoal_templates = []

    def __init__(self, world_state, args):
        self.subgoals = [] # list of subgoals (only if there is no action to be performed)
        self.world_state = world_state
        self.action = self.instantiate_action_template(world_state, args) # action to be performed (only if there are no subgoals)

        # self.guide = self.get_instance_guide()


    @classmethod
    def instantiate_action_template(cls, world_state, args):
        try:
            return cls.action_template(world_state, args)
        except TypeError:
            return None


    def get_instance_guide(self):
        if not self.action is None:
            for condition in self.action.conditions:
                start = condition.find(TARGET)
                if start != -1:
                    end = start + len(TARGET)
                    prefix = condition[:start]
                    postfix = condition[end:]
                    return (prefix, postfix)
        else:
            for subgoal in self.subgoals:
                guide = subgoal.get_instance_guide()
                if not guide is None:
                    return guide
            return None


    @classmethod
    def get_condition_templates(cls):
        try:
            return cls.action_template.condition_templates
        except AttributeError:
            try:
                return cls.condition_templates
            except AttributeError:
                cls.condition_templates = OrderedConsistentSet()
                consequences = OrderedConsistentSet()

                for subgoal in cls.subgoal_templates:
                    new_conditions = subgoal.get_condition_templates()
                    consequences += subgoal.get_consequence_templates()
                    # rospy.loginfo("new conditions:")
                    # rospy.loginfo(new_conditions)
                    # rospy.loginfo("new consequences:")
                    # rospy.loginfo(consequences)
                    for condition in new_conditions:
                        if not condition in consequences:
                            cls.condition_templates.append(condition)

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
                    cls.consequences += subgoal.get_consequences()
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
        query = ""
        for condition in cls.get_condition_templates():
            query += condition + " ^ "
        query = query[:-3]
        rospy.loginfo("BDI: asking MLN system; query: {:}".format(query))
        result = world_state.check(query)
        rospy.loginfo("BDI: received result for query: {:}".format(query))
        targets = []
        for key, value in result.results.items():
            if not value is None:
                rospy.loginfo("{:}: {:d}".format(key,value))
                #TODO: return constructor arguments

                # if not self.guide is None:
                #     start = key.find(self.guide[0]) + len(self.guide[0])
                #     end = key.find(self.guide[1])
                #     target = key[start:end]
                #     if not self.is_achieved(target):
                #         targets.append(target)
        return targets


    def is_achieved(self):
        for consequence in self.get_consequences():
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
        return self.get_action_queue()[0]


    def perform_action(self):
        action = self.get_action_queue().pop(0)
        action.perform(target)


    def get_cost(self, target):
        try:
            return self.cost
        except AttributeError:
            try:
                self.cost = self.action.get_cost(target)
            except AttributeError:
                self.cost = 0
                for subgoal in self.subgoals:
                    self.cost += subgoal.get_cost(target)
        return self.cost


    def get_gain(self, target):
        try:
            return self.gain
        except AttributeError:
            try:
                self.gain = self.action.gain
            except AttributeError:
                self.gain = 0
                for subgoal in self.subgoals:
                    self.gain += subgoal.get_gain(target)
        return self.gain



class MoveGoal(Goal):

    action_template = MoveAction

    def __init__(self, world_state, me, origin, destination):
        super(MoveGoal, self).__init__(world_state, [me, origin, destination])

class MoveTo(Goal):

    action_template = MoveToAction

    def __init__(self, world_state, me, target, origin, destination):
        super(MoveToGoal, self).__init__(world_state, [me, target, origin, destination])


class GiveCrateGoal(Goal):

    action_template = GiveCrateAction

    def __init__(self, world_state, me, picker):
        super(GiveCrateGoal, self).__init__(world_state, [me, picker])



class DeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,GiveCrateGoal]

    def __init__(self, world_state, me, picker, origin, destination):
        super(DeliverGoal, self).__init__(world_state, [me, picker, origin, destination])
        self.subgoals.append(MoveToGoal(world_state, me, picker, origin, destination))
        self.subgoals.append(GiveCrateGoal(world_state, me, picker))

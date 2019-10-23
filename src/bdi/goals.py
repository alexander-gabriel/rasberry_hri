import rospy

from actions import MoveAction, MoveToAction, GiveCrateAction, ExchangeCrateAction, Evade1Action, Evade2Action
from utils import OrderedConsistentSet, suppress

class Goal(object):

    action_template = None
    subgoal_templates = []

    def __init__(self, world_state, args):
        self.subgoals = [] # list of subgoals (only if there is no action to be performed)
        self.world_state = world_state
        self.action = self.instantiate_action_template(world_state, args) # action to be performed (only if there are no subgoals)


    @classmethod
    def instantiate_action_template(cls, world_state, args):
        try:
            return cls.action_template(world_state, args)
        except TypeError:
            return None


    @classmethod
    def get_condition_templates(cls):
        try:
            cls.guide = cls.action_template.get_instance_guide()
            return cls.action_template.condition_templates,
        except AttributeError:
            try:
                guide = cls.get_instance_guide()
                return cls.condition_templates, guide
            except AttributeError:
                cls.condition_templates = OrderedConsistentSet()
                consequences = OrderedConsistentSet()

                for subgoal in cls.subgoal_templates:
                    new_conditions, _ = subgoal.get_condition_templates()
                    consequences += subgoal.get_consequence_templates()
                    # rospy.loginfo("new conditions:")
                    # rospy.loginfo(new_conditions)
                    # rospy.loginfo("new consequences:")
                    # rospy.loginfo(consequences)
                    for condition in new_conditions:
                        if not condition in consequences:
                            cls.condition_templates.append(condition)

                return cls.condition_templates, guide


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
        placeholders = []
        for condition, placeholder in cls.get_condition_templates():
            query += condition + " ^ "
            placeholders.append(placeholder)
        query = [query[:-3].replace("me", world_state.me.capitalize())]
        rospy.loginfo("BDI: asking MLN system; query: {:}".format(query))
        prob, formula = world_state.check(query, placeholders)
        rospy.loginfo("BDI: received result for query: {:}".format(query))
        targets = []
        rospy.loginfo("{:f} {:}".format(prob, formula))
        rospy.loginfo(cls.get_instance_guide())
        for number in result:
            rospy.loginfo(number)
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

    def __init__(self, world_state, me, origin, destination, robco):
        super(MoveGoal, self).__init__(world_state, [me, origin, destination, robco])



class MoveTo(Goal):

    action_template = MoveToAction

    def __init__(self, world_state, me, target, origin, destination, robco):
        super(MoveToGoal, self).__init__(world_state, [me, target, origin, destination, robco])



class GiveCrateGoal(Goal):

    action_template = GiveCrateAction

    def __init__(self, world_state, me, picker):
        super(GiveCrateGoal, self).__init__(world_state, [me, picker])



class ExchangeCrateGoal(Goal):

    action_template = ExchangeCrateAction

    def __init__(self, world_state, me, picker):
        super(ExchangeCrateGoal, self).__init__(world_state, [me, picker])



class Evade1Goal(Goal):

    action_template = Evade1Action

    def __init__(self, world_state, me, picker, origin, destination, robco):
        super(Evade1Goal, self).__init__(world_state, [me, picker, origin, destination, robco])



class Evade2Goal(Goal):

    action_template = Evade2Action

    def __init__(self, world_state, me, picker, origin, destination, robco):
        super(Evade2Goal, self).__init__(world_state, [me, picker, origin, destination, robco])



class DeliverGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,GiveCrateGoal]

    def __init__(self, world_state, me, picker, origin, destination, robco):
        super(DeliverGoal, self).__init__(world_state, [me, picker, origin, destination])
        self.subgoals.append(MoveToGoal(world_state, me, picker, origin, destination, robco))
        self.subgoals.append(GiveCrateGoal(world_state, me, picker))
        self.subgoals.append(MoveToGoal(world_state, me, picker, destination, origin, robco))



class ExchangeGoal(Goal):

    action_template = None
    subgoal_templates = [MoveGoal,ExchangeCrateGoal]

    def __init__(self, world_state, me, picker, origin, destinatio, robcon):
        super(ExchangeGoal, self).__init__(world_state, [me, picker, origin, destination])
        self.subgoals.append(MoveToGoal(world_state, me, picker, origin, destination, robco))
        self.subgoals.append(ExchangeCrateGoal(world_state, me, picker))
        self.subgoals.append(MoveToGoal(world_state, me, picker, destination, origin, robco))

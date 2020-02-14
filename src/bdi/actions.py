from time import sleep
from utils import is_at, colocated, leads_to, has_crate, seen_picking, not_seen_picking, is_a, not_has_crate, not_same, free_path, approaching, standing, leaving

from opencog.type_constructors import *
import rospy


SPEED = rospy.get_param("robot_speed", 0.5) # m/s
MEAN_WAYPOINT_DISTANCE = 2.95 # m

class Action(object):

    placeholders = []
    gain = 0

    def __init__(self, world_state, args):
        self.instances = args
        rospy.logdebug("{:} has args: {}".format(self.__class__.__name__, args))
        self.conditions = []
        self.consequences = []
        self.world_state = world_state
        self.cost = 0
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
        rospy.loginfo("ACT: Performing: {:}".format(self))
        # for condition in self.conditions:
        #     if not condition in self.consequences:
        #         self.world_state.abandon_belief(condition)
        # for consequence in self.consequences:
        #     self.world_state.add_belief(consequence)


    def get_cost(self):
        return self.cost




class MoveAction(Action):

    condition_templates = [[is_at, ["me", "origin"]], [free_path, ["origin", "destination"]]]
    consequence_templates = [[is_at, ["me", "destination"]]]
    placeholders = ["me", "origin", "destination"] # in same order as constructor arguments
    gain = rospy.get_param("move_gain", 0)


    def __init__(self, world_state, robco, args):
        # [me, origin, destination]
        super(MoveAction, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args[2]
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))

    def perform(self):
        super(MoveAction, self).perform()
        result = self.robco.move_to(self.destination)
        return result.success


    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / SPEED


    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)



class MoveToAction(Action):

    condition_templates = [[is_at, ["me", "origin"]], [is_at, ["picker", "destination"]]]
    consequence_templates = [[is_at, ["me", "destination"]]]
    placeholders = ["me", "picker", "origin", "destination"] # in same order as constructor arguments
    gain = rospy.get_param("moveto_gain", 0)


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(MoveToAction, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args[3]
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))

    def perform(self):
        super(MoveToAction, self).perform()
        self.robco.move_to(self.destination)
        return self.robco.get_result().success


    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / SPEED


    def __repr__(self):
        return "<Action:Move to {:}>".format(self.destination)



class EvadeAction(Action):

    condition_templates = [[is_at, ["picker", "place1"]], [is_at, ["me", "origin"]], [leads_to, ["destination", "origin"]], [leads_to, ["origin", "place1"]], [approaching, ["picker"]], [has_crate, ["picker"]], [not_seen_picking, ["picker"]]]
    consequence_templates = [[is_at, ["me", "destination"]]]
    placeholders = ["me", "picker", "origin", "destination"] # in same order as constructor arguments
    gain = rospy.get_param("evade_gain", 200)


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(EvadeAction, self).__init__(world_state, args)
        self.robco = robco
        self.picker = args[1]
        self.destination = args[3]

    def perform(self):
        super(EvadeAction, self).perform()
        self.robco.move_to(self.destination)
        return self.robco.get_result().success

    def get_cost(self):
        return MEAN_WAYPOINT_DISTANCE / SPEED


    def __repr__(self):
        return "<Action:Evade {:} by moving to {:}>".format(self.picker, self.destination)



class GiveCrateAction(Action):

    condition_templates = [[not_seen_picking, ["picker"]], [not_has_crate, ["picker"]], [is_at, ["picker", "destination"]], [is_at, ["me", "destination"]], [standing, ["picker"]], [is_a, ["picker", "human"]]]
    consequence_templates = [[has_crate, ["picker"]]]
    placeholders = ["me", "picker", "destination"] # in same order as constructor arguments
    gain = rospy.get_param("give_gain", 240)
    cost = rospy.get_param("give_cost", 90)


    def __init__(self, world_state, robco, args):
        # [me, picker]
        super(GiveCrateAction, self).__init__(world_state, args)
        self.picker = args[1]
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))


    def get_cost(self):
        return self.cost


    def perform(self):
        super(GiveCrateAction, self).perform()
        for fun, args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(ConceptNode(arg))
            consequence = fun(*new_args)
            rospy.loginfo("Entering consequence: {}".format(consequence))
            consequence.truth_value(1,1)
        sleep(5)
        return True


    def __repr__(self):
        return "<Action:Give crate to {:}>".format(self.picker)



class ExchangeCrateAction(Action):

    condition_templates = [[seen_picking, ["picker"]], [has_crate, ["picker"]], [is_at, ["picker", "destination"]], [is_at, ["me", "destination"]], [standing, ["picker"]], [is_a, ["picker", "human"]]]
    consequence_templates = [[not_seen_picking, ["picker"]]]
    placeholders = ["me", "picker", "destination"] # in same order as constructor arguments
    gain = rospy.get_param("exchange_gain", 240)
    cost = rospy.get_param("exchange_cost", 120)


    def __init__(self, world_state, robco, args):
        # [me, picker, destination]
        super(ExchangeCrateAction, self).__init__(world_state, args)
        self.picker = args[1]

    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))


    def get_cost(self):
        return self.cost


    def perform(self):
        super(GiveCrateAction, self).perform()
        for fun,args in self.consequences:
            new_args = []
            for arg in args:
                new_args.append(ConceptNode(arg))
            consequence = fun(*new_args)
            consequence.truth_value(1,1)
        sleep(5)
        return True


    def __repr__(self):
        return "<Action:Exchange crate with {:}>".format(self.picker)

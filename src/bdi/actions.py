from time import sleep
import rospy

class Action(object):

    placeholders = []
    gain = 0

    def __init__(self, world_state, args):
        self.conditions = []
        self.consequences = []
        self.world_state = world_state
        self.cost = 0
        for index in range(len(args)):
            for condition in self.condition_templates:
                self.conditions.append(condition.replace("?{:}".format(self.placeholders[index]), args[index]))
            for consequence in self.consequence_templates:
                self.consequences.append(consequence.replace("?{:}".format(self.placeholders[index]), args[index]))


    def perform(self):
        rospy.loginfo("Performing action: {:}".format(self.__class__.__name__))
        # for condition in self.conditions:
        #     if not condition in self.consequences:
        #         self.world_state.abandon_belief(condition)
        # for consequence in self.consequences:
        #     self.world_state.add_belief(consequence)


    def get_cost(self):
        return self.cost




class MoveAction(Action):

    condition_templates = ["is_at(me,origin)"]
    consequence_templates = ["is_at(me,destination)"]
    placeholders = ["me", "origin", "destination"] # in same order as constructor arguments
    gain = 10


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
        rospy.loginfo("Result of Action move was {:}".format(result))
        sleep(5)


class MoveToAction(Action):

    condition_templates = ["is_at(me,origin)", "is_at(picker,destination)"]
    consequence_templates = ["is_at(me,destination)", "is_at(picker,destination)", "colocated(me,picker)", "colocated(picker,me)" ]
    placeholders = ["me", "picker", "origin", "destination"] # in same order as constructor arguments
    gain = 10


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



class Evade1Action(Action):

    condition_templates = ["is_at(human,place1)", "is_at(me,origin)", "leads_to(place1,origin)", "leads_to(origin,destination)", "!(has_crate(picker))", "seen_picking(picker)"]
    consequence_templates = ["is_at(me,destination)", "leads_to(place1,origin)", "leads_to(origin,destination)", "!(has_crate(picker))", "seen_picking(picker)"]
    placeholders = ["me", "picker", "origin", "destination"] # in same order as constructor arguments
    gain = 50


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(Evade1Action, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args[3]

    def perform(self):
        super(Evade1Action, self).perform()
        self.robco.move_to(self.destination)



class Evade2Action(Action):

    condition_templates = ["is_at(human,place1)", "is_at(me,origin)", "leads_to(place1,origin)", "leads_to(origin,destination)", "!(has_crate(picker))", "seen_picking(picker)", "is_a(picker,Human)"]
    consequence_templates = ["is_at(me,destination)", "leads_to(place1,origin)", "leads_to(origin,destination)", "!(seen_picking(picker))", "has_crate(picker)", "is_a(picker,Human)"]
    placeholders = ["me", "picker", "origin", "destination"] # in same order as constructor arguments
    gain = 50


    def __init__(self, world_state, robco, args):
        # [me, picker, origin, destination]
        super(Evade2Action, self).__init__(world_state, args)
        self.robco = robco
        self.destination = args[3]

    def perform(self):
        super(Evade2Action, self).perform()
        self.robco.move_to(self.destination)



class GiveCrateAction(Action):

    condition_templates = ["!(seen_picking(picker))", "!(has_crate(picker))", "is_at(picker, destination)", "is_at(me,destination)", "is_a(picker,Human)"]
    consequence_templates = ["has_crate(picker)", "is_at(picker, destination)", "is_at(me,destination)", "is_a(picker,Human)"]
    placeholders = ["me", "picker", "destination"] # in same order as constructor arguments
    gain = 100


    def __init__(self, world_state, robco, args):
        # [me, picker]
        super(GiveCrateAction, self).__init__(world_state, args)
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))


    def get_cost(self):
        return 5


    def perform(self):
        super(GiveCrateAction, self).perform()
        for condition in self.conditions:
            if not condition in self.consequences:
                self.world_state.abandon_belief(condition)
        for consequence in self.consequences:
            self.world_state.add_belief(consequence)



class ExchangeCrateAction(Action):

    condition_templates = ["seen_picking(picker)", "has_crate(picker)", "is_at(picker, destination)", "is_at(me,destination)", "is_a(picker,Human)"]
    consequence_templates = ["has_crate(picker)", "!(seen_picking(picker)", "is_at(picker, destination)", "is_at(me,destination)", "is_a(picker,Human)"]
    placeholders = ["me", "picker", "destination"] # in same order as constructor arguments
    gain = 100


    def __init__(self, world_state, robco, args):
        # [me, picker, destination]
        super(ExchangeCrateAction, self).__init__(world_state, args)
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))


    def get_cost(self):
        return 5


    def perform(self):
        super(ExchangeCrateAction, self).perform()
        for condition in self.conditions:
            if not condition in self.consequences:
                self.world_state.abandon_belief(condition)
        for consequence in self.consequences:
            self.world_state.add_belief(consequence)

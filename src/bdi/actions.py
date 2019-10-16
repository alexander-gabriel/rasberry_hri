
class Action:

    label = "Abstract"

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
        rospy.loginfo("Performing action: {:}".format(self.label))
        for condition in self.conditions:
            if not condition in self.consequences:
                self.world_state.abandon_belief(condition)
        # for consequence in self.consequences:
        #     self.world_state.add_belief(consequence)


    def get_cost(self):
        return self.cost




class MoveAction(Action):

    label = "Move"
    condition_templates = ["is_at(me,origin)"]
    consequence_templates = ["is_at(me,destination)"]
    placeholders = ["me", "origin", "destination"] # in same order as constructor arguments
    gain = 0


    def __init__(self, world_state, me, origin, destination, robco):
        super(MoveAction, self).__init__(world_state, [me, origin, destination])
        self.robco = robco
        self.destination = destination
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))

    def perform(self):
        super(Action, self).perform()
        self.robco.move_to(self.destination)


class MoveToAction(Action):

    label = "MoveTo"
    condition_templates = ["is_at(me,origin)", "is_at(target,destination)"]
    consequence_templates = ["is_at(me,destination)", "is_at(target,destination)", "colocated(me,target)", "colocated(target,me)" ]
    placeholders = ["me", "target", "origin", "destination"] # in same order as constructor arguments
    gain = 10


    def __init__(self, world_state, me, target, origin, destination, robco):
        super(MoveToAction, self).__init__(world_state, [me, target, origin, destination])
        self.robco = robco
        self.destination = destination
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))

    def perform(self):
        super(Action, self).perform()
        self.robco.move_to(self.destination)



class Evade1Action(Action):

    label = "Evade1"
    condition_templates = ["is_at(human,place1)", "is_at(robot,origin)", "leads_to(place1,origin)", "leads_to(origin,destination)" "!(has_crate(target))", "seen_picking(target)"]
    consequence_templates = ["is_at(me,destination)", "leads_to(place1,origin)", "leads_to(origin,destination)" "!(has_crate(target))", "seen_picking(target)"]
    placeholders = ["me", "target", "origin", "destination"] # in same order as constructor arguments
    gain = 10


    def __init__(self, world_state, me, target, origin, destination, robco):
        super(MoveToAction, self).__init__(world_state, [me, target, origin, destination])
        self.robco = robco
        self.destination = destination

    def perform(self):
        super(Action, self).perform()
        self.robco.move_to(self.destination)



class Evade2Action(Action):

    label = "Evade2"
    condition_templates = ["is_at(human,place1)", "is_at(robot,origin)", "leads_to(place1,origin)", "leads_to(origin,destination)" "!(has_crate(target))", "seen_picking(target)"]
    consequence_templates = ["is_at(me,destination)", "leads_to(place1,origin)", "leads_to(origin,destination)" "!(seen_picking(target))", "has_crate(target)"]
    placeholders = ["me", "target", "origin", "destination"] # in same order as constructor arguments
    gain = 10


    def __init__(self, world_state, me, target, origin, destination, robco):
        super(MoveToAction, self).__init__(world_state, [me, target, origin, destination])
        self.robco = robco
        self.destination = destination

    def perform(self):
        super(Action, self).perform()
        self.robco.move_to(self.destination)


class GiveCrateAction(Action):

    label = "Give Crate"
    condition_templates = ["colocated(me,target)", "!(seen_picking(target))", "!(has_crate(target))"]
    consequence_templates = ["colocated(me,target)", "has_crate(target)"]
    placeholders = ["me", "target"] # in same order as constructor arguments
    gain = 100


    def __init__(self, world_state, me, target):
        super(GiveCrateAction, self).__init__(world_state, [me, target])
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))

    def get_cost(self):
        return 5


class ExchangeCrateAction(Action):

    label = "Exchange Crate"
    condition_templates = ["colocated(me,target)", "seen_picking(target)", "has_crate(target)"]
    consequence_templates = ["colocated(me,target)", "has_crate(target)", "!(seen_picking(target)"]
    placeholders = ["me", "target"] # in same order as constructor arguments
    gain = 100


    def __init__(self, world_state, me, target):
        super(GiveCrateAction, self).__init__(world_state, [me, target])
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))

    def get_cost(self):
        return 5

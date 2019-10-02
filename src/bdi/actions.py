
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
        for consequence in self.consequences:
            self.world_state.add_belief(consequence)


    def get_cost(self):
        return self.cost



class MoveAction(Action):

    label = "Move"
    condition_templates = ["is_at(me,origin)"]
    consequence_templates = ["is_at(me,destination)"]
    placeholders = ["me", "origin", "destination"] # in same order as constructor arguments
    gain = 0


    def __init__(self, world_state, me, origin, destination):
        super(MoveAction, self).__init__(world_state, [me, origin, destination])
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))


class MoveToAction(Action):

    label = "MoveTo"
    condition_templates = ["is_at(me,origin)", "is_at(target,destination)"]
    consequence_templates = ["is_at(me,destination)", "is_at(target,destination)", "colocated(me,target)", "colocated(target,me)" ]
    placeholders = ["me", "target", "origin", "destination"] # in same order as constructor arguments
    gain = 10


    def __init__(self, world_state, me, target, origin, destination):
        super(MoveToAction, self).__init__(world_state, [me, target, origin, destination])
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))



class GiveCrateAction(Action):

    label = "Give Crate"
    condition_templates = ["colocated(me,target)", "!has_crate(target)", "has_requested_crate(target)"]
    consequence_templates = ["colocated(me,target)", "has_crate(target)", "!has_requested_crate(target)"]
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


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
    condition_templates = ["is_at(?me,?origin)"]
    consequence_templates = ["is_at(?me,?destination)"]
    placeholders = ["me", "origin", "destination"] # in same order as constructor arguments
    gain = 0


    def __init__(self, world_state, me, origin, destination):
        super(MoveAction, self).__init__(world_state, [me, origin, destination])
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?origin", origin).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?destination", destination).replace(ME, me))


class GiveCrateAction(Action):

    label = "Give Crate"
    condition_templates = ["sam(?me,?picker)", "!has_crate(?picker)", "has_requested_crate(?picker)"]
    consequence_templates = ["sam(?me,?picker)", "has_crate(?picker)", "!has_requested_crate(?picker)"]
    placeholders = ["me", "picker"] # in same order as constructor arguments
    gain = 100


    def __init__(self, world_state, me, picker):
        super(GiveCrateAction, self).__init__(world_state, [me, picker])
    #     for condition in self.condition_templates:
    #         self.conditions.append(condition.replace("?picker", picker).replace(ME, me))
    #     for consequence in self.consequence_templates:
    #         self.consequences.append(consequence.replace("?picker", picker).replace(ME, me))

    def get_cost(self):
        return 5

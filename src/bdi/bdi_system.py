
from utils import OrderedConsistentSet
from world_state import WorldState

MAX_COST = 1.0
MIN_GAIN = 1.0
TRUTH_THRESHOLD = 0.5
TARGET = "?target"
INF = float('inf')



class BDISystem:

    goals = []
    intentions = []


    def __init__(self, me):
        self.me = me
        self.world_state = WorldState()
        self.robot_track = []
        self.people_tracks = {}


    def generate_options(self):
        desires = []
        for goal in self.goals:
            targets = goal.find_instances()
            for target in targets:
                desires.append((goal, target))
        for intention in self.intentions:
            if intention in desires and intention[0].is_achieved(intention[1]):
                desires.remove(intention)
        return desires


    def filter(self, desires):
        intentions = []
        for desire in desires:
            gain = desire[0].get_gain(desire[1])
            cost = desire[0].get_cost(desire[1])
            if gain > MIN_GAIN and cost < MAX_COST:
                intentions.append(desire)
        return intentions


    def perform_action(self):
        chosen_intention = None
        min_cost = 999999
        for intention in self.intentions:
            action = intention[0].get_next_action()
            cost = action.get_cost(intention[1])
            if cost < min_cost:
                chosen_intention = intention
                min_cost  = cost
        if not chosen_intention is None:
            chosen_intention[0].perform_action(chosen_intention[1])


    def loop(self):
        self.update_beliefs()
        desires = self.generate_options()
        self.intentions = self.filter(desires)
        self.perform_action()


    def update_beliefs(self):
        world = World_Trace()
        robot_position = None
        self.robot_track.append(robot_position)
        world.add_object_state_series(self.robot_track)
        for person, position in self.latest_people_position.items():
            if not person in self.people_tracks:
                self.people_tracks[person] = [position]
            else:
                self.people_tracks[person].append(position)
            world.add_object_state_series(self.people_tracks[person])
        qsrlib_request_message = QSRlib_Request_Message(which_qsr="tpcc", input_data=world)
        # request your QSRs
        qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)

        pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message)

        #TODO: parse result and update beliefs


    def write(self):
        self.world_state.write()


    def save(self):
        self.world_state.save()


    def add_goal(self, goal):
        self.goals.append(goal)




class Action:

    conditions = []
    consequences = []
    label = ""
    gain = 1.5


    def perform(self, world_state, target):
        print(self.label)
        for consequence in self.consequences:
            world_state.db.add(consequence.replace(TARGET, target))


    def get_cost(self, target):
        return 0.0 # devise cost calculation



class Goal:

    subgoals = [] # list of subgoals (only if there is no action to be performed)
    action = None # action to be performed (only if there are no subgoals)
    instance = None


    def __init__(self, world_state, subgoals=[], action=None):
        self.world_state = world_state
        self.subgoals = subgoals
        self.action = action
        self.guide = self.get_instance_guide()


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


    def find_instances(self):
        query = ""
        for condition in self.get_conditions():
            query += condition + " ^ "
        query = query[:-3]
        result = self.world_state.check(query)
        targets = []
        for key, value in result.results.items():
            if not value is None:
                # print("{:}: {:d}".format(key,value))
                if not self.guide is None:
                    start = key.find(self.guide[0]) + len(self.guide[0])
                    end = key.find(self.guide[1])
                    target = key[start:end]
                    if not self.is_achieved(target):
                        targets.append(target)
        return targets


    def is_achieved(self, target):
        for consequence in self.get_consequences():
            truth = self.world_state.db.truth(consequence.replace(TARGET, target))
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


    def perform_action(self, target):
        action = self.get_action_queue().pop(0)
        action.perform(self.world_state, target)


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

from opencog.type_constructors import *

from bdi.goals import DeliverGoal, ExchangeGoal
from utils import not_called_robot, called_robot, seen_picking, not_seen_picking

TRUE = TruthValue(1,1)

class Config(object):
    picker_name = "Picker01"
    robot_name = "Robot01"


class Goal(Config):
    goal = None

    def set_facts(self):
        pass

class Deliver(Goal):
    goal = DeliverGoal
    log_name_prefix = "Deliver"

    def set_facts(self):
        not_called_robot(ConceptNode(self.picker_name)).tv = TRUE
        not_seen_picking(ConceptNode(self.picker_name)).tv = TRUE


class Exchange(Goal):
    goal = ExchangeGoal
    log_name_prefix = "Exchange"

    def set_facts(self):
        called_robot(ConceptNode(self.picker_name)).tv = TRUE
        seen_picking(ConceptNode(self.picker_name)).tv = TRUE




class Size(Config):
    size = 4
    log_name_postfix = "Size.log"



class Small(Size):
    size = 10
    log_name_postfix = "Small.log"



class Medium(Size):
    size = 100
    log_name_postfix = "Medium.log"



class Large(Size):
    size = 999
    log_name_postfix = "Large.log"




class Range(Config):
    picker_pos = 2
    robot_pos = 1
    log_name = "Range"



class Short(Range):
    picker_pos = 4
    log_name = "Short"
    iterations = "30"



class Middle(Range):
    picker_pos = 100
    log_name = "Middle"



class Long(Range):
    picker_pos = 1000
    log_name = "Long"

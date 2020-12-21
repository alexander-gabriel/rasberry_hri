# from threading import Lock
from os.path import join
from math import sqrt

# import time


import rospy
import rospkg

from opencog.type_constructors import (
    PredicateNode,
    ConceptNode,
    VariableNode,
    TypedVariableLink,
    FloatValue,
    TruthValue,
    AbsentLink,
    PresentLink,
    StateLink,
    ListLink,
    ExistsLink,
    AndLink,
    OrLink,
    NotLink,
    InheritanceLink,
    IdenticalLink,
    LinkValue,
    NumberNode,
    GreaterThanLink,
    ValueOfLink,
    SetValueLink,
    PlusLink,
    GetLink,
    EqualLink,
    EvaluationLink,
    VariableList,
    TypeNode,
    FalseLink,
    MinusLink,
)

# from opencog.utilities import initialize_opencog

from common.parameters import MINIMUM_DISTANCE, CRATE_CAPACITY, \
    TIMEOUT_LENGTH, NO_BERRY_PLACES, PICKER_DISTANCE_PREFERENCE

# from common.utils import atomspace

# from topological_navigation.tmap_utils import get_distance

# from bdi.knowledge_base import


# initialize_opencog(atsp)

rospack = rospkg.RosPack()
path = join(rospack.get_path("rasberry_hri"), "src", "bdi")


class WorldState(object):

    _size = PredicateNode("size")
    _intent = PredicateNode("intent")
    _position = PredicateNode("position")
    _berry_count = PredicateNode("berry count")
    _full_crate_count = PredicateNode("full crate count")
    _empty_crate_count = PredicateNode("empty crate count")
    _distance = PredicateNode("distance")
    _dismissed = PredicateNode("dismissed robot")
    _called = PredicateNode("called robot")
    _movement = PredicateNode("movement")
    _seen_picking = PredicateNode("seen_picking")
    _timeout = PredicateNode("timeout")

    def __init__(self, kb, me):
        self.CALLED_ROBOT = "CALLED_ROBOT"
        self.kb = kb
        self.me = me
        self.moving = False
        self.too_close = False

        # DefineLink(DefinedSchemaNode("NewName"), )

    # def set(self, link):
    #     return link
    #
    #
    def present(self, link):
        return PresentLink(link)

    def absent(self, link):
        return AbsentLink(link)

    # variance experiment
    def is_target(self, place):
        return self.state2(ConceptNode("target"), place)
    # variance experiment

    def state(self, concept, predicate, truth="TRUE"):
        # if not concept.type_name == "VariableNode":
        #     rospy.loginfo("KB: created StateLink(ListLink{:}, {:}), ConceptNode({:}))".format(concept, predicate, truth))
        return StateLink(ListLink(concept, predicate), ConceptNode(truth))

    def state2(self, concept, predicate):
        # if concept.type_name == "ConceptNode":
        #     rospy.loginfo("State of {:} changed to {:}".format(concept, predicate))
        # if not concept.type_name == "VariableNode":
        #     rospy.loginfo("KB: created StateLink({:}, {:})".format(concept, predicate))
        return StateLink(concept, predicate)

    def set_size(self, thing, width, length):
        thing.set_value(self._size, FloatValue([width, length]))

    def get_size(self, thing):
        return thing.get_value(self._size).to_list()

    def set_position(self, entity, x, y, date):
        try:
            positions = self.get_position(entity)
            positions.append(FloatValue([x, y, date]))
        except Exception:
            positions = [FloatValue([x, y, date])]
        try:
            position = LinkValue(positions[-10:])
        except Exception():
            position = LinkValue(positions)
        entity.set_value(self._position, position)

    def get_optimum_distance(self, person):
        try:
            distances = person.get_value(self._distance).to_list()
        except AttributeError:
            distances = [PICKER_DISTANCE_PREFERENCE]
        return sum(distances) / len(distances)

    def set_latest_distance(self, person, distance):
        try:
            distances = person.get_value(self._distance).to_list()
        except AttributeError:
            distances = []
        distances.append(distance)
        person.set_value(self._distance, FloatValue(distances))

    def false(self, thing):
        return FalseLink(thing)

    def update_position(self, person, place):
        self.is_at(person, place).tv = self.kb.TRUE
        if place.name in NO_BERRY_PLACES:
            rospy.loginfo("WST: Observation: {:} is at {:} (no berries)"
                          .format(person.name,place.name))
        else:
            rospy.loginfo("WST: Observation: {:} is at {:} (berries)"
                          .format(person.name,  place.name))

    def get_position(self, entity):
        return entity.get_value(self._position).to_list()

    def get_distance(self, thing1, thing2, only_x=False):
        p1x, p1y, _ = self.get_position(thing1)[-1].to_list()
        p2x, p2y, _ = self.get_position(thing2)[-1].to_list()
        dx = p1x - p2x
        dy = p1y - p2y
        dxs = dx * dx
        dys = dy * dy
        if only_x:
            try:
                w1, l1 = self.get_size(thing1)
                w2, l2 = self.get_size(thing2)
                return abs(dx) - 0.5 * (l1 + l2)
            except AttributeError:
                return abs(dx)
        else:
            try:
                w1, l1 = self.get_size(thing1)
                w2, l2 = self.get_size(thing2)
                if dxs > dys:
                    return sqrt(dxs + dys) - 0.5 * (l1 + l2)
                else:
                    return sqrt(dxs + dys) - 0.5 * (w1 + w2)
            except AttributeError:
                return sqrt(dxs + dys)

    def set_berry_state(self, place, ripe):
        place.set_value(self._berry_count, NumberNode(str(ripe)))

    def get_berry_state(self, place):
        return place.get_value(self._berry_count)

    def update_berry_state(self, place, ripe):
        old_ripe = self.get_berry_state(place)
        self.set_berry_state(place, old_ripe + ripe)

    def has_berries(self, place):
        has_berries = GreaterThanLink(
            ValueOfLink(place, self._berry_count), NumberNode("0")
        )
        return has_berries

    def not_has_berries(self, place):
        not_has_berries = EqualLink(
            ValueOfLink(place, self._berry_count), NumberNode("0")
        )
        return not_has_berries

    def has_crate(self, picker):
        return self.state(picker, PredicateNode("has crate"), "TRUE")

    def not_has_crate(self, picker):
        return self.state(picker, PredicateNode("has crate"), "FALSE")

    def robot_has_crate(self, robot, crate_type):
        # return self.state(robot, PredicateNode("has full crate"), "TRUE")
        has_crates = GreaterThanLink(
            ValueOfLink(robot, crate_type), NumberNode("0")
        )
        return has_crates

    def robot_not_has_crate(self, robot, crate_type):
        # return self.state(robot, PredicateNode("has full crate"), "TRUE")
        # has_no_crates = GreaterThanLink(
        #     NumberNode("1"), ValueOfLink(robot, crate_type)
        # )
        return NotLink(self.robot_has_crate(robot, crate_type))

    def robot_has_crate_capacity(self, robot, crate_type):
        # return self.state(robot, PredicateNode("has full crate"), "TRUE")
        has_crate_capacity = GreaterThanLink(
            NumberNode(str(CRATE_CAPACITY)), ValueOfLink(robot, crate_type)
        )
        return has_crate_capacity

    def robot_not_has_crate_capacity(self, robot, crate_type):
        # return self.state(robot, PredicateNode("has full crate"), "TRUE")
        return NotLink(self.robot_has_crate_capacity(robot, crate_type))

    # def not_has_full_crate(self, robot):
    #     return self.state(robot, PredicateNode("has full crate"), "FALSE")

    def robot_add_crate(self, robot, crate_type):
        add_crate = SetValueLink(
            robot,
            crate_type,
            PlusLink(NumberNode("1"), ValueOfLink(robot, crate_type)),
        )
        return add_crate

    def robot_add_crate2(self, robot, crate_type):
        value = robot.get_value(crate_type).to_list()[0]
        robot.set_value(crate_type, FloatValue(value+1))
        return robot

    def robot_remove_crate(self, robot, crate_type):
        value = robot.get_value(crate_type).to_list()[0]
        robot.set_value(crate_type, FloatValue(value-1))
        # remove_crate = SetValueLink(
        #     robot,
        #     crate_type,
        #     PlusLink(NumberNode("-1"), ValueOfLink(robot, crate_type)),
        # )
        return robot

    def robot_remove_crate2(self, robot, crate_type):
        value = robot.get_value(crate_type).to_list()[0]
        robot.set_value(crate_type, FloatValue(value-1))
        return robot

    def robot_set_crate_count(self, robot, crate_type, count):
        link = SetValueLink(robot, crate_type, count)
        return link

    def robot_set_crate_count2(self, robot, crate_type, count):
        robot.set_value(crate_type, count)
        link = SetValueLink(robot, crate_type, count)
        return link

    def robot_get_crate_count(self, robot, crate_type, result):
        link = EqualLink(result, ValueOfLink(robot, crate_type))
        return link

    def crate_full(self, picker):
        return self.state(picker, PredicateNode("crate is full"), "TRUE")

    def not_crate_full(self, picker):
        return self.state(picker, PredicateNode("crate is full"), "FALSE")

    def timeout_reset(self, picker):
        picker.set_value(self._timeout, NumberNode("-1"))

    def timeout_counter_increase(self, picker):
        timeout = picker.get_value(self._timeout).to_list()[0] + 1
        picker.set_value(self._timeout, NumberNode(str(timeout)))

    def timeout_not_reached(self, picker):
        timout_reached = GreaterThanLink(
            ValueOfLink(picker, self._timeout), NumberNode(str(TIMEOUT_LENGTH))
        )
        return timout_reached

    def wants_something(self, picker):
        return self.state2(
            ListLink(picker, self._intent), ConceptNode("something")
        )

    def wants_nothing(self, picker):
        return self.state2(
            ListLink(picker, self._intent), ConceptNode("nothing")
        )

    def wants_to_pass(self, picker):
        return self.state2(
            ListLink(picker, self._intent), ConceptNode("wants to pass")
        )

    def needs_help_soon(self, picker):
        return self.state2(
            ListLink(picker, self._intent), ConceptNode("needs help soon")
        )

    def wants_to_exchange_their_crate(self, picker):
        return self.state2(
            ListLink(picker, self._intent),
            ConceptNode("wants to exchange their crate"),
        )

    def wants_to_get_crate(self, picker):
        return self.state2(
            ListLink(picker, self._intent), ConceptNode("wants to get a crate")
        )

    # def wants_help_soon(self, picker):
    #     return self.state2(ListLink(picker, self._intent),
    #                        ConceptNode("unknown"))

    def full_crate_count(self, robot, crate_count):
        return self.state2(ListLink(robot, self._crate_count), crate_count)

    def is_at(self, thing, place):
        return self.state2(thing, place)

    def query_at(self, thing, place):
        # link = EqualLink(thing, GetLink(StateLink(VariableNode("x"), place)))
        return self.state2(thing, place)

    def query_not_at(self, thing, place):
        return self.absent(self.state2(thing, place))

    def is_occupied(self, place):
        someone = VariableNode("someone")
        link = ExistsLink(
            someone,
            AndLink(
                OrLink(
                    self.is_a(someone, ConceptNode("human")),
                    self.is_a(someone, ConceptNode("robot")),
                ),
                self.is_at(someone, place),
            ),
        )
        return link

    def is_not_occupied(self, place):
        link = NotLink(self.is_occupied(place))
        return link

    def is_a(self, thing, category):
        link = InheritanceLink(thing, category)
        return link

    def query_a(self, thing, category):
        link = IdenticalLink(
            category, GetLink(InheritanceLink(thing, VariableNode("x")))
        )
        # link = InheritanceLink(thing, category)
        return link

    def same(self, thing1, thing2):
        link = EqualLink(thing1, thing2)
        return link

    def not_same(self, thing1, thing2):
        link = NotLink(EqualLink(thing1, thing2))
        return link

    def colocated(self, thing1, thing2):
        link = EvaluationLink(
            PredicateNode("colocated"), ListLink(thing1, thing2)
        )
        return link

    def leads_to(self, origin, destination):
        link = EvaluationLink(
            PredicateNode("leads_to"), ListLink(origin, destination)
        )
        return link

    def linked(self, origin, destination):
        link = EvaluationLink(
            PredicateNode("linked"), ListLink(origin, destination)
        )
        return link

    def approaching(self, picker):
        return self.state(picker, self._movement, "APPROACHING")

    def not_approaching(self, picker):
        return self.absent(self.state(picker, self._movement, "APPROACHING"))

    def leaving(self, picker):
        return self.state(picker, self._movement, "LEAVING")

    def not_leaving(self, picker):
        return self.absent(self.state(picker, self._movement, "LEAVING"))

    def standing(self, picker):
        return self.state(picker, self._movement, "STANDING")

    def not_standing(self, picker):
        return self.absent(self.state(picker, self._movement, "STANDING"))

    def seen_picking(self, picker):
        return self.state(picker, self._seen_picking, "TRUE")

    def not_seen_picking(self, picker):
        return self.state(picker, self._seen_picking, "FALSE")

    def called_robot(self, picker):
        return self.state(picker, self._called, "TRUE")

    def not_called_robot(self, picker):
        return self.state(picker, self._called, "FALSE")

    def unknown_called_robot(self, picker):
        return self.absent(self.state(picker, self._called, "TRUE"))

    def dismissed_robot(self, picker):
        return self.state(picker, self._dismissed, "TRUE")

    def not_dismissed_robot(self, picker):
        return self.state(picker, self._dismissed, "FALSE")

    def unknown_dismissed_robot(self, picker):
        return self.absent(self.state(picker, self._dismissed, "TRUE"))

    # add a place ConceptNode to the KB
    def add_place(self, name, x, y, truth_value=None):
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        node1 = ConceptNode(name)
        self.set_position(node1, x, y, 0)
        node1.tv = truth_value
        link = self.is_a(node1, self.kb.place)
        link.tv = truth_value
        return node1

    # add two 'linked' places
    def add_place_link(self, place1, place2, truth_value=None):
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        p1 = ConceptNode(place1)
        p1.tv = truth_value
        self.is_a(p1, self.kb.place).tv = truth_value

        p2 = ConceptNode(place2)
        p2.tv = truth_value
        self.is_a(p2, self.kb.place).tv = truth_value
        # link = EvaluationLink(
        #     PredicateNode("leads_to"),
        #     ListLink(p1, p2))
        # link.tv = truth_value
        self.leads_to(p1, p2).tv = truth_value
        # self.leads_to(p2, p1).tv = truth_value
        EvaluationLink(
            PredicateNode("leads_to"), ListLink(p1, p2)
        ).tv = truth_value
        EvaluationLink(
            PredicateNode("linked"), ListLink(p1, p2)
        ).tv = truth_value
        rospy.logdebug(
            "WST: Adding place link: {:} to {:}".format(place1, place2)
        )

    # add arbitrary typed things to the KB
    def add_thing(self, name, klasse, truth_value=None):
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        node1 = ConceptNode(name)
        node1.tv = truth_value
        node2 = ConceptNode(klasse)
        node2.tv = truth_value
        link = self.is_a(node1, node2)
        link.tv = truth_value
        return node1

    def update_action(self, person, action):
        # _as = self.kb.queryspace
        person = ConceptNode(person)
        if action == "picking berries" or action == "picking_berries_right":
            self.seen_picking(person).tv = self.kb.TRUE
            rospy.loginfo(
                "WST: Observation: {:} is {:}".format(person.name, action)
            )
        elif action == "calling":
            self.called_robot(person).tv = self.kb.TRUE
            rospy.loginfo(
                "WST: Observation: {:} is {:}".format(person.name, action)
            )
        elif action == "gesture_cancel":
            self.dismissed_robot(ConceptNode(person)).tv = self.kb.TRUE
            rospy.loginfo(
                "WST: Observation: {:} is {:}".format(person.name, action)
            )
        elif action == "gesture_stop":
            pass
            # self.called_robot(ConceptNode(person)).tv = self.kb.TRUE
        elif action == "gesture_forward":
            pass
            # self.called_robot(ConceptNode(person)).tv = self.kb.TRUE
        elif action == "gesture_backward":
            pass
            # self.called_robot(ConceptNode(person)).tv = self.kb.TRUE
        elif action == "neutral" or action == "put_or_get_crate":
            pass
        else:
            rospy.logerr("WST: Observed unknown behaviour {}".format(action))
        # results = self.kb.reason(self.seen_picking(VariableNode("picker")),
        #                          VariableNode("picker"))

    def get_picker_locations(self):
        pickers = []
        picker = VariableNode("picker")
        location = VariableNode("location")
        variables = VariableList(
            TypedVariableLink(picker, TypeNode("ConceptNode")),
            TypedVariableLink(location, TypeNode("ConceptNode")),
        )
        query = AndLink(
            self.is_a(picker, ConceptNode("human")),
            self.is_at(picker, location),
        )
        results = self.kb.reason(GetLink(variables, query), variables)

        for list_link in results.get_out()[0].get_out():
            picker, location = list_link.get_out()
            pickers.append([picker, location])
        return pickers

    def get_location(self, target):
        location = VariableNode("location")
        variables = VariableList(
            TypedVariableLink(location, TypeNode("ConceptNode"))
        )
        query = self.is_at(target, location)
        results = self.kb.reason(GetLink(variables, query), variables)
        for concept_node in results.get_out()[0].get_out():
            return concept_node
        return None

from os.path import join
from math import sqrt
import traceback

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
    SetLink
)

from common.parameters import MINIMUM_DISTANCE, CRATE_CAPACITY, \
    TIMEOUT_LENGTH, NO_BERRY_PLACES, PICKER_DISTANCE_PREFERENCE, \
    BEHAVIOR_PERCEPTION, GESTURE_PERCEPTION, SIMPLE_MODE


rospack = rospkg.RosPack()
path = join(rospack.get_path("rasberry_hri"), "src", "bdi")


class WorldState(object):

    _size = PredicateNode("size")
    _intent = PredicateNode("intent")
    _position = PredicateNode("position")
    _berry_count = PredicateNode("berry count")
    _has_crate = PredicateNode("has crate")
    _crate_full = PredicateNode("crate is full")
    _full_crate_count = PredicateNode("full crate count")
    _empty_crate_count = PredicateNode("empty crate count")
    _distance = PredicateNode("distance")
    _dismissed = PredicateNode("dismissed robot")
    _called = PredicateNode("called robot")
    _movement = PredicateNode("movement")
    _seen_picking = PredicateNode("seen_picking")
    _timeout = PredicateNode("timeout")
    _approaching = "APPROACHING"
    _leaving = "LEAVING"
    _standing = "STANDING"

    def __init__(self, kb, me):
        self.CALLED_ROBOT = "CALLED_ROBOT"
        self.kb = kb
        self.me = me
        self.moving = False
        self.too_close = False

    def present(self, link):
        return PresentLink(link)

    def absent(self, link):
        return AbsentLink(link)

    # variance experiment
    def is_target(self, place):
        return self.state2(ConceptNode("target"), place)
    # variance experiment

    def state(self, concept, predicate, truth="TRUE"):
        statelink = StateLink(ListLink(concept, predicate), ConceptNode(truth))
        return statelink

    def state2(self, concept, predicate):
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
        return self.state(picker, self._has_crate, "TRUE")

    def not_has_crate(self, picker):
        return self.state(picker, self._has_crate, "FALSE")

    def robot_has_crate(self, robot, crate_type):
        has_crates = GreaterThanLink(
            ValueOfLink(robot, crate_type), NumberNode("0")
        )
        return has_crates

    def robot_not_has_crate(self, robot, crate_type):
        has_no_crates = EqualLink(
            ValueOfLink(robot, crate_type), NumberNode("0")
        )
        return has_no_crates

    def robot_has_crate_capacity(self, robot, crate_type):
        has_crate_capacity = GreaterThanLink(
            NumberNode(str(CRATE_CAPACITY)), ValueOfLink(robot, crate_type)
        )
        return has_crate_capacity

    def robot_not_has_crate_capacity(self, robot, crate_type):
        not_has_crate_capacity = EqualLink(ValueOfLink(robot, crate_type), NumberNode(str(CRATE_CAPACITY)))
        return not_has_crate_capacity

    def robot_add_crate(self, robot, crate_type):
        add_crate = SetValueLink(
            robot,
            crate_type,
            PlusLink(NumberNode("1"), ValueOfLink(robot, crate_type)),
        )
        return add_crate

    def robot_add_crate2(self, robot, crate_type):
        value = float(robot.get_value(crate_type).name)
        robot.set_value(crate_type, NumberNode(str(value+1)))
        return robot

    def robot_remove_crate(self, robot, crate_type):
        value = float(robot.get_value(crate_type).name)
        robot.set_value(crate_type, NumberNode(str(value-1)))
        return robot

    def robot_remove_crate2(self, robot, crate_type):
        value = float(robot.get_value(crate_type).name)
        robot.set_value(crate_type, NumberNode(str(value-1)))
        return robot

    def robot_set_crate_count(self, robot, crate_type, count):
        link = SetValueLink(robot, crate_type, count)
        return link

    def robot_set_crate_count2(self, robot, crate_type, count):
        robot.set_value(crate_type, count)
        return robot

    def robot_get_crate_count(self, robot, crate_type):
        try:
            return float(robot.get_value(crate_type).name)
        except AttributeError as err:
            rospy.logerr("WS: robot_get_crate_count AttributeError {}".format(err))
            return "unknown"
        except TypeError as err:
            rospy.logerr("WS: robot_get_crate_count TypeError {}".format(err))
            rospy.logwarn("WS: {}".format(robot.get_value(crate_type)))
            return "unknown"

    def crate_full(self, picker):
        return self.state(picker, self._crate_full, "TRUE")

    def not_crate_full(self, picker):
        return self.state(picker, self._crate_full, "FALSE")

    def timeout_reset(self, picker):
        picker.set_value(self._timeout, NumberNode("-1"))

    def timeout_counter_increase(self, picker):
        timeout = float(picker.get_value(self._timeout).name) + 1
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

    def full_crate_count(self, robot, crate_count):
        return self.state2(ListLink(robot, self._crate_count), crate_count)

    def is_at(self, thing, place):
        return self.state2(thing, place)

    def query_at(self, thing, place):
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
        return self.state(picker, self._movement, self._approaching)

    def not_approaching(self, picker):
        return self.absent(self.state(picker, self._movement, self._approaching))

    def is_approaching(self, picker):
        return self.is_movement(picker, self._approaching)


    def leaving(self, picker):
        return self.state(picker, self._movement, self._leaving)

    def not_leaving(self, picker):
        return self.absent(self.state(picker, self._movement, self._leaving))

    def is_leaving(self, picker):
        return self.is_movement(picker, self._leaving)

    def standing(self, picker):
        return self.state(picker, self._movement, self._standing)

    def is_standing(self, picker):
        return self.is_movement(picker, self._standing)

    def is_movement(self, picker, movement):
        # rospy.logwarn(self.kb.execute(MeetLink(StateLink(ListLink(picker, self._movement), VariableNode("approaching")))))
        result = self.kb.evaluate(EqualLink(SetLink(ConceptNode(movement)), GetLink(StateLink(ListLink(picker, self._movement), VariableNode("leaving")))))
        return result == self.kb.TRUE

    def not_standing(self, picker):
        return self.absent(self.state(picker, self._movement, self._standing))

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
        self.leads_to(p1, p2).tv = truth_value
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
        person = ConceptNode(person)
        if (action == "picking berries" or action == "picking berries right") and BEHAVIOR_PERCEPTION:
            self.seen_picking(person).tv = self.kb.TRUE
            rospy.loginfo(
                "WST: Observation: {:} is {:}".format(person.name, action)
            )
        elif action == "calling" and (GESTURE_PERCEPTION or SIMPLE_MODE):
            self.called_robot(person).tv = self.kb.TRUE
            rospy.loginfo(
                "WST: Observation: {:} is {:}".format(person.name, action)
            )
        elif action == "gesture cancel" and GESTURE_PERCEPTION:
            self.dismissed_robot(person).tv = self.kb.TRUE
            rospy.loginfo(
                "WST: Observation: {:} is {:}".format(person.name, action)
            )
        elif action == "gesture stop" and GESTURE_PERCEPTION:
            pass
            # self.called_robot(ConceptNode(person)).tv = self.kb.TRUE
        elif action == "gesture forward" and GESTURE_PERCEPTION:
            pass
            # self.called_robot(ConceptNode(person)).tv = self.kb.TRUE
        elif action == "gesture backward" and GESTURE_PERCEPTION:
            pass
            # self.called_robot(ConceptNode(person)).tv = self.kb.TRUE
        elif action == "neutral" or action == "put or get crate" and BEHAVIOR_PERCEPTION:
            pass
        else:
            rospy.logerr("WST: Did not observe gesture/behaviour {}".format(action))

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

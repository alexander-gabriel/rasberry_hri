from threading import Lock
from os.path import join
import time

import rospy
import rospkg

from parameters import *
from utils import suppress


rospack = rospkg.RosPack()
path = join(rospack.get_path('rasberry_hri'), 'src', 'bdi')




class WorldState(object):

    def __init__(self, me, kb):
        self.me = me
        self.kb = kb
        self.moving = False


    def is_at(self, thing, place, atsp=None):
        link = self.kb.state(thing, place, atsp=atsp)
        return link


    def query_at(self, thing, place, atsp=None):
        # link = self.kb.equal(thing, self.kb.get(self.kb.state(self.kb.variable("x"), place, atsp=atsp), atsp=atsp), atsp=atsp)
        link = self.kb.state(thing, place, atsp=atsp)
        return link

    def query_not_at(self, thing, place, atsp=None):
        link = self.kb.absent(self.kb.state(thing, place, atsp=atsp), atsp=atsp)
        return link


    def is_occupied(self, place, atsp=None):
        someone = self.kb.variable("someone", atsp=atsp)
        link = self.kb.exists(someone, self.kb.And(self.kb.Or(self.is_a(someone, self.kb.concept("human", atsp=atsp), atsp=atsp), self.is_a(someone, self.kb.concept("robot", atsp=atsp), atsp=atsp), atsp=atsp), self.is_at(someone, place, atsp=atsp), atsp=atsp), atsp=atsp)
        return link


    def is_not_occupied(self, place, atsp=None):
        link = self.kb.Not(self.is_occupied(place, atsp=atsp), atsp=atsp)
        return link


    def is_a(self, thing, category, atsp=None):
        link = self.kb.inheritance(thing, category, atsp=atsp)
        return link


    def query_a(self, thing, category, atsp=None):
        link = self.kb.identical(category, self.kb.get(self.kb.inheritance(thing, self.kb.variable("x"), atsp=atsp), atsp=atsp), atsp=atsp)
        # link = self.kb.inheritance(thing, category, atsp=atsp)
        return link


    def not_same(self, thing1, thing2, atsp=None):
        link = self.kb.Not(self.kb.identical(thing1, thing2, atsp=atsp), atsp=atsp)
        return link


    def colocated(self, thing1, thing2, atsp=None):
        link = self.kb.evaluation(self.kb.predicate("colocated", atsp=atsp), self.kb.list(thing1, thing2, atsp=atsp), atsp=atsp)
        return link


    def leads_to(self, origin, destination, atsp=None):
        link = self.kb.evaluation(
            self.kb.predicate("leads_to", atsp=atsp),
            self.kb.list(origin, destination, atsp=atsp), atsp=atsp)
        return link


    def linked(self, origin, destination, atsp=None):
        link = self.kb.evaluation(
            self.kb.predicate("linked", atsp=atsp),
            self.kb.list(origin, destination, atsp=atsp), atsp=atsp)
        return link


    def is_approaching(self, picker, atsp=None):
        link = self.kb.identical(picker, self.kb.get(self.kb.state(self.kb.variable("x", atsp=atsp), self.kb.concept("APPROACHING", atsp=atsp), atsp=atsp), atsp=atsp), atsp=atsp)
        # link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement", atsp=atsp), atsp=atsp), self.kb.concept("APPROACHING", atsp=atsp), atsp=atsp)
        return link


    def approaching(self, picker, atsp=None):
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement", atsp=atsp), atsp=atsp), self.kb.concept("APPROACHING", atsp=atsp), atsp=atsp)
        return link


    def is_leaving(self, picker, atsp=None):
        link = self.kb.identical(picker, self.kb.get(self.kb.state(self.kb.variable("x", atsp=atsp), self.kb.concept("LEAVING", atsp=atsp), atsp=atsp), atsp=atsp), atsp=atsp)
        # link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement", atsp=atsp), atsp=atsp), self.kb.concept("APPROACHING", atsp=atsp), atsp=atsp)
        return link


    def leaving(self, picker, atsp=None):
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement", atsp=atsp), atsp=atsp), self.kb.concept("LEAVING", atsp=atsp), atsp=atsp)
        return link


    def is_standing(self, picker, atsp=None):
        link = self.kb.identical(picker, self.kb.get(self.kb.state(self.kb.variable("x", atsp=atsp), self.kb.concept("STANDING", atsp=atsp), atsp=atsp), atsp=atsp), atsp=atsp)
        # link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement", atsp=atsp), atsp=atsp), self.kb.concept("APPROACHING", atsp=atsp), atsp=atsp)
        return link


    def standing(self, picker, atsp=None):
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement", atsp=atsp), atsp=atsp), self.kb.concept("STANDING", atsp=atsp), atsp=atsp)
        return link


    def seen_picking(self, picker, atsp=None):
        # link = self.kb.evaluation(self.kb.predicate("seen_picking", atsp=atsp), picker, atsp=atsp)
        link = self.kb.state(self.kb.list(picker, self.kb.predicate( "seen_picking", atsp=atsp), atsp=atsp), self.kb.concept("TRUE", atsp=atsp), atsp=atsp)
        return link


    def not_seen_picking(self, picker, atsp=None):
        # link = self.kb.Not(self.seen_picking(picker, atsp), atsp=atsp)
        link = self.kb.state(self.kb.list(picker, self.kb.predicate( "seen_picking", atsp=atsp), atsp=atsp), self.kb.concept("FALSE", atsp=atsp), atsp=atsp)
        return link


    def called_robot(self, picker, atsp=None):
        # link = self.kb.evaluation(self.kb.predicate("called_robot", atsp=atsp), picker, atsp=atsp)
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("called robot", atsp=atsp), atsp=atsp), self.kb.concept("TRUE", atsp=atsp), atsp=atsp)
        return link


    def not_called_robot(self, picker, atsp=None):
        # link = self.kb.Not(self.called_robot(picker, atsp), atsp=atsp)
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("called robot", atsp=atsp), atsp=atsp), self.kb.concept("FALSE", atsp=atsp), atsp=atsp)
        return link



    # add a place ConceptNode to the KB
    def add_place(self, name, truth_value=None):
        _as = self.kb.atomspace
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        node1 = self.kb.concept(name, atsp=_as)
        node1.tv = truth_value
        link = self.is_a(node1, self.kb.place, atsp=_as)
        link.tv = truth_value


    # add two 'linked' places
    def add_place_link(self, place1, place2, truth_value=None):
        _as = self.kb.atomspace
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        p1 = self.kb.concept(place1, atsp=_as)
        p1.tv = truth_value
        self.is_a(p1, self.kb.place, atsp=_as).tv = truth_value

        p2 = self.kb.concept(place2, atsp=_as)
        p2.tv = truth_value
        self.is_a(p2, self.kb.place, atsp=_as).tv = truth_value
        # link = self.kb.evaluation(
        #     self.kb.predicate("leads_to"),
        #     self.kb.list(p1, p2))
        # link.tv = truth_value
        self.kb.evaluation(
            self.kb.predicate("leads_to", atsp=_as),
            self.kb.list(p1, p2, atsp=_as), atsp=_as).tv = truth_value
        self.kb.evaluation(
            self.kb.predicate("linked", atsp=_as),
            self.kb.list(p1, p2, atsp=_as), atsp=_as).tv = truth_value
        rospy.logdebug("WST: Adding place link: {:} to {:}".format(place1, place2))


    # add arbitrary typed things to the KB
    def add_thing(self, name, klasse, truth_value=None):
        _as = self.kb.atomspace
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        node1 = self.kb.concept(name, atsp=_as)
        node1.tv = truth_value
        node2 = self.kb.concept(klasse, atsp=_as)
        node2.tv = truth_value
        link = self.is_a(node1, node2, atsp=_as)
        link.tv = truth_value


    def update_action(self, person, action):
        _as = self.kb.atomspace
        # _as = self.kb.queryspace
        person = self.kb.concept(person, atsp=_as)
        if action == "picking berries" or action == "picking_berries_right":
            self.seen_picking(person, atsp=_as).tv = self.kb.TRUE
            rospy.loginfo("WST: {:} was observed {:}".format(person.name, action))
        elif action == "calling":
            self.called_robot(person, atsp=_as).tv = self.kb.TRUE
            rospy.loginfo("WST: {:} was observed {:}".format(person.name, action))
        elif action == "gesture_cancel":
            pass
            # self.called_robot(self.kb.concept(person)).tv = self.kb.TRUE
        elif action == "gesture_stop":
            pass
            # self.called_robot(self.kb.concept(person)).tv = self.kb.TRUE
        elif action == "gesture_forward":
            pass
            # self.called_robot(self.kb.concept(person)).tv = self.kb.TRUE
        elif action == "gesture_backward":
            pass
            # self.called_robot(self.kb.concept(person)).tv = self.kb.TRUE
        elif action == "neutral" or action == "put_or_get_crate":
            pass
        else:
            rospy.logwarn("WST: Perceived unknown action {}".format(action))
        # results = self.kb.reason(self.seen_picking(self.kb.variable("picker")), self.kb.variable("picker"))


    def update_position(self, person, place):
        _as = self.kb.atomspace
        node1 = self.kb.concept(person, atsp=_as)
        node2 = self.kb.concept(place, atsp=_as)
        self.is_at(node1, node2).tv = self.kb.TRUE
        rospy.loginfo("WST: {:} is at {:}".format(person, place))


    def get_picker_locations(self):
        pickers = []
        picker = self.kb.variable("picker")
        location = self.kb.variable("location")
        variables = self.kb.variable_list(
            self.kb.typed_variable(picker, self.kb.type("ConceptNode")),
            self.kb.typed_variable(location, self.kb.type("ConceptNode")))
        query = self.kb.And(
            self.is_a(picker, self.kb.concept("human")),
            self.is_at(picker, location))
        results = self.kb.reason(self.kb.get(variables, query), variables)

        for listlink in results.get_out()[0].get_out():
            picker, location = listlink.get_out()
            pickers.append([picker, location])
        return pickers


    def get_location(self, target):
        location = self.kb.variable("location")
        variables = self.kb.variable_list(
            self.kb.typed_variable(location, self.kb.type("ConceptNode")))
        query = self.is_at(target, location)
        results = self.kb.reason(self.kb.get(variables, query), variables)
        for concept_node in results.get_out()[0].get_out():
            return concept_node
        return None

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


    def is_at(self, thing, place):
        link = self.kb.state(thing, place)
        return link


    def query_at(self, thing, place):
        # link = self.kb.equal(thing, self.kb.get(self.kb.state(self.kb.variable("x"), place)))
        link = self.kb.state(thing, place)
        return link

    def query_not_at(self, thing, place):
        link = self.kb.absent(self.kb.state(thing, place))
        return link


    def is_occupied(self, place):
        someone = self.kb.variable("someone")
        link = self.kb.exists(someone, self.kb.And(self.kb.Or(self.is_a(someone, self.kb.concept("human")), self.is_a(someone, self.kb.concept("robot"))), self.is_at(someone, place)))
        return link


    def is_not_occupied(self, place):
        link = self.kb.Not(self.is_occupied(place))
        return link


    def is_a(self, thing, category):
        link = self.kb.inheritance(thing, category)
        return link


    def query_a(self, thing, category):
        link = self.kb.identical(category, self.kb.get(self.kb.inheritance(thing, self.kb.variable("x"))))
        # link = self.kb.inheritance(thing, category)
        return link


    def not_same(self, thing1, thing2):
        link = self.kb.Not(self.kb.identical(thing1, thing2))
        return link


    def colocated(self, thing1, thing2):
        link = self.kb.evaluation(self.kb.predicate("colocated"), self.kb.list(thing1, thing2))
        return link


    def leads_to(self, origin, destination):
        link = self.kb.evaluation(
            self.kb.predicate("leads_to"),
            self.kb.list(origin, destination))
        return link


    def linked(self, origin, destination):
        link = self.kb.evaluation(
            self.kb.predicate("linked"),
            self.kb.list(origin, destination))
        return link


    def is_approaching(self, picker):
        link = self.kb.identical(picker, self.kb.get(self.kb.state(self.kb.variable("x"), self.kb.concept("APPROACHING"))))
        # link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement")), self.kb.concept("APPROACHING"))
        return link


    def approaching(self, picker):
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement")), self.kb.concept("APPROACHING"))
        return link


    def is_leaving(self, picker):
        link = self.kb.identical(picker, self.kb.get(self.kb.state(self.kb.variable("x"), self.kb.concept("LEAVING"))))
        # link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement")), self.kb.concept("APPROACHING"))
        return link


    def leaving(self, picker):
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement")), self.kb.concept("LEAVING"))
        return link


    def is_standing(self, picker):
        link = self.kb.identical(picker, self.kb.get(self.kb.state(self.kb.variable("x"), self.kb.concept("STANDING"))))
        # link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement")), self.kb.concept("APPROACHING"))
        return link


    def standing(self, picker):
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("movement")), self.kb.concept("STANDING"))
        return link


    def seen_picking(self, picker):
        # link = self.kb.evaluation(self.kb.predicate("seen_picking"), picker)
        link = self.kb.state(self.kb.list(picker, self.kb.predicate( "seen_picking")), self.kb.concept("TRUE"))
        return link


    def not_seen_picking(self, picker):
        # link = self.kb.Not(self.seen_picking(picker, atsp))
        link = self.kb.state(self.kb.list(picker, self.kb.predicate( "seen_picking")), self.kb.concept("FALSE"))
        return link


    def called_robot(self, picker):
        # link = self.kb.evaluation(self.kb.predicate("called_robot"), picker)
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("called robot")), self.kb.concept("TRUE"))
        return link


    def not_called_robot(self, picker):
        # link = self.kb.Not(self.called_robot(picker, atsp))
        link = self.kb.state(self.kb.list(picker, self.kb.predicate("called robot")), self.kb.concept("FALSE"))
        return link



    # add a place ConceptNode to the KB
    def add_place(self, name, truth_value=None):
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        node1 = self.kb.concept(name)
        node1.tv = truth_value
        link = self.is_a(node1, self.kb.place)
        link.tv = truth_value


    # add two 'linked' places
    def add_place_link(self, place1, place2, truth_value=None):
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        p1 = self.kb.concept(place1)
        p1.tv = truth_value
        self.is_a(p1, self.kb.place).tv = truth_value

        p2 = self.kb.concept(place2)
        p2.tv = truth_value
        self.is_a(p2, self.kb.place).tv = truth_value
        # link = self.kb.evaluation(
        #     self.kb.predicate("leads_to"),
        #     self.kb.list(p1, p2))
        # link.tv = truth_value
        self.kb.evaluation(
            self.kb.predicate("leads_to"),
            self.kb.list(p1, p2)).tv = truth_value
        self.kb.evaluation(
            self.kb.predicate("linked"),
            self.kb.list(p1, p2)).tv = truth_value
        rospy.logdebug("WST: Adding place link: {:} to {:}".format(place1, place2))


    # add arbitrary typed things to the KB
    def add_thing(self, name, klasse, truth_value=None):
        truth_value = self.kb.TRUE if truth_value is None else truth_value
        node1 = self.kb.concept(name)
        node1.tv = truth_value
        node2 = self.kb.concept(klasse)
        node2.tv = truth_value
        link = self.is_a(node1, node2)
        link.tv = truth_value


    def update_action(self, person, action):
        # _as = self.kb.queryspace
        person = self.kb.concept(person)
        if action == "picking berries" or action == "picking_berries_right":
            self.seen_picking(person).tv = self.kb.TRUE
            rospy.loginfo("WST: {:} was observed {:}".format(person.name, action))
        elif action == "calling":
            self.called_robot(person).tv = self.kb.TRUE
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
        node1 = self.kb.concept(person)
        node2 = self.kb.concept(place)
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

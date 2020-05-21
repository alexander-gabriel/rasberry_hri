
from opencog.type_constructors import *

import rospy
# Note: this is how you query for the state of a specific ConceptNode
# DefineLink(
#     DefinedPredicateNode("is in state"),
#     LambdaLink(
#          VariableList(VariableNode("target"), VariableNode("state")),
#          EqualLink(SetLink(VariableNode("state")), GetLink(VariableNode("anystate"), StateLink(VariableNode("target"), VariableNode("anystate"))))))
#
#
# def has_crate(human):
#     return EvaluationLink(DefinedPredicateNode("is in state"),
#          ListLink(human, ConceptNode("has crate"))))



class IntentionRecognition(object):

    def __init__(self, world_state):
        has_crate = PredicateNode("has crate")
        has_berries = PredicateNode("has berries")
        crate_full = PredicateNode("crate is full")
        movement = PredicateNode("movement")
        called_robot = PredicateNode("called robot")
        false = "FALSE"
        true = "TRUE"
        approaching = "APPROACHING"
        self.ws = world_state
        self.picker = VariableNode("picker")
        place = VariableNode("place")
        concept = TypeNode("ConceptNode")
        self.variables1 = TypedVariableLink(self.picker, concept)
        self.variables2 = VariableList(TypedVariableLink(self.picker, concept), TypedVariableLink(place, concept))
        # TODO: write functions that can check these things with ConceptNode input
        getting_crate = lambda picker : GetLink(
                                            self.variables1,
                                            PresentLink(
                                                picker,
                                                self.ws.state(picker, has_crate, false),
                                                self.ws.state(picker, movement, approaching)))

        exchanging_crate = lambda picker : GetLink(
                                                self.variables2,
                                                AndLink(PresentLink(picker),
                                                        self.ws.is_a(picker, ConceptNode("human")),
                                                        self.ws.state(picker, has_crate, true),
                                                        self.ws.state(picker, movement, approaching),
                                                        OrLink(
                                                            PresentLink(
                                                                self.ws.state(picker, called_robot, false),
                                                                self.ws.state(picker, crate_full, true)),
                                                            PresentLink(
                                                                self.ws.state(picker, called_robot, false),
                                                                self.ws.state(picker, crate_full, false),
                                                                self.ws.state(place, has_berries, true),
                                                                self.ws.state2(picker, place)
                                                            ),
                                                            PresentLink(self.ws.state(picker, called_robot, true))
                                                        )
                                                )
                                            )

        passing_us = lambda picker : GetLink(
                                        self.variables2,
                                        PresentLink(picker,
                                                self.ws.state(picker, has_crate, true),
                                                self.ws.state(picker, crate_full, false),
                                                self.ws.state(place, has_berries, false),
                                                self.ws.is_at(picker, place),
                                                self.ws.state(picker, called_robot, false),
                                                self.ws.state(picker, movement, approaching)
                                        ))

        no_help_needed = lambda picker : GetLink(
                                                self.variables1,
                                                AndLink(PresentLink(picker,
                                                            self.ws.state(picker, has_crate, true),
                                                            self.ws.state(picker, crate_full, false),
                                                            self.ws.state(picker, called_robot, false)),
                                                        AbsentLink(self.ws.state(picker, movement, approaching))))

        needs_help_soon = lambda picker : GetLink(
                                            self.variables1,
                                            AndLink(
                                                PresentLink(picker),
                                                AbsentLink(self.ws.state(picker, movement, approaching)),
                                                OrLink(
                                                        PresentLink(self.ws.state(picker, called_robot, true)),
                                                        PresentLink(self.ws.state(picker, called_robot, false),
                                                                    self.ws.state(picker, has_crate, false)),
                                                        # PresentLink(self.ws.state(picker, called_robot, false),
                                                        #             self.ws.state(picker, has_crate, true),
                                                        #             self.ws.state(picker, crate_full, true))
                                                )
                                            )
                                        )

        self.human_intention_templates = [(self.ws.wants_to_get_crate, getting_crate), (self.ws.wants_to_exchange_their_crate, exchanging_crate), (self.ws.wants_to_pass, passing_us), (self.ws.wants_nothing, no_help_needed), (self.ws.wants_help_soon, needs_help_soon)]

        # self.human_intention_templates = [(self.ws.wants_to_get_crate, getting_crate)] # works without place
        # self.human_intention_templates = [(self.ws.wants_to_exchange_their_crate, exchanging_crate)]
        # self.human_intention_templates = [(self.ws.wants_to_pass, passing_us)] # works when also asking for place
        # self.human_intention_templates = [(self.ws.wants_nothing, no_help_needed)] # works
        # self.human_intention_templates = [(self.ws.wants_help_soon, needs_help_soon)]


    def run_untargeted(self):
        for intention, query in self.human_intention_templates:
            results = self.ws.kb.execute(query(self.picker))
            pickers = []
            if intention in [self.ws.wants_to_pass, self.ws.wants_to_exchange_their_crate]:
                try:
                    pickers = results.get_out()
                    for listlink in pickers:
                        picker = listlink.get_out()[0]
                        i = intention(picker)
                        i.tv = self.ws.kb.TRUE
                        # print(i)
                except IndexError:
                    pass
            else:
                try:
                    pickers = results.get_out()
                    for picker in pickers:
                        i = intention(picker)
                        i.tv  = self.ws.kb.TRUE
                        # print(i)
                except IndexError:
                    pass


    def run_targeted(self): # this doesn't work yet. the queries are tailored to untargeted search
        for human in self.humans: # humans aren't defined yet
            for intention, query in self.human_intention_templates:
                result = self.ws.kb.evaluate(query(human))

                # TODO: check result
                # if result = True, set the respective intention
                if True:
                    intention(human).tv = self.ws.kb.TRUE

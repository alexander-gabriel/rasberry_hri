from opencog.type_constructors import (
    PredicateNode,
    VariableNode,
    ConceptNode,
    TypeNode,
    TypedVariableLink,
    GetLink,
    AndLink,
    AbsentLink,
    PresentLink,
    VariableList,
    ChoiceLink,
)

import rospy

# Note: this is how you query for the state of a specific ConceptNode
# DefineLink(
#     DefinedPredicateNode("is in state"),
#     LambdaLink(
#          VariableList(VariableNode("target"), VariableNode("state")),
#          EqualLink(SetLink(VariableNode("state")), GetLink(VariableNode("anystate"), StateLink(VariableNode("target"), VariableNode("anystate"))))))


class IntentionRecognition(object):
    def __init__(self, world_state):
        self.ws = world_state
        has_crate = self.ws._has_crate
        dismissed = self.ws._dismissed
        crate_full = self.ws._crate_full
        movement = self.ws._movement
        called_robot = self.ws._called
        false = "FALSE"
        true = "TRUE"
        approaching = self.ws._approaching
        self.intentions = set()
        self.picker = VariableNode("picker")
        self.place = VariableNode("place")
        concept = TypeNode("ConceptNode")
        human = ConceptNode("human")
        self.variables1 = TypedVariableLink(self.picker, concept)
        self.variables2 = VariableList(
            TypedVariableLink(self.picker, concept),
            TypedVariableLink(self.place, concept),
        )
        # TODO: write functions that can check these things with ConceptNode input
        self.getting_crate = lambda picker: GetLink(
            self.variables1,
            AndLink(
                PresentLink(
                    picker,
                    self.ws.is_a(picker, human),
                    self.ws.state(picker, has_crate, false),
                    self.ws.state(picker, dismissed, false)
                ),
                ChoiceLink(
                    self.ws.state(picker, movement, approaching),
                    self.ws.state(picker, called_robot, true)
                )
            )
        )

        self.getting_crate_soon = lambda picker: GetLink(
            self.variables1,
            AndLink(
                AbsentLink(self.ws.state(picker, movement, approaching)),
                PresentLink(
                    picker,
                    self.ws.is_a(picker, human),
                    self.ws.state(picker, has_crate, false)
                ),
                AbsentLink(self.ws.state(picker, called_robot, true))
            ),
        )

        self.exchanging_crate1 = lambda picker: GetLink(
            self.variables1,
            AndLink(
                PresentLink(picker),
                self.ws.is_a(picker, human),
                self.ws.state(picker, has_crate, true),
                PresentLink(self.ws.state(picker, called_robot, true))
            )
        )

        self.exchanging_crate2 = lambda picker: GetLink(
            self.variables2,
            AndLink(
                PresentLink(picker),
                self.ws.is_a(picker, human),
                self.ws.state(picker, has_crate, true),
                self.ws.state(picker, movement, approaching),
                self.ws.is_at(picker, self.place),
                self.ws.is_a(self.place, ConceptNode("place")),
                ChoiceLink(
                    self.ws.has_berries(self.place),
                    PresentLink(self.ws.state(picker, crate_full, true))
                )
            )
        )

        self.exchanging_crate_soon = lambda picker: GetLink(
            self.variables1,
            AndLink(
                AbsentLink(self.ws.state(picker, movement, approaching)),
                PresentLink(
                    picker,
                    self.ws.is_a(picker, human),
                    self.ws.state(picker, has_crate, true),
                    self.ws.state(picker, crate_full, true)
                ),
                AbsentLink(self.ws.state(picker, called_robot, true))
            ),
        )

        self.passing_us = lambda picker: GetLink(
            self.variables2,
            AndLink(
                self.ws.is_a(picker, human),
                self.ws.is_a(self.place, ConceptNode("place")),
                self.ws.not_has_berries(self.place),
                PresentLink(
                    picker,
                    self.ws.state(picker, has_crate, true),
                    self.ws.is_at(picker, self.place),
                    self.ws.state(picker, movement, approaching),
                ),
                AbsentLink(self.ws.state(picker, called_robot, true)),
                AbsentLink(self.ws.state(picker, crate_full, true))
            ),
        )

        self.no_help_needed = lambda picker: GetLink(
            self.variables1,
            AndLink(
                self.ws.is_a(picker, human),
                PresentLink(
                    picker,
                    self.ws.state(picker, has_crate, true),
                    self.ws.state(picker, crate_full, false)
                ),
                AbsentLink(self.ws.state(picker, called_robot, true)),
                AbsentLink(self.ws.state(picker, movement, approaching)),
            ),
        )

        # self.needs_help_soon = lambda picker: GetLink(self.variables1,
        #     AndLink(
        #         PresentLink(picker),
        #         self.ws.is_a(picker, human),
        #         AbsentLink(self.ws.state(picker, movement, approaching)),
        #         ChoiceLink(
        #             PresentLink(self.ws.state(picker, called_robot, true)),
        #             PresentLink(
        #                 self.ws.state(picker, called_robot, false),
        #                 self.ws.state(picker, has_crate, false)
        #                 ),
        #             PresentLink(
        #                 self.ws.state(picker, called_robot, false),
        #                 self.ws.state(picker, has_crate, true),
        #                 self.ws.state(picker, crate_full, true)
        #                 )
        #             )
        #         )
        #     )

        self.human_intention_templates = [
            (self.getting_crate, self.ws.wants_to_get_crate),
            (self.getting_crate_soon, self.ws.needs_help_soon),
            (self.exchanging_crate_soon, self.ws.needs_help_soon),
            # (self.needs_help_soon, self.ws.needs_help_soon),
            (self.exchanging_crate1, self.ws.wants_to_exchange_their_crate),
            (self.exchanging_crate2, self.ws.wants_to_exchange_their_crate),
            (self.passing_us, self.ws.wants_to_pass),
            (self.no_help_needed, self.ws.wants_nothing),
        ]

    def run_untargeted(self):
        intentions = set()
        for query, intention in self.human_intention_templates:
            results = self.ws.kb.execute(query(self.picker))
            pickers = []
            if query in [self.passing_us, self.exchanging_crate2]:
                try:
                    pickers = results.get_out()
                    for listlink in pickers:
                        # print(listlink)
                        picker = listlink.get_out()[0]
                        i = intention(picker)
                        i.tv = self.ws.kb.TRUE
                        intentions.add((picker.name, intention.__name__))
                        # rospy.logwarn("{:} intention: {:}".format(
                        #                 picker.name, intention.__name__))
                except IndexError:
                    pass
            else:
                try:
                    pickers = results.get_out()
                    for picker in pickers:
                        i = intention(picker)
                        i.tv = self.ws.kb.TRUE
                        # rospy.logwarn("{:} intention: {:}".format(
                        #                 picker.name, intention.__name__))
                        intentions.add((picker.name, intention.__name__))
                except IndexError:
                    pass
        if self.intentions ^ intentions:
            rospy.loginfo("IRE: Possible intentions: {}".format(intentions))
            self.intentions = intentions

    def run_targeted(
        self,
    ):  # this doesn't work yet. the queries are tailored to untargeted search
        for human in self.humans:  # humans aren't defined yet
            for intention, query in self.human_intention_templates:
                result = self.ws.kb.evaluate(query(human))
                # TODO: check result
                # if result = True, set the respective intention
                if True:
                    intention(human).tv = self.ws.kb.TRUE

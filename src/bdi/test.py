from opencog.atomspace import AtomSpace, types, TruthValue
from opencog.type_constructors import *
from opencog.scheme_wrapper import scheme_eval
from opencog.utilities import initialize_opencog
from opencog.ure import BackwardChainer, ForwardChainer
from opencog.bindlink import execute_atom, evaluate_atom

atomspace = AtomSpace()
initialize_opencog(atomspace)
set_type_ctor_atomspace(atomspace)

TRUE = TruthValue(1,1)
FALSE = TruthValue(0,1)


def build_ontology():
    thing = ConceptNode("thing").truth_value(1.0, 1.0)

    concept = ConceptNode("concept").truth_value(1.0, 1.0)
    InheritanceLink(concept, thing).truth_value(1.0, 1.0)

    place = ConceptNode("place").truth_value(1.0, 1.0)
    InheritanceLink(place, concept).truth_value(1.0, 1.0)


    entity = ConceptNode("entity").truth_value(1.0, 1.0)
    InheritanceLink(entity, thing).truth_value(1.0, 1.0)


    obj = ConceptNode("object").truth_value(1.0, 1.0)
    organism = ConceptNode("organism").truth_value(1.0, 1.0)
    InheritanceLink(organism, entity).truth_value(1.0, 1.0)
    InheritanceLink(obj, entity).truth_value(1.0, 1.0)

    plant = ConceptNode("plant").truth_value(1.0, 1.0)
    InheritanceLink(plant, organism).truth_value(1.0, 1.0)

    strawberryplant = ConceptNode("strawberryplant").truth_value(1.0, 1.0)
    InheritanceLink(strawberryplant, plant).truth_value(1.0, 1.0)

    creature = ConceptNode("creature").truth_value(1.0, 1.0)
    robot = ConceptNode("robot").truth_value(1.0, 1.0)
    human = ConceptNode("human").truth_value(1.0, 1.0)
    InheritanceLink(creature, organism).truth_value(1.0, 1.0)
    InheritanceLink(robot, creature).truth_value(1.0, 1.0)
    InheritanceLink(human, creature).truth_value(1.0, 1.0)

    crate = ConceptNode("crate").truth_value(1.0, 1.0)
    InheritanceLink(crate, obj).truth_value(1.0, 1.0)



    #
    # ForAllLink(
    #     VariableList(p1, p2, p3),
    #     ImplicationLink(
    #         AndLink(
    #             NotLink(
    #                 IdenticalLink(p1, p3)),
    #             EvaluationLink(
    #                 PredicateNode("linked"),
    #                 ListLink(p1, p2)),
    #             EvaluationLink(
    #                 PredicateNode("linked"),
    #                 ListLink(p2, p3))),
    #         EvaluationLink(
    #             PredicateNode("linked"),
    #             ListLink(p1, p3)).truth_value(1.0, 1.0)))

def deduction_formula(AC, AB, BC):
    tv1 = AB.tv
    tv2 = BC.tv

    if tv1.mean > 0.5 and tv2.mean > 0.5 and tv1.confidence > 0.5 and tv2.confidence > 0.5:
        AC.tv = TruthValue(1, 1)
    else:
        AC.tv = TruthValue(0, 0)

    return AC


def build_deduction_rulebase():
    deduction_rbs = ConceptNode("deduction-rule-base")
    InheritanceLink(
        deduction_rbs,
        ConceptNode("PLN"))
    execute_code = \
    '''
    (use-modules (opencog rule-engine))
    (ure-set-num-parameter (ConceptNode "deduction-rule-base") "URE:maximum-iterations" 30)
    '''
    scheme_eval(atomspace, execute_code)
    return deduction_rbs



def build_inheritance_deduction(deduction_rbs):
    deduction_rule = BindLink(
        VariableList(
            TypedVariableLink(
                VariableNode('$A'),
                TypeNode('ConceptNode')),
            TypedVariableLink(
                VariableNode('$B'),
                TypeNode('ConceptNode')),
            TypedVariableLink(
                VariableNode('$C'),
                TypeNode('ConceptNode'))),
        AndLink(
            InheritanceLink(
                VariableNode('$A'),
                VariableNode('$B')),
            InheritanceLink(
                VariableNode('$B'),
                VariableNode('$C')),
            NotLink(
                EqualLink(
                    VariableNode('$A'),
                    VariableNode('$C')))),
        ExecutionOutputLink(
            GroundedSchemaNode('py: deduction_formula'),
            ListLink(
                InheritanceLink(
                    VariableNode('$A'),
                    VariableNode('$C')),
                InheritanceLink(
                    VariableNode('$A'),
                    VariableNode('$B')),
                InheritanceLink(
                    VariableNode('$B'),
                    VariableNode('$C')))))
    deduction_rule_name = DefinedSchemaNode("inheritance-deduction-rule")
    DefineLink(
        deduction_rule_name,
        deduction_rule)
    MemberLink(deduction_rule_name, deduction_rbs)


def build_implication_deduction(deduction_rbs):
    deduction_rule = BindLink(
        VariableList(
            TypedVariableLink(
                VariableNode('$A'),
                TypeNode('ConceptNode')),
            TypedVariableLink(
                VariableNode('$B'),
                TypeNode('ConceptNode')),
            TypedVariableLink(
                VariableNode('$C'),
                TypeNode('ConceptNode'))),
        AndLink(
            ImplicationLink(
                VariableNode('$A'),
                VariableNode('$B')),
            ImplicationLink(
                VariableNode('$B'),
                VariableNode('$C')),
            NotLink(
                EqualLink(
                    VariableNode('$A'),
                    VariableNode('$C')))),
        ExecutionOutputLink(
            GroundedSchemaNode('py: deduction_formula'),
            ListLink(
                ImplicationLink(
                    VariableNode('$A'),
                    VariableNode('$C')),
                ImplicationLink(
                    VariableNode('$A'),
                    VariableNode('$B')),
                ImplicationLink(
                    VariableNode('$B'),
                    VariableNode('$C')))))
    deduction_rule_name = DefinedSchemaNode("implication-deduction-rule")
    DefineLink(
        deduction_rule_name,
        deduction_rule)
    MemberLink(deduction_rule_name, deduction_rbs)



def build_linked_deduction(deduction_rbs):
    deduction_rule = BindLink(
        VariableList(
            TypedVariableLink(
                VariableNode('$A'),
                TypeNode('ConceptNode')),
            TypedVariableLink(
                VariableNode('$B'),
                TypeNode('ConceptNode')),
            TypedVariableLink(
                VariableNode('$C'),
                TypeNode('ConceptNode'))),
        AndLink(
            EvaluationLink(
                PredicateNode("linked"),
                ListLink(
                    VariableNode('$A'),
                    VariableNode('$B'))),
            EvaluationLink(
                PredicateNode("linked"),
                ListLink(
                    VariableNode('$B'),
                    VariableNode('$C'))),
            NotLink(
                EqualLink(
                    VariableNode('$A'),
                    VariableNode('$C')))),
        ExecutionOutputLink(
            GroundedSchemaNode('py: deduction_formula'),
            ListLink(
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(
                        VariableNode('$A'),
                        VariableNode('$C'))),
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(
                        VariableNode('$A'),
                        VariableNode('$B'))),
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(
                        VariableNode('$B'),
                        VariableNode('$C'))))))
    deduction_rule_name = DefinedSchemaNode("linked-deduction-rule")
    DefineLink(
        deduction_rule_name,
        deduction_rule)
    MemberLink(deduction_rule_name, deduction_rbs)


def add_place_link(place1, place2, truth_value=TRUE):
    p1 = ConceptNode(place1)
    p1.tv = truth_value
    p2 = ConceptNode(place2)
    p2.tv = truth_value
    link = EvaluationLink(
        PredicateNode("leads_to"),
        ListLink(p1, p2))
    link.tv = truth_value


def add_thing(name, klasse, truth_value=TRUE):
    node1 = ConceptNode(name)
    node1.tv = truth_value
    node2 = ConceptNode(klasse)
    node2.tv = truth_value
    link = InheritanceLink(node1, node2)
    link.tv = truth_value

def check(query):
    print("\n")
    print("GOL: Asking MLN system; query: {:}".format(str(query)))
    max_prob = 0
    formula = None
    chainer = BackwardChainer(atomspace,
                      ConceptNode("deduction-rule-base"),
                      query)
    chainer.do_chain()
    results = chainer.get_results()
    # results = evaluate_atom(atomspace, query)
    # results = execute_atom(atomspace, query)
    print("\n")
    print("WS: Result is: {:}".format(results))
    print("\n")
    return results

if __name__ == '__main__':
    # print ("test_bc_deduction: pre scheme eval")
    # scheme_eval(atomspace, '(use-modules (opencog))')
    # scheme_eval(atomspace, '(use-modules (opencog exec))')
    # scheme_eval(atomspace, '(use-modules (opencog ure))')
    # print ("test_bc_deduction: pre scheme load")
    # scheme_eval(atomspace, '(load-from-path "/home/rasberry/catkin_ws/src/rasberry_hri/src/bdi/bc-deduction-config.scm")')
    # print ("test_bc_deduction: post scheme load")
    build_ontology()
    rbs = build_deduction_rulebase()
    build_linked_deduction(rbs)
    build_inheritance_deduction(rbs)
    build_implication_deduction(rbs)
    for waypoint in [("WayPoint133", "WayPoint102"), ("WayPoint102", "WayPoint103"), ("WayPoint103", "WayPoint104"), ("WayPoint104", "WayPoint105"), ("WayPoint105", "WayPoint106")]:
        add_thing(waypoint[0], "place")
        add_thing(waypoint[1], "place")
        add_place_link(waypoint[0], waypoint[1])


    p1 = VariableNode("place1")
    p2 = VariableNode("place2")
    results = execute_atom(atomspace, ForAllLink(
        VariableList(p1, p2),
        ImplicationLink(
            EvaluationLink(
                PredicateNode("leads_to"),
                ListLink(p1, p2)),
            EvaluationLink(
                PredicateNode("linked"),
                ListLink(p1, p2)).truth_value(1,1))))
    print("PRE: Result is: {:}".format(results))
    # p1 = VariableNode("place1")
    # p2 = VariableNode("place2")
    # p3 = VariableNode("place3")
    # t1 = VariableNode("thing1")
    # s1 = VariableNode("state1")
    # t2 = VariableNode("thing2")
    # hum1 = VariableNode("human1")
    # obj1 = VariableNode("object1")
    # cre1 = VariableNode("creature1")
    # execute_atom(atomspace, ForAllLink(
    #     VariableList(p1, p2),
    #     ImplicationLink(
    #         EvaluationLink(
    #             PredicateNode("leads_to"),
    #             ListLink(p1, p2)),
    #             EvaluationLink(
    #                 PredicateNode("linked"),
    #                 ListLink(p1, p2)).truth_value(1.0, 1.0))))



    ## works:
    # results = execute_atom(atomspace,
    #             TruthValueOfLink(ConceptNode("WayPoint102")))
    # results = execute_atom(atomspace,
    #             TruthValueOfLink(InheritanceLink(ConceptNode("WayPoint102"), ConceptNode("place"))))
    # results = evaluate_atom(atomspace, EvaluationLink(PredicateNode("leads_to"), ListLink(ConceptNode("WayPoint102"), ConceptNode("WayPoint103"))))
    # results = check(EvaluationLink(PredicateNode("leads_to"), ListLink(ConceptNode("WayPoint102"), ConceptNode("WayPoint103"))))
    # results = results.get_out()[0].tv
    # results = check(InheritanceLink(ConceptNode("WayPoint102"), ConceptNode("thing")))
    results = check(GetLink(VariableNode("bla"), InheritanceLink(ConceptNode("WayPoint102"), VariableNode("bla"))))
    # results = results.get_out()[0].tv
    # results = check(EvaluationLink(PredicateNode("linked"), ListLink(ConceptNode("WayPoint102"), ConceptNode("WayPoint104"))))
    results = results.get_out()[0].tv

    ## doesn't work:

    # results = execute_atom(atomspace, GetLink(MemberLink(VariableNode("var"), ConceptNode("PLN"))))


    # results = execute_atom(atomspace,
    #             TruthValueOfLink(results.get_out()[0]))

    # check(
    #     GetLink(
    #         VariableList(
    #             VariableNode("var")),
    #         InheritanceLink(
    #             VariableNode("var"),
    #             ConceptNode("place"))))
    # results = evaluate_atom(atomspace,
    #             EvaluationLink(
    #                 PredicateNode("leads_to"),
    #                     ListLink(
    #                         ConceptNode("WayPoint102"),
    #                         ConceptNode("Waypoint105"))))
    # results = execute_atom(atomspace,
    #             TruthValueOfLink(ConceptNode("WayPoint102")))

    # print("\n")
    print("POST: Result is: {:}".format(results))
    # print("\n")

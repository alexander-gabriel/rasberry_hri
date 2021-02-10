import cProfile
import pstats

from opencog.type_constructors import *
from opencog.bindlink import execute_atom, evaluate_atom
from opencog.ure import BackwardChainer, ForwardChainer
from opencog.utilities import initialize_opencog
from opencog.atomspace import AtomSpace, types
from opencog.scheme_wrapper import scheme_eval


from common.utils import OrderedConsistentSet, atomspace
from experiment_config import Small, Medium, Large, Short, Middle, Long, Deliver, Exchange
# scheme_eval(atomspace, execute_code)


TRUE = TruthValue(1.0, 1.0)

def deduction_formula(AC, AB, BC):
    tv1 = AB.tv
    tv2 = BC.tv
    if tv1.mean > 0.5 and tv2.mean > 0.5 and tv1.confidence > 0.5 and tv2.confidence > 0.5:
        AC.tv = TruthValue(1, 1)
    else:
        AC.tv = TruthValue(0, 0)
    return AC

def build_deduction_rulebase():
    rbs = ConceptNode("deduction-rule-base")
    execute_code = \
    '''
    (use-modules (opencog))
    (use-modules (opencog logger) (opencog ure) (opencog exec))
    (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/crisp/propositional/true-conjunction-introduction.scm")
    (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm")
    (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-disjunction-introduction.scm")
    (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/negation-introduction.scm")
    (define rbs (Concept "deduction-rule-base"))
    (ure-set-complexity-penalty rbs 0.1)
    '''
    scheme_eval(atomspace, execute_code)
    MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-6ary-rule"), rbs)
    return rbs

# custom deduction rule for the linked predicate
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
                IdenticalLink(
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

# basic structure of classes
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


# add a place conceptnode to the KB
def add_place(name, truth_value=TRUE):
    node1 = ConceptNode(name)
    node1.tv = truth_value
    link = InheritanceLink(node1, ConceptNode("place"))
    link.tv = truth_value

# add two 'linked' places
def add_place_link(place1, place2, truth_value=TRUE):
    p1 = ConceptNode(place1)
    p1.tv = truth_value
    link = InheritanceLink(p1, ConceptNode("place"))
    link.tv = truth_value
    p2 = ConceptNode(place2)
    p2.tv = truth_value
    link = InheritanceLink(p2, ConceptNode("place"))
    link.tv = truth_value
    link = EvaluationLink(
        PredicateNode("linked"),
        ListLink(p1, p2))
    link.tv = truth_value


# add arbitrary typed things to the KB
def add_thing(name, klasse, truth_value=TRUE):
    node1 = ConceptNode(name)
    node1.tv = truth_value
    node2 = ConceptNode(klasse)
    node2.tv = truth_value
    link = InheritanceLink(node1, node2)
    link.tv = truth_value
    return node1


def find_instances(config):
    templates = config.goal.get_condition_templates()
    conditions = []
    variables = OrderedConsistentSet()
    for fun, args in templates:
        new_args = []
        for arg in args:
            if arg == "me":
                new_args.append(ConceptNode(config.robot_name))
            elif fun.__name__ == "is_a" and args.index(arg) == 1:
                new_args.append(ConceptNode(arg))
            else:
                variable = VariableNode(arg)
                new_args.append(variable)
                variables.append(variable)
                # variables.append(TypedVariableLink(variable, TypeNode("ConceptNode")))
        condition = fun(*new_args)
        conditions.append(condition)
    return (VariableList(*variables.items), AndLink(*conditions))


class DeliverShortSmall(Deliver, Short, Small): pass
class DeliverShortMedium(Deliver, Short, Medium): pass
class DeliverShortLarge(Deliver, Short, Large): pass

class DeliverMiddleMedium(Deliver, Middle, Medium): pass
class DeliverMiddleLarge(Deliver, Middle, Large): pass

class DeliverLongLarge(Deliver, Long, Large): pass

class ExchangeShortSmall(Exchange, Short, Small): pass
class ExchangeShortMedium(Exchange, Short, Medium): pass
class ExchangeShortLarge(Exchange, Short, Large): pass

class ExchangeMiddleMedium(Exchange, Middle, Medium): pass
class ExchangeMiddleLarge(Exchange, Middle, Large): pass

class ExchangeLongLarge(Exchange, Long, Large): pass

if __name__ == '__main__':
    configs = [DeliverShortMedium()]
    for config in configs:
        # atomspace = AtomSpace()
        # initialize_opencog(atomspace)
        rbs = build_deduction_rulebase()
        ExecutionLink(SchemaNode("URE:maximum-iterations"), rbs, NumberNode(config.iterations))
        build_linked_deduction(rbs)
        build_ontology()

        p = add_thing(config.picker_name, "human")
        r = add_thing(config.robot_name, "robot")
        c = add_thing("crate01", "crate")
        s = add_thing("strawberry01", "strawberryplant")
        # build waypoints as defined in config
        for index in range(1,config.size):
            # add waypoint concepts and 'linked' predicate
            add_place_link("WayPoint{:03d}".format(index), "WayPoint{:03d}".format(index+1))
            # add statelinks to set picker and robot position according to config
            if config.picker_pos == index:
                link = StateLink(p, ConceptNode("WayPoint{:03d}".format(index)))
                link.tv = TRUE
            if config.robot_pos == index:
                link = StateLink(r, ConceptNode("WayPoint{:03d}".format(index)))
                link.tv = TRUE
        # add statelinks to set picker and robot position if configured for last WayPoint
        if config.picker_pos == config.size:
            link = StateLink(p, ConceptNode("WayPoint{:03d}".format(config.size)))
            link.tv = TRUE
        if config.robot_pos == config.size:
            link = StateLink(r, ConceptNode("WayPoint{:03d}".format(config.size)))
            link.tv = TRUE
        # add statelinks to set crate and plant position for waypoint 1
        link = StateLink(s, ConceptNode("WayPoint{:03d}".format(1)))
        link.tv = TRUE
        link = StateLink(c, ConceptNode("WayPoint{:03d}".format(1)))
        link.tv = TRUE

        # add config-defined facts to KB
        config.set_facts()
        variables, query = find_instances(config)

        variables = VariableList(VariableNode("picker"), VariableNode("origin"), VariableNode("destination"))
        query = AndLink(
                    StateLink(
                        ConceptNode(config.robot_name),
                        VariableNode("origin")),
                    StateLink(
                        VariableNode("picker"),
                        VariableNode("destination")),
                    StateLink(
                        ListLink(VariableNode("picker"), PredicateNode("seen_picking")),
                        ConceptNode("FALSE")),
                    InheritanceLink(
                        VariableNode("picker"),
                        ConceptNode("human")),
                    StateLink(
                        ListLink(VariableNode("picker"), PredicateNode("called_robot")),
                        ConceptNode("FALSE")),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(
                            VariableNode("origin"),
                            VariableNode("destination"))))
        print("Variables: {:}\n".format(variables))
        trace = AtomSpace()
        chainer = BackwardChainer(atomspace,
                          rbs,
                          query, vardecl=variables, trace_as=trace)
        # chainer.do_chain()
        logfilename = "_".join([config.log_name_prefix, config.log_name, config.log_name_postfix])
        cProfile.run('chainer.do_chain()', logfilename)
        p = pstats.Stats(logfilename)
        time = p.stats[('~', 0, "<method 'do_chain' of 'opencog.ure.BackwardChainer' objects>")][2]

        # p.strip_dirs().sort_stats(-1).print_stats('do_chain')
        # print(trace.get_atoms_by_type(types.Atom))
        results = chainer.get_results()
        # print("Result {:}".format(results))
        # print("Set Truth: {:}".format(results.tv))
        print("\n{:}\nSize: {:d}   Range: {:d}   Iterations: {:}   Time: {:f}".format(logfilename[:-4], config.size, config.picker_pos-config.robot_pos,config.iterations, time))
        print("{:};{:d};{:d};{:};{:f}".format(config.log_name_prefix, config.size, config.picker_pos-config.robot_pos,config.iterations, time))
        print("------------")
        print("Query:\n{:}\n\n\nResults:\n------------\n".format(query))
        try:
            for result in results.get_out():
                print("Result Truth: {:}\n".format(result.tv))
                print("Result:\n{:}".format(result))
                print("\n------\n")
        except:
            pass
        print("\n\n")

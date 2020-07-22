from opencog.type_constructors import *
from opencog.bindlink import execute_atom, evaluate_atom
from opencog.utilities import initialize_opencog
from opencog.ure import BackwardChainer, ForwardChainer
from opencog.atomspace import AtomSpace, types
from opencog.scheme_wrapper import scheme_eval
from opencog.logger import log

def print_results(query,results):
    print("Query:\n{:}\n\nResults:\n{:}\n\nDetails:\n--------".format(query, results))
    if query.type_name == "GetLink":
        for setlink in results.get_out():
            for result in setlink.get_out():
                print("Result Truth: {:}".format(result.tv))
                print("Result:\n{:}".format(result))
                print("------------------------\n")
    elif query.type_name == "AndLink":
        for result in results.get_out():
            print("Result Truth: {:}".format(result.tv))
            print("Result:\n{:}".format(result))
            for condition in result.get_out():
                print("------------------------")
                print("Condition:{:}".format(condition))
                print("Condition Truth: {:}".format(condition.tv))
                if condition.type_name == "NotLink":
                    subcondition = condition.get_out()[0]
                    print("    Subcondition: {:}".format(subcondition))
                    print("    Subcondition Truth: {:}".format(subcondition.tv))
            print("-----------------------------------------------------")
    else:
        for result in results.get_out():
            print("Result Truth: {:}".format(result.tv))
            print("Result:\n{:}".format(result))
            print("------------------------\n")


def leads_to(p2, p3):
    link = EvaluationLink(
        PredicateNode("leads_to"),
        ListLink(p2, p3))
    return link

def absent(link):
    return AbsentLink(link)

def state2(concept, predicate):
    return StateLink(concept, predicate)

def query_not_at(thing, place):
    return absent(state2(thing, place))


if __name__ == '__main__':

    log.use_stdout()
    log.set_level("DEBUG")

    atomspace = AtomSpace()
    initialize_opencog(atomspace)
    print(atomspace)
    a = ConceptNode("a")
    b = ConceptNode("b")
    c = ConceptNode("b")
    p1 = ConceptNode("p1")
    p2 = ConceptNode("p2")
    p3 = ConceptNode("p3")

    a_at_p1 = state2(a, p1)
    b_at_p2 = state2(b, p2)
    # c_at_p3 = state2(c, p3)
    true =  TruthValue(1, 1)

    leads1 = leads_to(p1, p2).tv = true
    leads2 = leads_to(p2, p3).tv = true

    va = VariableNode("a")
    vb = VariableNode("b")
    vp1 = VariableNode("p1")
    vp2 = VariableNode("p2")
    vp3 = VariableNode("p3")
    anything = VariableNode("anything")

    variables = VariableList(
        # TypedVariableLink(va, TypeNode("ConceptNode")),
        TypedVariableLink(vb, TypeNode("ConceptNode")),
        TypedVariableLink(vp1, TypeNode("ConceptNode")),
        TypedVariableLink(vp2, TypeNode("ConceptNode")),
        TypedVariableLink(vp3, TypeNode("ConceptNode")),
        TypedVariableLink(anything, TypeNode("ConceptNode")),

    )

    query = AndLink(leads_to(vp1, vp2),
                    leads_to(vp2, vp3),
                    state2(a, vp1),
                    query_not_at(anything, vp3),
                    state2(vb, vp2))

    # results = evaluate_atom(atomspace, query)
    # print(results)
    print(query)
    chainer = BackwardChainer(_as=atomspace,
                      rbs=ConceptNode("none"), trace_as=None, control_as=None, focus_set=None,
                      target=GetLink(variables,query), vardecl=variables)
    chainer.do_chain()
    results = chainer.get_results()
    print(results)

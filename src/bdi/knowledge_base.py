import sys
from threading import RLock
import time
from copy import copy

from opencog.atomspace import AtomSpace, TruthValue, createFloatValue, createLinkValue
from opencog.atomspace import types
# from opencog.type_constructors import *
from opencog.ure import BackwardChainer, ForwardChainer
from opencog.utilities import initialize_opencog
from opencog.scheme_wrapper import scheme_eval
from opencog.bindlink import execute_atom, evaluate_atom

import rospy

from parameters import *
# set_type_ctor_atomspace(self.atsp)

# def lock():
#     rospy.loginfo("{} wants lock".format(sys._getframe().f_back.f_code.co_name))
#     # return
#     LOCK.acquire()
#     rospy.loginfo("{} has lock".format(sys._getframe().f_back.f_code.co_name))
#
# def unlock():
#     # return
#     LOCK.release()
#     rospy.loginfo("{} released lock".format(sys._getframe().f_back.f_code.co_name))




class KnowledgeBase(object):

    def __init__(self):
        rospy.loginfo("KNB: Initializing Knowledge Base")
        self.atsp = AtomSpace()
        self.lock = RLock()
        self.TRUE = TruthValue(1,1)
        self.FALSE = TruthValue(0,1)
        rbs = self.build_deduction_rulebase()
        self.build_linked_deduction(rbs)
        self.build_ontology()
        # self.build_inheritance_deduction(rbs)
        rospy.loginfo("KNB: Initialization finished")


    def reason(self, query, variables):
        max_prob = 0
        formula = None
        rospy.logdebug("WST: Query:\n{:}\nVariables:\n{}".format(str(query), variables))
        start_time = time.time()
        with self.lock:
            chainer = BackwardChainer(self.atsp,
                          self.concept("deduction-rule-base"),
                          query, vardecl = variables)
            chainer.do_chain()
        rospy.logdebug("WST: do_chain -- {:.4f}".format(time.time() - start_time))
        results = chainer.get_results()

        got_one = False
        # if query.type_name == "GetLink":
        #     for result in results.get_out()[0].get_out():
        #         got_one = True
        #         # rospy.loginfo("WST: Chaining Result:{:}".format(result))
        #         # rospy.loginfo("WST: ---------------------")
        # else:
        #     for result in results.get_out():
        #         # if result.tv == self.TRUE:
        #         got_one = True
        #         if result.tv != self.TRUE:
        #             rospy.logwarn("Untrue Results:\n{}".format(results))
                # rospy.loginfo("WST: Chaining Result:{:}".format(result))
                # rospy.loginfo("WST: ---------------------")

        # if not got_one:
        #     if query.type_name == "GetLink":
        #         query = query.get_out()[1]
        #     # rospy.logwarn("Query:\n{:}\n\n\nResults:------------\n".format(query))
        #
        #     if query.type_name == "StateLink" or query.type_name == "EvaluationLink" or query.type_name == "InheritanceLink":
        #         with self.lock:
        #             # rospy.loginfo("WST: -----------")
        #             # rospy.loginfo("WST: Condition: {:}".format(query))
        #             chainer = BackwardChainer(self.atsp,
        #                           self.concept("deduction-rule-base"),
        #                           query)
        #             chainer.do_chain()
        #             res = chainer.get_results()
        #         got_anoter_one =False
        #         # rospy.logwarn("WST: Condition Results: {:}".format(res))
        #         for result in res.get_out():
        #             # rospy.loginfo("WST: Condition Result: {:}".format(result))
        #             # rospy.loginfo("WST: Condition truth: {:}".format(result.tv))
        #             got_anoter_one = True
        #             if result.tv != self.TRUE:
        #                 rospy.logwarn("WST: Non-True truth value: {:} -- {}".format(query, result.tv))
        #         if not got_anoter_one:
        #             rospy.logwarn("WST: No results for: {:}".format(query))
        #     else:
        #         # rospy.loginfo("default supported query? {}".format(query))
        #         for condition in query.get_out():
        #             with self.lock:
        #                 # rospy.loginfo("WST: -----------")
        #                 # rospy.loginfo("WST: Condition: {:}".format(condition))
        #                 chainer = BackwardChainer(self.atsp,
        #                               self.concept("deduction-rule-base"),
        #                               condition)
        #                 chainer.do_chain()
        #                 res = chainer.get_results()
        #             # rospy.logwarn("WST: Condition Results: {:}".format(res))
        #             got_anoter_one = False
        #             for result in res.get_out():
        #                 # rospy.loginfo("WST: Condition Result: {:}".format(result))
        #                 # rospy.loginfo("WST: Condition truth: {:}".format(result.tv))
        #                 got_anoter_one = True
        #                 if result.tv != self.TRUE:
        #                     rospy.logwarn("WST: Non-True truth value: {:} -- {}".format(condition, result.tv))
        #             if not got_anoter_one:
        #                 rospy.logwarn("WST: No results for: {:}".format(condition))
                        # if condition.type_name == "VariableList":
                        #     raise Exception()
        return results


    def check(self, query, variables):
        with self.lock:
            rospy.loginfo("check variables: {}".format(variables))
            rospy.loginfo("check query: {}".format(query))
            results = execute_atom(self.atsp, self.get(variables, query))
            rospy.loginfo("check results: {}".format(results))
        for result in results.get_out():
            if result.tv == self.TRUE:
                rospy.loginfo("WST: Execution Result Truth: {:}".format(result.tv))
                rospy.loginfo("WST: Execution Result:{:}".format(result))
                rospy.loginfo("WST: ---------------------")
        return results


    def check2(self, query):
        with self.lock:
            results = evaluate_atom(self.atsp, query)
        for result in results.get_out():
            if result.tv == self.TRUE:
                rospy.loginfo("WST: Evalutation Result Truth: {:}".format(result.tv))
                rospy.loginfo("WST: Evalutation Result:{:}".format(result))
                rospy.loginfo("WST: ---------------------")
        return results


    # likely doesn't work
    # def recursive_query_matcher(self, query, result, target):
    #     rospy.loginfo("---------\nquery:\n{}\nresult:\n{}".format(query, result))
    #     if query == result:
    #         return True
    #     elif query.is_link():
    #         subnodes_results = []
    #         query_out = query.get_out()
    #         result_out = result.get_out()
    #         if query.is_a(types.OrderedLink):
    #             for index in range(len(query_out)):
    #                 subnodes_results.append(self.recursive_query_matcher(query_out[index], result_out[index], target))
    #             if False in subnodes_results:
    #                 return False
    #             else:
    #                 return subnodes_results
    #         else:
    #             equality_counter = 0
    #             different_subqueries = []
    #             relevant_subresults = copy(result_out)
    #             for subquery in query_out:
    #                 if subquery in result_out:
    #                     relevant_subresults.remove(subquery)
    #                     equality_counter += 1
    #                 else:
    #                     different_subqueries.append(subquery)
    #             for subquery in different_subqueries:
    #                 candidates = []
    #                 for subresult in relevant_subresults:
    #                     if subresult.type_name == subquery.type_name:
    #                         subnodes_results.append(self.recursive_query_matcher(subquery, subresult, target))
    #                 filtered_subnode_results = []
    #                 for subnode_result in subnodes_results:
    #                     if subnode_result:
    #
    #     elif query.type_name == "VariableNode":
    #         return (query.name, result.name)
    #     else:
    #         return False


    def get_target(self, query, results, target):
        for result in results.get_out():
            if result.tv == self.TRUE:
                self.recursive_query_matcher(query, result, target)
        return target


    def add_link(self, link, truth_value=None):
        truth_value = self.TRUE if truth_value is None else truth_value
        with lock:
            self.atsp.add_link(link, tv=truth_value)


    def set_value(self, node, key, value):
        node.set_value(self.predicate(key), value)


    def get_value(self, node, key):
        return node.get_value(self.predicate(key))


    def FloatValue(self, value):
        return createFloatValue(value)


    def LinkValue(self, value):
        return createLinkValue(value)


    def add_node(self, node=None):
        with lock:
            self.atsp.add_node(node)


    def number(self, name):
        with self.lock:
            return self.atsp.add_node(types.NumberNode, name)


    def concept(self, name):
        with self.lock:
            return self.atsp.add_node(types.ConceptNode, name)


    def predicate(self, name):
        with self.lock:
            return self.atsp.add_node(types.PredicateNode, name)


    def type(self, name):
        with self.lock:
            return self.atsp.add_node(types.TypeNode, name)


    def variable(self, name):
        with self.lock:
            return self.atsp.add_node(types.VariableNode, name)


    def schema(self, name):
        with self.lock:
            return self.atsp.add_node(types.SchemaNode, name)


    def defined_schema(self, name):
        with self.lock:
            return self.atsp.add_node(types.DefinedSchemaNode, name)


    def grounded_schema(self, name):
        with self.lock:
            return self.atsp.add_node(types.GroundedSchemaNode, name)


    def variable_link(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.VariableLink, list(subnodes))


    def typed_variable(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.TypedVariableLink, list(subnodes))


    def variable_list(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.VariableList, list(subnodes))


    def list(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.ListLink, list(subnodes))


    def inheritance(self, *subnodes, **kwargs):
        with self.lock:
            # is_a = self.atsp.add_node(types.PredicateNode, "is a")
            # list_link = self.atsp.add_link(types.ListLink, list(subnodes))
            # return self.atsp.add_link(types.EvaluationLink, [is_a, list_link])
            return self.atsp.add_link(types.InheritanceLink, list(subnodes))


    def evaluation(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.EvaluationLink, list(subnodes))


    def execution(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.ExecutionLink, list(subnodes))


    def execution_output(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.ExecutionOutputLink, list(subnodes))


    def state(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.StateLink, list(subnodes))

    def absent(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.AbsentLink, list(subnodes))


    def present(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.PresentLink, list(subnodes))


    def exists(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.ExistsLink, list(subnodes))


    def for_all(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.ForAllLink, list(subnodes))


    def get(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.GetLink, list(subnodes))


    def And(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.AndLink, list(subnodes))


    def Or(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.OrLink, list(subnodes))


    def Not(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.NotLink, list(subnodes))


    def equal(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.NotLink, list(subnodes))


    def member(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.MemberLink, list(subnodes))


    def set(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.SetLink, list(subnodes))


    def identical(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.IdenticalLink, list(subnodes))


    def define(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.DefineLink, list(subnodes))


    def bind(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.BindLink, list(subnodes))


    def define(self, *subnodes, **kwargs):
        with self.lock:
            return self.atsp.add_link(types.DefineLink, list(subnodes))


    def build_deduction_rulebase(self):
        with self.lock:
            rbs = self.concept("deduction-rule-base")
            execute_code = \
            '''
            (use-modules (opencog))
            (use-modules (opencog logger) (opencog ure) (opencog exec) (opencog python))
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/term/inheritance-direct-introduction.scm") ; loading error
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/term/crisp-deduction.scm") ; loading error

            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/instantiation.scm")
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/crisp/propositional/true-conjunction-introduction.scm")
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/propositional/fuzzy-disjunction-introduction.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/propositional/modus-ponens.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/term/deduction.scm")
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/precise-modus-ponens.scm")
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/inheritance-to-member.scm")
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/member-to-evaluation.scm")
            ;(load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/member-to-inheritance.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/negation-introduction.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/not-simplification.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/not-elimination.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/wip/implication-instantiation.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/rules/predicate/conditional-direct-evaluation.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/meta-rules/predicate/conditional-full-instantiation.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/meta-rules/predicate/conditional-partial-instantiation.scm")
            (load-from-path "/home/rasberry/git/pln/opencog/pln/meta-rules/predicate/universal-full-instantiation.scm")
            (define rbs (Concept "deduction-rule-base"))
            (ure-set-complexity-penalty rbs {:f})
            '''.format(COMPLEXITY_PENALTY)
            scheme_eval(self.atsp, execute_code)
            # self.member(self.defined_schema("inheritance-to-member-rule"), rbs)
            # self.member(self.defined_schema("member-to-evaluation-0-rule"), rbs)
            # self.member(self.defined_schema("member-to-evaluation-1-rule"), rbs)
            # self.member(self.defined_schema("member-to-evaluation-2-1-rule"), rbs)
            # self.member(self.defined_schema("member-to-evaluation-2-2-rule"), rbs)
            # self.member(self.defined_schema("member-to-inheritance-rule"), rbs)
            # self.member(self.defined_schema("true-conjunction-introduction-1ary-rule"), rbs)
            # self.member(self.defined_schema("true-conjunction-introduction-2ary-rule"), rbs)
            # self.member(self.defined_schema("true-conjunction-introduction-3ary-rule"), rbs)
            # self.member(self.defined_schema("true-conjunction-introduction-4ary-rule"), rbs)
            # self.member(self.defined_schema("true-conjunction-introduction-5ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-1ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-2ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-3ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-4ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-5ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-6ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-7ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-8ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-9ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-10ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-11ary-rule"), rbs)
            self.member(self.defined_schema("fuzzy-conjunction-introduction-12ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-1ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-2ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-3ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-4ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-5ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-6ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-7ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-8ary-rule"), rbs)
            # self.member(self.defined_schema("fuzzy-disjunction-introduction-9ary-rule"), rbs)
            # self.member(self.defined_schema("bc-deduction-rule"), rbs)

            # self.member(self.defined_schema("inheritance-direct-introduction-rule"), rbs)
            # self.member(self.defined_schema("conditional-direct-evaluation-implication-scope-rule"), rbs)
            # self.member(self.defined_schema("conditional-full-instantiation-implication-meta-rule"), rbs)
            # self.member(self.defined_schema("conditional-full-instantiation-inheritance-meta-rule"), rbs)
            # self.member(self.defined_schema("conditional-partial-instantiation-meta-rule"), rbs)
            # self.member(self.defined_schema("universal-full-instantiation-forall-1ary-meta-rule"), rbs)
            # self.member(self.defined_schema("deduction-inheritance-rule"), rbs)
            # self.member(self.defined_schema("deduction-implication-rule"), rbs)
            # self.member(self.defined_schema("deduction-subset-rule"), rbs)
            # self.member(self.defined_schema("present-deduction-inheritance-rule"), rbs)
            # self.member(self.defined_schema("modus-ponens-inheritance-rule"), rbs)
            # self.member(self.defined_schema("modus-ponens-implication-rule"), rbs)
            # self.member(self.defined_schema("modus-ponens-subset-rule"), rbs)
            # self.member(self.defined_schema("crisp-contraposition-implication-scope-rule"), rbs)
            # self.member(self.defined_schema("contraposition-implication-rule"), rbs)
            # self.member(self.defined_schema("contraposition-inheritance-rule"), rbs)
            # self.member(self.defined_schema("implication-scope-to-implication-rule"), rbs)
            # self.member(self.defined_schema("implication-full-instantiation-rule"), rbs)
            # self.member(self.defined_schema("implication-partial-instantiation-rule"), rbs)
            self.member(self.defined_schema("negation-introduction-rule"), rbs)
            # self.member(self.defined_schema("not-simplification-rule"), rbs)
            # self.member(self.defined_schema("not-elimination-rule"), rbs)
            self.evaluation(self.predicate("URE:attention-allocation"), rbs).truth_value(0.1, 1)
            self.execution(self.schema("URE:maximum-iterations"), rbs, self.number(ITERATIONS))
        return rbs


    def build_inheritance_deduction(self,deduction_rbs):
        deduction_rule = self.bind(
            self.variable_list(
                self.typed_variable(
                    self.variable('$A'),
                    self.type('ConceptNode')),
                self.typed_variable(
                    self.variable('$B'),
                    self.type('ConceptNode')),
                self.typed_variable(
                    self.variable('$C'),
                    self.type('ConceptNode'))),
            self.And(
                self.inheritance(
                    self.variable('$A'),
                    self.variable('$B')),
                self.inheritance(
                    self.variable('$B'),
                    self.variable('$C')),
                self.Not(
                    self.equal(
                        self.variable('$A'),
                        self.variable('$C')))),
            self.execution_output(
                self.grounded_schema('py: deduction_formula'),
                self.list(
                    self.inheritance(
                        self.variable('$A'),
                        self.variable('$C')),
                    self.inheritance(
                        self.variable('$A'),
                        self.variable('$B')),
                    self.inheritance(
                        self.variable('$B'),
                        self.variable('$C')))))
        deduction_rule_name = self.defined_schema("inheritance-deduction-rule")
        self.define(
            deduction_rule_name,
            deduction_rule)
        self.member(deduction_rule_name, deduction_rbs)


    def build_implication_deduction(self, deduction_rbs):
        deduction_rule = self.bind(
            self.variable_list(
                self.typed_variable(
                    self.variable('$A'),
                    self.type('ConceptNode')),
                self.typed_variable(
                    self.variable('$B'),
                    self.type('ConceptNode')),
                self.typed_variable(
                    self.variable('$C'),
                    self.type('ConceptNode'))),
            self.And(
                self.implication(
                    self.variable('$A'),
                    self.variable('$B')),
                self.implication(
                    self.variable('$B'),
                    self.variable('$C')),
                self.Not(
                    self.equal(
                        self.variable('$A'),
                        self.variable('$C')))),
            self.execution_output(
                self.grounded_schema('py: deduction_formula'),
                self.list(
                    self.implication(
                        self.variable('$A'),
                        self.variable('$C')),
                    self.implication(
                        self.variable('$A'),
                        self.variable('$B')),
                    self.implication(
                        self.variable('$B'),
                        self.variable('$C')))))
        deduction_rule_name = self.defined_schema("implication-deduction-rule")
        self.define(
            deduction_rule_name,
            deduction_rule)
        self.member(deduction_rule_name, deduction_rbs)


    def build_linked_deduction(self, deduction_rbs):
        deduction_rule = self.bind(
            self.variable_list(
                self.typed_variable(
                    self.variable('$A'),
                    self.type('ConceptNode')),
                self.typed_variable(
                    self.variable('$B'),
                    self.type('ConceptNode')),
                self.typed_variable(
                    self.variable('$C'),
                    self.type('ConceptNode'))),
            self.And(
                self.evaluation(
                    self.predicate("linked"),
                    self.list(
                        self.variable('$A'),
                        self.variable('$B'))),
                self.evaluation(
                    self.predicate("linked"),
                    self.list(
                        self.variable('$B'),
                        self.variable('$C'))),
                self.Not(
                    self.identical(
                        self.variable('$A'),
                        self.variable('$C')))),
            self.execution_output(
                self.grounded_schema('py: deduction_formula'),
                self.list(
                    self.evaluation(
                        self.predicate("linked"),
                        self.list(
                            self.variable('$A'),
                            self.variable('$C'))),
                    self.evaluation(
                        self.predicate("linked"),
                        self.list(
                            self.variable('$A'),
                            self.variable('$B'))),
                    self.evaluation(
                        self.predicate("linked"),
                        self.list(
                            self.variable('$B'),
                            self.variable('$C'))))))
        deduction_rule_name = self.defined_schema("linked-deduction-rule")
        self.define(
            deduction_rule_name,
            deduction_rule)
        self.member(deduction_rule_name, deduction_rbs)


    def build_ontology(self):

        thing = self.concept("thing").truth_value(1.0, 1.0)

        concept = self.concept("concept").truth_value(1.0, 1.0)
        self.inheritance(concept, thing).truth_value(1.0, 1.0)

        self.place = self.concept("place").truth_value(1.0, 1.0)
        self.inheritance(self.place, concept).truth_value(1.0, 1.0)


        entity = self.concept("entity").truth_value(1.0, 1.0)
        self.inheritance(entity, thing).truth_value(1.0, 1.0)


        obj = self.concept("object").truth_value(1.0, 1.0)
        organism = self.concept("organism").truth_value(1.0, 1.0)
        self.inheritance(organism, entity).truth_value(1.0, 1.0)
        self.inheritance(obj, entity).truth_value(1.0, 1.0)

        plant = self.concept("plant").truth_value(1.0, 1.0)
        self.inheritance(plant, organism).truth_value(1.0, 1.0)

        strawberryplant = self.concept("strawberryplant").truth_value(1.0, 1.0)
        self.inheritance(strawberryplant, plant).truth_value(1.0, 1.0)

        creature = self.concept("creature").truth_value(1.0, 1.0)
        self.robot = self.concept("robot").truth_value(1.0, 1.0)
        self.human = self.concept("human").truth_value(1.0, 1.0)
        self.inheritance(creature, organism).truth_value(1.0, 1.0)
        self.inheritance(self.robot, creature).truth_value(1.0, 1.0)
        self.inheritance(self.human, creature).truth_value(1.0, 1.0)

        self.crate = self.concept("crate").truth_value(1.0, 1.0)
        self.inheritance(self.crate, obj).truth_value(1.0, 1.0)


        # p1 = self.variable("place1")
        # p2 = self.variable("place2")
        # p3 = self.variable("place3")
        # t1 = self.variable("thing1")
        # s1 = self.variable("state1")
        # t2 = self.variable("thing2")
        # hum1 = self.variable("human1")
        # obj1 = self.variable("object1")
        # cre1 = self.variable("creature1")

        # self.for_all(
        #     p1,
        #     self.evaluation(
        #         self.predicate("leads_to"),
        #         self.list(p1, p1)).truth_value(0.0, 1.0))
        #
        # self.for_all(
        #     self.variable_list(p1, p2),
        #     self.implication(
        #         self.evaluation(
        #             self.predicate("leads_to"),
        #             self.list(p1, p2)),
        #             self.evaluation(
        #                 self.predicate("linked"),
        #                 self.list(p1, p2)).truth_value(1.0, 1.0)))
        #
        # self.for_all(
        #     self.variable_list(p1, p2, p3),
        #     self.implication(
        #         self.And(
        #             self.Not(
        #                 self.identical(p1, p3)),
        #             self.evaluation(
        #                 self.predicate("linked"),
        #                 self.list(p1, p2)),
        #             self.evaluation(
        #                 self.predicate("linked"),
        #                 self.list(p2, p3))),
        #         self.evaluation(
        #             self.predicate("linked"),
        #             self.list(p1, p3)).truth_value(1.0, 1.0)))
        #
        # self.for_all(
        #     self.variable_list(p1, p2),
        #     self.implication(
        #         self.And(
        #             self.evaluation(
        #                 self.predicate("leads_to"),
        #                 self.list(p1, p2)),
        #             self.Not(
        #                 self.exists(t1, self.state(t1, p2))),
        #             self.or(
        #                 self.evaluation(
        #                     self.predicate("can_reach"),
        #                     self.list(p2, p3)),
        #                 self.identical(p1, p3))),
        #         self.evaluation(
        #             self.predicate("free_path"),
        #             self.list(p1, p3)).truth_value(1.0, 1.0)))
        #
        # self.for_all(
        #     self.variable_list(p1, p2, obj1, cre1),
        #     self.implication(
        #         self.or(
        #             self.inheritance(obj1, obj),
        #             self.inheritance(obj1, plant)),
        #         self.evaluation(
        #             self.predicate("can_reach"),
        #             self.list(p1, p2)).truth_value(0.0, 1.0)))
        # self.for_all(
        #     self.variable_list(p1, p2, obj1, cre1),
        #     self.implication(
        #         self.And(
        #             self.or(
        #                 self.inheritance(obj1, obj),
        #                 self.inheritance(obj1, plant)),
        #             self.inheritance(cre1, creature),
        #             self.state(cre1, p1),
        #             self.state(obj1, p2),
        #             self.or(
        #                 self.evaluation(
        #                     self.predicate("free_path"),
        #                     self.list(p1, p2)),
        #                 self.evaluation(
        #                     self.predicate("leads_to"),
        #                     self.list(p1, p2)))),
        #         self.evaluation(
        #             self.predicate("can_reach"),
        #             self.list(p1, p2)).truth_value(1.0, 1.0)))
        #
        # self.for_all(
        #     self.variable_list(t1, t2, p1),
        #     self.implication(
        #         self.And(
        #             self.Not(self.identical(t1, t2)),
        #             self.state(t1, p1),
        #             self.state(t2, p1)),
        #         self.evaluation(
        #             self.predicate("colocated"),
        #             self.list(t1, t2)).truth_value(1.0, 1.0)))
        # self.for_all(
        #     self.variable_list(t1, t2, p1, p2),
        #     self.implication(
        #         self.And(
        #             self.Not(self.identical(t1, t2)),
        #             self.Not(self.identical(p1, p2)),
        #             self.state(t1, p1),
        #             self.state(t2, p2)),
        #         self.evaluation(
        #             self.predicate("colocated"),
        #             self.list(t1, t2)).truth_value(0.0, 1.0)))
        # self.evaluation(
        #     self.identical(
        #         self.set(self.concept("has_crate")),
        #         self.get(self.state(hum1, s1))))

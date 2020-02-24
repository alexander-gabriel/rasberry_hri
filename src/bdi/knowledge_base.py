import sys
from threading import RLock
import time

from opencog.atomspace import AtomSpace, TruthValue
from opencog.atomspace import types
# from opencog.type_constructors import *
from opencog.ure import BackwardChainer, ForwardChainer
from opencog.utilities import initialize_opencog
from opencog.scheme_wrapper import scheme_eval
from opencog.bindlink import execute_atom, evaluate_atom

import rospy


ITERATIONS = rospy.get_param("iterations", "30")
COMPLEXITY_PENALTY = rospy.get_param("complexity_penalty", 0.1)
# set_type_ctor_atomspace(self.atomspace)

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
        self.queryspace = AtomSpace()
        self.atomspace = self.queryspace
        # self.atomspace = AtomSpace()
        rospy.loginfo("KNB: got atomspaces")
        self.lock = RLock()
        self.TRUE = TruthValue(1,1)
        self.FALSE = TruthValue(0,1)
        rbs = self.build_deduction_rulebase()
        self.build_linked_deduction(rbs)
        self.build_ontology()
        # self.build_inheritance_deduction(rbs)
        rospy.loginfo("KNB: Initialization finished")


    def reason(self, query, variables, atsp=None):
        max_prob = 0
        formula = None
        rospy.logdebug("WST: Asking KB; query: {:}\n {}".format(str(query), variables))
        start_time = time.time()
        with self.lock:
            chainer = BackwardChainer(self.atomspace,
                          self.concept("deduction-rule-base"),
                          query, vardecl = variables)
            chainer.do_chain()
        results = chainer.get_results()
        rospy.logdebug("WST: do_chain -- {:.4f}".format(time.time() - start_time))
        got_one = False
        for result in results.get_out():
            if result.tv == self.TRUE:
                got_one = True
                # rospy.loginfo("WST: Chaining Result Truth: {:}".format(result.tv))
                # rospy.loginfo("WST: Chaining Result:{:}".format(result))
                # rospy.loginfo("WST: ---------------------")
        try: #---this section is for result reasoning debugging...currently not working: linked, is_a
            if not got_one:

                if query.type_name == "GetLink":
                    query = query.get_out()[1]
                elif query.type_name == "StateLink" or query.type_name == "EvaluationLink":
                    with self.lock:
                        chainer = BackwardChainer(self.atomspace,
                                      self.concept("deduction-rule-base"),
                                      query)
                        chainer.do_chain()
                    got_anoter_one =False
                    for result in results.get_out():
                        got_anoter_one = True
                        if result.tv != self.TRUE:
                            rospy.logwarn("WST: Failed: {:}".format(query))
                    if not got_anoter_one:
                        rospy.logwarn("WST: No results for: {:}".format(query))
                else:
                    # rospy.loginfo("default supported query? {}".format(query))
                    for condition in query.get_out():
                        with self.lock:
                            chainer = BackwardChainer(self.atomspace,
                                          self.concept("deduction-rule-base"),
                                          condition)
                            chainer.do_chain()
                        got_anoter_one =False
                        for result in results.get_out():
                            got_anoter_one = True
                            if result.tv != self.TRUE:
                                rospy.logwarn("WST: Failed: {:}".format(condition))
                        if not got_anoter_one:
                            rospy.logwarn("WST: No results for: {:}".format(condition))
                            # if condition.type_name == "VariableList":
                            #     raise Exception()
        except Exception as err:
            rospy.logerr(err)
        return results


    def check(self, query, variables):
        with self.lock:
            rospy.loginfo("check variables: {}".format(variables))
            rospy.loginfo("check query: {}".format(query))
            results = execute_atom(self.atomspace, self.get(variables, query))
            rospy.loginfo("check results: {}".format(results))
        for result in results.get_out():
            if result.tv == self.TRUE:
                rospy.loginfo("WST: Execution Result Truth: {:}".format(result.tv))
                rospy.loginfo("WST: Execution Result:{:}".format(result))
                rospy.loginfo("WST: ---------------------")
        return results


    def check2(self, query):
        with self.lock:
            results = evaluate_atom(self.atomspace, query)
        for result in results.get_out():
            if result.tv == self.TRUE:
                rospy.loginfo("WST: Evalutation Result Truth: {:}".format(result.tv))
                rospy.loginfo("WST: Evalutation Result:{:}".format(result))
                rospy.loginfo("WST: ---------------------")
        return results


    def add_link(self, link, truth_value=None, atsp=None):
        truth_value = self.TRUE if truth_value is None else truth_value
        atsp = self.queryspace if atsp is None else atsp
        with lock:
            atsp.add_link(link, tv=truth_value)


    def add_node(self, atsp=None, node=None):
        atsp = self.queryspace if atsp is None else atsp
        with lock:
            atsp.add_node(node)


    def number(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.NumberNode, name)


    def concept(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.ConceptNode, name)


    def predicate(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.PredicateNode, name)


    def type(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.TypeNode, name)


    def variable(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.VariableNode, name)


    def schema(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.SchemaNode, name)


    def defined_schema(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.DefinedSchemaNode, name)


    def grounded_schema(self, name, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            return atsp.add_node(types.GroundedSchemaNode, name)


    def variable_link(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.VariableLink, subnodes)


    def typed_variable(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.TypedVariableLink, subnodes)


    def variable_list(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.VariableList, subnodes)


    def list(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.ListLink, subnodes)


    def inheritance(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            # is_a = atsp.add_node(types.PredicateNode, "is a")
            # list_link = atsp.add_link(types.ListLink, subnodes)
            # return atsp.add_link(types.EvaluationLink, [is_a, list_link])
            return atsp.add_link(types.InheritanceLink, subnodes)


    def evaluation(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.EvaluationLink, subnodes)


    def execution(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.ExecutionLink, subnodes)


    def execution_output(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.ExecutionOutputLink, subnodes)


    def state(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.StateLink, subnodes)


    def present(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.PresentLink, subnodes)


    def exists(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.ExistsLink, subnodes)


    def for_all(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.ForAllLink, subnodes)


    def get(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.GetLink, subnodes)


    def And(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.AndLink, subnodes)


    def Or(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.OrLink, subnodes)


    def Not(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.NotLink, subnodes)


    def equal(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.NotLink, subnodes)


    def member(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.MemberLink, subnodes)


    def set(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.SetLink, subnodes)


    def identical(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.IdenticalLink, subnodes)


    def define(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.DefineLink, subnodes)


    def bind(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.BindLink, subnodes)


    def define(self, *subnodes, **kwargs):
        atsp = kwargs["atsp"] if "atsp" in kwargs and kwargs["atsp"] is not None else self.queryspace
        with self.lock:
            return atsp.add_link(types.DefineLink, subnodes)


    def build_deduction_rulebase(self, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
        with self.lock:
            rbs = self.concept("deduction-rule-base")
            execute_code = \
            '''
            (use-modules (opencog))
            (use-modules (opencog logger) (opencog ure) (opencog exec) (opencog python))
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/term/inheritance-direct-introduction.scm") ; loading error
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/term/crisp-deduction.scm") ; loading error

            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/instantiation.scm")
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/crisp/propositional/true-conjunction-introduction.scm")
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-disjunction-introduction.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/modus-ponens.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/term/deduction.scm")
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/precise-modus-ponens.scm")
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/inheritance-to-member.scm")
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/member-to-evaluation.scm")
            ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/member-to-inheritance.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/negation-introduction.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/not-simplification.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/not-elimination.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/implication-instantiation.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/predicate/conditional-direct-evaluation.scm")

            (load-from-path "/home/rasberry/git/opencog/opencog/pln/meta-rules/predicate/conditional-full-instantiation.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/meta-rules/predicate/conditional-partial-instantiation.scm")
            (load-from-path "/home/rasberry/git/opencog/opencog/pln/meta-rules/predicate/universal-full-instantiation.scm")
            (define rbs (Concept "deduction-rule-base"))
            (ure-set-complexity-penalty rbs {:f})
            '''.format(COMPLEXITY_PENALTY)
            scheme_eval(atsp, execute_code)
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
            self.member(self.defined_schema("conditional-direct-evaluation-implication-scope-rule"), rbs)
            self.member(self.defined_schema("conditional-full-instantiation-implication-meta-rule"), rbs)
            self.member(self.defined_schema("conditional-full-instantiation-inheritance-meta-rule"), rbs)
            self.member(self.defined_schema("conditional-partial-instantiation-meta-rule"), rbs)
            # self.member(self.defined_schema("universal-full-instantiation-forall-1ary-meta-rule"), rbs)
            self.member(self.defined_schema("deduction-inheritance-rule"), rbs)
            self.member(self.defined_schema("deduction-implication-rule"), rbs)
            self.member(self.defined_schema("deduction-subset-rule"), rbs)
            # self.member(self.defined_schema("present-deduction-inheritance-rule"), rbs)
            self.member(self.defined_schema("modus-ponens-inheritance-rule"), rbs)
            self.member(self.defined_schema("modus-ponens-implication-rule"), rbs)
            self.member(self.defined_schema("modus-ponens-subset-rule"), rbs)
            # self.member(self.defined_schema("crisp-contraposition-implication-scope-rule"), rbs)
            # self.member(self.defined_schema("contraposition-implication-rule"), rbs)
            # self.member(self.defined_schema("contraposition-inheritance-rule"), rbs)
            # self.member(self.defined_schema("implication-scope-to-implication-rule"), rbs)
            self.member(self.defined_schema("implication-full-instantiation-rule"), rbs)
            self.member(self.defined_schema("implication-partial-instantiation-rule"), rbs)
            self.member(self.defined_schema("negation-introduction-rule"), rbs)
            self.member(self.defined_schema("not-simplification-rule"), rbs)
            self.member(self.defined_schema("not-elimination-rule"), rbs)
            self.evaluation(self.predicate("URE:attention-allocation"), rbs).truth_value(0, 1)
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



    def build_ontology(self, atsp=None):
        atsp = self.queryspace if atsp is None else atsp
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

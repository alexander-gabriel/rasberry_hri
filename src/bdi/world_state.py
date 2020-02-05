from threading import Lock
from os.path import join

import rospy
import rospkg

from opencog.atomspace import AtomSpace
from opencog.type_constructors import *
from opencog.ure import BackwardChainer, ForwardChainer
# from opencog.utilities import initialize_opencog
from opencog.scheme_wrapper import scheme_eval
from opencog.bindlink import execute_atom, evaluate_atom

# from probcog.MLN import MLN, Database
# from probcog.MLN.util import strFormula

from utils import suppress, TRUE, FALSE, atomspace




rospack = rospkg.RosPack()
path = join(rospack.get_path('rasberry_hri'), 'src', 'bdi')
MLN_FILENAME = join(path, "lcas_bdi.mln")
DB_FILENAME = join(path, "lcas_bdi.db")
VERBOSE = False
ITERATIONS = rospy.get_param("iterations", 30)
COMPLEXITY_PENALTY = rospy.get_param("complexity_penalty", 0.1)



class WorldState():

    def __init__(self, me):
        self.me = me
        self.lock = Lock()
        # set_type_ctor_atomspace(self.atomspace)


        rbs = self.build_deduction_rulebase()
        self.build_linked_deduction(rbs)
        self.build_ontology()
        # self.build_inheritance_deduction(rbs)


    def build_deduction_rulebase(self):
        rbs = ConceptNode("deduction-rule-base")
        execute_code = \
        '''
        (use-modules (opencog))
        (use-modules (opencog logger) (opencog ure) (opencog exec) (opencog python))
        ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/crisp/propositional/true-conjunction-introduction.scm")
        (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm")
        (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-disjunction-introduction.scm")
        (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/negation-introduction.scm")
        (define rbs (Concept "deduction-rule-base"))
        (ure-set-complexity-penalty rbs {:f})
        '''.format(COMPLEXITY_PENALTY)
        scheme_eval(atomspace, execute_code)
        # MemberLink(DefinedSchemaNode("true-conjunction-introduction-1ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("true-conjunction-introduction-2ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("true-conjunction-introduction-3ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("true-conjunction-introduction-4ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("true-conjunction-introduction-5ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-1ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-2ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-3ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-4ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-5ary-rule"), rbs)
        MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-6ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-7ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-8ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-9ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-1ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-2ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-3ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-4ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-5ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-6ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-7ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-8ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("fuzzy-disjunction-introduction-9ary-rule"), rbs)
        # MemberLink(DefinedSchemaNode("bc-deduction-rule"), rbs)
        # MemberLink(DefinedSchemaNode("conditional-full-instantiation-implication-meta-rule"), rbs)
        # MemberLink(DefinedSchemaNode("conditional-full-instantiation-inheritance-meta-rule"), rbs)
        # MemberLink(DefinedSchemaNode("conditional-partial-instantiation-meta-rule"), rbs)
        # MemberLink(DefinedSchemaNode("universal-full-instantiation-forall-1ary-meta-rule"), rbs)
        # MemberLink(DefinedSchemaNode("deduction-inheritance-rule"), rbs)
        # MemberLink(DefinedSchemaNode("deduction-implication-rule"), rbs)
        # MemberLink(DefinedSchemaNode("deduction-subset-rule"), rbs)
        # MemberLink(DefinedSchemaNode("modus-ponens-inheritance-rule"), rbs)
        # MemberLink(DefinedSchemaNode("modus-ponens-implication-rule"), rbs)
        # MemberLink(DefinedSchemaNode("modus-ponens-subset-rule"), rbs)
        # MemberLink(DefinedSchemaNode("crisp-contraposition-implication-scope-rule"), rbs)
        # MemberLink(DefinedSchemaNode("contraposition-implication-rule"), rbs)
        # MemberLink(DefinedSchemaNode("contraposition-inheritance-rule"), rbs)
        # MemberLink(DefinedSchemaNode("implication-scope-to-implication-rule"), rbs)
        # MemberLink(DefinedSchemaNode("implication-full-instantiation-rule"), rbs)
        # MemberLink(DefinedSchemaNode("implication-partial-instantiation-rule"), rbs)
        # MemberLink(DefinedSchemaNode("negation-introduction-rule"), rbs)
        # MemberLink(DefinedSchemaNode("not-simplification-rule"), rbs)
        # MemberLink(DefinedSchemaNode("not-elimination-rule"), rbs)
        EvaluationLink(PredicateNode("URE:attention-allocation"), rbs).truth_value(0, 1)
        ExecutionLink(SchemaNode("URE:maximum-iterations"), rbs, NumberNode(ITERATIONS))
        return rbs



    def build_inheritance_deduction(self,deduction_rbs):
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


    def build_implication_deduction(self, deduction_rbs):
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



    def build_linked_deduction(self, deduction_rbs):
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


    def build_ontology(self):
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
        self.robot = ConceptNode("robot").truth_value(1.0, 1.0)
        self.human = ConceptNode("human").truth_value(1.0, 1.0)
        InheritanceLink(creature, organism).truth_value(1.0, 1.0)
        InheritanceLink(self.robot, creature).truth_value(1.0, 1.0)
        InheritanceLink(self.human, creature).truth_value(1.0, 1.0)

        self.crate = ConceptNode("crate").truth_value(1.0, 1.0)
        InheritanceLink(self.crate, obj).truth_value(1.0, 1.0)


        # p1 = VariableNode("place1")
        # p2 = VariableNode("place2")
        # p3 = VariableNode("place3")
        # t1 = VariableNode("thing1")
        # s1 = VariableNode("state1")
        # t2 = VariableNode("thing2")
        # hum1 = VariableNode("human1")
        # obj1 = VariableNode("object1")
        # cre1 = VariableNode("creature1")

        # ForAllLink(
        #     p1,
        #     EvaluationLink(
        #         PredicateNode("leads_to"),
        #         ListLink(p1, p1)).truth_value(0.0, 1.0))
        #
        # ForAllLink(
        #     VariableList(p1, p2),
        #     ImplicationLink(
        #         EvaluationLink(
        #             PredicateNode("leads_to"),
        #             ListLink(p1, p2)),
        #             EvaluationLink(
        #                 PredicateNode("linked"),
        #                 ListLink(p1, p2)).truth_value(1.0, 1.0)))
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
        #
        # ForAllLink(
        #     VariableList(p1, p2),
        #     ImplicationLink(
        #         AndLink(
        #             EvaluationLink(
        #                 PredicateNode("leads_to"),
        #                 ListLink(p1, p2)),
        #             NotLink(
        #                 ExistsLink(t1, StateLink(t1, p2))),
        #             OrLink(
        #                 EvaluationLink(
        #                     PredicateNode("can_reach"),
        #                     ListLink(p2, p3)),
        #                 IdenticalLink(p1, p3))),
        #         EvaluationLink(
        #             PredicateNode("free_path"),
        #             ListLink(p1, p3)).truth_value(1.0, 1.0)))
        #
        # ForAllLink(
        #     VariableList(p1, p2, obj1, cre1),
        #     ImplicationLink(
        #         OrLink(
        #             InheritanceLink(obj1, obj),
        #             InheritanceLink(obj1, plant)),
        #         EvaluationLink(
        #             PredicateNode("can_reach"),
        #             ListLink(p1, p2)).truth_value(0.0, 1.0)))
        # ForAllLink(
        #     VariableList(p1, p2, obj1, cre1),
        #     ImplicationLink(
        #         AndLink(
        #             OrLink(
        #                 InheritanceLink(obj1, obj),
        #                 InheritanceLink(obj1, plant)),
        #             InheritanceLink(cre1, creature),
        #             StateLink(cre1, p1),
        #             StateLink(obj1, p2),
        #             OrLink(
        #                 EvaluationLink(
        #                     PredicateNode("free_path"),
        #                     ListLink(p1, p2)),
        #                 EvaluationLink(
        #                     PredicateNode("leads_to"),
        #                     ListLink(p1, p2)))),
        #         EvaluationLink(
        #             PredicateNode("can_reach"),
        #             ListLink(p1, p2)).truth_value(1.0, 1.0)))
        #
        # ForAllLink(
        #     VariableList(t1, t2, p1),
        #     ImplicationLink(
        #         AndLink(
        #             NotLink(IdenticalLink(t1, t2)),
        #             StateLink(t1, p1),
        #             StateLink(t2, p1)),
        #         EvaluationLink(
        #             PredicateNode("colocated"),
        #             ListLink(t1, t2)).truth_value(1.0, 1.0)))
        # ForAllLink(
        #     VariableList(t1, t2, p1, p2),
        #     ImplicationLink(
        #         AndLink(
        #             NotLink(IdenticalLink(t1, t2)),
        #             NotLink(IdenticalLink(p1, p2)),
        #             StateLink(t1, p1),
        #             StateLink(t2, p2)),
        #         EvaluationLink(
        #             PredicateNode("colocated"),
        #             ListLink(t1, t2)).truth_value(0.0, 1.0)))
        # EvaluationLink(
        #     IdenticalLink(
        #         SetLink(ConceptNode("has_crate")),
        #         GetLink(StateLink(hum1, s1))))


    # add a place ConceptNode to the KB
    def add_place(self, name, truth_value=TRUE):
        node1 = ConceptNode(name)
        node1.tv = truth_value
        link = InheritanceLink(node1, self.place)
        link.tv = truth_value

    # add two 'linked' places
    def add_place_link(self, place1, place2, truth_value=TRUE):
        p1 = ConceptNode(place1)
        p1.tv = truth_value
        link = InheritanceLink(p1, ConceptNode("place"))
        link.tv = truth_value
        p2 = ConceptNode(place2)
        p2.tv = truth_value
        link = InheritanceLink(p2, ConceptNode("place"))
        link.tv = truth_value
        # link = EvaluationLink(
        #     PredicateNode("leads_to"),
        #     ListLink(p1, p2))
        # link.tv = truth_value
        link = EvaluationLink(
            PredicateNode("linked"),
            ListLink(p1, p2))
        link.tv = truth_value
        rospy.logdebug("WST: Adding place link: {:} to {:}".format(place1, place2))

    # add arbitrary typed things to the KB
    def add_thing(self, name, klasse, truth_value=TRUE):
        node1 = ConceptNode(name)
        node1.tv = truth_value
        node2 = ConceptNode(klasse)
        node2.tv = truth_value
        link = InheritanceLink(node1, node2)
        link.tv = truth_value


    # not needed since initialisation in utils
    def add_link(self, link, truth_value=TRUE):
        raise NotImplementedError()
        atomspace.add_link(link, tv=truth_value)

    # not needed since initialisation in utils
    def add_node(self, node):
        raise NotImplementedError()
        atomspace.add_node(node)


    def get_probability(self, belief):
        raise NotImplementedError()
        self.lock.acquire()
        # probability = self.db.evidence[belief]
        probability = link.TRUE[0]
        self.lock.release()
        return probability


    def update_position(self, person, place):
        node1 = ConceptNode(person)
        node2 = ConceptNode(place)
        link = StateLink(node1, node2).tv = TRUE
        rospy.loginfo("WST: {:} is at: {:}".format(person, place))


    def set_probability(self, belief, truth):
        raise NotImplementedError()
        self.lock.acquire()
        belief.TRUE = truth
        # with suppress(Exception):
        #     self.db.readContent("{:f} {:}".format(truth, belief))
        #     rospy.logdebug("WST: Adding belief: {:f} {:}".format(truth, belief))
        self.lock.release()


    def abandon_belief(self, belief):
        raise NotImplementedError()
        self.lock.acquire()
        with suppress(KeyError):
            del self.db.evidence[belief]
            rospy.logdebug("WST: Retracting belief: {:}".format(belief))
        self.lock.release()


    def reason(self, query, variables):
        self.lock.acquire()
        max_prob = 0
        formula = None
        # query = EvaluationLink(PredicateNode("linked"),ListLink(ConceptNode("WayPoint104"), ConceptNode("WayPoint106")))
        # variables = VariableList()
        rospy.logdebug("WST: Asking MLN system; query: {:}".format(str(query)))
        chainer = BackwardChainer(atomspace,
                          ConceptNode("deduction-rule-base"),
                          query, vardecl = variables)
                          # GetLink(variables, query), vardecl = variables)
        chainer.do_chain()
        results = chainer.get_results()
        for result in results.get_out():
            rospy.logdebug("WST: Result Truth: {:}".format(result.tv))
            rospy.logdebug("WST: Result:{:}".format(result))
            rospy.logdebug("WST: ------")
        results = execute_atom(atomspace, GetLink(variables, query))
        self.lock.release()
        return results


    def check(self, query):
        self.lock.acquire()
        results = execute_atom(atomspace, query)
        self.lock.release()
        return results


    def write(self):
        raise NotImplementedError()
        self.lock.acquire()
        self.db.write()
        self.lock.release()


    def truth(self, belief):
        raise NotImplementedError()
        return self.db.truth(belief)
        self.lock.release()


    # def add_belief(self, belief, probability=1):
        # self.atomspace.add_node()
        # self.atomspace.add_link(types.SimilarityLink, [node1,node2])

        # self.set_probability(belief, probability)


    def add_constant(self, domain, constants):
        raise NotImplementedError()
        self.mln.constant(domain, constants)


    def add_predicate(self, predicate):
        raise NotImplementedError()
        self.mln.declare_predicate(predicate)


    def add_formula(self, formula, weight, fixweight=True):
        raise NotImplementedError()
        self.mln.formula(formula=formula, weight=weight, fixweight=fixweight, unique_templvars=None)


    # def ground(self):
    #     return self.mln.ground(self.db)


    def learn(self):
        raise NotImplementedError()
        # 'WCSP (exact MPE with toulbar2)'
        self.lock.acquire()
        self.mln.learn([self.db], method="<class 'pracmln.mln.learning.bpll.BPLL'>", **params)
        self.lock.release()


    # def _materialize(self):
    #     self.lock.acquire()
    #     self.mln.materialize(self.db)
    #     self.lock.release()


    def save(self):
        raise NotImplementedError()
        raise NotImplementedError
        with open(DB_FILENAME, "w") as file:
            self.lock.acquire()
            self.db.write(file, bars=False)
            self.lock.release()
        self.mln.tofile(MLN_FILENAME)


    def write_mln(self):
        raise NotImplementedError()
        self.mln.write()


    def write_evidence(self):
        raise NotImplementedError()
        self.lock.acquire()
        print(self.db.evidence)
        self.lock.release()



    def write(self):
        raise NotImplementedError()
        self.write_mln()
        print("-------------------------------------------------------------------")
        self.write_evidence()

from threading import Lock
from os.path import join

import rospy
import rospkg

from opencog.type_constructors import *
from opencog.ure import BackwardChainer, ForwardChainer
# from probcog.MLN import MLN, Database
# from probcog.MLN.util import strFormula

from utils import suppress, TRUE, FALSE




rospack = rospkg.RosPack()
path = join(rospack.get_path('rasberry_hri'), 'src', 'bdi')
MLN_FILENAME = join(path, "lcas_bdi.mln")
DB_FILENAME = join(path, "lcas_bdi.db")
VERBOSE = False
MAX_STEPS = 40
#INFERENCE_METHOD = 'WCSP (exact MPE with toulbar2)'
INFERENCE_METHOD = 'MC-SAT'



class WorldState():


    def __init__(self, me):
        self.me = me
        self.lock = Lock()

        # self.atomspace = AtomSpace()

        # set_type_ctor_atomspace(self.atomspace)
        self.build_ontology()


    def build_deduction(self):
        p1 = VariableNode("place1")
        p2 = VariableNode("place2")
        p3 = VariableNode("place3")
        InheritanceLink(ConceptNode("PLN-location-aware"), ConceptNode("PLN"))
        DefineLink(
            DefinedSchemaNode("linked-awareness"),
            BindLink(
                VariableList(p1, p2, p3),
                AndLink(
                    NotLink(
                        IdenticalLink(p1, p3)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p1, p2)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p2, p3))),
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(p1, p3)).truth_value(1.0, 1.0)))
        # DefineLink(DefinedSchemaNode("linked-awareness"), BindLink(VariableList(!!variables!!), AndLink(!!clauses!!), ExecutionOutputLink(!!formula!!, ListLink(!!conclusion!!, Setlink(!!premises!!)))))
        # DefineLink(DefinedSchemaNode("free-path-awareness"), BindLink(VariableList(!!variables!!), AndLink(!!clauses!!), ExecutionOutputLink(!!formula!!, ListLink(!!conclusion!!, Setlink(!!premises!!)))))
        MemberLink(DefinedSchemaNode("linked-awareness"), ConceptNode ("PLN-location-aware"))
        # MemberLink(DefinedSchemaNode("free-path-awareness"), ConceptNode ("PLN-location-aware"))

        # common configuration
        # ExecutionLink(SchemaNode("URE:maximum-iterations"), ConceptNode("PLN-location-aware"), NumberNode(20))
        # ExecutionLink(SchemaNode("URE:complexity-penalty"), ConceptNode("PLN-location-aware"), NumberNode(0.1))
        # fc configuration
        # EvaluationLink(PredicateNode("URE:FC:retry-exhausted-sources"), ConceptNode("PLN-location-aware")).truth_value(1,1)
        # bc configuration
        # ExecutionLink(SchemaNode("URE:BC:maximum-bit-size"), ConceptNode("PLN-location-aware"), NumberNode(1000))


    def build_ontology(self):
        thing = ConceptNode("thing").truth_value(1.0, 1.0)

        concept = ConceptNode("concept").truth_value(1.0, 1.0)
        InheritanceLink(concept, thing).truth_value(1.0, 1.0)

        self.place = ConceptNode("place").truth_value(1.0, 1.0)
        InheritanceLink(self.place, concept).truth_value(1.0, 1.0)


        entity = ConceptNode("entity").truth_value(1.0, 1.0)
        InheritanceLink(entity, thing).truth_value(1.0, 1.0)


        obj = ConceptNode("object").truth_value(1.0, 1.0)
        organism = ConceptNode("organism").truth_value(1.0, 1.0)
        InheritanceLink(organism, entity).truth_value(1.0, 1.0)
        InheritanceLink(obj, entity).truth_value(1.0, 1.0)

        plant = ConceptNode("plant").truth_value(1.0, 1.0)
        InheritanceLink(plant, organism).truth_value(1.0, 1.0)

        self.strawberryplant = ConceptNode("strawberryplant").truth_value(1.0, 1.0)
        InheritanceLink(self.strawberryplant, plant).truth_value(1.0, 1.0)

        creature = ConceptNode("creature").truth_value(1.0, 1.0)
        self.robot = ConceptNode("robot").truth_value(1.0, 1.0)
        self.human = ConceptNode("human").truth_value(1.0, 1.0)
        InheritanceLink(creature, organism).truth_value(1.0, 1.0)
        InheritanceLink(self.robot, creature).truth_value(1.0, 1.0)
        InheritanceLink(self.human, creature).truth_value(1.0, 1.0)

        self.crate = ConceptNode("crate").truth_value(1.0, 1.0)
        InheritanceLink(self.crate, obj).truth_value(1.0, 1.0)

        p1 = VariableNode("place1")
        p2 = VariableNode("place2")
        p3 = VariableNode("place3")
        t1 = VariableNode("thing1")
        s1 = VariableNode("state1")
        t2 = VariableNode("thing2")
        hum1 = VariableNode("human1")
        obj1 = VariableNode("object1")
        cre1 = VariableNode("creature1")

        ForAllLink(
            p1,
            EvaluationLink(
                PredicateNode("leads_to"),
                ListLink(p1, p1)).truth_value(0.0, 1.0))

        ForAllLink(
            VariableList(p1, p2),
            ImplicationLink(
                EvaluationLink(
                    PredicateNode("leads_to"),
                    ListLink(p1, p2)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p1, p2)).truth_value(1.0, 1.0)))

        ForAllLink(
            VariableList(p1, p2, p3),
            ImplicationLink(
                AndLink(
                    NotLink(
                        IdenticalLink(p1, p3)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p1, p2)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p2, p3))),
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(p1, p3)).truth_value(1.0, 1.0)))

        ForAllLink(
            VariableList(p1, p2),
            ImplicationLink(
                AndLink(
                    EvaluationLink(
                        PredicateNode("leads_to"),
                        ListLink(p1, p2)),
                    NotLink(
                        ExistsLink(t1, StateLink(t1, p2))),
                    OrLink(
                        EvaluationLink(
                            PredicateNode("can_reach"),
                            ListLink(p2, p3)),
                        IdenticalLink(p1, p3))),
                EvaluationLink(
                    PredicateNode("free_path"),
                    ListLink(p1, p3)).truth_value(1.0, 1.0)))

        ForAllLink(
            VariableList(p1, p2, obj1, cre1),
            ImplicationLink(
                OrLink(
                    InheritanceLink(obj1, obj),
                    InheritanceLink(obj1, plant)),
                EvaluationLink(
                    PredicateNode("can_reach"),
                    ListLink(p1, p2)).truth_value(0.0, 1.0)))
        ForAllLink(
            VariableList(p1, p2, obj1, cre1),
            ImplicationLink(
                AndLink(
                    OrLink(
                        InheritanceLink(obj1, obj),
                        InheritanceLink(obj1, plant)),
                    InheritanceLink(cre1, creature),
                    StateLink(cre1, p1),
                    StateLink(obj1, p2),
                    OrLink(
                        EvaluationLink(
                            PredicateNode("free_path"),
                            ListLink(p1, p2)),
                        EvaluationLink(
                            PredicateNode("leads_to"),
                            ListLink(p1, p2)))),
                EvaluationLink(
                    PredicateNode("can_reach"),
                    ListLink(p1, p2)).truth_value(1.0, 1.0)))

        ForAllLink(
            VariableList(t1, t2, p1),
            ImplicationLink(
                AndLink(
                    NotLink(IdenticalLink(t1, t2)),
                    StateLink(t1, p1),
                    StateLink(t2, p1)),
                EvaluationLink(
                    PredicateNode("colocated"),
                    ListLink(t1, t2)).truth_value(1.0, 1.0)))
        ForAllLink(
            VariableList(t1, t2, p1, p2),
            ImplicationLink(
                AndLink(
                    NotLink(IdenticalLink(t1, t2)),
                    NotLink(IdenticalLink(p1, p2)),
                    StateLink(t1, p1),
                    StateLink(t2, p2)),
                EvaluationLink(
                    PredicateNode("colocated"),
                    ListLink(t1, t2)).truth_value(0.0, 1.0)))
        # EvaluationLink(
        #     IdenticalLink(
        #         SetLink(ConceptNode("has_crate")),
        #         GetLink(StateLink(hum1, s1))))


    def add_place(self, name, truth_value=TRUE):
        node1 = ConceptNode(name)
        node1.tv = truth_value
        link = InheritanceLink(node1, self.place)
        link.tv = truth_value


    def add_place_link(self, place1, place2, truth_value=TRUE):
        p1 = ConceptNode(place1)
        p1.tv = truth_value
        p2 = ConceptNode(place2)
        p2.tv = truth_value
        link = EvaluationLink(
            PredicateNode("leads_to"),
            ListLink(p1, p2))
        link.tv = truth_value


    def add_thing(self, name, klasse, truth_value=TRUE):
        node1 = ConceptNode(name)
        node1.tv = truth_value
        node2 = ConceptNode(klasse)
        node2.tv = truth_value
        link = InheritanceLink(node1, node2)
        link.tv = truth_value


    def add_link(self, link, truth_value=TRUE):
        atomspace.add_link(link, tv=truth_value)


    def add_node(self, node):
        atomspace.add_node(node)


    def get_probability(self, belief):
        self.lock.acquire()
        # probability = self.db.evidence[belief]
        probability = link.TRUE[0]
        self.lock.release()
        return probability


    def update_position(self, person, place):
        node1 = ConceptNode(person)
        node2 = ConceptNode(place)
        link = StateLink(node1, node2)
        link.tv = TRUE


    def set_probability(self, belief, truth):
        self.lock.acquire()
        belief.TRUE = truth
        # with suppress(Exception):
        #     self.db.readContent("{:f} {:}".format(truth, belief))
        #     rospy.loginfo("WS: Adding belief: {:f} {:}".format(truth, belief))
        self.lock.release()


    def abandon_belief(self, belief):
        self.lock.acquire()
        with suppress(KeyError):
            del self.db.evidence[belief]
            rospy.loginfo("WS: Retracting belief: {:}".format(belief))
        self.lock.release()


    def reason(self):
        # mrf = self.mln.groundMRF(self.db)
        # result = mrf.inferMCSAT([], verbose=False)
        self.lock.acquire()
        # for belief, truth in result.result_dict().items():
        #     self.db.evidence[belief] = truth
        self.lock.release()


    def check(self, query):
        self.lock.acquire()
        rospy.loginfo("GOL: Asking MLN system; query: {:}".format(str(query)))
        max_prob = 0
        formula = None
        chainer = BackwardChainer(atomspace,
                          ConceptNode("PLN-location-aware"),
                          query)
        chainer.do_chain()
        results = chainer.get_results()
        rospy.loginfo("WS: Result is: {:}".format(results))
        self.lock.release()
        return results


    def write(self):
        self.lock.acquire()
        self.db.write()
        self.lock.release()


    def truth(self, belief):
        return self.db.truth(belief)
        self.lock.release()


    # def add_belief(self, belief, probability=1):
        # self.atomspace.add_node()
        # self.atomspace.add_link(types.SimilarityLink, [node1,node2])

        # self.set_probability(belief, probability)


    def add_constant(self, domain, constants):
        self.mln.constant(domain, constants)


    def add_predicate(self, predicate):
        self.mln.declare_predicate(predicate)


    def add_formula(self, formula, weight, fixweight=True):
        self.mln.formula(formula=formula, weight=weight, fixweight=fixweight, unique_templvars=None)


    # def ground(self):
    #     return self.mln.ground(self.db)


    def learn(self):
        # 'WCSP (exact MPE with toulbar2)'
        self.lock.acquire()
        self.mln.learn([self.db], method="<class 'pracmln.mln.learning.bpll.BPLL'>", **params)
        self.lock.release()


    # def _materialize(self):
    #     self.lock.acquire()
    #     self.mln.materialize(self.db)
    #     self.lock.release()


    def save(self):
        raise NotImplementedError
        with open(DB_FILENAME, "w") as file:
            self.lock.acquire()
            self.db.write(file, bars=False)
            self.lock.release()
        self.mln.tofile(MLN_FILENAME)


    def write_mln(self):
        self.mln.write()


    def write_evidence(self):
        self.lock.acquire()
        print(self.db.evidence)
        self.lock.release()



    def write(self):
        self.write_mln()
        print("-------------------------------------------------------------------")
        self.write_evidence()

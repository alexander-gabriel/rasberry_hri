from threading import Lock
from os.path import join

import rospy
import rospkg

from probcog.MLN import MLN, Database
from probcog.MLN.util import strFormula

from opencog.atomspace import AtomSpace, types, TruthValue
from opencog.utilities import initialize_opencog
from opencog.type_constructors import *
# from opencog.backwardchainer import BackwardChainer

from utils import suppress

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
        self.atomspace = AtomSpace()
        initialize_opencog(self.atomspace)
        set_type_ctor_atomspace(self.atomspace)
        self.build_ontology()

    def build_ontology(self):
        thing = ConceptNode("thing")

        concept = ConceptNode("concept")
        entity = ConceptNode("entity")
        IntensionalInheritanceLink(entity, thing)
        IntensionalInheritanceLink(concept, thing)

        obj = ConceptNode("object")
        organism = ConceptNode("organism")
        IntensionalInheritanceLink(organism, entity)
        IntensionalInheritanceLink(obj, entity)

        plant = ConceptNode("plant")
        IntensionalInheritanceLink(plant, organism)

        strawberryplant = ConceptNode("strawberryplant")
        IntensionalInheritanceLink(strawberryplant, plant)

        creature = ConceptNode("creature")
        robot = ConceptNode("robot")
        human = ConceptNode("human")
        IntensionalInheritanceLink(creature, organism)
        IntensionalInheritanceLink(robot, creature)
        IntensionalInheritanceLink(human, creature)

        crate = ConceptNode("crate")
        IntensionalInheritanceLink(crate, obj)

        p1 = VariableNode("place1")
        p2 = VariableNode("place2")
        p3 = VariableNode("place3")
        t1 = VariableNode("thing1")
        t2 = VariableNode("thing2")
        hum1 = VariableNode("human1")
        obj1 = VariableNode("object1")
        cre1 = VariableNode("creature1")

        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, human),
                NotLink(IntensionalInheritanceLink(t1, robot))))
        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, robot),
                NotLink(IntensionalInheritanceLink(t1, human))))

        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, creature),
                NotLink(IntensionalInheritanceLink(t1, plant))))
        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, plant),
                NotLink(IntensionalInheritanceLink(t1, creature))))

        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, obj),
                NotLink(IntensionalInheritanceLink(t1, organism))))
        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, organism),
                NotLink(IntensionalInheritanceLink(t1, obj))))

        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, entity),
                NotLink(IntensionalInheritanceLink(t1, concept))))
        ForAllLink(
            VariableList(t1),
            ImplicationLink(
                IntensionalInheritanceLink(t1, concept),
                NotLink(IntensionalInheritanceLink(t1, entity))))

        ForAllLink(
            VariableList(p1, p2),
            ImplicationLink(
                EqualLink(p1, p2),
                NotLink(
                    EvaluationLink(
                        PredicateNode("leads_to"),
                        ListLink(p1, p2)))))

        ForAllLink(
            VariableList(p1, p2),
            ImplicationLink(
                EvaluationLink(
                    PredicateNode("leads_to"),
                    ListLink(p1, p2)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p1, p2))))

        ForAllLink(
            VariableList(p1, p2, p3),
            ImplicationLink(
                AndLink(
                    NotLink(
                        EqualLink(p1, p3)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p1, p2)),
                    EvaluationLink(
                        PredicateNode("linked"),
                        ListLink(p2, p3))),
                EvaluationLink(
                    PredicateNode("linked"),
                    ListLink(p1, p3))))

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
                        EqualLink(p1, p3))),
                EvaluationLink(
                    PredicateNode("free_path"),
                    ListLink(p1, p3))))

        ForAllLink(
            VariableList(p1,p2,obj1,cre1),
            ImplicationLink(
                OrLink(
                    IntensionalInheritanceLink(obj1, obj),
                    IntensionalInheritanceLink(obj1, plant)),
                NotLink(EvaluationLink(
                    PredicateNode("can_reach"),
                    ListLink(p1, p2)))))
        ForAllLink(
            VariableList(p1,p2,obj1,cre1),
            ImplicationLink(
                AndLink(
                    OrLink(
                        IntensionalInheritanceLink(obj1, obj),
                        IntensionalInheritanceLink(obj1, plant)),
                    IntensionalInheritanceLink(cre1, creature),
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
                    ListLink(p1, p2))))

        ForAllLink(
            VariableList(t1, t2, p1),
            ImplicationLink(
                AndLink(
                    NotLink(EqualLink(t1,t2)),
                    StateLink(t1,p1),
                    StateLink(t2,p1)),
                EvaluationLink(
                    PredicateNode("colocated"),
                    ListLink(t1, t2))))
        ForAllLink(
            VariableList(t1, t2, p1, p2),
            ImplicationLink(
                AndLink(
                    NotLink(EqualLink(t1,t2)),
                    NotLink(EqualLink(p1,p2)),
                    StateLink(t1,p1),
                    StateLink(t2,p2)),
                NotLink(EvaluationLink(
                    PredicateNode("colocated"),
                    ListLink(t1, t2)))))
        ForAllLink(
            hum1,
            ImplicationLink(
                StateLink(ListLink(hum1, PredicateNode("has_crate")), NumberNode(1.0)),
                StateLink(ListLink(hum1, PredicateNode("has_requested_crate")), NumberNode(0.0))))



    def add_thing(self, name, klasse):
        node1 = ConceptNode(name)
        node2 = ConceptNode(klasse)
        IntensionalInheritanceLink(node1, node2)
        # link = atomspace.add_link(types.SimilarityLink, [node1,node2])
        # self.atomspace.add_node(node)


    def update_position(self, entity, place):
        node1 = ConceptNode(entity)
        node1 = ConceptNode(place)
        StateLink(entity, place)


    def update_property(self, thing, property, truth):
        node1 = ConceptNode(thing)
        node1 = PredicateNode(property)
        link1 = ListLink(node1,node2)
        link2 = StateLink(link1, NumberNode(truth))

    def add_link(self, link, truth, confidence):
        tv = TruthValue(truth, confidence)
        self.atomspace.add_link(link)


    def add_node(self, node):
        self.atomspace.add_node(node)


    def get_probability(self, belief):
        self.lock.acquire()
        # probability = self.db.evidence[belief]
        probability = link.tv[0]
        self.lock.release()
        return probability


    def set_probability(self, belief, truth):
        self.lock.acquire()
        belief.tv = truth
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


    def check(self, queries):
        self.lock.acquire()
        max_prob = 0
        formula = None
        # chainer = BackwardChainer(atomspace,
        #                   rule_base,
        #                   start_atom,
        #                   trace_as=trace_atomspace)
        # chainer.do_chain()
        # results = chainer.get_results()
        # with suppress(Exception):
        #     mrf = self.mln.groundMRF(self.db)
        #     rospy.logdebug("Grounded MRF")
        #
        #     results = mrf.inferMCSAT(queries, verbose=False, details=False, maxSteps=MAX_STEPS)
        #     index = results.index(max(results))
        #     max_prob = max(results)
        #     formula = mrf.mcsat.queries[index]
        #     # for evidence, truth in result.result_dict().items():
        #     #     bdi.world_state.db.add(evidence, truth)
        self.lock.release()
        return (max_prob, formula)


    def write(self):
        self.lock.acquire()
        self.db.write()
        self.lock.release()


    def truth(self, belief):
        return self.db.truth(belief)
        self.lock.release()


    def add_belief(self, belief, probability=1):
        pass
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

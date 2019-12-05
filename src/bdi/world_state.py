from threading import Lock
from os.path import join

import rospy
import rospkg

from probcog.MLN import MLN, Database
from probcog.MLN.util import strFormula

from opencog.atomspace import AtomSpace, types, TruthValue
from opencog.utilities import initialize_opencog
from opencog.type_constructors import *
from opencog.backwardchainer import BackwardChainer

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
        # self.mln = MLN(MLN_FILENAME)
        self.lock = Lock()
        self.atomspace = AtomSpace()
        initialize_opencog(self.atomspace)
        set_type_ctor_atomspace(self.atomspace)
        # self.db = Database(self.mln, dbfile=DB_FILENAME)


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
        chainer = BackwardChainer(atomspace,
                          rule_base,
                          start_atom,
                          trace_as=trace_atomspace)
        chainer.do_chain()
        results = chainer.get_results()
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

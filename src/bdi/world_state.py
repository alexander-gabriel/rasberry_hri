import rospkg
from os.path import join
from pracmln import MLN, Database, MLNQuery
from pracmln.mln import Predicate
from pracmln.utils.project import PRACMLNConfig, MLNProject

rospack = rospkg.RosPack()
path = join(rospack.get_path('rasberry_hri'), 'src', 'bdi')
MLN_FILENAME = join(path, "lcas_bdi.mln")
DB_FILENAME = join(path, "lcas_bdi.db")
VERBOSE = False
INFERENCE_METHOD = 'WCSP (exact MPE with toulbar2)'



class WorldState():


    def __init__(self):

        self.mln = MLN(mlnfile=MLN_FILENAME, grammar='PRACGrammar', logic='FuzzyLogic')
        self.db = Database(self.mln, dbfile=DB_FILENAME)
        self.query = MLNQuery(mln=self.mln, db=self.db, verbose=VERBOSE)
        self.query._config['method'] = INFERENCE_METHOD
        self.query._config['multicore'] = True


    def get_probability(self, belief, threshhold):
        return self.db.query(belief, threshhold)


    def set_probability(self, belief, strength):
        self.db[belief] = strength


    def abandon_belief(self, belief):
        del self.db[belief]


    def reason(self):
        self.query._config['queries'] = None
        result = self.query.run()
        for evidence, truth in result.result_dict().items():
            bdi.world_state.db.add(evidence, truth)


    def check(self, queries):
        self.query._config['queries'] = queries
        result = self.query.run()
        # for evidence, truth in result.result_dict().items():
        #     bdi.world_state.db.add(evidence, truth)
        return result


    def write(self):
        self.db.write()


    def truth(self, atom):
        return self.mln.truth(atom)


    def add_constant(self, domain, constants):
        self.mln.constant(domain, constants)


    def add_predicate(self, predicate):
        self.mln.declare_predicate(predicate)


    def add_formula(self, formula, weight, fixweight=True):
        self.mln.formula(formula=formula, weight=weight, fixweight=fixweight, unique_templvars=None)


    def ground(self):
        return self.mln.ground(self.db)


    def learn(self):
        # 'WCSP (exact MPE with toulbar2)'
        self.mln.learn([self.db], method="<class 'pracmln.mln.learning.bpll.BPLL'>", **params)


    def _materialize(self):
        self.mln.materialize(self.db)


    def save(self):
        with open(DB_FILENAME, "w") as file:
            self.db.write(file, bars=False)
        self.mln.tofile(MLN_FILENAME)


    def write_mln(self):
        self.mln.write()


    def write_evidence(self):
        self.db.write()


    def write(self):
        self.write_mln()
        print("-------------------------------------------------------------------")
        self.write_evidence()

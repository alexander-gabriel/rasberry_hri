from opencog.type_constructors import *
from opencog.utilities import initialize_opencog
from opencog.ure import BackwardChainer, ForwardChainer
from opencog.atomspace import AtomSpace, types
from opencog.scheme_wrapper import scheme_eval

atomspace = AtomSpace()
initialize_opencog(atomspace)

rbs = ConceptNode("deduction-rule-base")
# MemberLink(rbs, ConceptNode("URE"))


MemberLink(rbs, ConceptNode("URE"))
execute_code = \
  '''
  (use-modules (opencog))
  (use-modules (opencog logger) (opencog ure) (opencog exec))
  (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/crisp/propositional/true-conjunction-introduction.scm")
  (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm")
  (load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/propositional/fuzzy-disjunction-introduction.scm")
  ;(load-from-path "/home/rasberry/git/opencog/opencog/pln/rules/wip/negation-introduction.scm")

  (define rbs (Concept "deduction-rule-base"))
  (ure-set-complexity-penalty rbs 0.1)
  (ure-logger-set-level! "DEBUG")
  (cog-logger-set-level! "DEBUG")
  (cog-logger-set-stdout! #t)
  (cog-logger-set-timestamp! #f)
  (ure-logger-set-stdout! #t)
  (ure-logger-set-timestamp! #f)
  '''
scheme_eval(atomspace, execute_code)
# MemberLink(DefinedSchemaNode("true-conjunction-introduction-1ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("true-conjunction-introduction-2ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("true-conjunction-introduction-3ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("true-conjunction-introduction-4ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("true-conjunction-introduction-5ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-1ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-2ary-rule"), rbs)
MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-3ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-4ary-rule"), rbs)
# MemberLink(DefinedSchemaNode("fuzzy-conjunction-introduction-5ary-rule"), rbs)
ExecutionLink(SchemaNode("URE:maximum-iterations"), rbs, NumberNode("30"))

ConceptNode("A").truth_value(1,1)
ConceptNode("B").truth_value(1,1)
execute_code = \
  '''
(ure-logger-set-level! "debug")

(define A (PredicateNode "A" (stv 0.91 1.0)))
(define B (ConceptNode "B" (stv 0.92 0.71)))
(define C (ConceptNode "C" (stv 0.93 0.72)))
(define D (ConceptNode "D" (stv 0.94 0.73)))
(define SAB (StateLink A B (stv 0.95 0.74)))
(define EAC (EvaluationLink A C (stv 0.96 0.75)))
(define IAD (InheritanceLink A D (stv 0.97 0.76)))

(define SABEACIAD (AndLink SAB EAC IAD))
(cog-bc (ConceptNode "deduction-rule-base") SABEACIAD)
'''
scheme_eval(atomspace, execute_code)
# print(atomspace.get_atoms_by_type(types.Atom))
# query = AndLink(EvaluationLink(PredicateNode("A"), ConceptNode("B")),
# EvaluationLink(PredicateNode("A"), ConceptNode("C")))
# variables = None
# trace = None
# chainer = BackwardChainer(atomspace,
#                         rbs,
#                         query)
# chainer.do_chain()
# results = chainer.get_results()
# print("Result {:}".format(results))
# print("Set Truth: {:}".format(results.tv))
# print("------")
# try:
#     for result in results.get_out():
#         print("Result Truth: {:}".format(result.tv))
# except:
#     pass

#define DEBUG_TYPE "array-simplifying-solver"
#include "klee/Solver.h"
#include "klee/Expr.h"
#include "klee/Constraints.h"
#include "klee/SolverImpl.h"
#include "klee/Internal/Support/Debug.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/util/ExprVisitor.h"
#include "klee/util/ExprUtil.h"
#include "klee/util/Assignment.h"
#include <map>
#include <vector>
#include <ostream>
#include <list>

using namespace klee;
using namespace llvm;


class ArrayIndexCollector : public ExprVisitor {
public:
  std::map<const Array*, ref<Expr> > indices;

  Action visitRead(const ReadExpr& re) {
    // Sanity check
    if (re.getWidth() != Expr::Int8) {
      klee_error("Non-byte read!");
    }

    const Array* root = re.updates.root;
    if (indices.count(root) == 0) {
      indices[root] = re.index;
    } else {
      // ITE(a < b, b, a)
      // KLEE exprs should not use "gt" and be right-imbalanced, according to Expr's documentation
      indices[root] = SelectExpr::create(UltExpr::create(indices[root], re.index), re.index, indices[root]);
    }

    // I guess techically children could include a Read as well
    // maybe we should check that this doesn't happen though, idk if this optimization is sound in that case
    return Action::doChildren();
  }
};

class ArrayReadFixer : public ExprVisitor {
public:
  std::map<const Array*, const Array*> replacements;

  ArrayReadFixer(std::map<const Array*, const Array*> _replacements) : replacements(_replacements) {}

  Action visitRead(const ReadExpr& cre) {
    // This is stupid but it seems the method isn't called if the parameter isn't const...
    ReadExpr& re = (ReadExpr&) cre;

    if (replacements.count(re.updates.root) == 0) {
      // Nothing to do
      return Action::doChildren();
    }

    // replace the array - can't do it in the updatelist (its array is const), but ReadExpr has a non-const updates
    re.updates = UpdateList(replacements[re.updates.root], re.updates.head);

    return Action::doChildren();
  }
};

class ArraySimplifyingSolver : public SolverImpl {
private:
  Solver *solver;
  void fixQuery(const Query& query);

public:
  ArraySimplifyingSolver(Solver *_solver) : solver(_solver) {}
  ~ArraySimplifyingSolver() { delete solver; }

  bool computeTruth(const Query&, bool &isValid);
  bool computeValidity(const Query&, Solver::Validity &result);
  bool computeValue(const Query&, ref<Expr> &result);
  bool computeInitialValues(const Query& query,
                            const std::vector<const Array*> &objects,
                            std::vector< std::vector<unsigned char> > &values,
                            bool &hasSolution);
  SolverRunStatus getOperationStatusCode();
  char *getConstraintLog(const Query&);
  void setCoreSolverTimeout(double timeout);
};

void ArraySimplifyingSolver::fixQuery(const Query& query) {
  ArrayIndexCollector coll;
  coll.visit(query.expr);

  std::map<const Array*, const Array*> replacements;

  for(auto pair : coll.indices) {
    if (pair.first->size < 4096) {
      // not worth it
      continue;
    }

    Query rangeQuery(query.constraints, pair.second);
    auto range = solver->getRange(rangeQuery);

    Query maxQuery(query.constraints, range.second);
    ref<ConstantExpr> maxExpr;
    if(!solver->getValue(maxQuery, maxExpr)) {
      klee_error("couldn't compute max size of array");
    }

    uint64_t maxValue = maxExpr->getZExtValue();
    uint64_t size = maxValue + 1; // maxValue is a 0-based index, size is 1-based!

    klee_warning("Array %s is huge (%d), trimmed it to %d", pair.first->name.c_str(), pair.first->size, size);

    const Array* arr = pair.first;
    const Array* rep;
    if (arr->isConstantArray()) {
      rep = new Array(arr->name, size, &(arr->constantValues[0]), &(arr->constantValues[size]), arr->domain, arr->range);
    } else {
      rep = new Array(arr->name, size, nullptr, nullptr, arr->domain, arr->range);
    }

    replacements[arr] = rep;
  }

  ArrayReadFixer fixer(replacements);
  fixer.visit(query.expr);
}


bool ArraySimplifyingSolver::computeValidity(const Query& query, Solver::Validity &result) {
  fixQuery(query);
  return solver->impl->computeValidity(query, result);
}

bool ArraySimplifyingSolver::computeTruth(const Query& query, bool &isValid) {
  fixQuery(query);
  return solver->impl->computeTruth(query, isValid);
}

bool ArraySimplifyingSolver::computeValue(const Query& query, ref<Expr> &result) {
  fixQuery(query);
  return solver->impl->computeValue(query, result);
}

bool ArraySimplifyingSolver::computeInitialValues(const Query& query,
                                             const std::vector<const Array*> &objects,
                                             std::vector< std::vector<unsigned char> > &values,
                                             bool &hasSolution) {
  fixQuery(query);
  return solver->impl->computeInitialValues(query, objects, values, hasSolution);
}

SolverImpl::SolverRunStatus ArraySimplifyingSolver::getOperationStatusCode() {
  return solver->impl->getOperationStatusCode();
}

char *ArraySimplifyingSolver::getConstraintLog(const Query& query) {
  return solver->impl->getConstraintLog(query);
}

void ArraySimplifyingSolver::setCoreSolverTimeout(double timeout) {
  solver->impl->setCoreSolverTimeout(timeout);
}

Solver *klee::createArraySimplifyingSolver(Solver *s) {
  return new Solver(new ArraySimplifyingSolver(s));
}

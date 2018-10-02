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
#include <utility>
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
  std::map<const Array*, std::pair<const Array*, uint64_t>> replacements;

  ArrayReadFixer(std::map<const Array*, std::pair<const Array*, uint64_t>> _replacements) : replacements(_replacements) {}

  Action visitRead(const ReadExpr& re) {
    if (replacements.count(re.updates.root) == 0) {
      // Nothing to do
      return Action::doChildren();
    }

    auto rep = replacements[re.updates.root];

    // replace the index to account for new minValue, if needed
    auto index = re.index;
    if (rep.second != 0) {
      index = SubExpr::create(re.index, ConstantExpr::create(rep.second, rep.first->getDomain()));
    }

    // replace the array
    auto updates = UpdateList(rep.first, re.updates.head);

    return Action::changeTo(ReadExpr::create(updates, index));
  }
};

struct FixResult {
  ref<Expr> queryExpr;
  const std::vector<ref<Expr>>& queryConstraints;
  std::map<const Array*, std::pair<const Array*, uint64_t>> replacements;
};

class ArraySimplifyingSolver : public SolverImpl {
private:
  Solver *solver;
  FixResult fixQuery(const Query& query);

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

FixResult ArraySimplifyingSolver::fixQuery(const Query& query) {
  ArrayIndexCollector coll;
  coll.visit(query.expr);
  for (auto constraint : query.constraints) {
    coll.visit(constraint);
  }

  std::map<const Array*, std::pair<const Array*, uint64_t>> replacements;

  for(auto pair : coll.indices) {
    if (pair.first->size < 4096) {
      // not worth it
      continue;
    }

    Query rangeQuery(query.constraints, pair.second);
    auto range = solver->getRange(rangeQuery);

    Query minQuery(query.constraints, range.first);
    ref<ConstantExpr> minExpr;
    if(!solver->getValue(minQuery, minExpr)) {
      klee_error("couldn't compute min index of array");
    }
    uint64_t minValue = minExpr->getZExtValue();

    Query maxQuery(query.constraints, range.second);
    ref<ConstantExpr> maxExpr;
    if(!solver->getValue(maxQuery, maxExpr)) {
      klee_error("couldn't compute max index of array");
    }
    uint64_t maxValue = maxExpr->getZExtValue();

    uint64_t size = (maxValue - minValue) + 1;

    klee_warning("Array %s is huge (size %d), trimmed it to range %lu-%lu (size %lu)", pair.first->name.c_str(), pair.first->size, minValue, maxValue, size);

    const Array* arr = pair.first;
    const Array* rep;
    if (arr->isConstantArray()) {
      rep = new Array(arr->name, size, &(arr->constantValues[minValue]), &(arr->constantValues[minValue + size]), arr->domain, arr->range);
    } else {
      rep = new Array(arr->name, size, nullptr, nullptr, arr->domain, arr->range);
    }

    replacements[arr] = std::pair<const Array*, uint64_t>(rep, minValue);
  }

  ArrayReadFixer fixer(replacements);
  auto newQueryExpr = fixer.visit(query.expr);

  std::vector<ref<Expr>>* newConstraints = new std::vector<ref<Expr>>();
  for (auto constraint : query.constraints) {
    auto newConstraint = fixer.visit(constraint);
    newConstraints->push_back(newConstraint);
  }

  return {
    .queryExpr = newQueryExpr,
    .queryConstraints = *newConstraints,
    .replacements = replacements
  };
}


bool ArraySimplifyingSolver::computeValidity(const Query& query, Solver::Validity &result) {
  auto fix = fixQuery(query);
  return solver->impl->computeValidity(Query(ConstraintManager(fix.queryConstraints), fix.queryExpr), result);
}

bool ArraySimplifyingSolver::computeTruth(const Query& query, bool &isValid) {
  auto fix = fixQuery(query);
  return solver->impl->computeTruth(Query(ConstraintManager(fix.queryConstraints), fix.queryExpr), isValid);
}

bool ArraySimplifyingSolver::computeValue(const Query& query, ref<Expr> &result) {
  auto fix = fixQuery(query);
  return solver->impl->computeValue(Query(ConstraintManager(fix.queryConstraints), fix.queryExpr), result);
}

bool ArraySimplifyingSolver::computeInitialValues(const Query& query,
                                                  const std::vector<const Array*> &objects,
                                                  std::vector< std::vector<unsigned char> > &values,
                                                  bool &hasSolution) {
  auto fix = fixQuery(query);
  std::vector<const Array*> replacedObjects;
  for (auto obj : objects) {
    if (fix.replacements.count(obj) == 0) {
      replacedObjects.push_back(obj); // OK, as is
    } else {
      replacedObjects.push_back(fix.replacements[obj].first);
    }
  }

  bool result = solver->impl->computeInitialValues(Query(ConstraintManager(fix.queryConstraints), fix.queryExpr), replacedObjects, values, hasSolution);
  if (!result) {
    return false;
  }

  if (hasSolution) {
    for (unsigned n = 0; n < values.size(); n++) {
      // If necessary, create a fake value with 0 everywhere it wasn't used...
      if (fix.replacements.count(objects[n]) == 1) {
        uint64_t minValue = fix.replacements[objects[n]].second;
        values[n].insert(/* pos */ values[n].begin(), /* count */ minValue, /* value */ 0);
        values[n].resize(/* size */ objects[n]->size, /* value */ 0);
      }
    }
  }

  return true;
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

#include "klee/util/GetExprSymbols.h"

namespace klee {
template<>
SymbolSet ExprAnalysis<GetExprSymbols,
                       SymbolSet >::
visitChildren(const Expr& expr) {
  SymbolSet ret;
  for (unsigned i = 0; i < expr.getNumKids(); ++i) {
    ref<Expr> kid = expr.getKid(i);
    SymbolSet symbols = visit(kid);
    ret.insert(symbols.begin(), symbols.end());
  }
  return ret;
}

//TODO: rewrite in terms of individual read-exprs (parts of the
// arrays), instead of using the array-granularity
template<>
SymbolSet ExprAnalysis<GetExprSymbols,
                       SymbolSet >::
visitRead(const ReadExpr& expr) {
  SymbolSet ret = visit(expr.index);
  ret.insert(expr.updates.root);
  return ret;
}
}

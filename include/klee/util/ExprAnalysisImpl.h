//===-- ExprAnalysisImpl.h --------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXPRANALYSIS_H
#  error "This file is supposed to be included into ExprAnalysis.h"
#endif

namespace klee {

template<class Analysis, class Result>
Result ExprAnalysis<Analysis, Result>::visit(const ref<Expr> &e) {
  const Expr& ep = *e.get();
  switch(ep.getKind()) {
  case Expr::NotOptimized: return visitNotOptimized(static_cast<const NotOptimizedExpr&>(ep));
  case Expr::Read: return visitRead(static_cast<const ReadExpr&>(ep));
  case Expr::Select: return visitSelect(static_cast<const SelectExpr&>(ep));
  case Expr::Concat: return visitConcat(static_cast<const ConcatExpr&>(ep));
  case Expr::Extract: return visitExtract(static_cast<const ExtractExpr&>(ep));
  case Expr::ZExt: return visitZExt(static_cast<const ZExtExpr&>(ep));
  case Expr::SExt: return visitSExt(static_cast<const SExtExpr&>(ep));
  case Expr::Add: return visitAdd(static_cast<const AddExpr&>(ep));
  case Expr::Sub: return visitSub(static_cast<const SubExpr&>(ep));
  case Expr::Mul: return visitMul(static_cast<const MulExpr&>(ep));
  case Expr::UDiv: return visitUDiv(static_cast<const UDivExpr&>(ep));
  case Expr::SDiv: return visitSDiv(static_cast<const SDivExpr&>(ep));
  case Expr::URem: return visitURem(static_cast<const URemExpr&>(ep));
  case Expr::SRem: return visitSRem(static_cast<const SRemExpr&>(ep));
  case Expr::Not: return visitNot(static_cast<const NotExpr&>(ep));
  case Expr::And: return visitAnd(static_cast<const AndExpr&>(ep));
  case Expr::Or: return visitOr(static_cast<const OrExpr&>(ep));
  case Expr::Xor: return visitXor(static_cast<const XorExpr&>(ep));
  case Expr::Shl: return visitShl(static_cast<const ShlExpr&>(ep));
  case Expr::LShr: return visitLShr(static_cast<const LShrExpr&>(ep));
  case Expr::AShr: return visitAShr(static_cast<const AShrExpr&>(ep));
  case Expr::Eq: return visitEq(static_cast<const EqExpr&>(ep));
  case Expr::Ne: return visitNe(static_cast<const NeExpr&>(ep));
  case Expr::Ult: return visitUlt(static_cast<const UltExpr&>(ep));
  case Expr::Ule: return visitUle(static_cast<const UleExpr&>(ep));
  case Expr::Ugt: return visitUgt(static_cast<const UgtExpr&>(ep));
  case Expr::Uge: return visitUge(static_cast<const UgeExpr&>(ep));
  case Expr::Slt: return visitSlt(static_cast<const SltExpr&>(ep));
  case Expr::Sle: return visitSle(static_cast<const SleExpr&>(ep));
  case Expr::Sgt: return visitSgt(static_cast<const SgtExpr&>(ep));
  case Expr::Sge: return visitSge(static_cast<const SgeExpr&>(ep));
  case Expr::Constant: return visitConstantExpr(static_cast<const ConstantExpr&>(ep));
  default:
    assert(0 && "invalid expression kind");
  }
}

}

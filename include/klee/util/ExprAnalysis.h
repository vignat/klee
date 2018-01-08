//===-- ExprAnalysis.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXPRANALYSIS_H
#define KLEE_EXPRANALYSIS_H

#include "klee/Expr.h"

namespace klee {
  template<class Analysis, class Result>
  class ExprAnalysis {
  protected:

    static Result visitChildren(const Expr&);

    static Result visitConstantExpr(const ConstantExpr& arg) {return visitChildren(arg);}
    static Result visitNotOptimized(const NotOptimizedExpr& arg) {return visitChildren(arg);}
    static Result visitRead(const ReadExpr& arg) {return visitChildren(arg);}
    static Result visitSelect(const SelectExpr& arg) {return visitChildren(arg);}
    static Result visitConcat(const ConcatExpr& arg) {return visitChildren(arg);}
    static Result visitExtract(const ExtractExpr& arg) {return visitChildren(arg);}
    static Result visitZExt(const ZExtExpr& arg) {return visitChildren(arg);}
    static Result visitSExt(const SExtExpr& arg) {return visitChildren(arg);}
    static Result visitAdd(const AddExpr& arg) {return visitChildren(arg);}
    static Result visitSub(const SubExpr& arg) {return visitChildren(arg);}
    static Result visitMul(const MulExpr& arg) {return visitChildren(arg);}
    static Result visitUDiv(const UDivExpr& arg) {return visitChildren(arg);}
    static Result visitSDiv(const SDivExpr& arg) {return visitChildren(arg);}
    static Result visitURem(const URemExpr& arg) {return visitChildren(arg);}
    static Result visitSRem(const SRemExpr& arg) {return visitChildren(arg);}
    static Result visitNot(const NotExpr& arg) {return visitChildren(arg);}
    static Result visitAnd(const AndExpr& arg) {return visitChildren(arg);}
    static Result visitOr(const OrExpr& arg) {return visitChildren(arg);}
    static Result visitXor(const XorExpr& arg) {return visitChildren(arg);}
    static Result visitShl(const ShlExpr& arg) {return visitChildren(arg);}
    static Result visitLShr(const LShrExpr& arg) {return visitChildren(arg);}
    static Result visitAShr(const AShrExpr& arg) {return visitChildren(arg);}
    static Result visitEq(const EqExpr& arg) {return visitChildren(arg);}
    static Result visitNe(const NeExpr& arg) {return visitChildren(arg);}
    static Result visitUlt(const UltExpr& arg) {return visitChildren(arg);}
    static Result visitUle(const UleExpr& arg) {return visitChildren(arg);}
    static Result visitUgt(const UgtExpr& arg) {return visitChildren(arg);}
    static Result visitUge(const UgeExpr& arg) {return visitChildren(arg);}
    static Result visitSlt(const SltExpr& arg) {return visitChildren(arg);}
    static Result visitSle(const SleExpr& arg) {return visitChildren(arg);}
    static Result visitSgt(const SgtExpr& arg) {return visitChildren(arg);}
    static Result visitSge(const SgeExpr& arg) {return visitChildren(arg);}
  public:
    // apply the visitor to the expression and return a possibly
    // modified new expression.
    static Result visit(const ref<Expr> &e);
  };

}

#include "ExprAnalysisImpl.h"

#endif

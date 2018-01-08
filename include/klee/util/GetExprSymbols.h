//===-- GetExprSymbols.h -------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_GETEXPRSYMBOLS_H
#define KLEE_GETEXPRSYMBOLS_H

#include <llvm/ADT/SmallPtrSet.h>
#include "ExprAnalysis.h"

namespace klee {

typedef llvm::SmallPtrSet<const Array*, 100> SymbolSet;

class GetExprSymbols: public ExprAnalysis<GetExprSymbols, SymbolSet > {
};


template<>
SymbolSet ExprAnalysis<GetExprSymbols,
                       SymbolSet >::
visitChildren(const Expr& expr);

template<>
SymbolSet ExprAnalysis<GetExprSymbols,
                       SymbolSet >::
visitRead(const ReadExpr&);

}

#endif

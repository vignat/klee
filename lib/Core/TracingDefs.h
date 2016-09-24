//===-- TracingDefs.h -------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_TRACING_DEF_H
#define KLEE_TRACING_DEF_H

#include <memory>
#include <map>

#include "klee/tracing_definitions.h"

namespace klee {


  template<class K, class V>
  using TracingMap = std::map<K,V>;

  template<class T>
  using uptr = std::unique_ptr<T>;
} // End klee namespace

#endif //KLEE_TRACING_DEF_H

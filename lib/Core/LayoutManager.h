//===-- LayoutManager.h ----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_LAYOUT_MANAGER_H
#define KLEE_LAYOUT_MANAGER_H


#include "TracingDefs.h"
#include "klee/Internal/ADT/ImmutableMap.h"

namespace klee {

  class MetaValue;

  class LayoutManager {
  public:
    LayoutManager() = default;
    LayoutManager(const LayoutManager& lb);

    int plain(uint64_t);
    int ptr(int ptee);
    int firstField(int valLayout, int offset, const std::string& name);
    int nextField(int lastFieldLayout, int valLayout,
                  int offset, const std::string& name);
    int array(int cellLayout, int length);

    MetaValue *getLayout(int layout);

  private:
    ImmutableMap<int, uptr<MetaValue> > layoutParts;
    static int lastIdx;

    int freshId();
    int insertANewLayout(MetaValue *l);
  };
} // End klee namespace

#endif //KLEE_LAYOUT_MANAGER_H

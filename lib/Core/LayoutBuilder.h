//===-- LayoutBuilder.h ----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_LAYOUT_BUILDER_H
#define KLEE_LAYOUT_BUILDER_H


#include "TracingDefs.h"

namespace klee {

  class MetaValue;

  class LayoutBuilder {
  public:
    LayoutBuilder() = default;
    LayoutBuilder(const LayoutBuilder& lb);

    int plain(uint64_t);
    int ptr(int ptee);
    int firstField(int valLayout, int offset, const std::string& name);
    int nextField(int lastFieldLayout, int valLayout,
                  int offset, const std::string& name);
    int array(int cellLayout, int length);

    uptr<MetaValue> buildAndRemove(int layout);

  private:
    TracingMap<int, uptr<MetaValue> > layoutParts;
    int last_idx;
  };
} // End klee namespace

#endif //KLEE_LAYOUT_BUILDER_H

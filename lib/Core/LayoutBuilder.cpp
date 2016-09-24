//===-- LayoutBuilder.cpp -------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <assert.h>

#include "LayoutBuilder.h"
#include "Tracing.h"

using namespace klee;

int LayoutBuilder::plain(uint64_t) {
  assert(false && "unsupported");
  return 0;
}

int LayoutBuilder::ptr(int ptee) {
  assert(false && "unsupported");
  return 0;
}

int LayoutBuilder::firstField(int valLayout, int offset, const std::string& name) {
  assert(false && "unsupported");
  return 0;
}

int LayoutBuilder::nextField(int lastFieldLayout, int valLayout,
                             int offset, const std::string& name) {
  assert(false && "unsupported");
  return 0;
}

int LayoutBuilder::array(int cellLayout, int length) {
  assert(false && "unsupported");
  return 0;
}

uptr<MetaValue> LayoutBuilder::buildAndRemove(int layout) {
  assert(false && "unsupported");
  return 0;
}

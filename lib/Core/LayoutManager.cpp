//===-- LayoutManager.cpp -------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <assert.h>

#include "LayoutManager.h"
#include "Tracing.h"

using namespace klee;

LayoutManager::LayoutManager(const LayoutManager& lb) {
  assert(false && "unsupported");
}

int LayoutManager::plain(uint64_t) {
  assert(false && "unsupported");
  return 0;
}

int LayoutManager::ptr(int ptee) {
  assert(false && "unsupported");
  return 0;
}

int LayoutManager::firstField(int valLayout, int offset, const std::string& name) {
  assert(false && "unsupported");
  return 0;
}

int LayoutManager::nextField(int lastFieldLayout, int valLayout,
                             int offset, const std::string& name) {
  assert(false && "unsupported");
  return 0;
}

int LayoutManager::array(int cellLayout, int length) {
  assert(false && "unsupported");
  return 0;
}

MetaValue *LayoutManager::getLayout(int layout) {
  assert(false && "unsupported");
  return 0;
}

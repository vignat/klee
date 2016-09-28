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
#include "Context.h"
#include "klee/tracing_definitions.h"

using namespace klee;

int klee::LayoutManager::lastIdx = 0;

LayoutManager::LayoutManager(const LayoutManager& lb)
  :layoutParts(lb.layoutParts) {
}

int LayoutManager::plain(uint64_t type) {
  Expr::Width width;
  MetaPlainVal::Kind kind;
  switch (type) {
  case KLEE_TRACE_FLOAT:
  case KLEE_TRACE_DOUBLE:
    assert(false && "Tracing of floating values unsupported.");
    break;
  case KLEE_TRACE_SI16:
    kind = MetaPlainVal::Kind::MPV_SINTEGER;
    width = 16;
    break;
  case KLEE_TRACE_SI32:
    kind = MetaPlainVal::Kind::MPV_SINTEGER;
    width = 32;
    break;
  case KLEE_TRACE_SI64:
    kind = MetaPlainVal::Kind::MPV_SINTEGER;
    width = 64;
    break;
  case KLEE_TRACE_UI16:
    kind = MetaPlainVal::Kind::MPV_UINTEGER;
    width = 16;
    break;
  case KLEE_TRACE_UI32:
    kind = MetaPlainVal::Kind::MPV_UINTEGER;
    width = 32;
    break;
  case KLEE_TRACE_UI64:
    kind = MetaPlainVal::Kind::MPV_UINTEGER;
    width = 64;
    break;
  case KLEE_TRACE_FPTR:
    kind = MetaPlainVal::Kind::MPV_FUNPTR;
    width = Context::get().getPointerWidth();
    break;
  case KLEE_TRACE_APATH_PTR:
    kind = MetaPlainVal::Kind::MPV_APATHPTR;
    width = Context::get().getPointerWidth();
    break;
  default:
    assert(false && "unexpected plain value type");
  };
  return insertANewLayout(new MetaPlainVal(width, kind));
}

int LayoutManager::ptr(int ptee) {
  MetaValue *pteeLayout = getLayout(ptee);
  return insertANewLayout(new MetaPointer(pteeLayout));
}

int LayoutManager::firstField(int valLayout,
                              int offset,
                              const std::string& name) {
  MetaValue *fLayout = getLayout(valLayout);
  MetaStructure *str = new MetaStructure();
  str->addField(offset, name, fLayout);
  return insertANewLayout(str);
}

int LayoutManager::nextField(int lastFieldLayout,
                             int valLayout,
                             int offset,
                             const std::string& name) {
  MetaValue *fLayout = getLayout(valLayout);
  MetaStructure *lastStr =
    cast<MetaStructure>(getLayout(lastFieldLayout));
  MetaStructure *str = new MetaStructure(*lastStr);
  str->addField(offset, name, fLayout);
  return insertANewLayout(str);
}

int LayoutManager::array(int cellLayout, int length) {
  MetaValue *cLayout = getLayout(cellLayout);
  return insertANewLayout(new MetaArray(cLayout, length));
}

MetaValue *LayoutManager::getLayout(int layout) {
  auto found = layoutParts.find(layout);
  //TODO: instead of crash, generate a "user error" exception here,
  // because it essentially is.
  assert(found != layoutParts.end() && "layout not found");
  return found->second.get();
}

int LayoutManager::freshId() {
  ++lastIdx;
  return lastIdx;
}

int LayoutManager::insertANewLayout(MetaValue *l) {
  int id = freshId();
  layoutParts =
    layoutParts.insert(std::move(std::make_pair
                                 (id, uptr<MetaValue>(l))));
  return id;
}

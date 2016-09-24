//===-- Tracing.cpp -------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Tracing.h"

#include "klee/Expr.h"

using namespace klee;


uptr<RuntimeValue> MetaStructure::ReadValue(ref<Expr> val,
                                            ExecutionState *state) const {
  assert(false && "unimplemented");
  return nullptr;
}

uptr<RuntimeValue> MetaArray::ReadValue(ref<Expr> val,
                                        ExecutionState *state) const {
  assert(false && "unimplemented");
  return nullptr;
}

uptr<RuntimeValue> MetaPointer::ReadValue(ref<Expr> val,
                                          ExecutionState *state) const {
  assert(false && "unimplemented");
  return nullptr;
}

uptr<RuntimeValue> MetaPlainVal::ReadValue(ref<Expr> val,
                                           ExecutionState *state) const {
  assert(false && "unimplemented");
  return nullptr;
}


CallInfo::CallInfo(const CallInfo& ci) {
  assert(false && "unimplemented");
}

void CallInfo::traceArgument(std::string name, uptr<MetaValue> layout,
                   ref<Expr> val, const ExecutionState &state) {
  assert(false && "unimplemented");
}
void CallInfo::setReturnLayout(uptr<MetaValue> layout) {
  assert(false && "unimplemented");
}
void CallInfo::traceExtraPointer(std::string name, uptr<MetaValue> layout,
                       uint64_t addr, const ExecutionState &state) {
  assert(false && "unimplemented");
}


void CallInfo::traceFunOutput(ref<Expr> retValue, const ExecutionState &state) {
  assert(false && "unimplemented");
}

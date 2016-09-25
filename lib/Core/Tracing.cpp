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

CallTreeNode::CallTreeNode(const std::vector<CallInfo>& initialPath) {
  assert(false && "unimplemented");
}

void CallInfo::traceFunOutput(ref<Expr> retValue, const ExecutionState &state) {
  assert(false && "unimplemented");
}

void CallTreeNode::addCallPath(std::vector<CallInfo>::const_iterator begin,
                               std::vector<CallInfo>::const_iterator end,
                               int id) {
  assert(false && "unimplemented");
}

void CallTreeNode::dumpCallPrefixes(std::list<CallInfo> accumulatedPrefix,
                                    FileOpener* fileOpener) {
  assert(false && "unimplemented");
}

void CallTree::addCallPath(const std::vector<CallInfo> &path) {
  if (path.size() == 0) return;
  ++lastId;
  if (root) {
    root->addCallPath(path.begin(), path.end(), lastId);
  } else {
    root.reset(new CallTreeNode(path));
  }
}

void CallTree::dumpCallPrefixes(InterpreterHandler* handler) {
  if (root) {
    PrefixFileOpener fo(handler);
    root->dumpCallPrefixes(std::list<CallInfo>(), &fo);
  }
}

CallTree::PrefixFileOpener::PrefixFileOpener(InterpreterHandler* h) {
  assert(false && "unimplemented");
}

llvm::raw_ostream *CallTree::PrefixFileOpener::openAnotherFile() {
  assert(false && "unimplemented");
}

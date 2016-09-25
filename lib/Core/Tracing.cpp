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
#include "Context.h"

using namespace klee;

uptr<RuntimeValue> MetaValue::readValueByPtr(size_t addr,
                                             ExecutionState *state) const {
  return readValue(state->readMemoryChunk(addr, calculateWidth()), state);
}

uptr<RuntimeValue> MetaStructure::readValue(ref<Expr> val,
                                            ExecutionState *state) const {
  assert(false && "unsupported, read by pointer instead");
  return nullptr;
}

uptr<RuntimeValue> MetaStructure::readValueByPtr(size_t addr,
                                                 ExecutionState *state) const {
  uptr<StructureValue> ret(new StructureValue(this));
  for (const auto& fieldPair : fields) {
    MetaStructureField* field = fieldPair.second.get();
    uptr<RuntimeValue> fieldValue =
      field->layout->readValueByPtr(addr + field->offset, state);
    ret->addField(field->offset, std::move(fieldValue));
  }
  return ret;
}

Expr::Width MetaStructure::calculateWidth() const {
  Expr::Width width = 0;
  for (const auto& fieldPair : fields) {
    MetaStructureField* field = fieldPair.second.get();
    // TODO: can be optimized: take the right-most field and go from
    // there.
    Expr::Width ending = 8*field->offset + field->layout->calculateWidth();
    if (ending > width) width = ending;
  }
  return width;
}

uptr<RuntimeValue> MetaArray::readValue(ref<Expr> val,
                                        ExecutionState *state) const {
  uptr<ArrayValue> ret(new ArrayValue(this, len));
  assert(isa<klee::ConstantExpr>(val) &&
         "No support for symbolic pointer return values.");
  ref<klee::ConstantExpr> address = cast<klee::ConstantExpr>(val);
  size_t base = address->getZExtValue();
  Expr::Width cellWidth = cell->calculateWidth();
  for (size_t i = 0; i < len; ++i) {
    uptr<RuntimeValue> cellValue = cell->readValueByPtr(base + i*cellWidth,
                                                        state);
    ret->setCell(i, std::move(cellValue));
  }
  return ret;
}

Expr::Width MetaArray::calculateWidth() const {
  assert(cell && "Cannot calculate width without cell layout.");
  return len*cell->calculateWidth();
}

uptr<RuntimeValue> MetaPointer::readValue(ref<Expr> val,
                                          ExecutionState *state) const {
  uptr<PointerValue> ret(new PointerValue(this));
  assert(isa<klee::ConstantExpr>(val) &&
         "No support for symbolic pointer return values.");
  ref<klee::ConstantExpr> address = cast<klee::ConstantExpr>(val);
  size_t addr = address->getZExtValue();
  ret->setPtee(ptee->readValueByPtr(addr, state));
  return ret;
}

Expr::Width MetaPointer::calculateWidth() const {
  return Context::get().getPointerWidth();
}

uptr<RuntimeValue> MetaPlainVal::readValue(ref<Expr> val,
                                           ExecutionState *state) const {
  (void)state;
  assert(val->getWidth() == width);
  return uptr<PlainValue>(new PlainValue(this, val));
}

Expr::Width MetaPlainVal::calculateWidth() const {
  return width;
}

void StructureValue::addField(size_t offset, uptr<RuntimeValue> value) {
  assert(false && "unimplemeted");
}

bool StructureValue::dumpToStream(llvm::raw_ostream& file) {
  assert(false && "unimplemeted");
  return false;
}

bool StructureValue::eq(const class RuntimeValue& other) const {
  assert(false && "unimplemeted");
  return false;
}

SymbolSet StructureValue::collectSymbols() const {
  assert(false && "unimplemeted");
  return SymbolSet();
}

void ArrayValue::setCell(size_t index, uptr<RuntimeValue> value) {
  assert(false && "unimplemeted");
}

bool ArrayValue::dumpToStream(llvm::raw_ostream& file) {
  assert(false && "unimplemeted");
  return false;
}

bool ArrayValue::eq(const class RuntimeValue& other) const {
  assert(false && "unimplemeted");
  return false;
}

SymbolSet ArrayValue::collectSymbols() const {
  assert(false && "unimplemeted");
  return SymbolSet();
}


void PointerValue::setPtee(uptr<RuntimeValue> ptee) {
  assert(false && "unimplemeted");
}

bool PointerValue::dumpToStream(llvm::raw_ostream& file) {
  assert(false && "unimplemeted");
  return false;
}

bool PointerValue::eq(const class RuntimeValue& other) const {
  assert(false && "unimplemeted");
  return false;
}

SymbolSet PointerValue::collectSymbols() const {
  assert(false && "unimplemeted");
  return SymbolSet();
}

bool PlainValue::dumpToStream(llvm::raw_ostream& file) {
  assert(false && "unimplemeted");
  return false;
}

bool PlainValue::eq(const class RuntimeValue& other) const {
  assert(false && "unimplemeted");
  return false;
}

SymbolSet PlainValue::collectSymbols() const {
  assert(false && "unimplemeted");
  return SymbolSet();
}


CallInfo::CallInfo(const CallInfo& ci) {
  assert(false && "unimplemented");
}

void CallInfo::traceArgument(std::string name,
                             uptr<MetaValue> layout,
                             ref<Expr> val,
                             const ExecutionState &state) {
  assert(false && "unimplemented");
}
void CallInfo::setReturnLayout(uptr<MetaValue> layout) {
  assert(false && "unimplemented");
}
void CallInfo::traceExtraPointer(std::string name,
                                 uptr<MetaValue> layout,
                                 size_t addr,
                                 const ExecutionState &state) {
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

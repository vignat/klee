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
    const MetaStructureField &field = fieldPair.second;
    uptr<RuntimeValue> fieldValue =
      field.layout->readValueByPtr(addr + field.offset, state);
    ret->addField(field.offset, std::move(fieldValue));
  }
  return ret;
}

Expr::Width MetaStructure::calculateWidth() const {
  Expr::Width width = 0;
  for (const auto& fieldPair : fields) {
    const MetaStructureField &field = fieldPair.second;
    // TODO: can be optimized: take the right-most field and go from
    // there.
    Expr::Width ending = 8*field.offset + field.layout->calculateWidth();
    if (ending > width) width = ending;
  }
  return width;
}

MetaStructure::MetaStructureField::MetaStructureField(const std::string &name_,
                                                      size_t offset_,
                                                      uptr<MetaValue> layout_)
  :name(name_), offset(offset_), layout(std::move(layout_))
{}

bool MetaStructure::getFieldName(size_t offset,
                                 const std::string **outName) const {
  const auto& field = fields.find(offset);
  if (field != fields.end()) {
    *outName = &field->second.name;
    return true;
  }
  return false;
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
  assert(fields.find(offset) == fields.end());
  fields.emplace(offset, std::move(value));
}

void StructureValue::dumpToStream(llvm::raw_ostream& file) const {
  file << "(";
  const std::string *name = nullptr;
  for (const auto& fieldI : fields) {
    bool found = layout->getFieldName(fieldI.first, &name);
    assert(found);
    file <<"(" <<*name <<" ";
    fieldI.second->dumpToStream(file);
    file <<")\n";
  }
  file << ")";
}

bool StructureValue::eq(const class RuntimeValue& other) const {
  if (const StructureValue * otherStr =
      dyn_cast<const StructureValue>(&other)) {
    if (fields.size() != otherStr->fields.size()) return false;
    for (const auto& fieldPair : fields) {
      auto otherIter = otherStr->fields.find(fieldPair.first);
      if (otherIter == otherStr->fields.end()) return false;
      if (!fieldPair.second->eq(*otherIter->second)) return false;
    }
    return true;
  }
  return false;
}

SymbolSet StructureValue::collectSymbols() const {
  SymbolSet ret;
  for (const auto& fieldPair : fields) {
    SymbolSet fieldSymbols = fieldPair.second->collectSymbols();
    ret.insert(fieldSymbols.begin(), fieldSymbols.end());
  }
  return ret;
}

RuntimeValue *StructureValue::makeCopy() const {
  StructureValue *ret(new StructureValue(layout));
  for (const auto& fieldPair : fields) {
    ret->fields.emplace_hint(ret->fields.end(),
                             fieldPair.first,
                             uptr<RuntimeValue>(fieldPair.second->makeCopy()));
  }
  return ret;
}

void ArrayValue::setCell(size_t index, uptr<RuntimeValue> value) {
  cells[index] = std::move(value);
}

void ArrayValue::dumpToStream(llvm::raw_ostream& file) const {
  assert(layout->getLen() == cells.size());
  file << "((length " <<cells.size() << ")\n(data \n";
  for (const auto& cell : cells) {
    cell->dumpToStream(file);
    file <<" ";
  }
  file <<")";
}

bool ArrayValue::eq(const class RuntimeValue& other) const {
  if (const ArrayValue *otherArr =
      dyn_cast<const ArrayValue>(&other)) {
    if (cells.size() != otherArr->cells.size()) return false;
    for (size_t i = 0; i < cells.size(); ++i) {
      if (!cells[i]->eq(*otherArr->cells[i])) return false;
    }
    return true;
  }
  return false;
}

SymbolSet ArrayValue::collectSymbols() const {
  SymbolSet ret;
  for (const auto& cell : cells) {
    SymbolSet cellSymbols = cell->collectSymbols();
    ret.insert(cellSymbols.begin(), cellSymbols.end());
  }
  return ret;
}

RuntimeValue *ArrayValue::makeCopy() const {
  ArrayValue *ret(new ArrayValue(layout, cells.size()));
  for (size_t i = 0; i < cells.size(); ++i) {
    ret->cells[i].reset(cells[i]->makeCopy());
  }
  return ret;
}


void PointerValue::setPtee(uptr<RuntimeValue> ptee_) {
  ptee = std::move(ptee_);
}

void PointerValue::dumpToStream(llvm::raw_ostream& file) const {
  file <<"(Curioptr " <<value << " (ptee ";
  ptee->dumpToStream(file);
  file <<"))";
}

bool PointerValue::eq(const class RuntimeValue& other) const {
  if (const PointerValue * otherPtr =
      dyn_cast<const PointerValue>(&other)) {
    if (0 != value->compare(*otherPtr->value)) return false;
    return ptee->eq(*otherPtr->ptee);
  }
  return false;
}

SymbolSet PointerValue::collectSymbols() const {
  assert(isa<ConstantExpr>(value));
  return ptee->collectSymbols();
}

RuntimeValue *PointerValue::makeCopy() const {
  PointerValue *ret(new PointerValue(layout));
  ret->ptee.reset(ptee->makeCopy());
  ret->value = value;
  return ret;
}

void PlainValue::dumpToStream(llvm::raw_ostream& file) const {
  file <<"((is_signed " <<layout->isSigned()
       <<") (width " <<layout->getWidth()
       <<") (value " <<val <<"))";
}

bool PlainValue::eq(const class RuntimeValue& other) const {
  if (const PlainValue *otherPV =
      dyn_cast<const PlainValue>(&other)) {
    return 0 == val->compare(*otherPV->val);
  }
  return false;
}

SymbolSet PlainValue::collectSymbols() const {
  return GetExprSymbols::visit(val);
}

RuntimeValue *PlainValue::makeCopy() const {
  return new PlainValue(layout, val);
}


CallInfo::CallInfo(const CallInfo& ci)
  : argsLayout(ci.argsLayout),
    extraPtrs(ci.extraPtrs),
    retLayout(ci.retLayout),
    retValue(ci.retValue->makeCopy()),
    callContext(ci.callContext),
    returnContext(ci.returnContext),
    funName(ci.funName),
    returned(ci.returned)
{
  for (const auto &abc : ci.argsBeforeCall) {
    argsBeforeCall.emplace_hint(argsBeforeCall.end(),
                                abc.first,
                                uptr<RuntimeValue>(abc.second->makeCopy()));
  }
  for (const auto &aac : ci.argsAfterCall) {
    argsAfterCall.emplace_hint(argsAfterCall.end(),
                               aac.first,
                               uptr<RuntimeValue>(aac.second->makeCopy()));
  }
  for (const auto &epbc : ci.extraPtrsBeforeCall) {
    extraPtrsBeforeCall.emplace_back(cast<PointerValue>(epbc->makeCopy()));
  }
  for (const auto &epac : ci.extraPtrsAfterCall) {
    extraPtrsAfterCall.emplace_back(cast<PointerValue>(epac->makeCopy()));
  }
}

void CallInfo::traceArgument(std::string name,
                             const MetaValue *layout,
                             ref<Expr> val,
                             ExecutionState *state) {
  StructuredArg sarg;
  sarg.layout = layout;
  sarg.value = val;
  argsLayout.insert(std::make_pair(name, sarg));
  uptr<RuntimeValue> valBeforeCall = layout->readValue(val, state);
  addCallConstraintsFor(valBeforeCall.get(), state);
  argsBeforeCall.insert(std::make_pair(name, std::move(valBeforeCall)));
}

void CallInfo::addCallConstraintsFor(RuntimeValue* val,
                                     ExecutionState* state) {
  SymbolSet symbols = val->collectSymbols();
  symbolsIn.insert(symbols.begin(), symbols.end());
  //TODO: this gets called on every "trace" call for the same
  // function. Do it only once.
  // can not do this on the "trace output" event, because
  // there may new constraints appear.
  callContext = state->relevantConstraints(symbolsIn);
}

void CallInfo::setReturnLayout(const MetaValue *layout) {
  retLayout = layout;
}

void CallInfo::traceExtraPointer(std::string name,
                                 const MetaValue *layout,
                                 size_t addr,
                                 ExecutionState *state) {
  StructuredPtr sptr;
  sptr.layout = layout;
  sptr.addr = addr;
  sptr.name = name;

  extraPtrs.push_back(sptr);
  uptr<RuntimeValue> valBeforeCall = layout->readValueByPtr(addr, state);
  addCallConstraintsFor(valBeforeCall.get(), state);
  extraPtrsBeforeCall.emplace_back(std::move(valBeforeCall));
}

void CallInfo::traceFunOutput(ref<Expr> retValue_, ExecutionState *state) {
  SymbolSet symbols = symbolsIn;
  for (const auto& argLayoutPair : argsLayout) {
    const StructuredArg &sarg = argLayoutPair.second;
    uptr<RuntimeValue> valAfterCall =
      sarg.layout->readValue(sarg.value, state);
    SymbolSet argSymbols = valAfterCall->collectSymbols();
    symbols.insert(argSymbols.begin(), argSymbols.end());
    argsAfterCall.insert(std::make_pair(argLayoutPair.first,
                                        std::move(valAfterCall)));
  }
  for (const auto& extraPtr : extraPtrs) {
    uptr<RuntimeValue> valAfterCall =
      extraPtr.layout->readValueByPtr(extraPtr.addr, state);
    SymbolSet ptrSymbols = valAfterCall->collectSymbols();
    symbols.insert(ptrSymbols.begin(), ptrSymbols.end());
    extraPtrsAfterCall.emplace_back(std::move(valAfterCall));
  }
  retValue = retLayout->readValue(retValue_, state);
  SymbolSet retSymbols = retValue->collectSymbols();
  symbols.insert(retSymbols.begin(), retSymbols.end());
  returnContext = state->relevantConstraints(symbols);
  returned = true;
}

void CallTreeNode::addCallPath(std::vector<CallInfo>::const_iterator begin,
                               std::vector<CallInfo>::const_iterator end,
                               int id) {
  if (begin == end) return;

  std::vector<CallInfo>::const_iterator next = begin;
  ++next;
  std::vector<CallTreeNode>::iterator
    i = children.begin(),
    ie = children.end();
  for (; i != ie; ++i) {
    if ((*i).tip.tipCall.eq(*begin)) {
      (*i).addCallPath(next, end, id);
      return;
    }
  }
  children.emplace_back();
  CallTreeNode& n = children.back();
  n.tip.tipCall = *begin;
  n.tip.pathId = id;
  n.addCallPath(next, end, id);
}

void CallTreeNode::dumpCallPrefixes(std::list<CallInfo> accumulatedPrefix,
                                    FileOpener* fileOpener) {
  assert(false && "unimplemented");
}

void CallTree::addCallPath(const std::vector<CallInfo> &path) {
  if (path.size() == 0) return;
  ++lastId;
  root.addCallPath(path.begin(), path.end(), lastId);
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

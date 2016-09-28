//===-- Tracing.cpp -------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <sstream>
#include <iomanip>

#include "Tracing.h"

#include "klee/Expr.h"
#include "Context.h"
#include "klee/Interpreter.h"

#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 3)
#include <llvm/IR/Function.h>
#else//LLVM_VERSION_CODE...
#include <llvm/Function.h>
#endif//LLVM_VERSION_CODE...

using namespace klee;

uptr<RuntimeValue> MetaValue::readValueByPtr(size_t addr,
                                             ExecutionState *state) const {
  return readValue(state->readMemoryChunk(addr, calculateWidth()), state);
}

MetaStructure::MetaStructure(const MetaStructure& other)
  :MetaValue(MVK_STRUCT), fields(other.fields) {
}

void MetaStructure::addField(size_t offset,
                             const std::string &name,
                             MetaValue *layout) {
  MetaStructureField field(name, offset, layout);
  //TODO: assert there is no existing field by this offset.
  fields.insert(std::make_pair(offset,field));
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
                                                      MetaValue *layout_)
  :name(name_), offset(offset_), layout(layout_)
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
    if (value.get() == otherPtr->value.get()) return true;
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
  const char *kindName = 0;
  switch (layout->getValueKind()) {
  case MetaPlainVal::Kind::MPV_SINTEGER:
    kindName = "sint";
    break;
  case MetaPlainVal::Kind::MPV_UINTEGER:
    kindName = "uint";
    break;
  case MetaPlainVal::Kind::MPV_APATHPTR:
    kindName = "apathptr";
    break;
  case MetaPlainVal::Kind::MPV_FUNPTR:
    kindName = "funptr";
    break;
  default:
    assert(false && "Unknown plain value kind.");
  }

  file <<"((kind " <<kindName
       <<") (width " <<layout->getWidth() <<") ";
  //TODO: differentiate here: value melts together two different types!
  if (layout->getValueKind() == MetaPlainVal::MPV_FUNPTR) {
    ref<klee::ConstantExpr> address = cast<klee::ConstantExpr>(val);
    file <<"(value \"" <<((llvm::Function*)address->getZExtValue())->getName() <<"\")";
  } else {
    file <<"(value " <<val <<")";
  }
  file <<")";
}

bool PlainValue::eq(const class RuntimeValue& other) const {
  if (const PlainValue *otherPV =
      dyn_cast<const PlainValue>(&other)) {
    if (val.get() == otherPV->val.get()) return true;
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
    extraPtees(ci.extraPtees),
    retLayout(ci.retLayout),
    retValue(ci.retValue ? ci.retValue->makeCopy() : nullptr),
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
  for (const auto &epbc : ci.extraPteesBeforeCall) {
    extraPteesBeforeCall.emplace_back(epbc->makeCopy());
  }
  for (const auto &epac : ci.extraPteesAfterCall) {
    extraPteesAfterCall.emplace_back(epac->makeCopy());
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

  extraPtees.push_back(sptr);
  uptr<RuntimeValue> valBeforeCall = layout->readValueByPtr(addr, state);
  addCallConstraintsFor(valBeforeCall.get(), state);
  extraPteesBeforeCall.emplace_back(std::move(valBeforeCall));
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
  for (const auto& extraPtr : extraPtees) {
    uptr<RuntimeValue> valAfterCall =
      extraPtr.layout->readValueByPtr(extraPtr.addr, state);
    SymbolSet ptrSymbols = valAfterCall->collectSymbols();
    symbols.insert(ptrSymbols.begin(), ptrSymbols.end());
    extraPteesAfterCall.emplace_back(std::move(valAfterCall));
  }
  retValue = retLayout->readValue(retValue_, state);
  SymbolSet retSymbols = retValue->collectSymbols();
  symbols.insert(retSymbols.begin(), retSymbols.end());
  returnContext = state->relevantConstraints(symbols);
  returned = true;
}

bool CallInfo::eq(const CallInfo& other) const {
  if (!sameInvocation(other)) return false;
  if (argsAfterCall.size() != other.argsAfterCall.size())
    return false;
  for (auto& aac : argsAfterCall) {
    auto otherAAC = other.argsAfterCall.find(aac.first);
    if (otherAAC == other.argsAfterCall.end()) return false;
    if (aac.second) {
      if (!otherAAC->second) return false;
      if (!aac.second->eq(*otherAAC->second)) return false;
    } else {
      if (otherAAC->second) return false;
    }
  }
  if (extraPteesAfterCall.size() != other.extraPteesAfterCall.size())
    return false;
  for (size_t i = 0; i < extraPteesAfterCall.size(); ++i) {
    auto &epac = extraPteesAfterCall[i];
    auto &otherEPAC = other.extraPteesAfterCall[i];
    if (epac) {
      if (!otherEPAC) return false;
      if (!epac->eq(*otherEPAC)) return false;
    } else {
      if (otherEPAC) return false;
    }
  }

  if (retValue) {
    if (!other.retValue) return false;
    if (!retValue->eq(*other.retValue)) return false;
  } else {
    if (other.retValue) return false;
  }

  // Comparing the expressions just by the ref<...> pointer values.
  // it is an approximation, but it is not bad.
  if (returnContext.size() != other.returnContext.size()) return false;
  for (auto constraint : returnContext) {
    if (other.returnContext.find(constraint) ==
        other.returnContext.end()) return false;
  }

  if (funName != other.funName) return false;
  return returned == other.returned;
}

bool CallInfo::sameInvocation(const CallInfo& other) const {
  if (argsBeforeCall.size() != other.argsBeforeCall.size())
    return false;
  for (auto& abc : argsBeforeCall) {
    auto otherABC = other.argsBeforeCall.find(abc.first);
    if (otherABC == other.argsBeforeCall.end()) return false;
    if (abc.second) {
      if (!otherABC->second) return false;
      if (!abc.second->eq(*otherABC->second)) return false;
    } else {
      if (otherABC->second) return false;
    }
  }
  if (extraPteesBeforeCall.size() !=
      other.extraPteesBeforeCall.size())
    return false;
  for (size_t i = 0; i < extraPteesBeforeCall.size(); ++i) {
    auto &epbc = extraPteesBeforeCall[i];
    auto &otherEPBC = other.extraPteesBeforeCall[i];
    if (epbc) {
      if (!otherEPBC) return false;
      if (!epbc->eq(*otherEPBC)) return false;
    } else {
      if (otherEPBC) return false;
    }
  }
  // Comparing the expressions just by the ref<...> pointer values.
  // it is an approximation, but it is not bad.
  if (callContext.size() != other.callContext.size()) return false;
  for (auto constraint : callContext) {
    if (other.callContext.find(constraint) ==
        other.callContext.end()) return false;
  }
  return true;
}

void CallInfo::dumpToStream(llvm::raw_ostream& file) const {
  assert(returned);
  file <<"((fun_name \"" <<funName <<"\")\n (args (";
  for (auto &argPair : argsBeforeCall) {
    auto iterAfter = argsAfterCall.find(argPair.first);
    file <<"\n((aname \"" <<argPair.first <<"\")\n";
    file <<"(value_before ";
    argPair.second->dumpToStream(file);
    file <<") (value_after ";
    iterAfter->second->dumpToStream(file);
    file <<")";
  }
  file <<"))\n";
  file <<"(extra_ptrs (";
  for (size_t i = 0; i < extraPtees.size(); ++i) {
    file <<"\n((pname \"" <<extraPtees[i].name <<"\")";
    file <<"\n(addr " <<extraPtees[i].addr <<")";
    file <<"\n(before_ptee ";
    extraPteesBeforeCall[i]->dumpToStream(file);
    file <<")";
    file <<"\n(after_ptee ";
    extraPteesAfterCall[i]->dumpToStream(file);
    file <<"))";
  }
  file <<"))\n";
  file <<"(call_context (";
  for (auto &constraint : callContext) {
    file <<constraint <<"\n";
  }
  file <<"))\n";
  file <<"(ret_context (";
  for (auto &constraint : returnContext) {
    file <<constraint <<"\n";
  }
  file <<")))\n";
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
  children.emplace_back(id, *begin);
  CallTreeNode& n = children.back();
  n.addCallPath(next, end, id);
}

std::vector<std::vector<CallTreeNode::PathTip*> >
CallTreeNode::groupChildren() {
  std::vector<std::vector<CallTreeNode::PathTip*> > ret;
  for (unsigned ci = 0; ci < children.size(); ++ci) {
    PathTip* current = &children[ci].tip;
    bool groupNotFound = true;
    for (unsigned gi = 0; gi < ret.size(); ++gi) {
      if (current->tipCall.sameInvocation(ret[gi][0]->tipCall)) {
        ret[gi].push_back(current);
        groupNotFound = false;
        break;
      }
    }
    if (groupNotFound) {
      ret.push_back(std::vector<CallTreeNode::PathTip*>());
      ret.back().push_back(current);
    }
  }
  return ret;
}

void CallTreeNode::dumpCallPrefixes(std::list<CallInfo> accumulatedPrefix,
                                    FileOpener* fileOpener) {
  std::vector<std::vector<CallTreeNode::PathTip*> > tipCalls = groupChildren();
  auto ti = tipCalls.begin(), te = tipCalls.end();
  for (; ti != te; ++ti) {
    llvm::raw_ostream* file = fileOpener->openAnotherFile();
    *file <<"((history (\n";
    std::list<CallInfo>::iterator
      ai = accumulatedPrefix.begin(),
      ae = accumulatedPrefix.end();
    for (; ai != ae; ++ai) {
      ai->dumpToStream(*file);
    }
    *file <<"))\n";
    //FIXME: currently there can not be more than one alternative.
    *file <<"(tip_calls (\n";
    for (std::vector<CallTreeNode::PathTip*>::const_iterator
           chi = ti->begin(),
           che = ti->end();
         chi != che; ++chi) {
      *file <<"\n; id: " <<(**chi).pathId <<"\n";
      (**chi).tipCall.dumpToStream(*file);
    }
    *file <<")))\n";
    delete file;
  }
  std::vector< CallTreeNode >::iterator
    ci = children.begin(),
    ce = children.end();
  for (; ci != ce; ++ci) {
    accumulatedPrefix.push_back(ci->tip.tipCall);
    ci->dumpCallPrefixes(accumulatedPrefix, fileOpener);
    accumulatedPrefix.pop_back();
  }
}

void CallTree::addCallPath(const std::vector<CallInfo> &path) {
  if (path.size() == 0) return;
  ++lastId;
  root.addCallPath(path.begin(), path.end(), lastId);
}

void CallTree::dumpCallPrefixes(InterpreterHandler *handler) {
  PrefixFileOpener fo(handler);
  root.dumpCallPrefixes(std::list<CallInfo>(), &fo);
}

CallTree::PrefixFileOpener::PrefixFileOpener(InterpreterHandler *h)
  :numOpenedFiles(0), handler(h) {
}

llvm::raw_ostream *CallTree::PrefixFileOpener::openAnotherFile() {
  size_t id = ++numOpenedFiles;
  std::stringstream filename;
  filename << "call-prefix" << std::setfill('0') << std::setw(6) << id << '.' << "txt";
  return handler->openOutputFile(filename.str());
}

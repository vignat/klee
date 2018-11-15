//===-- ExecutionState.cpp ------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <stdio.h>
#include <iostream>
#include <ostream>
#include <fstream>

#include "klee/ExecutionState.h"

#include "klee/Internal/Module/Cell.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/Internal/Module/InstructionInfoTable.h"
#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "TimingSolver.h"
#include "klee/LoopAnalysis.h"

#include "klee/Expr.h"

#include "Memory.h"
#include "llvm/IR/Function.h"
#include "llvm/DebugInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <map>
#include <regex>
#include <set>
#include <stdarg.h>
#include <tuple>

using namespace llvm;
using namespace klee;

namespace { 
  cl::opt<bool>
  DebugLogStateMerge("debug-log-state-merge");
}

/***/

StackFrame::StackFrame(KInstIterator _caller, KFunction *_kf)
  : caller(_caller), kf(_kf), callPathNode(0),
    minDistToUncoveredOnReturn(0), varargs(0) {
  locals = new Cell[kf->numRegisters];
}

StackFrame::StackFrame(const StackFrame &s) 
  : caller(s.caller),
    kf(s.kf),
    callPathNode(s.callPathNode),
    allocas(s.allocas),
    minDistToUncoveredOnReturn(s.minDistToUncoveredOnReturn),
    varargs(s.varargs) {
  locals = new Cell[s.kf->numRegisters];
  for (unsigned i=0; i<s.kf->numRegisters; i++)
    locals[i] = s.locals[i];
}

StackFrame::~StackFrame() { 
  delete[] locals; 
}

/***/

ExecutionState::ExecutionState(KFunction *kf) :
    pc(kf->instructions),
    prevPC(pc),

    executionStateForLoopInProcess(0),

    queryCost(0.),
    weight(1),
    depth(0),

    instsSinceCovNew(0),
    coveredNew(false),
    forkDisabled(false),
    ptreeNode(0),
    steppedInstructions(0),
    relevantSymbols(),
    condoneUndeclaredHavocs(false) {
  pushFrame(0, kf);
}

ExecutionState::ExecutionState(const std::vector<ref<Expr> > &assumptions)
  : executionStateForLoopInProcess(0),
    constraints(assumptions),
    queryCost(0.), ptreeNode(0),
    relevantSymbols(),
    condoneUndeclaredHavocs(false) {}

ExecutionState::~ExecutionState() {
  for (unsigned int i=0; i<symbolics.size(); i++)
  {
    const MemoryObject *mo = symbolics[i].first;
    assert(mo->refCount > 0);
    mo->refCount--;
    if (mo->refCount == 0)
      delete mo;
  }

  for(auto it = havocs.begin(); it != havocs.end(); ++it) {
    const MemoryObject *mo = it->first;
    assert(mo->refCount > 0);
    mo->refCount--;
    if (mo->refCount == 0)
      delete mo;
  }
  delete executionStateForLoopInProcess;

  for (auto cur_mergehandler: openMergeStack){
    cur_mergehandler->removeOpenState(this);
  }


  while (!stack.empty()) popFrame();
}

ExecutionState::ExecutionState(const ExecutionState& state):
    fnAliases(state.fnAliases),
    readsIntercepts(state.readsIntercepts),
    writesIntercepts(state.writesIntercepts),
    pc(state.pc),
    prevPC(state.prevPC),
    stack(state.stack),
    incomingBBIndex(state.incomingBBIndex),


    addressSpace(state.addressSpace),
    loopInProcess(state.loopInProcess) ,
    analysedLoops(state.analysedLoops),
    executionStateForLoopInProcess(0),
    constraints(state.constraints),

    queryCost(state.queryCost),
    weight(state.weight),
    depth(state.depth),

    pathOS(state.pathOS),
    symPathOS(state.symPathOS),

    instsSinceCovNew(state.instsSinceCovNew),
    coveredNew(state.coveredNew),
    forkDisabled(state.forkDisabled),
    coveredLines(state.coveredLines),
    ptreeNode(state.ptreeNode),
    symbolics(state.symbolics),
    havocs(state.havocs),
    havocNames(state.havocNames),
    arrayNames(state.arrayNames),
    openMergeStack(state.openMergeStack),
    steppedInstructions(state.steppedInstructions),
    callPath(state.callPath),
    relevantSymbols(state.relevantSymbols),
    condoneUndeclaredHavocs(state.condoneUndeclaredHavocs)
{
  for (unsigned int i=0; i<symbolics.size(); i++)
    symbolics[i].first->refCount++;

  for (auto cur_mergehandler: openMergeStack)
    cur_mergehandler->addOpenState(this);
  for(auto it = havocs.begin(); it != havocs.end(); ++it) {
    it->first->refCount++;
  }
  LOG_LA("Cloning ES " << (void*)this << " from " << (void*)&state);
}

void ExecutionState::addHavocInfo(const MemoryObject *mo,
                                  const std::string &name) {
  havocs[mo].name = name;
  havocs[mo].havoced = false;
  havocs[mo].mask = BitArray();
  mo->refCount++;
}

ExecutionState *ExecutionState::branch() {
  depth++;

  ExecutionState *falseState = new ExecutionState(*this);
  falseState->coveredNew = false;
  falseState->coveredLines.clear();

  weight *= .5;
  falseState->weight -= weight;

  return falseState;
}

void ExecutionState::pushFrame(KInstIterator caller, KFunction *kf) {
  stack.push_back(StackFrame(caller,kf));
}

void ExecutionState::popFrame() {
  StackFrame &sf = stack.back();
  for (std::vector<const MemoryObject*>::iterator it = sf.allocas.begin(), 
         ie = sf.allocas.end(); it != ie; ++it)
    addressSpace.unbindObject(*it);
  stack.pop_back();
}

void ExecutionState::addSymbolic(const MemoryObject *mo, const Array *array) { 
  mo->refCount++;
  symbolics.push_back(std::make_pair(mo, array));
}
///

std::string ExecutionState::getFnAlias(std::string fn) {
  for (auto& candidate : fnAliases) {
    if (candidate.isRegex) {
      if (std::regex_match(fn, candidate.nameRegex)) {
        return candidate.alias;
      }
    } else if (fn == candidate.name) {
      return candidate.alias;
    }
  }

  return "";
}

void ExecutionState::addFnAlias(std::string old_fn, std::string new_fn) {
  removeFnAlias(old_fn);

  FunctionAlias alias {
    .isRegex = false,
    .nameRegex = std::regex(""),
    .name = old_fn,
    .alias = new_fn
  };
  fnAliases.push_back(alias);
}

void ExecutionState::addFnRegexAlias(std::string fn_regex, std::string new_fn) {
  removeFnAlias(fn_regex);

  FunctionAlias alias = {
    .isRegex = true,
    .nameRegex = std::regex(fn_regex),
    .name = fn_regex,
    .alias = new_fn
  };
  fnAliases.push_back(alias);
}

void ExecutionState::removeFnAlias(std::string fn) {
  fnAliases.erase(std::remove_if(fnAliases.begin(), fnAliases.end(),
                                 [fn](FunctionAlias candidate) {
                                   return candidate.name == fn;
                                 }),
                  fnAliases.end());
}

std::string ExecutionState::getInterceptReader(uint64_t addr) {
  auto it = readsIntercepts.find(addr);
  if (it == readsIntercepts.end()) {
    return "";
  }

  return it->second;
}

std::string ExecutionState::getInterceptWriter(uint64_t addr) {
  auto it = writesIntercepts.find(addr);
  if (it == writesIntercepts.end()) {
    return "";
  }

  return it->second;
}

void ExecutionState::addReadsIntercept(uint64_t addr, std::string reader) {
  readsIntercepts[addr] = reader;
}

void ExecutionState::addWritesIntercept(uint64_t addr, std::string writer) {
  writesIntercepts[addr] = writer;
}

/**/

llvm::raw_ostream &klee::operator<<(llvm::raw_ostream &os, const MemoryMap &mm) {
  os << "{";
  MemoryMap::iterator it = mm.begin();
  MemoryMap::iterator ie = mm.end();
  if (it!=ie) {
    os << "MO" << it->first->id << ":" << it->second;
    for (++it; it!=ie; ++it)
      os << ", MO" << it->first->id << ":" << it->second;
  }
  os << "}";
  return os;
}

bool ExecutionState::merge(const ExecutionState &b) {
  if (DebugLogStateMerge)
    llvm::errs() << "-- attempting merge of A:" << this << " with B:" << &b
                 << "--\n";
  if (pc != b.pc)
    return false;

  if ((!loopInProcess.isNull()) || !b.loopInProcess.isNull()) {
    llvm::errs() <<"-- Loop in process: merge unsupported "
                 << "for loop invariant analysis.\n";
    return false;
  }

  // XXX is it even possible for these to differ? does it matter? probably
  // implies difference in object states?
  if (symbolics!=b.symbolics)
    return false;

  {
    std::vector<StackFrame>::const_iterator itA = stack.begin();
    std::vector<StackFrame>::const_iterator itB = b.stack.begin();
    while (itA!=stack.end() && itB!=b.stack.end()) {
      // XXX vaargs?
      if (itA->caller!=itB->caller || itA->kf!=itB->kf)
        return false;
      ++itA;
      ++itB;
    }
    if (itA!=stack.end() || itB!=b.stack.end())
      return false;
  }

  std::set< ref<Expr> > aConstraints(constraints.begin(), constraints.end());
  std::set< ref<Expr> > bConstraints(b.constraints.begin(), 
                                     b.constraints.end());
  std::set< ref<Expr> > commonConstraints, aSuffix, bSuffix;
  std::set_intersection(aConstraints.begin(), aConstraints.end(),
                        bConstraints.begin(), bConstraints.end(),
                        std::inserter(commonConstraints, commonConstraints.begin()));
  std::set_difference(aConstraints.begin(), aConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(aSuffix, aSuffix.end()));
  std::set_difference(bConstraints.begin(), bConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(bSuffix, bSuffix.end()));
  if (DebugLogStateMerge) {
    llvm::errs() << "\tconstraint prefix: [";
    for (std::set<ref<Expr> >::iterator it = commonConstraints.begin(),
                                        ie = commonConstraints.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tA suffix: [";
    for (std::set<ref<Expr> >::iterator it = aSuffix.begin(),
                                        ie = aSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tB suffix: [";
    for (std::set<ref<Expr> >::iterator it = bSuffix.begin(),
                                        ie = bSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
  }

  // We cannot merge if addresses would resolve differently in the
  // states. This means:
  // 
  // 1. Any objects created since the branch in either object must
  // have been free'd.
  //
  // 2. We cannot have free'd any pre-existing object in one state
  // and not the other

  if (DebugLogStateMerge) {
    llvm::errs() << "\tchecking object states\n";
    llvm::errs() << "A: " << addressSpace.objects << "\n";
    llvm::errs() << "B: " << b.addressSpace.objects << "\n";
  }
    
  std::set<const MemoryObject*> mutated;
  MemoryMap::iterator ai = addressSpace.objects.begin();
  MemoryMap::iterator bi = b.addressSpace.objects.begin();
  MemoryMap::iterator ae = addressSpace.objects.end();
  MemoryMap::iterator be = b.addressSpace.objects.end();
  for (; ai!=ae && bi!=be; ++ai, ++bi) {
    if (ai->first != bi->first) {
      if (DebugLogStateMerge) {
        if (ai->first < bi->first) {
          llvm::errs() << "\t\tB misses binding for: " << ai->first->id << "\n";
        } else {
          llvm::errs() << "\t\tA misses binding for: " << bi->first->id << "\n";
        }
      }
      return false;
    }
    if (ai->second != bi->second) {
      if (DebugLogStateMerge)
        llvm::errs() << "\t\tmutated: " << ai->first->id << "\n";
      mutated.insert(ai->first);
    }
  }
  if (ai!=ae || bi!=be) {
    if (DebugLogStateMerge)
      llvm::errs() << "\t\tmappings differ\n";
    return false;
  }
  
  // merge stack

  ref<Expr> inA = ConstantExpr::alloc(1, Expr::Bool);
  ref<Expr> inB = ConstantExpr::alloc(1, Expr::Bool);
  for (std::set< ref<Expr> >::iterator it = aSuffix.begin(), 
         ie = aSuffix.end(); it != ie; ++it)
    inA = AndExpr::create(inA, *it);
  for (std::set< ref<Expr> >::iterator it = bSuffix.begin(), 
         ie = bSuffix.end(); it != ie; ++it)
    inB = AndExpr::create(inB, *it);

  // XXX should we have a preference as to which predicate to use?
  // it seems like it can make a difference, even though logically
  // they must contradict each other and so inA => !inB

  std::vector<StackFrame>::iterator itA = stack.begin();
  std::vector<StackFrame>::const_iterator itB = b.stack.begin();
  for (; itA!=stack.end(); ++itA, ++itB) {
    StackFrame &af = *itA;
    const StackFrame &bf = *itB;
    for (unsigned i=0; i<af.kf->numRegisters; i++) {
      ref<Expr> &av = af.locals[i].value;
      const ref<Expr> &bv = bf.locals[i].value;
      if (av.isNull() || bv.isNull()) {
        // if one is null then by implication (we are at same pc)
        // we cannot reuse this local, so just ignore
      } else {
        av = SelectExpr::create(inA, av, bv);
      }
    }
  }

  for (std::set<const MemoryObject*>::iterator it = mutated.begin(), 
         ie = mutated.end(); it != ie; ++it) {
    const MemoryObject *mo = *it;
    const ObjectState *os = addressSpace.findObject(mo);
    const ObjectState *otherOS = b.addressSpace.findObject(mo);
    assert(os && !os->readOnly && 
           "objects mutated but not writable in merging state");
    assert(otherOS);
    assert(os->isAccessible() && otherOS->isAccessible() &&
           "Merging of inaccessible objects is not supported.");

    ObjectState *wos = addressSpace.getWriteable(mo, os);
    for (unsigned i=0; i<mo->size; i++) {
      ref<Expr> av = wos->read8(i);
      ref<Expr> bv = otherOS->read8(i);
      wos->write(i, SelectExpr::create(inA, av, bv));
    }
  }

  constraints = ConstraintManager();
  for (std::set< ref<Expr> >::iterator it = commonConstraints.begin(), 
         ie = commonConstraints.end(); it != ie; ++it)
    constraints.addConstraint(*it);
  constraints.addConstraint(OrExpr::create(inA, inB));

  return true;
}

void ExecutionState::dumpStack(llvm::raw_ostream &out) const {
  unsigned idx = 0;
  const KInstruction *target = prevPC;
  for (ExecutionState::stack_ty::const_reverse_iterator
         it = stack.rbegin(), ie = stack.rend();
       it != ie; ++it) {
    const StackFrame &sf = *it;
    Function *f = sf.kf->function;
    const InstructionInfo &ii = *target->info;
    out << "\t#" << idx++;
    std::stringstream AssStream;
    AssStream << std::setw(8) << std::setfill('0') << ii.assemblyLine;
    out << AssStream.str();
    out << " in " << f->getName().str() << " (";
    // Yawn, we could go up and print varargs if we wanted to.
    unsigned index = 0;
    for (Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
         ai != ae; ++ai) {
      if (ai!=f->arg_begin()) out << ", ";

      out << ai->getName().str();
      // XXX should go through function
      ref<Expr> value = sf.locals[sf.kf->getArgRegister(index++)].value;
      if (value.get() && isa<ConstantExpr>(value))
        out << "=" << value;
    }
    out << ")";
    if (ii.file != "")
      out << " at " << ii.file << ":" << ii.line;
    out << "\n";
    target = sf.caller;
  }
}

bool symbolSetsIntersect(const SymbolSet& a, const SymbolSet& b) {
  if (a.size() > b.size()) return symbolSetsIntersect(b, a);
  for (SymbolSet::const_iterator i = a.begin(), e = a.end(); i != e; ++i) {
    if (b.count(*i)) return true;
  }
  return false;
}

std::vector<ref<Expr> > ExecutionState::
relevantConstraints(SymbolSet symbols) const {
  std::vector<ref<Expr> > ret;
  llvm::SmallPtrSet<Expr*, 100> insertedConstraints;
  bool newSymbols = false;
  do {
    newSymbols = false;
    for (ConstraintManager::constraint_iterator ci = constraints.begin(),
           cEnd = constraints.end(); ci != cEnd; ++ci) {
      if (insertedConstraints.count((*ci).get())) continue;
      SymbolSet constrainedSymbols = GetExprSymbols::visit(*ci);
      if (symbolSetsIntersect(constrainedSymbols, symbols)) {
        for (SymbolSet::const_iterator csi = constrainedSymbols.begin(),
               cse = constrainedSymbols.end();
             csi != cse; ++csi) {
          bool inserted = symbols.insert(*csi);
          newSymbols = newSymbols || inserted;
        }
        symbols.insert(constrainedSymbols.begin(), constrainedSymbols.end());
        ret.push_back(*ci);
        insertedConstraints.insert((*ci).get());
      }
    }
  } while (newSymbols);
  return ret;
}

bool ExecutionState::isAccessibleAddr(ref<Expr> addr) const {
  ObjectPair op;
  ref<klee::ConstantExpr> address = cast<klee::ConstantExpr>(addr);
  bool success = addressSpace.resolveOne(address, op);
  assert(success && "Unknown pointer result!");
  const ObjectState *os = op.second;
  return os->isAccessible();
}

ref<Expr> ExecutionState::readMemoryChunk(ref<Expr> addr,
                                          Expr::Width width,
                                          bool circumventInaccessibility) const {
  ObjectPair op;
  ref<klee::ConstantExpr> address = cast<klee::ConstantExpr>(addr);
  bool success = addressSpace.resolveOne(address, op);
  assert(success && "Unknown pointer result!");
  const MemoryObject *mo = op.first;
  const ObjectState *os = op.second;
  //FIXME: assume inbounds.
  ref<Expr> offset = mo->getOffsetExpr(address);
  assert(0 < width && "Can not read a zero-length value.");
  return os->read(offset, width, circumventInaccessibility);
}

void ExecutionState::trace() {
  CallInfo info;
  info.function = stack.back().kf;
  info.fillValuesBefore(this);

  callPath.push_back(info);
}

void ExecutionState::recordRetConstraints(CallInfo *info) const {
  assert(!callPath.empty() &&
         info->function == stack.back().kf->function);

  SymbolSet symbols = info->getSymbols();
  std::vector<ref<Expr> > constrs = relevantConstraints(symbols);
  info->returnContext.insert(info->returnContext.end(),
                             constrs.begin(), constrs.end());
}

void ExecutionState::symbolizeConcretes() {
  for (MemoryMap::iterator obj_I = addressSpace.objects.begin(),
         obj_E = addressSpace.objects.end(); obj_I != obj_E; ++obj_I) {
    const MemoryObject *mo = obj_I->first;
    ObjectState *os = obj_I->second;
    if (!os->readOnly && os->isAccessible()) {
      ObjectState *osw = addressSpace.getWriteable(mo, os);
      const Array *array = osw->forgetAll();
      symbolics.push_back(std::make_pair(mo, array));
    }
  }
}

ExecutionState* ExecutionState::finishLoopRound(KFunction *kf) {
  bool analysisFinished = false;
  ExecutionState *nextRoundState =
    loopInProcess->nextRoundState(&analysisFinished);
  //TODO: here analysis may never finish, e.g. if inside the analysed loop
  // there is an infinite loop somewhere. If we cound detect infinite loops,
  // we may ensure that the analysis finishes. Otherwise, Klee will be blocked
  // on this loop, even though there are paths that avoid that infinite loop,
  // and the search heuristic may guide execution away.
  if (analysisFinished) {
    kf->insert(loopInProcess->getLoop(),
               loopInProcess->getChangedBytes(),
               loopInProcess->getEntryState());
    LOG_LA("[" << loopInProcess->getLoop() << "]analysis finished, loop inserted");
  }
  return nextRoundState;
}

void ExecutionState::loopRepetition(const llvm::Loop *dstLoop,
                                    TimingSolver *solver,
                                    bool *terminate) {
  //TODO: detect infinite loops.
  LOG_LA("Loop repetition");
  if (!loopInProcess.isNull()) {
    if (loopInProcess->getLoop() == dstLoop) {
      LOG_LA("[" << loopInProcess->getLoop() << "]The loop is in process.")
      loopInProcess->updateChangedObjects(*this, solver);
      LOG_LA("refcount: " <<loopInProcess->refCount);
      LOG_LA("Terminating the loop-repeating state.");
      *terminate = true;
      return;
    }
  }
  if (analysedLoops.count(dstLoop)) {
    LOG_LA("terminate loop repeating state for an analyzed loop.");
    *terminate = true;
    return;
  }
  *terminate = false;
}

void ExecutionState::loopEnter(const llvm::Loop *dstLoop) {
  LOG_LA("Loop enter");
  // Get ready for the next analysis run, which may have
  // different starting conditions.
  LOG_LA("Remove the loop from the analyzed set - prepare"
         " to repeat the analysis.");
  analysedLoops = analysedLoops.remove(dstLoop);
  /// Remember the initial state for this loop header in
  /// case ther is an klee_induce_invariants call following.
  LOG_LA("store the loop-head entering state,"
         " just in case.");
  executionStateForLoopInProcess = branch();
  executionStateForLoopInProcess->loopInProcess = 0;
}

void ExecutionState::loopExit(const llvm::Loop *srcLoop,
                              bool *terminate) {
  LOG_LA("Loop exit");
  if (!loopInProcess.isNull()) {
    if (loopInProcess->getLoop() == srcLoop) {
      LOG_LA("[" << loopInProcess->getLoop() << "]The loop is in process");
      LOG_LA("Terminating loop-escaping state.");
      *terminate = true;
      return;
    }
  }
  *terminate = false;
}

void ExecutionState::updateLoopAnalysisForBlockTransfer
                      (BasicBlock *dst, BasicBlock *src,
                       TimingSolver* solver,
                       bool *terminate) {
  KFunction *kf = stack.back().kf;
  const llvm::Loop *dstLoop = kf->loopInfo.getLoopFor(dst);
  const llvm::Loop *srcLoop = kf->loopInfo.getLoopFor(src);
  *terminate = false;
  if (srcLoop) {
    if (dstLoop) {
      if (srcLoop == dstLoop) {
        if (dst == dstLoop->getHeader()) {
          //Loop repetition.
          loopRepetition(dstLoop, solver, terminate);
        } else {
          //In-loop transition
        }
      } else if (srcLoop->contains(dstLoop)) {
        //Nested loop enter
        assert(dstLoop->getHeader() == dst);
        loopEnter(dstLoop);
      } else if (dstLoop->contains(srcLoop)) {
        //Nested loop exit
        loopExit(srcLoop, terminate);
        if (dst == dstLoop->getHeader()) {
          //Loop repetition.
          loopRepetition(dstLoop, solver, terminate);
        }
      } else {
        //Transition from one loop to another
        //Loop enter + Loop exit
        assert(dstLoop->getHeader() == dst);
        loopEnter(dstLoop);
        loopExit(srcLoop, terminate);
      }
    } else {
      //Loop exit
      loopExit(srcLoop, terminate);
    }
  } else {
    if (dstLoop) {
      //Loop enter
      assert(dstLoop->getHeader() == dst);
      loopEnter(dstLoop);
    } else {
      //Out-of-loop transition.
    }
  }
}

void ExecutionState::terminateState(ExecutionState** replace) {
  LOG_LA("Terminating: " << (void*)this);
  if (!loopInProcess.isNull()) {
    *replace = finishLoopRound(stack.back().kf);
    loopInProcess = 0;
    LOG_LA(" - replacing with: " <<(void*)(*replace));
  }
}

void ExecutionState::startInvariantSearch() {
  KInstruction *inst = prevPC;
  llvm::Instruction *linst = inst->inst;
  assert(linst);
  LOG_LA(linst->getOpcodeName());
  BasicBlock *bb = linst->getParent();
  assert(bb);

  KFunction *kf = stack.back().kf;
  const KFunction::LInfo &loopInfo = kf->loopInfo;

  const llvm::Loop* loop = loopInfo.getLoopFor(bb);

  LOG_LA("loop being analysed:" <<loop);
  if ((loopInProcess.isNull() ||
       loopInProcess->getLoop() != loop) &&
      analysedLoops.count(loop) == 0) {
    LOG_LA("Start search for loop invariants.");

    assert(executionStateForLoopInProcess &&
           "The initial execution state must have been stored at the entrance"
           " of the loop header block.");

    assert(!loopInfo.empty());
    assert(loop &&
           "The klee_induce_invariants must be placed into the condition"
           " of a loop.");
    assert(loopInfo.isLoopHeader(bb) &&
           "The klee_induce_invariants must be placed into the condition"
           " of a loop.");

    loopInProcess =
      new LoopInProcess(loop,
                        executionStateForLoopInProcess,
                        loopInProcess);
    executionStateForLoopInProcess = 0;
  } else {
    LOG_LA("Already analysed, or being analysed at this very moment");
  }
}

void ExecutionState::induceInvariantsForThisLoop(KInstruction *target) {
  startInvariantSearch();

  //The return value of the intrinsic function call.
  stack.back().locals[target->dest].value =
    ConstantExpr::create(0xffffffff, Expr::Int32);
}

void CallValue::fill(const ExecutionState& state, const DataLayout* layout) {
  if (expr == NULL) {
    // If expr isn't set, address must be
    expr = state.readMemoryChunk(address, layout->getTypeAllocSizeInBits(type), true);
  }

  if (type->isStructTy()) {
    // Note that this case cannot happen for a parameter, so we know we have an address

    StructType* structTy = dynamic_cast<StructType*>(type);
    const StructLayout* structLayout = layout->getStructLayout(structTy);

    for (int n = 0; n < structTy->elements().size(); ++n) {
      uint64_t offset = structLayout->getElementOffsetInBits(n);

      CallValue* val = new CallValue();
      val->type = structTy->elements()[n];
      val->address = ConstantExpr::create(address->getZExtValue() + offset);
      val->fill(state, layout);

      children.push_back(val);
    }
  } else if(type->isPointerTy()) {
    // We may not have an address here since it could be a param, but we don't need it

    // First, is it symbolic? We can't trace that.
    if (!isa<ConstantExpr>(expr)) {
      return;
    }

    // Then check for null
    if (dynamic_cast<ConstantExpr>(expr)->getZExtValue() == 0) {
      return;
    }

    PointerType* pointerTy = dynamic_cast<PointerType*>(type);

    // We also don't follow function pointers
    if (pointerTy->getElementType()->isFunctionType()) {
      return;
    }

    pointee = new CallValue();
    pointee->type = pointerTy->getElementType();
    pointee->address = dynamic_cast<ConstantExpr>(expr);
    pointee->fill(state, layout);
  } else if(type->isIntegerTy()) {
    // Nothing to do here, expr must be set by this point
  } else {
    klee_error("Trying to trace a type that is not integer, pointer or struct.");
  }
}

ref<Expr> getArgExpr(const ExecutionState& state, KFunction* function, int index) {
  // HACK copy paste of what's in Executor because it'd be stupid to depend on it
  return state.stack.back().locals[function->getArgRegister(index)].value;
}

void CallInfo::fillValuesBefore(const ExecutionState& state) {
  if (!state.callPath.empty()) {
    auto symbols = state.callPath.back().getSymbols();
    relevantSymbols.insert(symbols.begin(), symbols.end());
  }

  auto constraints = relevantConstraints(relevantSymbols);
  callContext.insert(callContext.end(), constraints.begin(), constraints.end());

  for (int n = 0; n < function->numArgs; ++n) {
    CallValue* val = new CallValue();
    val->type = function->function->arguments()[n]->getType();
    val->expr = getArgExpr(state, function, n);
    val->fill(state, function->module->targetData);

    argumentsBefore.push_back(val);
  }
}

void CallInfo::fillValuesAfter(const ExecutionState& state, ref<Expr> result) {
  auto symbols = getSymbols();
  auto constraints = relevantConstraints(symbols);
  returnContext.insert(returnContext.end(), constraints.begin(), constraints.end());

  returnValue = new CallValue();
  returnValue->type = function->function->getReturnType();
  returnValue->expr = result;
  returnValue->fill(state, function->module->targetData);

  for (int n = 0; n < function->numArgs; ++n) {
    CallValue* val = new CallValue();
    val->type = function->function->arguments()[n]->getType();
    val->expr = getArgExpr(state, function, n);
    val->fill(state, function->module->targetData);

    argumentsAfter.push_back(val);
  }
}

bool exprsEqual(ref<Expr> a, ref<Expr> b) {
  return a.isNull() ? b.isNull() : (!b.isNull() && a == b));
}

bool CallValue::equals(const CallValue& other) const {
  if (!exprsEqual(value, other.value)) return false;
  if (!exprsEqual(pointee, other.pointee)) return false;
  if (pointeeChildren.size() != other.pointeeChildren.size()) return false;

  for (int n = 0; n < pointeeChildren.size(); ++n) {
    if (!exprsEqual(pointeeChildren[n], other.pointeeChildren[n])) return false;
  }

  return true;
}

bool equalContexts(const std::vector<ref<Expr> >& a,
                   const std::vector<ref<Expr> >& b) {
  // TODO: Structural-only comparison here, ideally we'd ask the solver about it
  // FIXME: quadratic! ouch!
  if (a.size() != b.size()) return false;
  for (unsigned i = 0; i < a.size(); ++i) {
    bool notFound = true;
    for (unsigned j = 0; j < b.size(); ++j) {
      if ((*a[i]).compare(*b[j]) == 0) {
        notFound = false;
        break;
      }
    }
    if (notFound) return false;
  }
  for (unsigned i = 0; i < b.size(); ++i) {
    bool notFound = true;
    for (unsigned j = 0; j < a.size(); ++j) {
      if ((*a[i]).compare(*b[j]) == 0) {
        notFound = false;
        break;
      }
    }
    if (notFound) return false;
  }
  return true;
}

bool CallInfo::equals(const CallInfo& other, bool compareOutputs) const {
  if (argsBefore.size() != other.argsBefore.size()) return false;
  for (unsigned i = 0; i < argsBefore.size(); ++i) {
    if (!argsBefore[i].equals(other.argsBefore[i])) return false;
  }

  if(compareOutputs) {
    if (argsAfter.size() != other.argsAfter.size()) return false;
    for (unsigned i = 0; i < argsAfter.size(); ++i) {
      if (!argsAfter[i].equals(other.argsAfter[i])) return false;
    }
  }

  return function == other.function &&
         equalContexts(callContext, other.callContext) &&
         (!compareOutputs || (returnValue.equals(other.returnValue) &&
                              equalContexts(returnContext, other.returnContext)));
}

SymbolSet CallValue::getSymbols() const {
  SymbolSet symbols = GetExprSymbols::visit(expr);

  if (pointee != nullptr) {
    auto pointeeSymbols = pointee->getSymbols();
    symbols.insert(pointeeSymbols.begin(), pointeeSymbols.end());
  }

  for (auto val : children) {
    auto valSymbols = val->getSymbols();
    symbols.insert(valSymbols.begin(), valSymbols.end());
  }

  return symbols;
}

SymbolSet CallInfo::getSymbols() const {
  SymbolSet symbols;

  if (!returnValue->expr.isNull()) {
    auto retSymbols = returnValue->getSymbols();
    symbols.insert(retSymbols.begin(), retSymbols.end());
  }

  for (auto val : argsAfter) {
    auto valSymbols = val->getSymbols();
    symbols.insert(valSymbols.begin(), valSymbols.end());
  }

  return symbols;
}

LoopInProcess::LoopInProcess(const llvm::Loop *_loop,
                             ExecutionState *_headerState,
                             const ref<LoopInProcess> &_outer)
  :refCount(0), outer(_outer), loop(_loop), restartState(_headerState),
   lastRoundUpdated(false)
{
  //TODO: this can not belong here. It has nothing to do with execution state,
  // nor with ptree node.
  restartState->ptreeNode = 0;
}

LoopInProcess::~LoopInProcess() {
  for (std::map<const MemoryObject *, BitArray *>::iterator
         i = changedBytes.begin(),
         e = changedBytes.end();
       i != e; ++i) {
    delete i->second;
  }
  assert(restartState);
  delete restartState;
}

unsigned countBitsSet(const BitArray *arr, unsigned size) {
  unsigned rez = 0;
  for (unsigned i = 0; i < size; ++i) {
    if (arr->get(i)) ++rez;
  }
  return rez;
}

ExecutionState *LoopInProcess::makeRestartState() {
  ExecutionState *newState = restartState->branch();
  LOG_LA("Making restart state " << (void*)newState <<" from " <<(void*)restartState);
  for (std::map<const MemoryObject *, BitArray *>::iterator
         i = changedBytes.begin(),
         e = changedBytes.end();
       i != e; ++i) {
    const MemoryObject *mo = i->first;
    const BitArray *bytes = i->second;
    if (mo->allocSite) {
      LOG_LA(" Forgetting: [" <<countBitsSet(bytes, mo->size)
             <<"/" <<mo->size <<"]" <<*mo->allocSite);
    } else {
      LOG_LA(" Forgetting something.\n");
    }
    const ObjectState *os =
      newState->addressSpace.findObject(mo);
    assert(os != 0 &&
           "changedObjects must contain only existing objects.");
    assert(!os->readOnly &&
           "Read only object can not have been changed");
    ObjectState *wos;
    bool wasInaccessible = !os->isAccessible();
    if (wasInaccessible) {
      wos = newState->addressSpace.allowAccess(mo, os);
    } else {
      wos = newState->addressSpace.getWriteable(mo, os);
    }
    const Array *array = wos->forgetThese(bytes);
    if (wasInaccessible) {
      wos->forbidAccessWithLastMessage();
    }

    auto havoc_info = newState->havocs.find(mo);
    if (havoc_info == newState->havocs.end() &&
        !restartState->condoneUndeclaredHavocs) {
      printf("Unexpected memory location being havoced.\n");
      assert(0 && "Possible havoc location must have been predelcared");
    }

    if (havoc_info != newState->havocs.end()) {
      // Remember the generated value for later reporting in the ktest file.
      havoc_info->second.value = array;
      havoc_info->second.havoced = true;
      havoc_info->second.mask = BitArray(*bytes, bytes->size());
      LOG_LA("Adding havoc here: " << havoc_info->second.name << " in: " << (void*)newState);
    }

    // Do not record this symbol, as it was not generated with klee_make_symbolic.
    //newState->symbolics.push_back(std::make_pair(mo, array));
  }
  if (lastRoundUpdated) {
    LOG_LA("[" << loop << "]Some more objects were changed."
           " repeat the loop.");
    lastRoundUpdated = false;
    //This works, because refCount is the internal field.
    newState->loopInProcess = this;
  } else {
    LOG_LA("[" << loop << "]Nothing else changed."
           " Restart loop "
           " in the normal mode.");
    newState->loopInProcess = outer;
    newState->analysedLoops = newState->analysedLoops.insert(loop);
  }
  return newState;
}

std::string __attribute__((weak)) numToStr(long long n) {
  std::stringstream ss;
  ss << n;
  return ss.str();
}

//TODO: move this into not-yet existing LoopAnalysis.cpp
bool klee::updateDiffMask(StateByteMask* mask,
                          const AddressSpace& refValues,
                          const ExecutionState& state,
                          TimingSolver* solver) {
  bool updated = false;
  for (MemoryMap::iterator
         i = refValues.objects.begin(),
         e = refValues.objects.end();
       i != e; ++i) {
    const MemoryObject *obj = i->first;
    const ObjectState *refOs = i->second;
    const ObjectState *os = state.addressSpace.findObject(obj);
    if (refOs == os) continue;
    if (refOs->isAccessible() != os->isAccessible()) {
      std::string inacc_msg;
      if (refOs->isAccessible()) {
        inacc_msg = "cand " + os->inaccessible_message;
      } else {
        inacc_msg = "ref " + refOs->inaccessible_message;
      }
      printf("No support for accessibility alternation "
             "between loop iterations. Inaccessibility reason: %s\n",
             inacc_msg.c_str());
      exit(1);
    }
    assert(refOs->isAccessible() == os->isAccessible() &&
           "No support for accessibility alteration "
           "between loop iterations.");
    std::pair<std::map<const MemoryObject *, BitArray *>::iterator, bool>
      insRez = mask->insert
      (std::pair<const MemoryObject *, BitArray *>(obj, 0));

    if (state.havocs.find(obj) == state.havocs.end() &&
        !state.condoneUndeclaredHavocs) {
      ref<Expr> firstByteRef = refOs->read8(0, true);
      ref<Expr> firstByte = os->read8(0, true);
      fprintf(stderr, "First byte before: ");
      fflush(stderr);
      firstByteRef->dump();
      fprintf(stderr, "First byte after: ");
      fflush(stderr);
      firstByte->dump();
      fprintf(stderr, "Type: ");
      fflush(stderr);
      obj->allocSite->getType()->dump();
      fprintf(stderr, "\n");
      std::string metadata;
      if (isa<llvm::Instruction>(obj->allocSite)) {
        const llvm::Instruction *inst = dyn_cast<llvm::Instruction>(obj->allocSite);
        if (llvm::MDNode *node = inst->getMetadata("dbg")) {
          llvm::DILocation loc(node);
          metadata = loc.getDirectory().str() + "/" +
            loc.getFilename().str() + ":" +
            numToStr(loc.getLineNumber());
        } else {
          metadata = "(unknown)";
        }
      } else {
        metadata = "(not an instruciton)";
      }
      klee_error("Unexpected memory location changed its value during invariant analysis:\n"
                 "  name: %s\n  location: %s\n"
                 "  local: %s\n  global: %s\n"
                 "  fixed: %s\n  size: %u\n"
                 "  address: 0x%lx\n  metadata: %s",
                 obj->name.c_str(),
                 obj->allocSite->getName().str().c_str(),
                 obj->isLocal ? "true" : "false",
                 obj->isGlobal ? "true" : "false",
                 obj->isFixed ? "true" : "false",
                 obj->size,
                 obj->address,
                 metadata.c_str());
    }

    if (insRez.second) insRez.first->second =
                         new BitArray(obj->size);
    BitArray *bytes = insRez.first->second;
    assert(bytes != 0);
    unsigned size = obj->size;
    for (unsigned j = 0; j < size; ++j) {
      if (bytes->get(j)) continue;
      ref<Expr> refVal = refOs->read8(j, true);
      ref<Expr> val = os->read8(j, true);
      if (0 != refVal->compare(*val)) {
        //So: this byte was not diferent on the previous round,
        // it also differs structuraly now. It is time to make
        // sure it can be really different.

        solver->setTimeout(0.01);//TODO: determine a correct argument here.
        bool mayDiffer = true;
        bool solverRes = solver->mayBeFalse(state, EqExpr::create(refVal, val),
                                            /*&*/mayDiffer);
        solver->setTimeout(0);
        //assert(solverRes &&
        //       "Solver failed in computing whether a byte changed or not.");
        if (solverRes && mayDiffer) {
          bytes->set(j);
          updated = true;
        }
      }
    }
  }
  return updated;
}

void LoopInProcess::updateChangedObjects(const ExecutionState& current,
                                         TimingSolver* solver) {
  bool updated = updateDiffMask(&changedBytes,
                                restartState->addressSpace,
                                current,
                                solver);
  if (updated) lastRoundUpdated = true;
}

ExecutionState *LoopInProcess::nextRoundState(bool *analysisFinished) {
  if (refCount == 1) {
    //The last state in the round.
    if (!lastRoundUpdated) {
      LOG_LA("[" << loop << "]Fixpoint reached. Time to"
             " restart the iteration in the normal mode.");
      *analysisFinished = true;
    } else {
      *analysisFinished = false;
    }
    // Order is important; makeRestartState clears the
    // lastRoundUpdated flag.
    LOG_LA("[" << loop << "]Schedule a fresh copy of the"
           " restart state for the loop");
    return makeRestartState();
  }
  *analysisFinished = false;
  return 0;
}

void ExecutionState::dumpConstraints() const {
  const char* digits[10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
  static int cnt = 0;
  ++cnt;
  std::string fname = "constraints";

  int tmp = cnt;
  while (0 < tmp) {fname = digits[tmp%10] + fname; tmp /= 10;}
  fname += ".txt";
  std::string Error;
  llvm::raw_ostream *file = new llvm::raw_fd_ostream(fname.c_str(), Error, llvm::sys::fs::F_None);
  if (!Error.empty()) {
    printf("error opening file \"%s\".  KLEE may have run out of file "
           "descriptors: try to increase the maximum number of open file "
           "descriptors by using ulimit (%s).",
           fname.c_str(), Error.c_str());
    delete file;
    file = NULL;
    return;
  }
  *file <<";;-- Constraints --\n";
  for (ConstraintManager::constraint_iterator ci = constraints.begin(),
         cEnd = constraints.end(); ci != cEnd; ++ci) {
    *file <<**ci<<"\n";
  }
  delete file;
  // for (ConstraintManager::constraint_iterator ci = constraints.begin(),
  //        cEnd = constraints.end(); ci != cEnd; ++ci) {
  //   const ref<Expr> constraint = *ci;
  //   std::cout <<*constraint <<std::endl;
  //}
}

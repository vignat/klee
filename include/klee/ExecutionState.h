//===-- ExecutionState.h ----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXECUTIONSTATE_H
#define KLEE_EXECUTIONSTATE_H

#include "klee/Constraints.h"
#include "klee/Expr.h"
#include "klee/Internal/ADT/TreeStream.h"
#include "klee/MergeHandler.h"
#include "klee/Internal/ADT/ImmutableSet.h"
#include "klee/util/GetExprSymbols.h"
#include "klee/LoopAnalysis.h"

// FIXME: We do not want to be exposing these? :(
#include "../../lib/Core/AddressSpace.h"
#include "klee/Internal/Module/KInstIterator.h"

//TODO: generalize for otehr LLVM versions like the above
#include <llvm/Analysis/LoopInfo.h>

#include <map>
#include <regex>
#include <set>
#include <vector>

namespace llvm {
  class Function;
  class BasicBlock;
}

namespace klee {
class Array;
class CallPathNode;
struct Cell;
struct KFunction;
struct KInstruction;
class MemoryObject;
class PTreeNode;
struct InstructionInfo;
class TimingSolver;

llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const MemoryMap &mm);

struct StackFrame {
  KInstIterator caller;
  KFunction *kf;
  CallPathNode *callPathNode;

  std::vector<const MemoryObject *> allocas;
  Cell *locals;

  /// Minimum distance to an uncovered instruction once the function
  /// returns. This is not a good place for this but is used to
  /// quickly compute the context sensitive minimum distance to an
  /// uncovered instruction. This value is updated by the StatsTracker
  /// periodically.
  unsigned minDistToUncoveredOnReturn;

  // For vararg functions: arguments not passed via parameter are
  // stored (packed tightly) in a local (alloca) memory object. This
  // is set up to match the way the front-end generates vaarg code (it
  // does not pass vaarg through as expected). VACopy is lowered inside
  // of intrinsic lowering.
  MemoryObject *varargs;

  StackFrame(KInstIterator caller, KFunction *kf);
  StackFrame(const StackFrame &s);
  ~StackFrame();
};

struct FunctionAlias {
  bool isRegex;
  std::regex nameRegex;
  std::string name;
  std::string alias;
};

struct CallValue {
  ref<Expr> value;
  std::vector<CallValue*> children;
};

struct CallInfo {
  Function* function;

  // 0 is return value, rest are args
  std::vector<CallValue> valuesBefore;
  std::vector<CallValue> valuesAfter;

  std::vector<ref<Expr>> callContext;
  std::vector<ref<Expr>> returnContext;

  SymbolSet computeRetSymbolSet() const;
};

struct HavocInfo {
  std::string name;
  bool havoced;
  BitArray mask;
  const Array* value;
};

class ExecutionState;

/// @brief LoopInProcess keeps all the necessary information for
/// dynamic loop invariant deduction.
class LoopInProcess {
public:
  /// for the ref class. This count also determines how many
  /// paths are in the loop.
  int refCount;
private:
  ref<LoopInProcess> outer;
  const llvm::Loop *loop; //Owner: KFunction::loopInfo
  // No circular dependency here: the restartState must not have
  // loop in process.
  ExecutionState *restartState; //Owner.
  bool lastRoundUpdated;
  //Owner for the bitarrays.
  StateByteMask changedBytes;
  //std::set<const MemoryObject *> changedObjects;

  ExecutionState *makeRestartState();

public:
  // Captures ownership of the _headerState.
  // TODO: rewrite in terms of std::uniquePtr
  LoopInProcess(const llvm::Loop *_loop, ExecutionState *_headerState,
                const ref<LoopInProcess> &_outer);
  ~LoopInProcess();

  void updateChangedObjects(const ExecutionState& current, TimingSolver* solver);
  ExecutionState* nextRoundState(bool *analysisFinished);

  const llvm::Loop *getLoop() const { return loop; }
  const StateByteMask &getChangedBytes() const { return changedBytes; }
  const ExecutionState &getEntryState() const { return *restartState; }
  const ref<LoopInProcess> &getOuter() const { return outer; }
};

/// @brief ExecutionState representing a path under exploration
class ExecutionState {
public:
  typedef std::vector<StackFrame> stack_ty;

private:
  // unsupported, use copy constructor
  ExecutionState &operator=(const ExecutionState &);

  std::vector<FunctionAlias> fnAliases;
  std::map<uint64_t, std::string> readsIntercepts;
  std::map<uint64_t, std::string> writesIntercepts;

public:
  // Execution - Control Flow specific

  /// @brief Pointer to instruction to be executed after the current
  /// instruction
  KInstIterator pc;

  /// @brief Pointer to instruction which is currently executed
  KInstIterator prevPC;

  /// @brief Stack representing the current instruction stream
  stack_ty stack;

  /// @brief Remember from which Basic Block control flow arrived
  /// (i.e. to select the right phi values)
  unsigned incomingBBIndex;

  // Overall state of the state - Data specific

  /// @brief Address space used by this state (e.g. Global and Heap)
  AddressSpace addressSpace;

  /// @brief Information necessary for loop invariant induction.
  /// This is a stack capable of tracking loop nests.
  /// Owner.
  ref<LoopInProcess> loopInProcess;

  /// @brief A message from the final round of loop analysis to the
  /// klee_induce_invariants handler, to allow analysed loops
  /// skip it with no effect.
  ImmutableSet<const llvm::Loop*> analysedLoops;

  /// @brief This pointer keeps a copy of the state in case
  ///  we will need to process this loop. Owner.
  // TODO: replace with std::unique_ptr;
  ExecutionState *executionStateForLoopInProcess;

  /// @brief Constraints collected so far
  ConstraintManager constraints;

  /// Statistics and information

  /// @brief Costs for all queries issued for this state, in seconds
  mutable double queryCost;

  /// @brief Weight assigned for importance of this state.  Can be
  /// used for searchers to decide what paths to explore
  double weight;

  /// @brief Exploration depth, i.e., number of times KLEE branched for this state
  unsigned depth;

  /// @brief History of complete path: represents branches taken to
  /// reach/create this state (both concrete and symbolic)
  TreeOStream pathOS;

  /// @brief History of symbolic path: represents symbolic branches
  /// taken to reach/create this state
  TreeOStream symPathOS;

  /// @brief Counts how many instructions were executed since the last new
  /// instruction was covered.
  unsigned instsSinceCovNew;

  /// @brief Whether a new instruction was covered in this state
  bool coveredNew;

  /// @brief Disables forking for this state. Set by user code
  bool forkDisabled;

  /// @brief Set containing which lines in which files are covered by this state
  std::map<const std::string *, std::set<unsigned> > coveredLines;

  /// @brief Pointer to the process tree of the current state
  PTreeNode *ptreeNode;

  /// @brief Ordered list of symbolics: used to generate test cases.
  //
  // FIXME: Move to a shared list structure (not critical).
  std::vector<std::pair<const MemoryObject *, const Array *> > symbolics;

  /// @brief The list of possibly havoced memory locations with their names
  ///  and values placed at the last havoc event.
  std::map<const MemoryObject *, HavocInfo> havocs;

  /// @brief The list of registered havoc mem location names, used to guarantee
  ///  uniqueness of each name.
  std::set<std::string> havocNames;

  /// @brief Set of used array names for this state.  Used to avoid collisions.
  std::set<std::string> arrayNames;

  std::vector<CallInfo> callPath;
  SymbolSet relevantSymbols;

  /// @brief: a flag indicating that the state is genuine and not
  ///  a product of some ancillary analysis, like loop-invariant search.
  bool doTrace;

  /// @brief: Whether to forgive the undeclared memory location changing their
  ///  value during loop invariant analysis.
  bool condoneUndeclaredHavocs;


  std::string getFnAlias(std::string fn);
  void addFnAlias(std::string old_fn, std::string new_fn);
  void addFnRegexAlias(std::string fn_regex, std::string new_fn);
  void removeFnAlias(std::string fn);

  std::string getInterceptReader(uint64_t addr);
  std::string getInterceptWriter(uint64_t addr);
  void addReadsIntercept(uint64_t addr, std::string reader);
  void addWritesIntercept(uint64_t addr, std::string writer);

  // The objects handling the klee_open_merge calls this state ran through
  std::vector<ref<MergeHandler> > openMergeStack;

  // The numbers of times this state has run through Executor::stepInstruction
  std::uint64_t steppedInstructions;

private:
  ExecutionState() : ptreeNode(0) {}


  void loopRepetition(const llvm::Loop *dstLoop,
                      TimingSolver *solver,
                      bool *terminate);
  void loopEnter(const llvm::Loop *dstLoop);
  void loopExit(const llvm::Loop *srcLoop,
                bool *terminate);

public:
  ExecutionState(KFunction *kf);

  // XXX total hack, just used to make a state so solver can
  // use on structure
  ExecutionState(const std::vector<ref<Expr> > &assumptions);

  ExecutionState(const ExecutionState &state);

  ~ExecutionState();

  ExecutionState *branch();

  void addHavocInfo(const MemoryObject *mo, const std::string &name);

  void pushFrame(KInstIterator caller, KFunction *kf);
  void popFrame();

  void addSymbolic(const MemoryObject *mo, const Array *array);
  void addConstraint(ref<Expr> e) { constraints.addConstraint(e); }

  bool merge(const ExecutionState &b);
  void dumpStack(llvm::raw_ostream &out) const;
  bool isAccessibleAddr(ref<Expr> addr) const;
  ref<Expr> readMemoryChunk(ref<Expr> addr,
                            Expr::Width width,
                            bool circumventInaccessibility) const;
  void traceArgValue(ref<Expr> val, std::string name);
  void traceArgPtr(ref<Expr> arg, Expr::Width width,
                   std::string name,
                   std::string type,
                   bool tracePointeeIn,
                   bool tracePointeeOut);
  void traceArgFunPtr(ref<Expr> arg,
                      std::string name);
  void trace();
  void recordRetConstraints(CallInfo *info) const;

  void dumpConstraints() const;
  void symbolizeConcretes();
  ExecutionState* finishLoopRound(KFunction *kf);
  void updateLoopAnalysisForBlockTransfer(llvm::BasicBlock *dst,
                                          llvm::BasicBlock *src,
                                          TimingSolver *solver,
                                          bool *terminate);
  std::vector<ref<Expr> > relevantConstraints(SymbolSet symbols) const;
  void terminateState(ExecutionState **replace);
  void induceInvariantsForThisLoop(KInstruction *target);
  void startInvariantSearch();
};
}

#endif

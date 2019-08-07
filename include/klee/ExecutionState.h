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

  friend bool operator< (const FunctionAlias &c1, const FunctionAlias &c2);
};

struct FieldDescr {
  Expr::Width width;
  std::string type;
  std::string name;
  size_t addr;
  bool doTraceValueIn;
  bool doTraceValueOut;
  ref<Expr> inVal;
  ref<Expr> outVal;
  std::map<int, FieldDescr> fields;

  bool sameInvocationValue(const FieldDescr& other) const;
  bool eq(const FieldDescr& other) const;
};

struct CallArg {
  ref<Expr> expr;
  bool isPtr;
  llvm::Function* funPtr;
  std::string name;

  FieldDescr pointee;

  bool eq(const CallArg& other) const;
  bool sameInvocationValue(const CallArg& other) const;
};

struct RetVal {
  ref<Expr> expr;
  bool isPtr;
  llvm::Function* funPtr;

  FieldDescr pointee;

  bool eq(const RetVal& other) const;
};

struct CallExtraPtr {
  size_t ptr;
  FieldDescr pointee;
  bool accessibleIn;
  bool accessibleOut;

  std::string name;

  bool sameInvocationValue(const CallExtraPtr& other) const;
  bool eq(const CallExtraPtr& other) const;
};

//TODO: Store assumptions increment as well. it is an important part of the call
// these assumptions allow then to correctly match and distinguish call path prefixes.
struct CallInfo {
  llvm::Function* f;
  std::vector<CallArg> args;
  std::map<size_t, CallExtraPtr> extraPtrs;
  RetVal ret;
  bool returned;
  std::vector< ref<Expr> > callContext;
  std::vector< ref<Expr> > returnContext;
  llvm::DebugLoc callPlace;

  CallArg* getCallArgPtrp(ref<Expr> ptr);
  bool eq(const CallInfo& other) const;
  bool sameInvocation(const CallInfo* other) const;
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

  ImmutableSet<FunctionAlias> fnAliases;
  ImmutableMap<uint64_t, std::string> readsIntercepts;
  ImmutableMap<uint64_t, std::string> writesIntercepts;

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
  ImmutableMap<const MemoryObject *, HavocInfo> havocs;

  /// @brief The list of never havoced memory locations with their names.
  ImmutableMap<const MemoryObject *, std::string> noHavocs;

  /// @brief The list of registered havoc mem location names, used to guarantee
  ///  uniqueness of each name.
  ImmutableSet<std::string> havocNames;

  /// @brief The list of registered never-havoc mem location names, used to guarantee
  ///  uniqueness of each name.
  ImmutableSet<std::string> noHavocNames;

  /// @brief Set of used array names for this state.  Used to avoid collisions.
  ImmutableSet<std::string> arrayNames;

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
  void addNoHavocInfo(const MemoryObject *mo, const std::string &name);

  void pushFrame(KInstIterator caller, KFunction *kf);
  void popFrame();

  void addSymbolic(const MemoryObject *mo, const Array *array);
  void addConstraint(ref<Expr> e) { constraints.addConstraint(e); }

  bool merge(const ExecutionState &b);
  void dumpStack() const;
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
  void traceArgArr(ref<Expr> arg, Expr::Width width, size_t count,
                   std::string name,
                   std::string type,
                   bool tracePointeeIn,
                   bool tracePointeeOut);
  void traceArgFunPtr(ref<Expr> arg,
                      std::string name);
  void traceRet();
  void traceRetPtr(Expr::Width width,
                   bool tracePointee);
  void traceArgPtrField(ref<Expr> arg, int offset,
                        Expr::Width width, std::string name,
                        bool doTraceValueIn,
                        bool doTraceValueOut);
  void traceArgPtrFieldArr(ref<Expr> arg, int offset,
                           Expr::Width el_width, int count, std::string name,
                           bool doTraceValueIn,
                           bool doTraceValueOut);
  void traceArgPtrNestedField(ref<Expr> arg, int base_offset, int offset,
                              Expr::Width width, std::string name,
                              bool trace_in, bool trace_out);
  void traceArgPtrNestedFieldArr(ref<Expr> arg, int base_offset, int offset,
                                 Expr::Width width, int count, std::string name,
                                 bool trace_in, bool trace_out);
  void traceArgPtrNestedNestedField(ref<Expr> arg,
                                    int base_base_offset,
                                    int base_offset,
                                    int offset,
                                    Expr::Width width,
                                    std::string name,
                                    bool trace_in, bool trace_out);
  void traceExtraPtr(size_t ptr, Expr::Width width,
                     std::string name,
                     std::string type,
                     bool trace_in, bool trace_out);
  void traceExtraPtrArr(size_t ptr, Expr::Width width, size_t count,
                        std::string name,
                        std::string type,
                        bool trace_in, bool trace_out);
  void traceExtraPtrField(size_t ptr, int offset,
                          Expr::Width width, std::string name,
                          bool trace_in, bool trace_out);
  void traceExtraPtrFieldArray(size_t ptr, int offset,
                               Expr::Width el_width, int count, std::string name,
                               bool trace_in, bool trace_out);
  void traceExtraPtrNestedField(size_t ptr,
                                int base_offset,
                                int offset,
                                Expr::Width width,
                                std::string name,
                                bool trace_in, bool trace_out);
  void traceExtraPtrNestedFieldArr(size_t ptr,
                                   int base_offset,
                                   int offset,
                                   Expr::Width width,
                                   int count,
                                   std::string name,
                                   bool trace_in, bool trace_out);
  void traceExtraPtrNestedNestedField(size_t ptr,
                                      int base_base_offset,
                                      int base_offset,
                                      int offset,
                                      Expr::Width width,
                                      std::string name,
                                      bool trace_in, bool trace_out);
  void traceRetPtrField(int offset, Expr::Width width, std::string name,
                        bool doTraceValue);
  void traceRetPtrNestedField(int base_offset, int offset,
                              Expr::Width width, std::string name);

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

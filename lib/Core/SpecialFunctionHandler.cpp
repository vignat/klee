//===-- SpecialFunctionHandler.cpp ----------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <llvm/IR/Instructions.h>
#include "Memory.h"
#include "SpecialFunctionHandler.h"
#include "TimingSolver.h"

#include "klee/ExecutionState.h"

#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "klee/Internal/Support/Debug.h"
#include "klee/Internal/Support/ErrorHandling.h"

#include "Executor.h"
#include "MemoryManager.h"

#include "klee/CommandLine.h"

#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 3)
#include "llvm/IR/Module.h"
#else
#include "llvm/Module.h"
#endif
#include "llvm/ADT/Twine.h"

#if LLVM_VERSION_CODE <= LLVM_VERSION(3, 1)
#include "llvm/Target/TargetData.h"
#elif LLVM_VERSION_CODE <= LLVM_VERSION(3, 2)
#include "llvm/DataLayout.h"
#else
#include "llvm/IR/DataLayout.h"
#endif

#include <errno.h>

using namespace llvm;
using namespace klee;

namespace {
  cl::opt<bool>
  ReadablePosix("readable-posix-inputs",
            cl::init(false),
            cl::desc("Prefer creation of POSIX inputs (command-line arguments, files, etc.) with human readable bytes. "
                     "Note: option is expensive when creating lots of tests (default=false)"));

  cl::opt<bool>
  SilentKleeAssume("silent-klee-assume",
                   cl::init(false),
                   cl::desc("Silently terminate paths with an infeasible "
                            "condition given to klee_assume() rather than "
                            "emitting an error (default=false)"));
}


/// \todo Almost all of the demands in this file should be replaced
/// with terminateState calls.

///



// FIXME: We are more or less committed to requiring an intrinsic
// library these days. We can move some of this stuff there,
// especially things like realloc which have complicated semantics
// w.r.t. forking. Among other things this makes delayed query
// dispatch easier to implement.
static SpecialFunctionHandler::HandlerInfo handlerInfo[] = {
#define add(name, handler, ret) { name, \
                                  &SpecialFunctionHandler::handler, \
                                  false, ret, false }
#define addDNR(name, handler) { name, \
                                &SpecialFunctionHandler::handler, \
                                true, false, false }
  addDNR("__assert_rtn", handleAssertFail),
  addDNR("__assert_fail", handleAssertFail),
  addDNR("_assert", handleAssert),
  addDNR("abort", handleAbort),
  addDNR("_exit", handleExit),
  { "exit", &SpecialFunctionHandler::handleExit, true, false, true },
  addDNR("klee_abort", handleAbort),
  addDNR("klee_silent_exit", handleSilentExit),  
  addDNR("klee_report_error", handleReportError),

  add("calloc", handleCalloc, true),
  add("free", handleFree, false),
  add("klee_assume", handleAssume, false),
  add("klee_check_memory_access", handleCheckMemoryAccess, false),
  add("klee_get_valuef", handleGetValue, true),
  add("klee_get_valued", handleGetValue, true),
  add("klee_get_valuel", handleGetValue, true),
  add("klee_get_valuell", handleGetValue, true),
  add("klee_get_value_i32", handleGetValue, true),
  add("klee_get_value_i64", handleGetValue, true),
  add("klee_define_fixed_object", handleDefineFixedObject, false),
  add("klee_get_obj_size", handleGetObjSize, true),
  add("klee_get_errno", handleGetErrno, true),
  add("klee_is_symbolic", handleIsSymbolic, true),
  add("klee_make_symbolic", handleMakeSymbolic, false),
  add("klee_mark_global", handleMarkGlobal, false),
  add("klee_merge", handleMerge, false),
  add("klee_prefer_cex", handlePreferCex, false),
  add("klee_posix_prefer_cex", handlePosixPreferCex, false),
  add("klee_print_expr", handlePrintExpr, false),
  add("klee_print_range", handlePrintRange, false),
  add("klee_set_forking", handleSetForking, false),
  add("klee_stack_trace", handleStackTrace, false),
  add("klee_warning", handleWarning, false),
  add("klee_warning_once", handleWarningOnce, false),
  add("klee_alias_function", handleAliasFunction, false),
  add("malloc", handleMalloc, true),
  add("realloc", handleRealloc, true),
  add("klee_trace_paramf", handleTraceParam, false),
  add("klee_trace_paramd", handleTraceParam, false),
  add("klee_trace_paraml", handleTraceParam, false),
  add("klee_trace_paramll", handleTraceParam, false),
  add("klee_trace_param_i32", handleTraceParam, false),
  add("klee_trace_param_i64", handleTraceParam, false),
  add("klee_trace_param_ptr", handleTraceParamPtr, false),
  add("klee_trace_param_ptr_directed", handleTraceParamPtrDirected, false),
  add("klee_trace_param_tagged_ptr", handleTraceParamTaggedPtr, false),
  add("klee_trace_param_just_ptr", handleTraceParamJustPtr, false),
  add("klee_trace_param_fptr", handleTraceParamFPtr, false),
  add("klee_trace_ret", handleTraceRet, false),
  add("klee_trace_ret_ptr", handleTraceRetPtr, false),
  add("klee_trace_ret_just_ptr", handleTraceRetJustPtr, false),
  add("klee_trace_param_ptr_field", handleTraceParamPtrField, false),
  add("klee_trace_param_ptr_field_directed",
      handleTraceParamPtrFieldDirected, false),
  add("klee_trace_param_ptr_field_just_ptr",
      handleTraceParamPtrFieldJustPtr, false),
  add("klee_trace_ret_ptr_field", handleTraceRetPtrField, false),
  add("klee_trace_ret_ptr_field_just_ptr", handleTraceRetPtrFieldJustPtr, false),
  add("klee_trace_param_ptr_nested_field", handleTraceParamPtrNestedField, false),
  add("klee_trace_ret_ptr_nested_field", handleTraceRetPtrNestedField, false),
  add("klee_trace_extra_ptr", handleTraceExtraPtr, false),
  add("klee_trace_extra_ptr_field", handleTraceExtraPtrField, false),
  add("klee_trace_extra_ptr_field_just_ptr",
      handleTraceExtraPtrFieldJustPtr, false),
  add("klee_trace_extra_ptr_nested_field", handleTraceExtraPtrNestedField, false),
  add("klee_trace_extra_ptr_nested_nested_field",
      handleTraceExtraPtrNestedNestedField, false),
  add("klee_forget_all", handleForgetAll, false),
  add("klee_induce_invariants", handleInduceInvariants, true),
  add("klee_forbid_access", handleForbidAccess, false),
  add("klee_allow_access", handleAllowAccess, false),
  add("klee_dump_constraints", handleDumpConstraints, false),

  // operator delete[](void*)
  add("_ZdaPv", handleDeleteArray, false),
  // operator delete(void*)
  add("_ZdlPv", handleDelete, false),

  // operator new[](unsigned int)
  add("_Znaj", handleNewArray, true),
  // operator new(unsigned int)
  add("_Znwj", handleNew, true),

  // FIXME-64: This is wrong for 64-bit long...

  // operator new[](unsigned long)
  add("_Znam", handleNewArray, true),
  // operator new(unsigned long)
  add("_Znwm", handleNew, true),

  // clang -fsanitize=unsigned-integer-overflow
  add("__ubsan_handle_add_overflow", handleAddOverflow, false),
  add("__ubsan_handle_sub_overflow", handleSubOverflow, false),
  add("__ubsan_handle_mul_overflow", handleMulOverflow, false),
  add("__ubsan_handle_divrem_overflow", handleDivRemOverflow, false),

#undef addDNR
#undef add  
};

SpecialFunctionHandler::const_iterator SpecialFunctionHandler::begin() {
  return SpecialFunctionHandler::const_iterator(handlerInfo);
}

SpecialFunctionHandler::const_iterator SpecialFunctionHandler::end() {
  // NULL pointer is sentinel
  return SpecialFunctionHandler::const_iterator(0);
}

SpecialFunctionHandler::const_iterator& SpecialFunctionHandler::const_iterator::operator++() {
  ++index;
  if ( index >= SpecialFunctionHandler::size())
  {
    // Out of range, return .end()
    base=0; // Sentinel
    index=0;
  }

  return *this;
}

int SpecialFunctionHandler::size() {
	return sizeof(handlerInfo)/sizeof(handlerInfo[0]);
}

SpecialFunctionHandler::SpecialFunctionHandler(Executor &_executor) 
  : executor(_executor) {}


void SpecialFunctionHandler::prepare() {
  unsigned N = size();

  for (unsigned i=0; i<N; ++i) {
    HandlerInfo &hi = handlerInfo[i];
    Function *f = executor.kmodule->module->getFunction(hi.name);
    
    // No need to create if the function doesn't exist, since it cannot
    // be called in that case.
  
    if (f && (!hi.doNotOverride || f->isDeclaration())) {
      // Make sure NoReturn attribute is set, for optimization and
      // coverage counting.
      if (hi.doesNotReturn)
#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 3)
        f->addFnAttr(Attribute::NoReturn);
#elif LLVM_VERSION_CODE >= LLVM_VERSION(3, 2)
        f->addFnAttr(Attributes::NoReturn);
#else
        f->addFnAttr(Attribute::NoReturn);
#endif

      // Change to a declaration since we handle internally (simplifies
      // module and allows deleting dead code).
      if (!f->isDeclaration())
        f->deleteBody();
    }
  }
}

void SpecialFunctionHandler::bind() {
  unsigned N = sizeof(handlerInfo)/sizeof(handlerInfo[0]);

  for (unsigned i=0; i<N; ++i) {
    HandlerInfo &hi = handlerInfo[i];
    Function *f = executor.kmodule->module->getFunction(hi.name);
    
    if (f && (!hi.doNotOverride || f->isDeclaration()))
      handlers[f] = std::make_pair(hi.handler, hi.hasReturnValue);
  }
}


bool SpecialFunctionHandler::handle(ExecutionState &state, 
                                    Function *f,
                                    KInstruction *target,
                                    std::vector< ref<Expr> > &arguments) {
  handlers_ty::iterator it = handlers.find(f);
  if (it != handlers.end()) {    
    Handler h = it->second.first;
    bool hasReturnValue = it->second.second;
     // FIXME: Check this... add test?
    if (!hasReturnValue && !target->inst->use_empty()) {
      executor.terminateStateOnExecError(state, 
                                         "expected return value from void special function");
    } else {
      (this->*h)(state, target, arguments);
    }
    return true;
  } else {
    return false;
  }
}

/****/

// reads a concrete string from memory
std::string 
SpecialFunctionHandler::readStringAtAddress(ExecutionState &state, 
                                            ref<Expr> addressExpr) {
  ObjectPair op;
  addressExpr = executor.toUnique(state, addressExpr);
  ref<ConstantExpr> address = cast<ConstantExpr>(addressExpr);
  if (!state.addressSpace.resolveOne(address, op))
    assert(0 && "XXX out of bounds / multiple resolution unhandled");
  bool res __attribute__ ((unused));
  assert(executor.solver->mustBeTrue(state, 
                                     EqExpr::create(address, 
                                                    op.first->getBaseExpr()),
                                     res) &&
         res &&
         "XXX interior pointer unhandled");
  const MemoryObject *mo = op.first;
  const ObjectState *os = op.second;

  char *buf = new char[mo->size];

  unsigned i;
  for (i = 0; i < mo->size - 1; i++) {
    ref<Expr> cur = os->read8(i);
    cur = executor.toUnique(state, cur);
    assert(isa<ConstantExpr>(cur) && 
           "hit symbolic char while reading concrete string");
    buf[i] = cast<ConstantExpr>(cur)->getZExtValue(8);
  }
  buf[i] = 0;
  
  std::string result(buf);
  delete[] buf;
  return result;
}

/****/

void SpecialFunctionHandler::handleAbort(ExecutionState &state,
                           KInstruction *target,
                           std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==0 && "invalid number of arguments to abort");
  executor.terminateStateOnError(state, "abort failure", Executor::Abort);
}

void SpecialFunctionHandler::handleExit(ExecutionState &state,
                           KInstruction *target,
                           std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 && "invalid number of arguments to exit");
  executor.terminateStateOnExit(state);
}

void SpecialFunctionHandler::handleSilentExit(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 && "invalid number of arguments to exit");
  executor.terminateState(state);
}

void SpecialFunctionHandler::handleAliasFunction(ExecutionState &state,
						 KInstruction *target,
						 std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==2 && 
         "invalid number of arguments to klee_alias_function");
  std::string old_fn = readStringAtAddress(state, arguments[0]);
  std::string new_fn = readStringAtAddress(state, arguments[1]);
  KLEE_DEBUG_WITH_TYPE("alias_handling", llvm::errs() << "Replacing " << old_fn
                                           << "() with " << new_fn << "()\n");
  if (old_fn == new_fn)
    state.removeFnAlias(old_fn);
  else state.addFnAlias(old_fn, new_fn);
}

void SpecialFunctionHandler::handleAssert(ExecutionState &state,
                                          KInstruction *target,
                                          std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==3 && "invalid number of arguments to _assert");  
  executor.terminateStateOnError(state,
				 "ASSERTION FAIL: " + readStringAtAddress(state, arguments[0]),
				 Executor::Assert);
}

void SpecialFunctionHandler::handleAssertFail(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==4 && "invalid number of arguments to __assert_fail");
  executor.terminateStateOnError(state,
				 "ASSERTION FAIL: " + readStringAtAddress(state, arguments[0]),
				 Executor::Assert);
}

void SpecialFunctionHandler::handleReportError(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==4 && "invalid number of arguments to klee_report_error");
  
  // arguments[0], arguments[1] are file, line
  executor.terminateStateOnError(state,
				 readStringAtAddress(state, arguments[2]),
				 Executor::ReportError,
				 readStringAtAddress(state, arguments[3]).c_str());
}

void SpecialFunctionHandler::handleMerge(ExecutionState &state,
                           KInstruction *target,
                           std::vector<ref<Expr> > &arguments) {
  // nop
}

void SpecialFunctionHandler::handleNew(ExecutionState &state,
                         KInstruction *target,
                         std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to new");

  executor.executeAlloc(state, arguments[0], false, target);
}

void SpecialFunctionHandler::handleDelete(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments) {
  // FIXME: Should check proper pairing with allocation type (malloc/free,
  // new/delete, new[]/delete[]).

  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to delete");
  executor.executeFree(state, arguments[0]);
}

void SpecialFunctionHandler::handleNewArray(ExecutionState &state,
                              KInstruction *target,
                              std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to new[]");
  executor.executeAlloc(state, arguments[0], false, target);
}

void SpecialFunctionHandler::handleDeleteArray(ExecutionState &state,
                                 KInstruction *target,
                                 std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to delete[]");
  executor.executeFree(state, arguments[0]);
}

void SpecialFunctionHandler::handleMalloc(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==1 && "invalid number of arguments to malloc");
  executor.executeAlloc(state, arguments[0], false, target);
}

void SpecialFunctionHandler::handleAssume(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_assume");
  
  ref<Expr> e = arguments[0];
  
  if (e->getWidth() != Expr::Bool)
    e = NeExpr::create(e, ConstantExpr::create(0, e->getWidth()));
  
  bool res;
  bool success __attribute__ ((unused)) = executor.solver->mustBeFalse(state, e, res);
  assert(success && "FIXME: Unhandled solver failure");
  if (res) {
    if (SilentKleeAssume) {
      executor.terminateState(state);
    } else {
      executor.terminateStateOnError(state,
                                     "invalid klee_assume call (provably false)",
                                     Executor::User);
    }
  } else {
    executor.addConstraint(state, e);
  }
}

void SpecialFunctionHandler::handleIsSymbolic(ExecutionState &state,
                                KInstruction *target,
                                std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_is_symbolic");

  executor.bindLocal(target, state, 
                     ConstantExpr::create(!isa<ConstantExpr>(arguments[0]),
                                          Expr::Int32));
}

void SpecialFunctionHandler::handlePreferCex(ExecutionState &state,
                                             KInstruction *target,
                                             std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_prefex_cex");

  ref<Expr> cond = arguments[1];
  if (cond->getWidth() != Expr::Bool)
    cond = NeExpr::create(cond, ConstantExpr::alloc(0, cond->getWidth()));

  Executor::ExactResolutionList rl;
  executor.resolveExact(state, arguments[0], rl, "prefex_cex");
  
  assert(rl.size() == 1 &&
         "prefer_cex target must resolve to precisely one object");

  rl[0].first.first->cexPreferences.push_back(cond);
}

void SpecialFunctionHandler::handlePosixPreferCex(ExecutionState &state,
                                             KInstruction *target,
                                             std::vector<ref<Expr> > &arguments) {
  if (ReadablePosix)
    return handlePreferCex(state, target, arguments);
}

void SpecialFunctionHandler::handlePrintExpr(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_print_expr");

  std::string msg_str = readStringAtAddress(state, arguments[0]);
  llvm::errs() << msg_str << ":" << arguments[1] << "\n";
}

void SpecialFunctionHandler::handleSetForking(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_set_forking");
  ref<Expr> value = executor.toUnique(state, arguments[0]);
  
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(value)) {
    state.forkDisabled = CE->isZero();
  } else {
    executor.terminateStateOnError(state, 
                                   "klee_set_forking requires a constant arg",
                                   Executor::User);
  }
}

void SpecialFunctionHandler::handleStackTrace(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments) {
  state.dumpStack(outs());
}

void SpecialFunctionHandler::handleWarning(ExecutionState &state,
                                           KInstruction *target,
                                           std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 && "invalid number of arguments to klee_warning");

  std::string msg_str = readStringAtAddress(state, arguments[0]);
  klee_warning("%s: %s", state.stack.back().kf->function->getName().data(), 
               msg_str.c_str());
}

void SpecialFunctionHandler::handleWarningOnce(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_warning_once");

  std::string msg_str = readStringAtAddress(state, arguments[0]);
  klee_warning_once(0, "%s: %s", state.stack.back().kf->function->getName().data(),
                    msg_str.c_str());
}

void SpecialFunctionHandler::handlePrintRange(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_print_range");

  std::string msg_str = readStringAtAddress(state, arguments[0]);
  llvm::errs() << msg_str << ":" << arguments[1];
  if (!isa<ConstantExpr>(arguments[1])) {
    // FIXME: Pull into a unique value method?
    ref<ConstantExpr> value;
    bool success __attribute__ ((unused)) = executor.solver->getValue(state, arguments[1], value);
    assert(success && "FIXME: Unhandled solver failure");
    bool res;
    success = executor.solver->mustBeTrue(state, 
                                          EqExpr::create(arguments[1], value), 
                                          res);
    assert(success && "FIXME: Unhandled solver failure");
    if (res) {
      llvm::errs() << " == " << value;
    } else { 
      llvm::errs() << " ~= " << value;
      std::pair< ref<Expr>, ref<Expr> > res =
        executor.solver->getRange(state, arguments[1]);
      llvm::errs() << " (in [" << res.first << ", " << res.second <<"])";
    }
  }
  llvm::errs() << "\n";
}

void SpecialFunctionHandler::handleGetObjSize(ExecutionState &state,
                                  KInstruction *target,
                                  std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_get_obj_size");
  Executor::ExactResolutionList rl;
  executor.resolveExact(state, arguments[0], rl, "klee_get_obj_size");
  for (Executor::ExactResolutionList::iterator it = rl.begin(), 
         ie = rl.end(); it != ie; ++it) {
    executor.bindLocal(
        target, *it->second,
        ConstantExpr::create(it->first.first->size,
                             executor.kmodule->targetData->getTypeSizeInBits(
                                 target->inst->getType())));
  }
}

void SpecialFunctionHandler::handleGetErrno(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==0 &&
         "invalid number of arguments to klee_get_errno");
  executor.bindLocal(target, state,
                     ConstantExpr::create(errno, Expr::Int32));
}

void SpecialFunctionHandler::handleCalloc(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==2 &&
         "invalid number of arguments to calloc");

  ref<Expr> size = MulExpr::create(arguments[0],
                                   arguments[1]);
  executor.executeAlloc(state, size, false, target, true);
}

void SpecialFunctionHandler::handleRealloc(ExecutionState &state,
                            KInstruction *target,
                            std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==2 &&
         "invalid number of arguments to realloc");
  ref<Expr> address = arguments[0];
  ref<Expr> size = arguments[1];

  Executor::StatePair zeroSize = executor.fork(state, 
                                               Expr::createIsZero(size), 
                                               true);
  
  if (zeroSize.first) { // size == 0
    executor.executeFree(*zeroSize.first, address, target);   
  }
  if (zeroSize.second) { // size != 0
    Executor::StatePair zeroPointer = executor.fork(*zeroSize.second, 
                                                    Expr::createIsZero(address), 
                                                    true);
    
    if (zeroPointer.first) { // address == 0
      executor.executeAlloc(*zeroPointer.first, size, false, target);
    } 
    if (zeroPointer.second) { // address != 0
      Executor::ExactResolutionList rl;
      executor.resolveExact(*zeroPointer.second, address, rl, "realloc");
      
      for (Executor::ExactResolutionList::iterator it = rl.begin(), 
             ie = rl.end(); it != ie; ++it) {
        executor.executeAlloc(*it->second, size, false, target, false, 
                              it->first.second);
      }
    }
  }
}

void SpecialFunctionHandler::handleFree(ExecutionState &state,
                          KInstruction *target,
                          std::vector<ref<Expr> > &arguments) {
  // XXX should type check args
  assert(arguments.size()==1 &&
         "invalid number of arguments to free");
  executor.executeFree(state, arguments[0]);
}

void SpecialFunctionHandler::handleCheckMemoryAccess(ExecutionState &state,
                                                     KInstruction *target,
                                                     std::vector<ref<Expr> > 
                                                       &arguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_check_memory_access");

  ref<Expr> address = executor.toUnique(state, arguments[0]);
  ref<Expr> size = executor.toUnique(state, arguments[1]);
  if (!isa<ConstantExpr>(address) || !isa<ConstantExpr>(size)) {
    executor.terminateStateOnError(state, 
                                   "check_memory_access requires constant args",
				   Executor::User);
  } else {
    ObjectPair op;

    if (!state.addressSpace.resolveOne(cast<ConstantExpr>(address), op)) {
      executor.terminateStateOnError(state,
                                     "check_memory_access: memory error",
				     Executor::Ptr, NULL,
                                     executor.getAddressInfo(state, address));
    } else {
      ref<Expr> chk = 
        op.first->getBoundsCheckPointer(address, 
                                        cast<ConstantExpr>(size)->getZExtValue());
      if (!chk->isTrue()) {
        executor.terminateStateOnError(state,
                                       "check_memory_access: memory error",
				       Executor::Ptr, NULL,
                                       executor.getAddressInfo(state, address));
      }
    }
  }
}

void SpecialFunctionHandler::handleGetValue(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_get_value");

  executor.executeGetValue(state, arguments[0], target);
}

void SpecialFunctionHandler::handleDefineFixedObject(ExecutionState &state,
                                                     KInstruction *target,
                                                     std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==2 &&
         "invalid number of arguments to klee_define_fixed_object");
  assert(isa<ConstantExpr>(arguments[0]) &&
         "expect constant address argument to klee_define_fixed_object");
  assert(isa<ConstantExpr>(arguments[1]) &&
         "expect constant size argument to klee_define_fixed_object");
  
  uint64_t address = cast<ConstantExpr>(arguments[0])->getZExtValue();
  uint64_t size = cast<ConstantExpr>(arguments[1])->getZExtValue();
  MemoryObject *mo = executor.memory->allocateFixed(address, size, state.prevPC->inst);
  executor.bindObjectInState(state, mo, false);
  mo->isUserSpecified = true; // XXX hack;
}

void SpecialFunctionHandler::handleMakeSymbolic(ExecutionState &state,
                                                KInstruction *target,
                                                std::vector<ref<Expr> > &arguments) {
  std::string name;

  // FIXME: For backwards compatibility, we should eventually enforce the
  // correct arguments.
  if (arguments.size() == 2) {
    name = "unnamed";
  } else {
    // FIXME: Should be a user.err, not an assert.
    assert(arguments.size()==3 &&
           "invalid number of arguments to klee_make_symbolic");  
    name = readStringAtAddress(state, arguments[2]);
  }

  Executor::ExactResolutionList rl;
  executor.resolveExact(state, arguments[0], rl, "make_symbolic");

  for (Executor::ExactResolutionList::iterator it = rl.begin(), 
         ie = rl.end(); it != ie; ++it) {
    const MemoryObject *mo = it->first.first;
    mo->setName(name);

    const ObjectState *old = it->first.second;
    ExecutionState *s = it->second;

    if (old->readOnly) {
      executor.terminateStateOnError(*s, "cannot make readonly object symbolic",
                                     Executor::User);
      return;
    }
    if (!old->accessible) {
      executor.terminateStateOnError
        (*s, llvm::Twine("cannot make inaccessible object symbolic") +
         "the object was rendered inaccessible due to:" +
         old->inaccessible_message, Executor::Inaccessible);
      return;
    }

    // FIXME: Type coercion should be done consistently somewhere.
    bool res;
    bool success __attribute__ ((unused)) =
      executor.solver->mustBeTrue(*s, 
                                  EqExpr::create(ZExtExpr::create(arguments[1],
                                                                  Context::get().getPointerWidth()),
                                                 mo->getSizeExpr()),
                                  res);
    assert(success && "FIXME: Unhandled solver failure");
    
    if (res) {
      executor.executeMakeSymbolic(*s, mo, name);
    } else {      
      executor.terminateStateOnError(*s, 
                                     "wrong size given to klee_make_symbolic[_name]", 
                                     Executor::User);
    }
  }
}

void SpecialFunctionHandler::handleMarkGlobal(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments) {
  assert(arguments.size()==1 &&
         "invalid number of arguments to klee_mark_global");  

  Executor::ExactResolutionList rl;
  executor.resolveExact(state, arguments[0], rl, "mark_global");
  
  for (Executor::ExactResolutionList::iterator it = rl.begin(), 
         ie = rl.end(); it != ie; ++it) {
    const MemoryObject *mo = it->first.first;
    assert(!mo->isLocal);
    mo->isGlobal = true;
  }
}

void SpecialFunctionHandler::handleAddOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments) {
  executor.terminateStateOnError(state, "overflow on unsigned addition",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleSubOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments) {
  executor.terminateStateOnError(state, "overflow on unsigned subtraction",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleMulOverflow(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments) {
  executor.terminateStateOnError(state, "overflow on unsigned multiplication",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleDivRemOverflow(ExecutionState &state,
                                                  KInstruction *target,
                                                  std::vector<ref<Expr> > &arguments) {
  executor.terminateStateOnError(state,
                                 "overflow on division or remainder",
                                 Executor::Overflow);
}

void SpecialFunctionHandler::handleTraceRet(ExecutionState &state,
                                            KInstruction *target,
                                            std::vector<ref<Expr> > &arguments) {
  state.traceRet();
}

void SpecialFunctionHandler::handleTraceRetPtr(ExecutionState &state,
                                               KInstruction *target,
                                               std::vector<ref<Expr> > &arguments) {
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[0]))->getZExtValue();
  width = width * 8;
  state.traceRetPtr(width, true);
}

void SpecialFunctionHandler::handleTraceRetJustPtr(ExecutionState &state,
                                                   KInstruction *target,
                                                   std::vector<ref<Expr> >
                                                   &arguments) {
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[0]))->getZExtValue();
  width = width * 8;
  state.traceRetPtr(width, false);
}

void SpecialFunctionHandler::handleTraceRetPtrField(ExecutionState &state,
                                                    KInstruction *target,
                                                    std::vector<ref<Expr> > &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[0]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[2]);
  width = width * 8;//Convert to bits.
  state.traceRetPtrField(offset, width, name, true);
}

void SpecialFunctionHandler::handleTraceRetPtrFieldJustPtr
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> > &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[0]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[2]);
  width = width * 8;//Convert to bits.
  state.traceRetPtrField(offset, width, name, false);
}

void SpecialFunctionHandler::handleTraceRetPtrNestedField
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> > &arguments) {
  int base_offset = (cast<klee::ConstantExpr>(arguments[0]))->getZExtValue();
  int offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[3]);
  width = width * 8;//Convert to bits.
  state.traceRetPtrNestedField(base_offset, offset, width, name);
}

void SpecialFunctionHandler::handleTraceExtraPtr(ExecutionState &state,
                                                 KInstruction *target,
                                                 std::vector<ref<Expr> >
                                                 &arguments) {
  assert(isa<klee::ConstantExpr>(arguments[1]) && "Width must be a static constant.");
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  width = width * 8;//Convert to bits.
  std::string name = readStringAtAddress(state, arguments[2]);
  std::string type = readStringAtAddress(state, arguments[3]);
  size_t ptr = (cast<ConstantExpr>(arguments[0]))->getZExtValue();
  state.traceExtraPtr(ptr, width, name, type, true);
}


void SpecialFunctionHandler::handleTraceParamPtrNestedField
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> > &arguments) {
  int base_offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  int offset = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[3]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[4]);
  width = width * 8;//Convert to bits.
  state.traceArgPtrNestedField(arguments[0], base_offset, offset, width, name);
}

void SpecialFunctionHandler::handleTraceParamPtrField(ExecutionState &state,
                                                      KInstruction *target,
                                                      std::vector<ref<Expr> >
                                                      &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[3]);
  width = width * 8;//Convert to bits.
  state.traceArgPtrField(arguments[0], offset, width, name, true, true);
}

void SpecialFunctionHandler::handleTraceParamPtrFieldDirected(ExecutionState &state,
                                                              KInstruction *target,
                                                              std::vector<ref<Expr> >
                                                              &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[3]);
  width = width * 8;//Convert to bits.
  size_t direction = (cast<klee::ConstantExpr>(arguments[4]))->getZExtValue();
  bool trace_in = false, trace_out = false;
  switch(direction) {
  case 0: trace_in = false; trace_out = false; break;
  case 1: trace_in = true;  trace_out = false; break;
  case 2: trace_in = false; trace_out = true; break;
  case 3: trace_in = true;  trace_out = true; break;
  default:
    executor.terminateStateOnError
      (state, "Unrecognized tracing direction",
       Executor::User);
    return;
  }
  state.traceArgPtrField(arguments[0], offset, width, name, trace_in, trace_out);
}

void SpecialFunctionHandler::handleTraceExtraPtrField(ExecutionState &state,
                                                      KInstruction *target,
                                                      std::vector<ref<Expr> >
                                                      &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[3]);
  width = width * 8;//Convert to bits.
  size_t ptr = (cast<ConstantExpr>(arguments[0]))->getZExtValue();
  state.traceExtraPtrField(ptr, offset, width, name, true);
}

void SpecialFunctionHandler::handleTraceExtraPtrNestedField
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> >
 &arguments) {
  size_t ptr = (cast<ConstantExpr>(arguments[0]))->getZExtValue();
  int base_offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  int offset = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[3]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[4]);
  width = width * 8;//Convert to bits.
  state.traceExtraPtrNestedField(ptr, base_offset, offset, width, name, true);
}

void SpecialFunctionHandler::handleTraceExtraPtrNestedNestedField
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> >
 &arguments) {
  size_t ptr = (cast<ConstantExpr>(arguments[0]))->getZExtValue();
  int base_base_offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  int base_offset = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  int offset = (cast<klee::ConstantExpr>(arguments[3]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[4]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[5]);
  width = width * 8;//Convert to bits.
  state.traceExtraPtrNestedNestedField(ptr, base_base_offset,
                                       base_offset, offset, width, name, true);
}

void SpecialFunctionHandler::handleTraceExtraPtrFieldJustPtr
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> >
 &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[3]);
  width = width * 8;//Convert to bits.
  size_t ptr = (cast<ConstantExpr>(arguments[0]))->getZExtValue();
  state.traceExtraPtrField(ptr, offset, width, name, false);
}

void SpecialFunctionHandler::handleTraceParamPtrFieldJustPtr
(ExecutionState &state,
 KInstruction *target,
 std::vector<ref<Expr> >
 &arguments) {
  int offset = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[2]))->getZExtValue();
  std::string name = readStringAtAddress(state, arguments[3]);
  width = width * 8;//Convert to bits.
  state.traceArgPtrField(arguments[0], offset, width, name, false, false);
}

void SpecialFunctionHandler::handleTraceParam(ExecutionState &state,
                                              KInstruction *target,
                                              std::vector<ref<Expr> > &arguments) {
  std::string name = readStringAtAddress(state, arguments[1]);
  state.traceArgValue(arguments[0], name);
}

void SpecialFunctionHandler::handleTraceParamTaggedPtr(ExecutionState &state,
                                                       KInstruction *target,
                                                       std::vector<ref<Expr> >
                                                       &arguments) {
  if (!isa<klee::ConstantExpr>(arguments[1])) {
    executor.terminateStateOnError
      (state, "Width must be a static constant.",
       Executor::User);
    return;
  }
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  width = width * 8;//Convert to bits.
  std::string name = readStringAtAddress(state, arguments[2]);
  std::string type = readStringAtAddress(state, arguments[3]);
  state.traceArgPtr(arguments[0], width, name, type, true, true);
}

void SpecialFunctionHandler::handleTraceParamPtr(ExecutionState &state,
                                                 KInstruction *target,
                                                 std::vector<ref<Expr> >
                                                 &arguments) {
  if (!isa<klee::ConstantExpr>(arguments[1])) {
    executor.terminateStateOnError
      (state, "Width must be a static constant.",
       Executor::User);
    return;
  }
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  width = width * 8;//Convert to bits.
  std::string name = readStringAtAddress(state, arguments[2]);
  state.traceArgPtr(arguments[0], width, name, "", true, true);
}

void SpecialFunctionHandler::handleTraceParamPtrDirected(ExecutionState &state,
                                                         KInstruction *target,
                                                         std::vector<ref<Expr> >
                                                         &arguments) {
  if (!isa<klee::ConstantExpr>(arguments[1])) {
    executor.terminateStateOnError
      (state, "Width must be a static constant.",
       Executor::User);
    return;
  }
  if (!isa<klee::ConstantExpr>(arguments[3])) {
    executor.terminateStateOnError
      (state, "Direction must be a static constant.",
       Executor::User);
    return;
  }
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  width = width * 8;//Convert to bits.
  std::string name = readStringAtAddress(state, arguments[2]);
  size_t direction = (cast<klee::ConstantExpr>(arguments[3]))->getZExtValue();
  bool trace_in = false, trace_out = false;
  switch(direction) {
  case 0: trace_in = false; trace_out = false; break;
  case 1: trace_in = true;  trace_out = false; break;
  case 2: trace_in = false; trace_out = true; break;
  case 3: trace_in = true;  trace_out = true; break;
  default:
    executor.terminateStateOnError
      (state, "Unrecognized tracing direction",
       Executor::User);
    return;
  }
  state.traceArgPtr(arguments[0], width, name, "", trace_in, trace_out);
}


void SpecialFunctionHandler::handleTraceParamJustPtr(ExecutionState &state,
                                                     KInstruction *target,
                                                     std::vector<ref<Expr> >
                                                     &arguments) {
  if (!isa<klee::ConstantExpr>(arguments[1])) {
    executor.terminateStateOnError
      (state, "Width must be a static constant.",
       Executor::User);
    return;
  }
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  width = width * 8;//Convert to bits.
  std::string name = readStringAtAddress(state, arguments[2]);
  state.traceArgPtr(arguments[0], width, name, "", false, false);
}

void SpecialFunctionHandler::handleTraceParamFPtr(ExecutionState &state,
                                                  KInstruction *target,
                                                  std::vector<ref<Expr> > &arguments) {
  std::string name = readStringAtAddress(state, arguments[1]);
  state.traceArgFunPtr(arguments[0], name);
}

void SpecialFunctionHandler::handleForgetAll
(ExecutionState &state, KInstruction *target, std::vector<ref<Expr> > &arguments) {
  state.constraints.clear();
  state.symbolizeConcretes();
}

void SpecialFunctionHandler::handleInduceInvariants
(ExecutionState &state, KInstruction *target, std::vector<ref<Expr> > &arguments) {
  state.induceInvariantsForThisLoop(target);
}

void SpecialFunctionHandler::handleForbidAccess
(ExecutionState &state, KInstruction *target,
 std::vector<ref<Expr> > &arguments) {
  if (!isa<klee::ConstantExpr>(arguments[0])) {
    executor.terminateStateOnError
      (state, "Symbolic address for klee_forbid_access is not supported",
       Executor::Unhandled);
    return;
  }
  if (!isa<klee::ConstantExpr>(arguments[1])) {
    executor.terminateStateOnError
      (state, "Symbolic width for klee_forbid_access is not supported",
       Executor::Unhandled);
    return;
  }
  ref<ConstantExpr> addr = cast<klee::ConstantExpr>(arguments[0]);
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();
  std::string message = readStringAtAddress(state, arguments[2]);

  ObjectPair op;
  bool success = state.addressSpace.resolveOne(addr, op);
  if (!success) {
    executor.terminateStateOnError
      (state, "The address does not exist.", Executor::User);
    return;
  }
  const MemoryObject *mo = op.first;
  if (mo->size != width) {
    executor.terminateStateOnError
      (state, "The provided size does not match the size of the object.",
       Executor::User);
    return;
  }
  const ObjectState *os = op.second;
  if (os->readOnly) {
    executor.terminateStateOnError
      (state, "The object is readonly, can not render it inaccessible",
       Executor::User);
    return;
  }
  if (!os->isAccessible()) {
    executor.terminateStateOnError
      (state, "The object is already inaccessible.",
       Executor::User);
    return;
  }
  ObjectState *wos = state.addressSpace.getWriteable(mo, os);
  wos->forbidAccess(message);
}

void SpecialFunctionHandler::handleAllowAccess
(ExecutionState &state, KInstruction *target,
 std::vector<ref<Expr> > &arguments) {
  if (!isa<klee::ConstantExpr>(arguments[0])) {
    executor.terminateStateOnError
      (state, "Symbolic address for klee_allow_access is not supported",
       Executor::Unhandled);
    return;
  }
  if (!isa<klee::ConstantExpr>(arguments[1])) {
    executor.terminateStateOnError
      (state, "Symbolic width for klee_allow_access is not supported",
       Executor::Unhandled);
    return;
  }
  ref<ConstantExpr> addr = cast<klee::ConstantExpr>(arguments[0]);
  Expr::Width width = (cast<klee::ConstantExpr>(arguments[1]))->getZExtValue();

  ObjectPair op;
  bool success = state.addressSpace.resolveOne(addr, op);
  if (!success) {
    executor.terminateStateOnError
      (state, "The address does not exist.", Executor::User);
    return;
  }
  const MemoryObject *mo = op.first;
  if (mo->size != width) {
    executor.terminateStateOnError
      (state, "The provided size does not match the size of the object.",
       Executor::User);
    return;
  }
  const ObjectState *os = op.second;
  if (os->isAccessible()) {
    executor.terminateStateOnError
      (state, "The object is already accessible.",
       Executor::User);
    return;
  }
  state.addressSpace.allowAccess(mo, os);
}

void SpecialFunctionHandler::handleDumpConstraints
(ExecutionState &state, KInstruction *target,
 std::vector<ref<Expr> > &arguments) {
  state.dumpConstraints();
}

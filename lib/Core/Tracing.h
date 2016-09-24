//===-- Tracing.h -----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_TRACING_H
#define KLEE_TRACING_H

#include <map>
#include <string>
#include <vector>

#include "TracingDefs.h"
#include "klee/Expr.h"
#include "klee/ExecutionState.h"
#include "klee/util/GetExprSymbols.h"

namespace klee {

  class RuntimeValue;
  class MetaValue;

  /// MetaValue -- holds the memory layout for one element in the hierarchy.
  class MetaValue {

  protected:

  public:
    MetaValue(Expr::Width width);
    virtual ~MetaValue();

    virtual uptr<RuntimeValue> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaStructure -- holds the memory layout for a structure
  class MetaStructure :public MetaValue {
    class MetaStructureField {
      std::string name;
      unsigned offset;
      MetaValue* layout;
    };
    TracingMap<unsigned, uptr<MetaStructureField> > fields;

  public:
    virtual uptr<RuntimeValue> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaArray -- holds the memory layout for an array
  class MetaArray :public MetaValue {
    uptr<MetaValue> cell;
    uint64_t len;

  public:
    virtual uptr<RuntimeValue> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaPointer -- holds the memory layout for a pointer
  class MetaPointer :public MetaValue {
    uptr<MetaValue> ptee;

  public:
    virtual uptr<RuntimeValue> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaPlainVal -- holds the memory layout for a plain value.
  class MetaPlainVal :public MetaValue {
    Expr::Width width;
    bool is_signed;

  public:
    virtual uptr<RuntimeValue> ReadValue(uint64_t addr, ExecutionState *state) const;
  };


  class RuntimeValue {
  protected:
    wptr<MetaValue> layout;
  public:
    virtual ~RuntimeValue();
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet&& getSymbols() const;
  };

  // StructureValue -- holds runtime value of a structure object.
  class StructureValue :public RuntimeValue {
    TracingMap<unsigned, uptr<RuntimeValue> > fields;

  public:
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
  };

  /// ArrayValue -- holds runtime value for an array;
  class ArrayValue :public RuntimeValue {
    std::vector<uptr<RuntimeValue> > cells;

  public:
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
  };

  class PointerValoue :public RuntimeValue {
    uptr<RuntimeValue> ptee;

  public:
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
  };

  class CallInfo {
    TracingMap<std::string, uptr<MetaValue> > argsLayout;
    TracingMap<std::string, uptr<RuntimeValue> > argsBeforeCall;
    TracingMap<std::string, uptr<RuntimeValue> > argsAfterCall;

    uptr<MetaValue> retLayout;
    uptr<RuntimeValue> retValue;

    std::string funName;

  public:

    void traceArgument(std::string name, uptr<MetaValue> layout,
                       ref<Expr> value, const ExecutionState &state);
    void setReturnLayout(uptr<MetaValue> layout);


    void traceFunOutput(ref<Expr> retValue);
  };

} // End klee namespace

#endif //KLEE_TRACING_H

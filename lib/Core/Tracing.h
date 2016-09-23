//===-- Context.h -----------------------------------------------*- C++ -*-===//
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

#include "klee/Expr.h"
#include "klee/ExecutionState.h"
#include "klee/util/GetExprSymbols.h"

namespace klee {

  template<class K, class V>
  using TracingMap = std::map<K,V>;

  template<class T>
  using uptr = std::unique_ptr<T>;

  template<class T>
  using wptr = std::weak_ptr<T>;

  class Value;
  class MetaValue;

  /// MetaValue -- holds the memory layout for one element in the hierarchy.
  class MetaValue {

  protected:
    Expr::Width width;

  public:
    virtual ~MetaValue();

    virtual uptr<Value> ReadValue(uint64_t addr, ExecutionState *state) const;
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
    virtual uptr<Value> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaArray -- holds the memory layout for an array
  class MetaArray :public MetaValue {
    uptr<MetaValue> cell;
    uint64_t len;

  public:
    virtual uptr<Value> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaPointer -- holds the memory layout for a pointer
  class MetaPointer :public MetaValue {
    uptr<MetaValue> ptee;

  public:
    virtual uptr<Value> ReadValue(uint64_t addr, ExecutionState *state) const;
  };

  /// MetaPlainVal -- holds the memory layout for a plain value.
  class MetaPlainVal :public MetaValue {
    bool is_signed;

  public:
    virtual uptr<Value> ReadValue(uint64_t addr, ExecutionState *state) const;
  };


  class RuntimeValue {
  protected:
    wptr<MetaValue> layout;
  public:
    virtual ~RuntimeValue();
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class Value& other) const;
    virtual SymbolSet&& getSymbols() const;
  };

  // StructureValue -- holds runtime value of a structure object.
  class StructureValue :public RuntimeValue {
    TracingMap<unsigned, uptr<Value> > fields;

  public:
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class Value& other) const;
  };

  /// ArrayValue -- holds runtime value for an array;
  class ArrayValue :public RuntimeValue {
    std::vector<uptr<Value> > cells;

  public:
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class Value& other) const;
  };

  class PointerValoue :public RuntimeValue {
    uptr<RuntimeValue> ptee;

  public:
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class Value& other) const;
  };

  // Value -- holds runtime value for a memory element;
  class Value {
    MetaValue* layout;

    ref<Expr> plain;
    uint64_t pointer;
    std::string funName;
    ArrayValue arr;
    StructureValue str;
    Value* ptee;

  public:
    bool dumpToStream(llvm::raw_ostream& file);
    bool eq(const class Value& other) const;
  };
} // End klee namespace

#endif //KLEE_TRACING

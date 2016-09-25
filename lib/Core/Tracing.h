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
#include <list>

#include "TracingDefs.h"
#include "klee/Expr.h"
#include "klee/ExecutionState.h"
#include "klee/util/GetExprSymbols.h"

namespace klee {

  class RuntimeValue;
  class MetaValue;
  class InterpreterHandler;

  /// MetaValue -- holds the memory layout for one element in the hierarchy.
  class MetaValue {
  public:
    MetaValue() = default;
    MetaValue(const MetaValue&) = default;
    virtual ~MetaValue() = default;

    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const = 0;
    virtual uptr<RuntimeValue> readValueByPtr(size_t addr,
                                              ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const = 0;
  };

  /// MetaStructure -- holds the memory layout for a structure
  class MetaStructure :public MetaValue {
    struct MetaStructureField {
      std::string name;
      size_t offset;
      uptr<MetaValue> layout;
    };
    TracingMap<size_t, uptr<MetaStructureField> > fields;

  public:
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual uptr<RuntimeValue> readValueByPtr(size_t addr,
                                              ExecutionState *state) const = 0;
    virtual Expr::Width calculateWidth() const;
  };

  /// MetaArray -- holds the memory layout for an array
  class MetaArray :public MetaValue {
    uptr<MetaValue> cell;
    size_t len;

  public:
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;
  };

  /// MetaPointer -- holds the memory layout for a pointer
  class MetaPointer :public MetaValue {
    uptr<MetaValue> ptee;

  public:
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;
  };

  /// MetaPlainVal -- holds the memory layout for a plain value.
  class MetaPlainVal :public MetaValue {
    Expr::Width width;
    bool is_signed;

  public:
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;
  };


  class RuntimeValue {
  protected:
    const MetaValue *layout;
  public:
    RuntimeValue(const MetaValue* layout_):layout(layout_) {}
    virtual ~RuntimeValue() {};
    virtual bool dumpToStream(llvm::raw_ostream& file) = 0;
    virtual bool eq(const class RuntimeValue& other) const = 0;
    virtual SymbolSet collectSymbols() const = 0;
  };

  // StructureValue -- holds runtime value of a structure object.
  class StructureValue :public RuntimeValue {
    TracingMap<size_t, uptr<RuntimeValue> > fields;

  public:
    StructureValue(const MetaStructure* layout_):RuntimeValue(layout_) {}

    void addField(size_t offset, uptr<RuntimeValue> value);

    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
  };

  /// ArrayValue -- holds runtime value for an array;
  class ArrayValue :public RuntimeValue {
    std::vector<uptr<RuntimeValue> > cells;

  public:
    ArrayValue(const MetaArray *layout_,
               size_t length): RuntimeValue(layout_),
                               cells(length){}

    void setCell(size_t index, uptr<RuntimeValue> value);

    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
  };

  class PointerValue :public RuntimeValue {
    uptr<RuntimeValue> ptee;

  public:
    PointerValue(const MetaPointer *layout_):RuntimeValue(layout_) {}

    void setPtee(uptr<RuntimeValue> ptee);

    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
  };

  class PlainValue :public RuntimeValue {
    ref<Expr> val;

  public:
    PlainValue(const MetaPlainVal *layout_,
               ref<Expr> val_):RuntimeValue(layout_), val(val_) {}
    virtual bool dumpToStream(llvm::raw_ostream& file);
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
  };

  class CallInfo {
  public:
    CallInfo() = default;
    CallInfo(const CallInfo& ci);

    void traceArgument(std::string name, uptr<MetaValue> layout,
                       ref<Expr> val, const ExecutionState &state);
    void setReturnLayout(uptr<MetaValue> layout);
    void traceExtraPointer(std::string name, uptr<MetaValue> layout,
                           size_t addr, const ExecutionState &state);

    void traceFunOutput(ref<Expr> retValue, const ExecutionState &state);

    bool eq(const CallInfo& other) const;
    bool sameInvocation(const CallInfo& other) const;
  private:
    struct StructuredArg {
      uptr<MetaValue> layout;
      ref<Expr> value;
    };
    struct StructuredPtr {
      uptr<MetaValue> layout;
      size_t addr;
      std::string name;
    };
    TracingMap<std::string, StructuredArg > argsLayout;
    TracingMap<std::string, uptr<RuntimeValue> > argsBeforeCall;
    TracingMap<std::string, uptr<RuntimeValue> > argsAfterCall;

    std::vector<StructuredPtr> extraPtrs;
    std::vector<StructuredPtr> extraPtrsBeforeCall;
    std::vector<StructuredPtr> extraPtrsAfterCall;


    uptr<MetaValue> retLayout;
    uptr<RuntimeValue> retValue;

    std::vector< ref<Expr> > callContext;
    std::vector< ref<Expr> > returnContext;

    std::string funName;

    bool returned;
  };

  class FileOpener {
  public:
    virtual llvm::raw_ostream *openAnotherFile() = 0;
  };

  class CallTreeNode {
  public:
    CallTreeNode(const std::vector<CallInfo>& initialPath);
    void addCallPath(std::vector<CallInfo>::const_iterator begin,
                     std::vector<CallInfo>::const_iterator end,
                     int id);
    void dumpCallPrefixes(std::list<CallInfo> accumulatedPrefix,
                          FileOpener* fileOpener);
  private:
    class PathTip {
      int pathId;
      CallInfo tipCall;
    };
    PathTip tip;
    std::vector< uptr<CallTreeNode> > children;
  };

  class CallTree {
  public:
    void addCallPath(const std::vector<CallInfo> &path);
    void dumpCallPrefixes(InterpreterHandler* handler);

  private:
    int lastId;
    uptr<CallTreeNode> root;

    class PrefixFileOpener :public FileOpener {
    public:
      PrefixFileOpener(InterpreterHandler* handler);

      virtual llvm::raw_ostream *openAnotherFile();
    };
  };

} // End klee namespace

#endif //KLEE_TRACING_H

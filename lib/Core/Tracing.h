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
#include <set>
#include <llvm/Support/Casting.h>

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
    enum MetaValueKind {
      MVK_STRUCT,
      MVK_ARRAY,
      MVK_POINTER,
      MVK_PLAIN
    };
    MetaValue(MetaValueKind kind_) :kind(kind_) {}
    MetaValue(const MetaValue&) = default;
    virtual ~MetaValue() = default;

    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const = 0;
    virtual uptr<RuntimeValue> readValueByPtr(size_t addr,
                                              ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const = 0;
    MetaValueKind getKind() const { return kind; }

  private:
    MetaValueKind kind;
  };

  /// MetaStructure -- holds the memory layout for a structure
  class MetaStructure :public MetaValue {
    struct MetaStructureField {
      std::string name;
      size_t offset;
      MetaValue *layout;

      MetaStructureField(const std::string &name_,
                         size_t offset_,
                         MetaValue *layout_);
    };
    TracingMap<size_t, MetaStructureField> fields;

  public:
    MetaStructure() :MetaValue(MVK_STRUCT) {}
    MetaStructure(const MetaStructure&);

    void addField(size_t offset, const std::string &name,
                  MetaValue *layout);
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual uptr<RuntimeValue> readValueByPtr(size_t addr,
                                              ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;
    static bool classof(const MetaValue *mv)
    { return mv->getKind() == MVK_STRUCT; }

    bool getFieldName(size_t offset,
                      const std::string **outName) const;
  };

  /// MetaArray -- holds the memory layout for an array
  class MetaArray :public MetaValue {
    MetaValue *cell;
    size_t len;

  public:
    MetaArray(MetaValue *cell_, size_t len_)
      :MetaValue(MVK_ARRAY), cell(cell_), len(len_) {}
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;
    static bool classof(const MetaValue *mv)
    { return mv->getKind() == MVK_ARRAY; }

    size_t getLen() const { return len; }
  };

  /// MetaPointer -- holds the memory layout for a pointer
  class MetaPointer :public MetaValue {
    MetaValue *ptee;

  public:
    MetaPointer(MetaValue *ptee_) :MetaValue(MVK_POINTER), ptee(ptee_) {}

    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;
    static bool classof(const MetaValue *mv)
    { return mv->getKind() == MVK_POINTER; }
  };

  /// MetaPlainVal -- holds the memory layout for a plain value.
  class MetaPlainVal :public MetaValue {
  public:
    enum Kind {
      MPV_SINTEGER,
      MPV_UINTEGER,
      MPV_APATHPTR,
      MPV_FUNPTR
    };
    MetaPlainVal(Expr::Width width_, Kind kind_)
      :MetaValue(MVK_PLAIN), width(width_), kind(kind_) {}
    virtual uptr<RuntimeValue> readValue(ref<Expr> val,
                                         ExecutionState *state) const;
    virtual Expr::Width calculateWidth() const;

    Expr::Width getWidth() const { return width; }
    Kind getValueKind() const { return kind; }
    static bool classof(const MetaValue *mv)
    { return mv->getKind() == MVK_PLAIN; }

  private:
    Expr::Width width;
    Kind kind;
  };


  class RuntimeValue {
  protected:
  public:
    enum RuntimeValueKind {
      RVK_STRUCT,
      RVK_ARRAY,
      RVK_POINTER,
      RVK_PLAIN
    };

    RuntimeValue(RuntimeValueKind k):kind(k) {}
    virtual ~RuntimeValue() {};
    virtual void dumpToStream(llvm::raw_ostream& file) const = 0;
    virtual bool eq(const class RuntimeValue& other) const = 0;
    virtual SymbolSet collectSymbols() const = 0;
    virtual RuntimeValue *makeCopy() const = 0;
    RuntimeValueKind getKind() const { return kind; }
  private:
    RuntimeValueKind kind;
  };

  // StructureValue -- holds runtime value of a structure object.
  class StructureValue :public RuntimeValue {
    const MetaStructure *layout;
    TracingMap<size_t, uptr<RuntimeValue> > fields;

  public:
    StructureValue(const MetaStructure* layout_):RuntimeValue(RVK_STRUCT),
                                                 layout(layout_) {}

    void addField(size_t offset, uptr<RuntimeValue> value);

    virtual void dumpToStream(llvm::raw_ostream& file) const;
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
    virtual RuntimeValue *makeCopy() const;
    static bool classof(const RuntimeValue *rv)
    { return rv->getKind() == RVK_STRUCT; }
  };

  /// ArrayValue -- holds runtime value for an array;
  class ArrayValue :public RuntimeValue {
    const MetaArray *layout;
    std::vector<uptr<RuntimeValue> > cells;

  public:
    ArrayValue(const MetaArray *layout_,
               size_t length): RuntimeValue(RVK_ARRAY),
                               layout(layout_),
                               cells(length){}

    void setCell(size_t index, uptr<RuntimeValue> value);

    virtual void dumpToStream(llvm::raw_ostream& file) const;
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
    virtual RuntimeValue *makeCopy() const;
    static bool classof(const RuntimeValue *rv)
    { return rv->getKind() == RVK_ARRAY; }
  };

  class PointerValue :public RuntimeValue {
    const MetaPointer *layout;
    uptr<RuntimeValue> ptee;
    ref<Expr> value;

  public:
    PointerValue(const MetaPointer *layout_):RuntimeValue(RVK_POINTER),
                                             layout(layout_) {}

    void setPtee(uptr<RuntimeValue> ptee);

    virtual void dumpToStream(llvm::raw_ostream& file) const;
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
    virtual RuntimeValue *makeCopy() const;
    static bool classof(const RuntimeValue *rv)
    { return rv->getKind() == RVK_POINTER; }
  };

  class PlainValue :public RuntimeValue {
    const MetaPlainVal *layout;
    ref<Expr> val;

  public:
    PlainValue(const MetaPlainVal *layout_,
               ref<Expr> val_):RuntimeValue(RVK_PLAIN),
                               layout(layout_),
                               val(val_) {}
    virtual void dumpToStream(llvm::raw_ostream& file) const;
    virtual bool eq(const class RuntimeValue& other) const;
    virtual SymbolSet collectSymbols() const;
    virtual RuntimeValue *makeCopy() const;
    static bool classof(const RuntimeValue *rv)
    { return rv->getKind() == RVK_PLAIN; }
  };

  class CallInfo {
  public:
    CallInfo(std::string funName_) :funName(funName_) {}
    CallInfo(const CallInfo& ci);

    void traceArgument(std::string name, const MetaValue *layout,
                       ref<Expr> val, ExecutionState *state);
    void setReturnLayout(const MetaValue *layout);
    void traceExtraPointer(std::string name, const MetaValue *layout,
                           size_t addr, ExecutionState *state);

    void traceFunOutput(ref<Expr> retValue, ExecutionState *state);

    bool eq(const CallInfo& other) const;
    bool sameInvocation(const CallInfo& other) const;
    void dumpToStream(llvm::raw_ostream& file) const;
  private:
    struct StructuredArg {
      const MetaValue *layout;
      ref<Expr> value;
    };
    struct StructuredPtr {
      const MetaValue *layout;
      size_t addr;
      std::string name;
    };
    TracingMap<std::string, StructuredArg > argsLayout;
    TracingMap<std::string, uptr<RuntimeValue> > argsBeforeCall;
    TracingMap<std::string, uptr<RuntimeValue> > argsAfterCall;

    std::vector<StructuredPtr> extraPtees;
    std::vector<uptr<RuntimeValue> > extraPteesBeforeCall;
    std::vector<uptr<RuntimeValue> > extraPteesAfterCall;


    const MetaValue *retLayout;
    uptr<RuntimeValue> retValue;

    SymbolSet symbolsIn;
    std::set< ref<Expr> > callContext;
    std::set< ref<Expr> > returnContext;

    std::string funName;
    bool returned;

    void addCallConstraintsFor(RuntimeValue* val, ExecutionState* state);
  };

  class FileOpener {
  public:
    virtual llvm::raw_ostream *openAnotherFile() = 0;
  };

  class CallTreeNode {
  public:
    CallTreeNode(int pathId, const CallInfo& tipCall)
      :tip(pathId, tipCall) {}

    void addCallPath(std::vector<CallInfo>::const_iterator begin,
                     std::vector<CallInfo>::const_iterator end,
                     int id);
    void dumpCallPrefixes(std::list<CallInfo> accumulatedPrefix,
                          FileOpener* fileOpener);
  private:
    struct PathTip {
      int pathId;
      CallInfo tipCall;

      PathTip(int pathId_, const CallInfo& tipCall_)
        :pathId(pathId_), tipCall(tipCall_) {}
    };
    PathTip tip;
    std::vector< CallTreeNode > children;

    std::vector<std::vector<CallTreeNode::PathTip*> > groupChildren();
  };

  class CallTree {
  public:
    CallTree() :root(-1, CallInfo("root")) {}
    void addCallPath(const std::vector<CallInfo> &path);
    void dumpCallPrefixes(InterpreterHandler *handler);

  private:
    int lastId;
    CallTreeNode root;

    class PrefixFileOpener :public FileOpener {
    public:
      PrefixFileOpener(InterpreterHandler *handler);

      virtual llvm::raw_ostream *openAnotherFile();
    private:
      size_t numOpenedFiles;
      InterpreterHandler *handler;
    };
  };

} // End klee namespace

#endif //KLEE_TRACING_H

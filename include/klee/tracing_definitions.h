/*===-- tracing_definitions.h -----------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===*/

#ifndef __TRACING_DEFINITIONS_H__
#define __TRACING_DEFINITIONS_H__

#ifdef __cplusplus
extern "C" {
#endif

enum KleeTraceType {
  KLEE_TRACE_FIRST_TYPE = 0,
  KLEE_TRACE_FLOAT = KLEE_TRACE_FIRST_TYPE,
  KLEE_TRACE_DOUBLE = 1,
  KLEE_TRACE_SI16 = 2,
  KLEE_TRACE_SI32 = 3,
  KLEE_TRACE_SI64 = 4,
  KLEE_TRACE_UI16 = 5,
  KLEE_TRACE_UI32 = 6,
  KLEE_TRACE_UI64 = 7,
  KLEE_TRACE_FPTR = 8,
  KLEE_TRACE_APATH_PTR = 9,
  KLEE_TRACE_LAST_TYPE = KLEE_TRACE_APATH_PTR,
};

#ifdef __cplusplus
}
#endif

#endif//__TRACING_DEFINITIONS_H__

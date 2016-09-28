// RUN: %llvmgcc -emit-llvm -g -c %s -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee -dump-call-trace-prefixes --output-dir=%t.klee-out %t.bc > %t.log

#include "klee/klee.h"
#include <stdio.h>

int traced_fun(int x) {
  klee_trace_arg_i32(x, "x", klee_l_plain(KLEE_TRACE_SI32));
  klee_trace_ret(klee_l_plain(KLEE_TRACE_SI32));
  return x;
}

int main() {
  traced_fun(34);
  return 0;
}

// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 3;
  while(klee_induce_invariants() & --x) {
    printf("inloop\n");
    // CHECK: inloop
  }
  printf("afterloop\n");
  // CHECK: afterloop
  return 0;
}

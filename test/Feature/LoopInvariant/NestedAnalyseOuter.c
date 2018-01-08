// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 10;
  int y = klee_int("y");
  int z = 0;

  int i = 20;

  while (klee_induce_invariants() & i--) {
    x = 10;
    while(--x) {
    }
  }

  printf("\n ----- afterloop ---- \n");
  // CHECK: ----- afterloop ----
  return 0;
}

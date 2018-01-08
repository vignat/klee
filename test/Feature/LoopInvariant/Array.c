// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x[3] = {1, 20, 3};
  while(klee_induce_invariants() & x[1]) {
    x[1] -- ;
    if (x[0] < 4) {
      printf("x[0] may be less than 4\n");
      // CHECK: x[0] may be less than 4
    } else {
      printf("x[0] may be more\n");
      // CHECK-NOT: x[0] may be more
    }
    if (x[2] == 3) {
      printf("x[2] may == 3\n");
      // CHECK: x[2] may == 3
    } else {
      printf("x[2] may != 3\n");
      // CHECK-NOT: x[2] may != 3
    }
    if (x[1] < 100) {
      printf("x[1] may be less than 100.\n");
      // CHECK: x[1] may be less than 100.
    } else {
      printf("x[1] may be more\n");
      // CHECK: x[1] may be more
    }
  }
  klee_assert(x[2] == 3);
  printf("afterloop\n");
  // CHECK: afterloop
  return 0;
}

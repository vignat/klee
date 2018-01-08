// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 3;
  int y = 5;
  int z = klee_int("z");
  int w = z;
  while(klee_induce_invariants()) {
    x -- ;
    if (x < 4) {
      printf("x may be less than 4\n");
      // CHECK: x may be less than 4
    } else {
      printf("x may be more\n");
      // CHECK: x may be more
    }
    if (y == 5) {
      printf("y may == 5\n");
      // CHECK: y may == 5
    } else {
      printf("y may != 5\n");
      // CHECK-NOT: y may != 5
    }
  }
  klee_assert(w == z);
  printf("afterloop");
  // CHECK-NOT: afterloop
  return 0;
}

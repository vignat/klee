// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>


int main() {
  int x = 10;
  int y = klee_int("y");
  int z = 0;

  if (5 < y) {
    z = 1;
    printf("taking this one");
    // CHECK: taking this one
  } else {
    z = 2;
    printf("taking OTHER one");
    // CHECK: taking OTHER one
  }

  while(klee_induce_invariants() & x--) {
    if (z == 1) {
      printf("z may be 1\n");
      // CHECK: z may be 1
    } else {
      printf("z may be not 1\n");
      // CHECK: z may be not 1
    }
  }

  if (z == 3) {
    printf("at the end, z may be 3, who knows\n");
    // CHECK-NOT: at the end, z may be 3, who knows
  }

  printf("afterloop\n");
  // CHECK: afterloop
  // CHECK: afterloop
  return 0;
}

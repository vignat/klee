// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out --exit-on-error %t1.bc > %t1.log

// RUN: grep "bigger" %t1.log
// RUN: grep "less" %t1.log
// RUN: grep "eq5" %t1.log
// RUN: grep "noneq5" %t1.log
// RUN: grep "eq10" %t1.log
// RUN: grep "noneq10" %t1.log

#include <klee/klee.h>

#include <stdlib.h>
#include <stdio.h>

int glo = 42;

int main() {
  int x = klee_int("x");
  int y = klee_int("y");
  int z = 10;
  klee_assume(y==5);
  if (3 < x) {
    klee_forget_all();
    if (3 < x) {
      printf("bigger\n");
    } else {
      printf("less\n");
    }
    if (y == 5) {
      printf("eq5\n");
    } else {
      printf("noneq5\n");
    }
    if (z == 10) {
      printf("eq10\n");
    } else {
      printf("noneq10\n");
    }
  }
}

// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc 2>&1 | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int foo(int *x) {
  return *x;
}

int main() {
  int x = 3;
  int y = 2;
  printf("first here\n");
  // CHECK: first here
  klee_forbid_access(&x, sizeof(int), "message");
  printf("y value = %d\n", foo(&y));
  // CHECK: y value = 2
  klee_allow_access(&x, sizeof(int));
  x = 18;
  printf("x value = %d\n", foo(&x));
  // CHECK: x value = 18
  klee_forbid_access(&x, sizeof(int), "message");
  printf("now y value is %d\n", foo(&y));
  // CHECK: now y value is 2
  printf("now x value is %d\n", foo(&x));
  // CHECK-NOT: now x value is 18
  printf("exitting normally\n");
  // CHECK-NOT: exitting normally
  return 0;
}

// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc 2>&1 | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

void forbid(int *x) {
  klee_forbid_access(x, sizeof(int), "message");
}

void allow(int *x) {
  klee_allow_access(x, sizeof(int));
}

int main() {
  int x = 3;
  int y = 2;
  printf("first here\n");
  // CHECK: first here
  forbid(&x);
  forbid(&y);
  allow(&x);
  x = 18;
  printf("x value = %d\n", x);
  // CHECK: x value = 18
  printf("y value = %d\n", y);
  // CHECK-NOT: y value = 2
  printf("exitting normally\n");
  // CHECK-NOT: exitting normally
  return 0;
}

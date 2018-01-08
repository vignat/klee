// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc 2>&1 | FileCheck %s
// RUN: [ -e %t.klee-out/test000001.inaccessible.err ]

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 3;
  int y = 2;
  printf("first here\n");
  // CHECK: first here
  klee_forbid_access(&x, sizeof(int), "message");
  y = 2;
  printf("y assigned\n");
  // CHECK: y assigned
  x = 18;
  printf("exitting normally\n");
  // CHECK-NOT: exitting normally
  return 0;
}

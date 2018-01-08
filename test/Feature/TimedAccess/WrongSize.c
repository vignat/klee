// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc 2>&1 | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 3;
  int y = 2;
  printf("first here\n");
  // CHECK: first here
  klee_forbid_access(&x, sizeof(char), "message");
  printf("exitting normally\n");
  // CHECK-NOT: exitting normally
  return 0;
}

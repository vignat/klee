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
  klee_forbid_access(&x, sizeof(int), "message");
  klee_allow_access(&x, sizeof(int));
  x = 18;
  printf("exitting normally\n");
  // CHECK: exitting normally
  return 0;
}

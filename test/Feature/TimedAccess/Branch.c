// RUN: %llvmgcc %s -emit-llvm -g -c -o %t1.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out %t1.bc 2>&1 | FileCheck %s

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 3;
  int y = klee_int("y");
  printf("first here\n");
  // CHECK: first here
  klee_forbid_access(&x, sizeof(int), "message");
  if (y < 3) {
    printf("something unimportant\n");
  } else {
    // VV this is legit, but if branching of an object state is broken,
    //    it might fail.
    klee_allow_access(&x, sizeof(int));
    printf("got to the else branch\n");
    // CHECK: got to the else branch
  }

  printf("successfully returned\n");
  // CHECK: successfully returned
  return 0;
}

#include <klee/klee.h>
#include <stdio.h>

int main() {
  int x = 3;
  while(klee_induce_invariants() & --x)
    {}
  printf("afterloop");
  return 0;
}

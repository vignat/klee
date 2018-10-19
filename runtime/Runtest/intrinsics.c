//===-- intrinsics.c ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

/* Straight C for linking simplicity */

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>

#include "klee/klee.h"

#include "klee/Internal/ADT/KTest.h"

#define MAX_HAVOCED_PLACES (1048576)
#define NUM_LEN (4)

struct havoced_place {
  char* name;
  void* ptr;
  unsigned width;
};

static KTest *testData = 0;
static unsigned testPosition = 0;
struct havoced_place havoced_places[MAX_HAVOCED_PLACES];
unsigned next_havoced_place = 0;


static unsigned char rand_byte(void) {
  unsigned x = rand();
  x ^= x>>16;
  x ^= x>>8;
  return x & 0xFF;
}

static void report_internal_error(const char *msg, ...)
    __attribute__((format(printf, 1, 2)));
static void report_internal_error(const char *msg, ...) {
  fprintf(stderr, "KLEE_RUN_TEST_ERROR: ");
  va_list ap;
  va_start(ap, msg);
  vfprintf(stderr, msg, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  char *testErrorsNonFatal = getenv("KLEE_RUN_TEST_ERRORS_NON_FATAL");
  if (testErrorsNonFatal) {
    fprintf(stderr, "KLEE_RUN_TEST_ERROR: Forcing execution to continue\n");
  } else {
    exit(1);
  }
}

static void init_test_data() {
  assert(!testData);
  char tmp[256];
  char *name = getenv("KTEST_FILE");

  if (!name) {
    fprintf(stdout, "KLEE-RUNTIME: KTEST_FILE not set, please enter .ktest path: ");
    fflush(stdout);
    name = tmp;
    if (!fgets(tmp, sizeof tmp, stdin) || !strlen(tmp)) {
      fprintf(stderr, "KLEE-RUNTIME: cannot replay, no KTEST_FILE or user input\n");
      exit(1);
    }
    tmp[strlen(tmp)-1] = '\0'; /* kill newline */
  }
  testData = kTest_fromFile(name);
  if (!testData) {
    fprintf(stderr, "KLEE-RUNTIME: unable to open .ktest file\n");
    exit(1);
  }
}

void klee_make_symbolic(void *array, size_t nbytes, const char *name) {
  static int rand_init = -1;

  if (rand_init == -1) {
    if (getenv("KLEE_RANDOM")) {
      struct timeval tv;
      gettimeofday(&tv, 0);
      rand_init = 1;
      srand(tv.tv_sec ^ tv.tv_usec);
    } else {
      rand_init = 0;
    }
  }

  if (rand_init) {
    if (!strcmp(name,"syscall_a0")) {
      unsigned long long *v = array;
      assert(nbytes == 8);
      *v = rand() % 69;
    } else {
      char *c = array;
      size_t i;
      for (i=0; i<nbytes; i++)
        c[i] = rand_byte();
    }
    return;
  }

  if (!testData) {
    init_test_data();
  }

  for (;; ++testPosition) {
    if (testPosition >= testData->numObjects) {
      report_internal_error("out of inputs. Will use zero if continuing.");
      memset(array, 0, nbytes);
      break;
    } else {
      KTestObject *o = &testData->objects[testPosition];
      if (strcmp("model_version", o->name) == 0 &&
          strcmp("model_version", name) != 0) {
        // Skip over this KTestObject because we've hit
        // `model_version` which is from the POSIX runtime
        // and the caller didn't ask for it.
        continue;
      }
      if (strcmp(name, o->name) != 0) {
        report_internal_error(
            "object name mismatch. Requesting \"%s\" but returning \"%s\"\n",
            name, o->name);
      }
      memcpy(array, o->bytes, nbytes < o->numBytes ? nbytes : o->numBytes);
      if (nbytes != o->numBytes) {
        report_internal_error("object sizes differ. Expected %zu but got %u",
                              nbytes, o->numBytes);
        if (o->numBytes < nbytes)
          memset((char *)array + o->numBytes, 0, nbytes - o->numBytes);
      }
      ++testPosition;
      break;
    }
  }
}

unsigned count_reuse(char* name) {
  unsigned i;
  int reuse_count = 0;
  for (i = 0; i < next_havoced_place; ++i) {
    unsigned name_len = strlen(name);
    unsigned candidate_len = strlen(havoced_places[i].name);
    // Compare only the prefix, skip the _### suffixes.
    if (name_len <= candidate_len &&
        0 == strncmp(name, havoced_places[i].name, name_len)) {
      unsigned j = name_len;
      int num_suffix = 1;
      int non_numerical = 0;
      if (name_len < candidate_len &&
          havoced_places[i].name[name_len] != '_') {
        continue;
      }
      for (j = name_len + 1; j < candidate_len; ++j) {
        if (havoced_places[i].name[j] < '0' || '9' < havoced_places[i].name[j]) {
          non_numerical = 1;
          break;
        }
      }
      if (non_numerical) continue;
      ++reuse_count;
    }
  }
  return reuse_count;
}

char* allocate_unique_name(char* orig, unsigned reuse_count) {
  int size = strlen(orig) + 1 + NUM_LEN + 1;
  char* unique_name = malloc(size);
  assert(unique_name);
  snprintf(unique_name, size, "%s_%d", orig, reuse_count);
  return unique_name;
}

void klee_possibly_havoc(void* ptr, int width, char* name) {
  assert(next_havoced_place < MAX_HAVOCED_PLACES);
  unsigned reuse_count = count_reuse(name);
  if (0 < reuse_count) {
    char* unique_name = allocate_unique_name(name, reuse_count);
    havoced_places[next_havoced_place].name = unique_name;
    //printf("%s reused: %d times, giving %s\n", name, reuse_count, unique_name);
  } else {
    havoced_places[next_havoced_place].name = name;
    //printf("%s not reused yet\n", name);
  }
  havoced_places[next_havoced_place].ptr = ptr;
  havoced_places[next_havoced_place].width = width;
  ++next_havoced_place;
  assert(next_havoced_place < MAX_HAVOCED_PLACES);
}

int klee_induce_invariants() {
  unsigned i, j, byte;

  //TODO: support partial havoc (only selected bytes of an array)
  if (!testData) {
    init_test_data();
  }

  for (i = 0; i < testData->numHavocs; ++i) {
    char* name = testData->havocs[i].name;
    int found = 0;
    //printf("name: %s; ", name);
    //fflush(stdout);
    for (j = 0; j < next_havoced_place; ++j) {
      if (strcmp(name, havoced_places[j].name) == 0) {
        assert(!found);
        found = 1;
        //printf("%d - %d\n", testData->havocs[i].numBytes, havoced_places[j].width);
        assert(testData->havocs[i].numBytes == havoced_places[j].width);
        for (byte = 0; byte < testData->havocs[i].numBytes; ++byte) {
          uint32_t byte_word = byte/32;
          uint32_t selector = 1 << (byte - byte_word*32);
          if (testData->havocs[i].mask[byte_word] & selector) {
            ((uint8_t*)(havoced_places[j].ptr))[byte] = testData->havocs[i].bytes[byte];
          }
        }
      }
    }
    assert(found);
  }

  return 1;
}


void klee_silent_exit(int x) {
  exit(x);
}

uintptr_t klee_choose(uintptr_t n) {
  uintptr_t x;
  klee_make_symbolic(&x, sizeof x, "klee_choose");
  if(x >= n)
    report_internal_error("klee_choose failure. max = %ld, got = %ld\n", n, x);
  return x;
}

void klee_assume(uintptr_t x) {
  if (!x) {
    report_internal_error("invalid klee_assume");
  }
}

#define KLEE_GET_VALUE_STUB(suffix, type)	\
	type klee_get_value##suffix(type x) { \
		return x; \
	}

KLEE_GET_VALUE_STUB(f, float)
KLEE_GET_VALUE_STUB(d, double)
KLEE_GET_VALUE_STUB(l, long)
KLEE_GET_VALUE_STUB(ll, long long)
KLEE_GET_VALUE_STUB(_i32, int32_t)
KLEE_GET_VALUE_STUB(_i64, int64_t)

#undef KLEE_GET_VALUE_STUB

int klee_range(int begin, int end, const char* name) {
  int x;
  klee_make_symbolic(&x, sizeof x, name);
  if (x<begin || x>=end) {
    report_internal_error("invalid klee_range(%u,%u,%s) value, got: %u\n",
                          begin, end, name, x);
  }
  return x;
}

void klee_prefer_cex(void *object, uintptr_t condition) { }

void klee_abort() {
  exit(-1);
}

int klee_int(const char *name) {
  int x;
  klee_make_symbolic(&x, sizeof x, name);
  return x;
}

/* not sure we should even define.  is for debugging. */
void klee_print_expr(const char *msg, ...) { }

void klee_set_forking(unsigned enable) { }

//Vigor: just ignore them

#define KLEE_TRACE_PARAM_PROTO(suffix, type) \
  void klee_trace_param##suffix(type param, const char* name) {}
  KLEE_TRACE_PARAM_PROTO(f, float);
  KLEE_TRACE_PARAM_PROTO(d, double);
  KLEE_TRACE_PARAM_PROTO(l, long);
  KLEE_TRACE_PARAM_PROTO(ll, long long);
  KLEE_TRACE_PARAM_PROTO(_u16, uint16_t);
  KLEE_TRACE_PARAM_PROTO(_i32, int32_t);
  KLEE_TRACE_PARAM_PROTO(_u32, uint32_t);
  KLEE_TRACE_PARAM_PROTO(_i64, int64_t);
  KLEE_TRACE_PARAM_PROTO(_u64, uint64_t);
#undef KLEE_TRACE_PARAM_PROTO
void klee_trace_param_ptr(void* ptr, int width, const char* name) {}
  void klee_trace_param_ptr_directed(void* ptr, int width,
                                     const char* name,
                                     TracingDirection td){}
  void klee_trace_param_tagged_ptr(void* ptr, int width,
                                   const char* name, const char* type,
                                   TracingDirection td){}
  void klee_trace_param_just_ptr(void* ptr, int width, const char* name){}
  void klee_trace_param_fptr(void* ptr, const char* name){}
  void klee_trace_ret(){}
  void klee_trace_ret_ptr(int width){}
  void klee_trace_ret_just_ptr(int width){}

  void klee_trace_param_ptr_field(void* ptr, int offset, int width, const char* name){}
  void klee_trace_param_ptr_field_directed(void* ptr, int offset,
                                           int width, const char* name,
                                           TracingDirection td){}
  void klee_trace_param_ptr_field_just_ptr(void* ptr, int offset,
                                           int width, const char* name){}
  void klee_trace_ret_ptr_field(int offset, int width, const char* name){}
  void klee_trace_ret_ptr_field_just_ptr(int offset, int width, const char* name){}
  void klee_trace_param_ptr_nested_field(void* ptr, int base_offset,
                                         int offset, int width, const char* name){}
  void klee_trace_param_ptr_nested_field_directed(void* ptr, int base_offset,
                                                  int offset, int width, const char* name,
                                                  TracingDirection td){}
  void klee_trace_ret_ptr_nested_field(int base_offset,
                                       int offset, int width, const char* name){}
  void klee_trace_extra_ptr(void* ptr, int width, const char* name, const char* type, TracingDirection td){}
  void klee_trace_extra_ptr_field(void* ptr, int offset, int width, const char* name, TracingDirection td){}
  void klee_trace_extra_ptr_field_just_ptr(void* ptr, int offset,
                                           int width, const char* name){}
  void klee_trace_extra_ptr_nested_field(void* ptr, int base_offset,
                                         int offset, int width, const char* name, TracingDirection td){}
  void klee_trace_extra_ptr_nested_nested_field(void* ptr, int base_base_offset,
                                                int base_offset, int offset,
                                                int width, const char* name, TracingDirection td){}

  void klee_forget_all(){}


  void klee_forbid_access(void* ptr, int width, char* message){}
  void klee_allow_access(void* ptr, int width){}

  void klee_dump_constraints(){}

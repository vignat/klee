#!/bin/bash

set -euo pipefail

KLEE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[ -d "$KLEE_DIR/build" ] || mkdir "$KLEE_DIR/build"

cd "$KLEE_DIR/build"

[ -f "Makefile" ] || CXXFLAGS="-D_GLIBCXX_USE_CXX11_ABI=0" \
                     CMAKE_PREFIX_PATH="$KLEE_DIR/../z3/build" \
                     CMAKE_INCLUDE_PATH="$KLEE_DIR/../z3/build/include/" \
                     cmake \
                         -DENABLE_UNIT_TESTS=OFF \
                         -DBUILD_SHARED_LIBS=OFF \
                         -DLLVM_CONFIG_BINARY="$KLEE_DIR/../llvm/Release/bin/llvm-config" \
                         -DLLVMCC="$KLEE_DIR/../llvm/Release/bin/clang" \
                         -DLLVMCXX="$KLEE_DIR/../llvm/Release/bin/clang++" \
                         -DENABLE_SOLVER_Z3=ON \
                         -DENABLE_KLEE_UCLIBC=ON \
                         -DKLEE_UCLIBC_PATH="$KLEE_DIR/../klee-uclibc" \
                         -DENABLE_POSIX_RUNTIME=ON \
                         -DCMAKE_BUILD_TYPE=Debug \
                         -DENABLE_KLEE_ASSERTS=ON \
                         -DENABLE_DOXYGEN=ON \
                         ..


make -kj $(nproc)

#!/usr/bin/env bash
# Build and run the host-side tests. Pure C++ (no Pico SDK)
#   ./test/run.sh        (or  CXX=clang++ ./test/run.sh)

cd "$(dirname "$0")/.." || exit 2          # -> Hexapod-Firmware/
CXX=${CXX:-g++}
FLAGS="-std=c++17 -Wall -Wextra -O1 -Isrc -Itest"
out=$(mktemp -d)
status=0

for src in test/test_*.cpp; do
    name=$(basename "$src" .cpp)
    echo "=== $name ==="
    if ! $CXX $FLAGS "$src" -o "$out/$name"; then
        echo "BUILD FAILED: $name"; status=1; continue
    fi
    "$out/$name" || status=1
    echo
done

[ $status -eq 0 ] && echo "ALL HOST TESTS PASSED" || echo "HOST TESTS FAILED"
exit $status

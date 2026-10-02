#!/bin/bash
#
# Run one Catch2 test suite with a tag filter, from the macOS arm64 build tree
# or, with TESTS_DIR set, from any other build tree (the Windows workflow sets
# TESTS_DIR=build/tests).
#
# Usage: .github/run-suite.sh <suite> '<catch2 filter>'
#   suite   one of libslic3r, slic3rutils, fff_print
#   filter  a Catch2 tag expression, e.g. '[AppConfig],[Config]'
#
# The generator decides whether the binary sits under a config subdirectory, and
# whether it is a bare executable or inside a .app, so the binary is located
# rather than assumed.

set -euo pipefail

SUITE="${1:?usage: run-suite.sh <suite> <filter>}"
FILTER="${2:?usage: run-suite.sh <suite> <filter>}"
ARCH="${ARCH:-arm64}"

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

SUITE_DIR="${TESTS_DIR:-build/${ARCH}/tests}/${SUITE}"
if [ ! -d "$SUITE_DIR" ]; then
    echo "ERROR: no build output at $SUITE_DIR. Did the build step run?" >&2
    exit 1
fi

BINARY="$(find "$SUITE_DIR" -type f \( -name "${SUITE}_tests" -perm -u+x -o -name "${SUITE}_tests.exe" \) -print | head -n 1)"
if [ -z "$BINARY" ]; then
    echo "ERROR: could not find ${SUITE}_tests under $SUITE_DIR" >&2
    find "$SUITE_DIR" -maxdepth 3 -print >&2
    exit 1
fi

mkdir -p test-output
LOG="test-output/${SUITE}.log"

echo "Suite:  ${SUITE}"
echo "Binary: ${BINARY}"
echo "Filter: ${FILTER}"

set +e
"$BINARY" "$FILTER" --reporter compact 2>&1 | tee "$LOG"
status="${PIPESTATUS[0]}"
set -e

echo "Exit: ${status}"
exit "$status"

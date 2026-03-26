#!/usr/bin/env bash
set -euo pipefail

# ── Defaults ────────────────────────────────────────────────────────────────
BUILD_DIST=ON
SHARED_LIBS=ON
USE_GLM=OFF
BUILD_DIR=""
RUN_TESTS=OFF
PRESET=""

MANUAL_FLAGS=false

# ── Helpers ──────────────────────────────────────────────────────────────────
log()  { echo "▶  $*"; }
step() {
  local bar
  bar=$(printf '═%.0s' {1..44})
  echo
  echo "$bar"
  echo "   $*"
  echo "$bar"
}
pause() {
  echo
  read -r -p "Press Enter to close..."
}

# ── Usage ────────────────────────────────────────────────────────────────────
usage() {
  cat <<USAGE
Usage: $(basename "$0") [OPTIONS]

Options:
  --preset NAME     Use a CMake preset (skips manual config flags)
  --no-dist         Disable FIZZIKS_BUILD_DIST      (default: ON)
  --no-shared       Disable BUILD_SHARED_LIBS       (default: ON)
  --use-glm         Enable FIZZIKS_USE_GLM          (default: OFF)
  --run-tests       Build and run tests             (default: OFF)
  --build-dir DIR   Set build directory             (default: build | preset name)
  -h, --help        Show this help message

Examples:
  $(basename "$0")                                 # manual, all configs
  $(basename "$0") --no-shared --use-glm           # manual, static + GLM
  $(basename "$0") --preset x64-shared-eigen-debug # single preset config
USAGE
  pause
  exit 0
}

# ── Trap ─────────────────────────────────────────────────────────────────────
trap 'echo; echo "✖  Script failed (line $LINENO)"; pause' ERR

# ── Argument parsing ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --preset)      PRESET="$2";    shift ;;
    --build-dir)   BUILD_DIR="$2"; shift ;;
    --no-dist)     BUILD_DIST=OFF   MANUAL_FLAGS=true;;
    --no-shared)   SHARED_LIBS=OFF; MANUAL_FLAGS=true;;
    --use-glm)     USE_GLM=ON       MANUAL_FLAGS=true;;
    --run-tests)   RUN_TESTS=ON;    MANUAL_FLAGS=true;;
    -h|--help)     usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
  shift
done

# warn if manual-mode flags were passed alongside a preset
if [[ -n "$PRESET" && "$MANUAL_FLAGS" == true ]]; then
  echo "⚠  --no-dist, --no-shared, --use-glm, --run-tests are ignored when --preset is used" >&2
fi

# ── Test runner ───────────────────────────────────────────────────────────────
run_tests() {
  local build_dir="$1"
  local config="${2:-Debug}"

  # Find the test executable — check both flat and config-subdirectory layouts
  local exe=""
  for candidate in \
      "$build_dir/bin/$config/FizziksTests.exe" \
      "$build_dir/bin/FizziksTests.exe" \
      "$build_dir/tests/$config/FizziksTests.exe" \
      "$build_dir/tests/FizziksTests.exe"
  do
    if [[ -f "$candidate" ]]; then
      exe="$candidate"
      break
    fi
  done

  if [[ -z "$exe" ]]; then
    echo "⚠  Could not find FizziksTests.exe under $build_dir — skipping tests." >&2
    return 0
  fi

  step "Test · $config"
  log "Running $exe"
  echo

  # Run GTest once with colour output and XML output simultaneously.
  # --gtest_catch_exceptions=1 ensures crashes are caught as test failures
  # rather than popping a Windows error dialog that blocks the terminal.
  # timeout kills the process if it somehow hangs beyond 120 seconds.
  local xml_out="$build_dir/test_results.xml"
  local gtest_exit=0
  "$exe" \
    --gtest_color=yes \
    --gtest_output="xml:$xml_out" \
    --gtest_catch_exceptions=1 \
    --gtest_break_on_failure=0 \
    & local gtest_pid=$!

  # Kill the process if it runs too long
  ( sleep 120 && kill "$gtest_pid" 2>/dev/null ) & local watchdog_pid=$!

  wait "$gtest_pid" || gtest_exit=$?
  kill "$watchdog_pid" 2>/dev/null || true
  wait "$watchdog_pid" 2>/dev/null || true

  if [[ $gtest_exit -eq 143 || $gtest_exit -eq 137 ]]; then
    echo "✖  Test run timed out after 120 seconds — possible hang or crash."
    return 1
  fi

  echo
  if [[ $gtest_exit -eq 0 ]]; then
    echo "✔  All tests passed."
  else
    echo "✖  Some tests FAILED (exit code $gtest_exit)."
    echo

    # Parse the XML for failed test names using grep (no extra deps required)
    if [[ -f "$xml_out" ]]; then
      echo "── Failed tests ────────────────────────────────"
      grep 'failure' "$xml_out" -B5 \
        | grep 'testcase name=' \
        | sed 's/.*testcase name="\([^"]*\)".*/  ✖  \1/' \
        | sort -u
      echo "────────────────────────────────────────────────"
    fi

    # Propagate failure so the script exit code reflects the test result
    return $gtest_exit
  fi
}

# ── Preset mode ──────────────────────────────────────────────────────────────
if [[ -n "$PRESET" ]]; then
  [[ -z "$BUILD_DIR" ]] && BUILD_DIR="build/$PRESET"

  step "Configure · $PRESET"
  cmake --preset "$PRESET" -B "$BUILD_DIR"

  step "Build · $PRESET"
  cmake --build "$BUILD_DIR"

  if [[ "$PRESET" == *-tests ]]; then
    run_tests "$BUILD_DIR" "Debug"
  fi

  step "Complete"
  pause
  exit 0
fi

# ── Manual mode ──────────────────────────────────────────────────────────────
[[ -z "$BUILD_DIR" ]] && BUILD_DIR="build"

step "Configure"
log "FIZZIKS_BUILD_DIST  = $BUILD_DIST"
log "BUILD_SHARED_LIBS   = $SHARED_LIBS"
log "FIZZIKS_USE_GLM     = $USE_GLM"
log "FIZZIKS_BUILD_TESTS = $RUN_TESTS"
log "Build directory     = $BUILD_DIR"

mkdir -p "$BUILD_DIR"

cmake -S . -B "$BUILD_DIR" \
  -DFIZZIKS_BUILD_DIST="$BUILD_DIST" \
  -DBUILD_SHARED_LIBS="$SHARED_LIBS" \
  -DFIZZIKS_BUILD_TESTS="$RUN_TESTS" \
  -DFIZZIKS_USE_GLM="$USE_GLM"

for config in Debug Release RelWithDebInfo MinSizeRel; do
  step "Build · $config"
  cmake --build "$BUILD_DIR" --config "$config" --target Fizziks
  if [[ "$RUN_TESTS" == ON ]]; then
    run_tests "$BUILD_DIR" "Debug"
  fi
done

step "Copy dist"
if [[ -d "$BUILD_DIR/dist" ]]; then
  log "Copying $BUILD_DIR/dist → ./dist"
  rm -rf ./dist
  cp -r "$BUILD_DIR/dist" ./dist
  log "Done."
else
  echo "⚠  $BUILD_DIR/dist not found — skipping copy." >&2
fi

step "Complete"
pause
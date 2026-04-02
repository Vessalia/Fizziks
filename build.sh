#!/usr/bin/env bash
set -euo pipefail

# ── Defaults ────────────────────────────────────────────────────────────────
BUILD_DIST=ON
SHARED_LIBS=ON
USE_GLM=OFF
BUILD_DIR=""
ASSETS_DIR=""
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
  --assets-dir DIR  Set assets directory            (default: assets)
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
    --preset)      PRESET="$2";     shift ;;
    --build-dir)   BUILD_DIR="$2";  shift ;;
	--assets-dir)  ASSETS_DIR="$2"; shift ;;
    --no-dist)     BUILD_DIST=OFF;  MANUAL_FLAGS=true;;
    --no-shared)   SHARED_LIBS=OFF; MANUAL_FLAGS=true;;
    --use-glm)     USE_GLM=ON;      MANUAL_FLAGS=true;;
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

  step "Test · $config"
  ctest --test-dir "$build_dir" -C "$config" \
    --output-on-failure \
    --timeout 120
}

# ── Preset mode ──────────────────────────────────────────────────────────────
if [[ -n "$PRESET" ]]; then
  [[ -z "$BUILD_DIR" ]] && BUILD_DIR="build/$PRESET"
  [[ -z "$ASSETS_DIR" ]] && ASSETS_DIR="assets"

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
[[ -z "$ASSETS_DIR" ]] && ASSETS_DIR="assets"

step "Configure"
log "FIZZIKS_BUILD_DIST  = $BUILD_DIST"
log "BUILD_SHARED_LIBS   = $SHARED_LIBS"
log "FIZZIKS_USE_GLM     = $USE_GLM"
log "FIZZIKS_BUILD_TESTS = $RUN_TESTS"
log "Build directory     = $BUILD_DIR"
log "Assets directory    = $ASSETS_DIR"

mkdir -p "$BUILD_DIR"

cmake -S . -B "$BUILD_DIR" \
  -DFIZZIKS_BUILD_DIST="$BUILD_DIST" \
  -DBUILD_SHARED_LIBS="$SHARED_LIBS" \
  -DFIZZIKS_BUILD_TESTS="$RUN_TESTS" \
  -DFIZZIKS_USE_GLM="$USE_GLM"

for config in Debug Release RelWithDebInfo MinSizeRel; do
  step "Build · $config"
  cmake --build "$BUILD_DIR" --config "$config"
done


if [[ "$RUN_TESTS" == ON ]]; then
  run_tests "$BUILD_DIR" "Debug"
fi

step "Copy dist"
if [[ -d "$BUILD_DIR/dist" ]]; then
  log "Copying $BUILD_DIR/dist → ./dist"
  rm -rf ./dist
  cp -r "$BUILD_DIR/dist" ./dist
  log "Done."
else
  echo "⚠  $BUILD_DIR/dist not found — skipping copy." >&2
fi

step "Copy default CMakeLists.txt"
if [[ -d "$BUILD_DIR/dist" && -f "$ASSETS_DIR/CMakeLists.txt" ]]; then
  log "Copying $ASSETS_DIR/CMakeLists.txt → ./dist/Fizziks"
  cp "$ASSETS_DIR/CMakeLists.txt" ./dist/Fizziks
  log "Done."
else
  echo "⚠  $BUILD_DIR/dist, or $ASSET_DIR/CMakeLists.txt not found — skipping copy." >&2
fi

step "Complete"
pause
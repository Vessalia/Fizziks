#!/usr/bin/env bash
set -euo pipefail

# ── Defaults ────────────────────────────────────────────────────────────────
BUILD_DIST=ON
SHARED_LIBS=ON
USE_GLM=OFF
BUILD_DIR=""
PRESET=""

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
  --no-shared       Disable BUILD_SHARED_LIBS        (default: ON)
  --use-glm         Enable FIZZIKS_USE_GLM            (default: OFF)
  --build-dir DIR   Set build directory              (default: build | preset name)
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
    --preset)     PRESET="$2";    shift ;;
    --no-dist)    BUILD_DIST=OFF        ;;
    --no-shared)  SHARED_LIBS=OFF       ;;
    --use-glm)    USE_GLM=ON            ;;
    --build-dir)  BUILD_DIR="$2"; shift ;;
    -h|--help)    usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
  shift
done

# ── Preset mode ──────────────────────────────────────────────────────────────
if [[ -n "$PRESET" ]]; then
  # build dir defaults to preset name if not explicitly set
  [[ -z "$BUILD_DIR" ]] && BUILD_DIR="build/$PRESET"

  step "Configure · $PRESET"
  cmake --preset "$PRESET" -B "$BUILD_DIR"

  step "Build · $PRESET"
  cmake --build "$BUILD_DIR"

  step "Complete"
  pause
  exit 0
fi

# ── Manual mode ──────────────────────────────────────────────────────────────
[[ -z "$BUILD_DIR" ]] && BUILD_DIR="build"

step "Configure"
log "FIZZIKS_BUILD_DIST = $BUILD_DIST"
log "BUILD_SHARED_LIBS  = $SHARED_LIBS"
log "FIZZIKS_USE_GLM    = $USE_GLM"
log "Build directory    = $BUILD_DIR"

mkdir -p "$BUILD_DIR"

cmake -S . -B "$BUILD_DIR" \
  -DFIZZIKS_BUILD_DIST="$BUILD_DIST" \
  -DBUILD_SHARED_LIBS="$SHARED_LIBS" \
  -DFIZZIKS_USE_GLM="$USE_GLM"

for config in Debug Release RelWithDebInfo MinSizeRel; do
  step "Build · $config"
  cmake --build "$BUILD_DIR" --config "$config"
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
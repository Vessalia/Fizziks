#!/usr/bin/env bash
set -euo pipefail
 
# ── Defaults ────────────────────────────────────────────────────────────────
BUILD_DIST=ON
SHARED_LIBS=ON
BUILD_DIR="build"
 
# ── Usage ────────────────────────────────────────────────────────────────────
usage() {
  cat <<USAGE
Usage: $(basename "$0") [OPTIONS]
 
Options:
  --no-dist         Disable FIZZIKS_BUILD_DIST      (default: ON)
  --no-shared       Disable BUILD_SHARED_LIBS        (default: ON)
  --build-dir DIR   Set build directory              (default: build)
  -h, --help        Show this help message
USAGE
  read -r -p "Press Enter to close..."
  exit 0
}
 
# ── Argument parsing ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-dist)    BUILD_DIST=OFF  ;;
    --no-shared)  SHARED_LIBS=OFF ;;
    --build-dir)  BUILD_DIR="$2"; shift ;;
    -h|--help)    usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
  shift
done
 
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
 
# ── Build ────────────────────────────────────────────────────────────────────
step "Configure"
log "FIZZIKS_BUILD_DIST = $BUILD_DIST"
log "BUILD_SHARED_LIBS  = $SHARED_LIBS"
log "Build directory    = $BUILD_DIR"
 
mkdir -p "$BUILD_DIR"
 
cmake -S . -B "$BUILD_DIR" \
  -DFIZZIKS_BUILD_DIST="$BUILD_DIST" \
  -DBUILD_SHARED_LIBS="$SHARED_LIBS"
 
for config in Debug Release RelWithDebInfo MinSizeRel; do
  step "Build · $config"
  cmake --build "$BUILD_DIR" --config "$config"
done
 
step "Copy dist"
if [[ -d "$BUILD_DIR/dist" ]]; then
  log "Copying $BUILD_DIR/dist → ./dist"
  cp -r "$BUILD_DIR/dist" ./dist
  log "Done."
else
  echo "⚠  $BUILD_DIR/dist not found — skipping copy." >&2
fi
#!/usr/bin/env bash
# Build the Doxygen documentation for the mapping package.
# Works inside the Docker container (Dockerfile.doxygen) and locally.
set -euo pipefail

DOCS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SOURCE_DIR:-$(cd "$DOCS_DIR/.." && pwd)}"
BUILD_DIR="${BUILD_DIR:-/tmp/docs-build}"

cmake -S "$SOURCE_DIR/docs" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR"

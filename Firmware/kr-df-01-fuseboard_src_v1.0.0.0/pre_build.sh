#!/bin/sh
#
# This script gathers build data (date, time, Git commit ID, etc.)
# and writes them as macros into build_info.h.
#
# Usage example:
#   sh pre_build.sh
#   (This will overwrite the existing build_info.h file.)

# Optionally adapt FW_VERSION or pass it in via environment variable if needed.
FW_VERSION="${FW_VERSION:-1.0.0.0}"
BUILD_DATE="$(date +%Y-%m-%d)"
BUILD_TIME="$(date +%H:%M:%S)"

# Retrieves the short commit hash from Git.
# If this script is not run inside a Git repository, you may need to provide the commit ID another way.
GIT_COMMIT_ID="$(git rev-parse --short HEAD 2>/dev/null || echo "UNKNOWN")"

# Write macros to build_info.h
cat <<EOF > Core/Inc/build_info.h
#pragma once

#define FW_VERSION    "${FW_VERSION}"
#define BUILD_DATE    "${BUILD_DATE}"
#define BUILD_TIME    "${BUILD_TIME}"
#define GIT_COMMIT_ID "${GIT_COMMIT_ID}"

EOF

echo "Generated build_info.h with:"
echo "  FW_VERSION    = ${FW_VERSION}"
echo "  BUILD_DATE    = ${BUILD_DATE}"
echo "  BUILD_TIME    = ${BUILD_TIME}"
echo "  GIT_COMMIT_ID = ${GIT_COMMIT_ID}"

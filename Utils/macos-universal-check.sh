#!/bin/bash

# Check if the user provided a directory
if [ -z "$1" ]; then
  echo "Usage: $0 <directory>"
  exit 1
fi

ROOT_DIR="$1"

# Recursively find all .dylib and .a files
find "$ROOT_DIR" -type f \( -name "*.dylib" -o -name "*.a" \) | while read -r lib; do
  # Get architectures in the binary
  ARCHS=$(lipo -archs "$lib" 2>/dev/null)

  # Skip files that aren't valid Mach-O binaries
  if [ $? -ne 0 ]; then
    continue
  fi

  # Check if both x86_64 and arm64 are present
  echo "$ARCHS" | grep -q "x86_64" && HAS_X86_64=1 || HAS_X86_64=0
  echo "$ARCHS" | grep -q "arm64" && HAS_ARM64=1 || HAS_ARM64=0

  if [ "$HAS_X86_64" -eq 0 ] || [ "$HAS_ARM64" -eq 0 ]; then
    echo -e "\n\n\033[0;31m❌\033[0m $lib only has archs $ARCHS!\n\n"
  else
    echo -e  "\033[0;32m✅\033[0m $lib is universal."
  fi
done

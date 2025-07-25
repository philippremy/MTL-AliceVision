#!/bin/bash

DIR="$1"
if [[ -z "$DIR" ]]; then
  echo "Usage: $0 <directory>"
  exit 2
fi

for file in "$DIR"/*; do
  if [[ -x "$file" && ! -d "$file" ]]; then
    echo "Checking executable: $file"
    "$file" > /dev/null 2>&1
    code=$?
    if [[ $code -ne 1 && $code -ne 0 ]]; then
      echo "Executable '$file' exited with code $code"
    fi
  fi
done

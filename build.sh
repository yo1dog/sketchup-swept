#!/usr/bin/env bash
# Build vehicle_swept_path.rbz — an .rbz is just a ZIP of the loader + folder.
set -euo pipefail
cd "$(dirname "$0")"

OUT="vehicle_swept_path.rbz"
rm -f "$OUT"

# Zip the loader and the extension folder at the archive root.
zip -r "$OUT" swept_path.rb swept_path \
  -x '*/.*' -x '*__MACOSX*' >/dev/null

echo "Built $OUT"
unzip -l "$OUT"

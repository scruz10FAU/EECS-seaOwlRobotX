#!/bin/bash
set -euo pipefail

ROOT_DIR="/opt/sysroots"

## GPT generated list of core libs we probably should leave alone. This may end
## up being a bad idea, for example librt.so.2 needs a librt.so symlink for
## ceres to solve but used to be on this list.
EXCLUDE_REGEX="lib(c|m|pthread|dl|gcc_s|resolv|crypt|nsl|nss|nss_|selinux|anl)\.so"

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
fi

echo "Scanning sysroots in $ROOT_DIR ..."
echo

find "$ROOT_DIR"  \( -name "lib*.so.*.*.*" -o -name "lib*.so.*" \) | while read -r fullpath; do
    dir=$(dirname "$fullpath")
    base=$(basename "$fullpath")


    # Extract base name up to ".so"
    libbase="${base%%.so*}"
    linkname="${base%%.so.*}.so"

    # Exclude core system libraries
    if [[ "$linkname" =~ $EXCLUDE_REGEX ]]; then
        echo "regex excluding $fullpath"
        continue
    fi

    # Only create the symlink if it does not already exist
    if [[ ! -e "$dir/$linkname" && ! -L "$dir/$linkname" ]]; then
        if $DRY_RUN; then
            echo "[DRY RUN] Would create: $dir/$linkname -> $fullpath"
        else
            echo "Creating: $dir/$linkname -> $fullpath"
            ln -s "$fullpath" "$dir/$linkname"
        fi
    fi
done

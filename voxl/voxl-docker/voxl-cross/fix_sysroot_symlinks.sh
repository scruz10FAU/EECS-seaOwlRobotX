#!/bin/bash
set -e

echo "Fixing broken symlinks in sysroots..."


SYSROOTS_BASE="/opt/sysroots"

# Resolve symlink chain under sysroot base, handling relative and absolute links
resolve_symlink_chain() {
    local base="$1"    # e.g. /opt/sysroots/qrb5165_2
    local path="$2"    # target from base path like usr/include/foo.h

    local current="$base$path"
    local seen=()

    # Prevent infinite loops by tracking visited links
    while [ -L "$current" ]; do
        # Detect loops
        for s in "${seen[@]}"; do
            if [ "$s" == "$current" ]; then
                echo "Symlink loop detected at $current" >&2
                break 2
            fi
        done
        seen+=("$current")

        local target
        target=$(readlink "$current")

        if [[ "$target" == /* ]]; then
            # If the symlink target is absolute, treat it as 
            # relative to the base directory (e.g., base + /usr/include/foo.h)
            current="$base$target"
        else
            # If it's relative, resolve it relative to the current symlink's directory
            current="$(dirname "$current")/$target"
        fi

        # Normalize path (collapse .. and .)
        current=$(realpath -m "$current")
    done

    echo "$current"
}

for SYSROOT in "$SYSROOTS_BASE"/*; do
    [ -d "$SYSROOT" ] || continue  # Skip if not a directory

    echo "Scanning sysroot: $SYSROOT"

    find "$SYSROOT" -type l | while read -r link; do
        target=$(readlink "$link")
        if [[ "$target" == /* ]]; then
            # Resolve the full symlink chain.
            #
            # This recursively follows symlinks to ensure we reach the actual file 
            # being pointed to, regardless of how many intermediate symlinks exist.
            # This guarantees that we always point to the real file, even if the 
            # target is a chain of links or discovered in a different order.
            resolved_target=$(resolve_symlink_chain "$SYSROOT" "$target")

            if [ -e "$resolved_target" ]; then
                link_dir=$(dirname "$link")
                rel_target=$(realpath --relative-to="$link_dir" "$resolved_target")

                echo "Rewriting $link -> $rel_target"
                rm -f "$link"
                ln -s "$rel_target" "$link"
            else
                echo "Warning: target $resolved_target does not exist. Skipping $link"
            fi
        fi
    done

    echo "Finished: $SYSROOT"
    echo
done


# this is necessary since qrb5165_1 does contain a spare lib32 directory with
# core libraries, but not the symlinks to common *.so names, so gcc doesn't
# recognize them. Add those in here to fix that.
echo "manually adding some qrb5165_1 32-bit symlinks"
PATH32="/opt/sysroots/qrb5165_1/usr/lib32"
ln -s $PATH32/libc.so.6       $PATH32/libc.so
ln -s $PATH32/libgcc_s.so.1   $PATH32/libgcc_s.so
ln -s $PATH32/libm.so.6       $PATH32/libm.so
ln -s $PATH32/libpthread.so.0 $PATH32/libpthread.so

## QVIO needs:
 # 0x00000001 (NEEDED)                     Shared library: [libc.so.6]
 # 0x00000001 (NEEDED)                     Shared library: [libgcc_s.so.1]
 # 0x00000001 (NEEDED)                     Shared library: [libm.so.6]
 # 0x00000001 (NEEDED)                     Shared library: [libmv1.so]
 # 0x00000001 (NEEDED)                     Shared library: [libpthread.so.0]
 # 0x00000001 (NEEDED)                     Shared library: [ld-linux.so.3]


echo "✅ Done fixing sysroot symlinks."

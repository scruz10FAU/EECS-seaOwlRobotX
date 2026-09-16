#!/bin/bash

# Inline function definition
replace_in_file() {
	if [[ $# -ne 3 ]]; then
	echo "Usage: replace_in_file <search_string> <replacement_string> <file_path>"
	return 1
	fi

	local search="$1"
	local replace="$2"
	local file="$3"

	if [[ ! -f "$file" ]]; then
	echo "Error: File '$file' does not exist."
	return 1
	fi

	# Escape slashes and ampersands
	local escaped_search
	local escaped_replace
	escaped_search=$(printf '%s\n' "$search" | sed 's/[&/\]/\\&/g')
	escaped_replace=$(printf '%s\n' "$replace" | sed 's/[&/\]/\\&/g')

	sed -i "s/${escaped_search}/${escaped_replace}/g" "$file"
}

# fix the google benchmark cmake file which hard-codes librt path and can therefore not cross compile
replace_in_file \
	"/usr/lib/aarch64-linux-gnu/librt.so" \
	"librt.so" \
	"/opt/sysroots/qrb5165_2/usr/lib/aarch64-linux-gnu/cmake/benchmark/benchmarkTargets.cmake"


# remove libfc_sensor from qrb5165-1 sysroot. It's old, should be removed from the sysroot
# and is replaced during installation of voxl-suite on target. Remove it here to
# prevent conflicts or accidentally using the old lib.
rm -rf /opt/sysroots/qrb5165_1/usr/include/fc_sensor.h
rm -rf /opt/sysroots/qrb5165_1/usr/lib/libfc_sensor.so

# qrb5165-2 is missing the fastcv header, just copy it over from qrb5165-1
# this may be added to the qrb5165-2 sysroot sometime in the future so only copy if missing
if ! [ -f /opt/sysroots/qrb5165_2/usr/include/fastcv/fastcv.h ]; then
	mkdir -p /opt/sysroots/qrb5165_2/usr/include/fastcv/
	cp /opt/sysroots/qrb5165_1/usr/include/fastcv/fastcv.h /opt/sysroots/qrb5165_2/usr/include/fastcv/fastcv.h
fi

#!/bin/bash


# Define target path and URL
TARGET_FILE="/tmp/check-linter-script.sh"
URL="https://gitlab.com/voxl-public/voxl-sdk/ci-tools/-/raw/master/scripts/check-linter-script.sh"


print_usage(){
	echo ""
	echo " Runs the ModalAI voxl-cross project linter in the current working directory"
	echo ""
	echo " Usage:"
	echo ""
	echo "  voxl-cross(4.3):~/libs/libmodal-json(dev)(0.4.7)$ voxl-cross-linter"
	echo ""
	echo ""
	echo ""
}


# Check for help argument
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    print_usage
    exit 0
fi

# Check if the file already exists
if [ -f "$TARGET_FILE" ]; then
    echo "File already exists: $TARGET_FILE"
else
    echo "Downloading script..."
    curl -L -o "$TARGET_FILE" "$URL"

    # Check if download succeeded
    if [ $? -eq 0 ]; then
        echo "Downloaded successfully to $TARGET_FILE"
    else
        echo "Download failed" >&2
        echo "gitlab may be down, please try again later"
        exit 1
    fi
fi

## execute!
bash ${TARGET_FILE}


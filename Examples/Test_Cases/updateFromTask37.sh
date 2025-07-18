#!/bin/bash

# Change the directory to the script's location
cd "$(dirname "$0")"

# Create a temporary directory for cloning
temp_dir=$(mktemp -d)
echo "Created temporary directory: $temp_dir"

# Clone the repository
git clone -b develop https://github.com/IEAWindTask37/IEA-15-240-RWT "$temp_dir"

# Check if clone was successful
if [ $? -ne 0 ]; then
    echo "Failed to clone repository"
    rm -rf "$temp_dir"
    exit 1
fi

# Create IEA-15-240-RWT directory if it doesn't exist
mkdir -p IEA-15-240-RWT

# Copy OpenFAST directory contents
cp -r "$temp_dir/OpenFAST/"* IEA-15-240-RWT/

# Clean up temporary directory
rm -rf "$temp_dir"

echo "Files copied successfully to IEA-15-240-RWT directory"
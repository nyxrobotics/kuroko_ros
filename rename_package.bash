#!/bin/bash

# Check for arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <old_name> <new_name>"
    exit 1
fi

OLD_NAME="$1"
NEW_NAME="$2"

# 1. Rename directory base names (from deepest level first)
find . -type d -not -path "*/.git/*" -not -path "*/.vscode/*" \
    | awk -F/ '{print NF, $0}' | sort -nr | cut -d" " -f2- \
    | while IFS= read -r dir; do
    base_name=$(basename "$dir")
    parent_dir=$(dirname "$dir")
    new_base_name=$(echo "$base_name" | sed "s/${OLD_NAME}/${NEW_NAME}/g")
    new_dir="${parent_dir}/${new_base_name}"
    if [ "$dir" != "$new_dir" ]; then
        echo "Renaming directory: $dir -> $new_dir"
        mv "$dir" "$new_dir"
    fi
done

# 2. Rename file names
find . -type f -not -path "*/.git/*" -not -path "*/.vscode/*" \
    | grep "${OLD_NAME}" \
    | while IFS= read -r file; do
    new_name=$(echo "$file" | sed "s/${OLD_NAME}/${NEW_NAME}/g")
    if [ "$file" != "$new_name" ]; then
        echo "Renaming file: $file -> $new_name"
        mv "$file" "$new_name"
    fi
done

# 3. Replace contents of files
find . -type f -not -path "*/.git/*" -not -path "*/.vscode/*" \
    -exec grep -l "${OLD_NAME}" {} \; \
    | while IFS= read -r file; do
    echo "Updating file content: $file"
    sed -i "s/${OLD_NAME}/${NEW_NAME}/g" "$file"
done

# Convert to different naming conventions
UPPER_OLD_NAME=$(echo "$OLD_NAME" | tr 'a-z' 'A-Z')
UPPER_NEW_NAME=$(echo "$NEW_NAME" | tr 'a-z' 'A-Z')
CAMEL_OLD_NAME=$(echo "$OLD_NAME" | sed -E 's/(^|_)([a-z])/\U\2/g')
CAMEL_NEW_NAME=$(echo "$NEW_NAME" | sed -E 's/(^|_)([a-z])/\U\2/g')
PASCAL_OLD_NAME=$(echo "$CAMEL_OLD_NAME" | sed -E 's/^([a-z])/\U\1/')
PASCAL_NEW_NAME=$(echo "$CAMEL_NEW_NAME" | sed -E 's/^([a-z])/\U\1/')

# 4. Replace additional naming conventions in file contents
find . -type f -not -path "*/.git/*" -not -path "*/.vscode/*" \
    | while IFS= read -r file; do
    echo "Updating file content for different naming conventions: $file"
    sed -i "s/${UPPER_OLD_NAME}/${UPPER_NEW_NAME}/g" "$file"
    sed -i "s/${CAMEL_OLD_NAME}/${CAMEL_NEW_NAME}/g" "$file"
    sed -i "s/${PASCAL_OLD_NAME}/${PASCAL_NEW_NAME}/g" "$file"
done

echo "Replacement complete: ${OLD_NAME} -> ${NEW_NAME}"

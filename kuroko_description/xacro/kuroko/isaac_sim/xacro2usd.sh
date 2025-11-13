#!/bin/bash
set -euo pipefail

# Get current script directory
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE:-$0}")"; pwd)
# Get Kuroko description directory (3 levels up; adjust if needed)
KUROKO_DESCRIPTION_DIR=$(cd "$(dirname "${BASH_SOURCE:-$0}")"; cd ../../..; pwd)
# Set export directory
EXPORT_DIR="${SCRIPT_DIR}/export"

# Ensure export directory exists
mkdir -p "${EXPORT_DIR}"

echo ""
echo "================================="
echo " Starting Kuroko Model Conversion"
echo "================================="

# Display parameter values
echo ">>> Checking paths..."
echo "SCRIPT_DIR           : ${SCRIPT_DIR}"
echo "KUROKO_DESCRIPTION_DIR: ${KUROKO_DESCRIPTION_DIR}"
echo "EXPORT_DIR           : ${EXPORT_DIR}"
echo "---------------------------------"

# Copy base xacro
echo ">>> Copying base xacro file..."
cp "${KUROKO_DESCRIPTION_DIR}/xacro/kuroko/kuroko.xacro" "${EXPORT_DIR}/kuroko_abs.xacro"
echo "Copied: ${KUROKO_DESCRIPTION_DIR}/xacro/kuroko/kuroko.xacro -> ${EXPORT_DIR}/kuroko_abs.xacro"

# Replace "$(find kuroko_description)/xacro" -> "${KUROKO_DESCRIPTION_DIR}/xacro"
echo ">>> Replacing path references in xacro (xacro path)..."
sed -i -e "s|\$(find kuroko_description)/xacro|${KUROKO_DESCRIPTION_DIR}/xacro|g" "${EXPORT_DIR}/kuroko_abs.xacro"

# Replace "package://kuroko_description/meshes" -> "${KUROKO_DESCRIPTION_DIR}/meshes"
echo ">>> Replacing path references in xacro (meshes path)..."
sed -i -e "s|package://kuroko_description/meshes|${KUROKO_DESCRIPTION_DIR}/meshes|g" "${EXPORT_DIR}/kuroko_abs.xacro"

# Convert xacro to urdf
echo ">>> Converting xacro -> urdf ..."
xacro "${EXPORT_DIR}/kuroko_abs.xacro" > "${EXPORT_DIR}/kuroko.urdf"
echo "Generated: ${EXPORT_DIR}/kuroko.urdf"

# Convert urdf to sdf
echo ">>> Converting urdf -> sdf ..."
gz sdf -p "${EXPORT_DIR}/kuroko.urdf" > "${EXPORT_DIR}/kuroko.sdf"
echo "Generated: ${EXPORT_DIR}/kuroko.sdf"

# Convert sdf to usd
echo ">>> Converting sdf -> usd ..."
sdf2usd "${EXPORT_DIR}/kuroko.sdf" "${EXPORT_DIR}/kuroko.usd"
echo "Generated: ${EXPORT_DIR}/kuroko.usd"

# Convert usd to usda (ASCII)
echo ">>> Converting usd -> usda (ASCII format)..."
usdcat "${EXPORT_DIR}/kuroko.usd" > "${EXPORT_DIR}/kuroko.usda"
echo "Generated: ${EXPORT_DIR}/kuroko.usda"

echo ""
echo "✅ Conversion completed successfully!"
echo "All exported files are available in:"
echo "    ${EXPORT_DIR}"
echo "================================="

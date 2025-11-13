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

# Display parameter values
echo "===== Parameter Information ====="
echo "SCRIPT_DIR: ${SCRIPT_DIR}"
echo "KUROKO_DESCRIPTION_DIR: ${KUROKO_DESCRIPTION_DIR}"
echo "EXPORT_DIR: ${EXPORT_DIR}"
echo "================================="

# Copy base xacro
cp "${KUROKO_DESCRIPTION_DIR}/xacro/kuroko/kuroko.xacro" "${EXPORT_DIR}/kuroko_abs.xacro"

# Replace "$(find kuroko_description)/xacro" -> "${KUROKO_DESCRIPTION_DIR}/xacro"
sed -i -e "s|\$(find kuroko_description)/xacro|${KUROKO_DESCRIPTION_DIR}/xacro|g" "${EXPORT_DIR}/kuroko_abs.xacro"

# Replace "package://kuroko_description/meshes" -> "${KUROKO_DESCRIPTION_DIR}/meshes"
sed -i -e "s|package://kuroko_description/meshes|${KUROKO_DESCRIPTION_DIR}/meshes|g" "${EXPORT_DIR}/kuroko_abs.xacro"

# Convert xacro to urdf
xacro "${EXPORT_DIR}/kuroko_abs.xacro" > "${EXPORT_DIR}/kuroko.urdf"

# Convert urdf to sdf
gz sdf -p "${EXPORT_DIR}/kuroko.urdf" > "${EXPORT_DIR}/kuroko.sdf"

# Convert sdf to usd
sdf2usd "${EXPORT_DIR}/kuroko.sdf" "${EXPORT_DIR}/kuroko.usd"

# Convert usd to usda (ASCII)
usdcat "${EXPORT_DIR}/kuroko.usd" > "${EXPORT_DIR}/kuroko.usda"

echo "Done. Files written to: ${EXPORT_DIR}"

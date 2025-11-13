#!/bin/bash

# Get current script directory
SCRIPT_DIR=$(cd $(dirname ${BASH_SOURCE:-$0}); pwd)
# Get Kuroko description directory (4 levels up)
KUROKO_DESCRIPTION_DIR=$(cd $(dirname ${BASH_SOURCE:-$0}); cd ../../..; pwd)
EXPORT_DIR=${SCRIPT_DIR}/export

# Display parameter values
echo "===== Parameter Information ====="
echo "SCRIPT_DIR: ${SCRIPT_DIR}"
echo "KUROKO_DESCRIPTION_DIR: ${KUROKO_DESCRIPTION_DIR}"
echo "EXPORT_DIR: ${EXPORT_DIR}"
echo "================================="

cp ${KUROKO_DESCRIPTION_DIR}/xacro/kuroko/kuroko.xacro ${EXPORT_DIR}/kuroko_abs.xacro
# Replace "$(find kuroko_description)/xacro" -> "${KUROKO_DESCRIPTION_DIR}/xacro"
# TODO:
# Replace "package://kuroko_description/meshes" -> "${KUROKO_DESCRIPTION_DIR}/meshes"
# TODO:
# Convert xacro to urdf
xacro ${EXPORT_DIR}/kuroko_abs.xacro > ${EXPORT_DIR}/kuroko.urdf
# Convert urdf to sdf
gz sdf -p ${EXPORT_DIR}/kuroko.urdf > ${EXPORT_DIR}/kuroko.sdf
# Convert sdf to usd
sdf2usd export/kuroko.sdf export/kuroko.usd
# Convert usd to usda
# TODO:

#!/bin/bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

XACRO_PATH=${SCRIPT_DIR}/model.sdf.xacro
MODEL_PATH=${SCRIPT_DIR}/models/gimbal_small_2d/model.sdf
MESHES_PATH=/gimbal_small_2d/meshes

xacro ${XACRO_PATH} \
    ros_namespace:=${GIMBAL_NAMESPACE} \
    camera_name:=${CAMERA_NAME:-camera} \
    camera_height:=${CAMERA_HEIGHT:-480} \
    camera_width:=${CAMERA_WIDTH:-640} \
    camera_rotation:=${CAMERA_ROTATION:-3.14159} \
    gimbal_initial_pitch:=${GIMBAL_INITIAL_PITCH:-0.0} \
    gimbal_initial_yaw:=${GIMBAL_INITIAL_YAW:-0.0} \
    gimbal_meshes_directory:=${MESHES_PATH} \
    -o ${MODEL_PATH}

echo "Written gimbal sdf to: ${MODEL_PATH}"

export GAZEBO_MODEL_PATH="${SCRIPT_DIR}/models:${GAZEBO_MODEL_PATH}"

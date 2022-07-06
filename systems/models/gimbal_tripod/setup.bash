#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

XACRO_PATH=${SCRIPT_DIR}/model.sdf.xacro
MODEL_PATH=${SCRIPT_DIR}/models/gimbal_tripod/model.sdf

xacro ${XACRO_PATH} \
    ros_namespace:=${GIMBAL_NAMESPACE} \
    height:=${GIMBAL_HEIGHT:-1.5} \
    -o ${MODEL_PATH}

echo "Written Gimbal model to ${MODEL_PATH}"

export GAZEBO_MODEL_PATH="${SCRIPT_DIR}/models:${GAZEBO_MODEL_PATH}"
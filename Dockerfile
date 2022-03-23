ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-iris:latest

# Copy in the xacro & bash file to setup the model
# Using ros.env.d automatic sourcing on entrypoint (see /simulator/base/core/Dockerfile)
COPY flightarena /ros.env.d/02_flightarena
COPY systems/models/gimbal_small_2d /ros.env.d/03_gimbal_small_2d
COPY systems/models/gimbal_tripod /ros.env.d/04_gimbal_tripod

# Build gimbal plugin with ROS2 on path and add plugin to path
COPY systems/gimbal_plugin /ros_ws/src/gimbal_plugin
RUN cd /ros_ws \
    && . /opt/ros/foxy/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --cmake-force-configure \
    && rm -r build \
    && echo 'export GAZEBO_PLUGIN_PATH=/ros_ws/install/gimbal_plugin/lib:${GAZEBO_PLUGIN_PATH}' >> /ros.env.d/03_gimbal_small_2d/setup.bash

# Copy in the vehicle launch file
COPY system.launch.xml /ros_ws/launch/

WORKDIR /ros_ws

ENV GIMBAL_NAMESPACE gimbal_1

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/system.launch.xml" ]
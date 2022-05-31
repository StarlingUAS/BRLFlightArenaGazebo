SRC_PATH="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Add flightarena.world to the Gazebo resource path
export GAZEBO_RESOURCE_PATH=$SRC_PATH/worlds:${GAZEBO_RESOURCE_PATH}
echo "RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"

# Add the 'grass_box' model to the Gazebo model path
export GAZEBO_MODEL_PATH=$SRC_PATH/models:${GAZEBO_MODEL_PATH}
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"

# Make the default media files work
cp -r /usr/share/gazebo-11/media $GZWEB_WS/http/client/assets/
(cd /root/gzweb/http/client/assets/media/materials/textures \
    && for f in *jpg; do convert $f ${f%.*}.png; done)

# Enable motion planner
if [[ -f "/ros_ws/install/setup.bash" ]]; then
    source "/ros_ws/install/setup.bash"
fi
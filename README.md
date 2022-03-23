# Flight Arena Gazebo in Starling

This repository contains a simple gazebo model of the flight arena within a docker-container for use with the Starling eco-system. This example currently ships with a PX4 based SITL by default.

![flightarena](flightarena.png)

## Shape

Flight arena is 15.6m x 11.8m x 5m (x, y, z) tall.

```
_________________________________
|                                |
|                                |
|                                |
|_                              L|
|C|
|  |                             |
|__|____________                 |
|___|_V_|___|___|                |
|                     _____ _____|
|_    _____    ______|__E__|_____|
```


The coordinate space is x positive is up and y postivie is left (w.r.t the ascii figure).

The flight arena origin is offset by (0.5, 0.7, 0.0) from the center of the space. This coincides with the vicon center

## Usage

Build and run the associated world and element into a container by running the following
```sh
make run

# just running `make` will build but not run it
```

The Gazebo web interface is then available on [localhost:8080](http://localhost:8080).

On Linux, any MAVLink compatible GCS can be connected to UDP 14550. Many GCS will do this automatically.

On Windows, any MAVLink compatible GCS can be connected to TCP 5761.

If you wish to run with additional drone, then you can use the associated docker-compose file

```sh
make
docker-compose up
```

The docker-compose file also includes the UI which you can run to double check the image output of the gimbal. Accessible via `localhost:3000`

To check the ROS network, you can either `docker exec -it <containerhash> bash` into the container, or you can run `make run_bash` which will put you in a shell on the same network. Then to access the ros2 topics:
```
source /opt/ros/foxy/setup.bash
ros2 topic list
```

### Troubleshooting

The simulator needs to download some model files when it is first run so it will likely fail to spawn the vehicle.
Leave it running for a few minutes then use `Ctrl+C` to exit and run it again. This should allow the model to spawn.
The first time the model attempts to spawn, the simulator will need to download files for it too so it may be slow to
start. Subsequent runs should be much faster.

When running the docker-compose, it will create a docker network called `BRLFightlArenaGazebo_default` which any additional calls to `docker run` can use.

## Files

### Flight Arena world file and models

The BRL world file is in the `flightarena` folder. The elements within the flight arena are all hard coded boxes.

> Note: need to find a way to apply texture

### 3D Gimbal on tripod

The gimbal is split into 3 parts.

1. The gimbal model which is located in `systems/models/gimbal_small_2d`
2. The gimbal plugin i.e. the gimbal controller in `systems/gimbal_plugin`
3. The gimbal tripod with camera which is located in `systems/models/gimbal_tripod`

Any changes made to any folder can be rebuilt when run `make`

## Developing your own ROS2 controller

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:

```
# Download the latest container
docker pull uobflightlabstarling/example_controller_python

docker run -it --rm --network fenswoodscenario_default uobflightlabstarling/example_controller_python
```

See [the docs](https://docs.starlinguas.dev/guide/single-drone-local-machine/#2-running-example-ros2-offboard-controller-node) for further details

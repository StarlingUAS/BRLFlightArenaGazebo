# Flight Arena Gazebo in Starling

This repository contains a simple gazebo model of the flight arena within a docker-container for use with the Starling eco-system. This example currently ships with a PX4 based SITL by default.

![flightarena](flightarena.png)

## Usage

First, ensure all the required images are downloaded and up-to-date:

```sh
docker-compose -f docker-compose.linux.yml pull
# or on windows:
docker-compose -f docker-compose.windows.yml pull
```

Then launch the scenario with:

```sh
docker-compose -f docker-compose.linux.yml up
# or on windows:
docker-compose -f docker-compose.windows.yml up
```

The Gazebo web interface is then available on [localhost:8080](http://localhost:8080).

On Linux, any MAVLink compatible GCS can be connected to UDP 14550. Many GCS will do this automatically.

On Windows, any MAVLink compatible GCS can be connected to TCP 5761.

### Troubleshooting

The simulator needs to download some model files when it is first run so it will likely fail to spawn the vehicle.
Leave it running for a few minutes then use `Ctrl+C` to exit and run it again. This should allow the model to spawn.
The first time the model attempts to spawn, the simulator will need to download files for it too so it may be slow to
start. Subsequent runs should be much faster.

## Windows and Linux

Each example file has a *linux* and *windows* variant.

- The *linux* variant allows you to run bare-metal application such as rviz2 or your own controllers natively. You do not need to wrap your own controllers in a docker container. Any exposed ports are automatically exposed to `localhost`.
- The *windows* variant runs inside a docker-compose network named `fenswoodscenario_default`. This network is segregated from your local network traffic *except* for the exposed ports in the docker-compose file which are now accessible from `localhost`. Any other ROS2 nodes will need to be wrapped in a docker container for running and run with `--network fenswoodscenario_default`.

## Developing your own ROS2 controller

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:

```
# Download the latest container
docker pull uobflightlabstarling/example_controller_python

docker run -it --rm --network fenswoodscenario_default uobflightlabstarling/example_controller_python
```

See [the docs](https://docs.starlinguas.dev/guide/single-drone-local-machine/#2-running-example-ros2-offboard-controller-node) for further details

ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-iris-ap:latest

WORKDIR /ros.env.d

RUN mkdir fenswood

COPY flightarena flightarena

CMD ["ros2", "launch", "/ros.env.d/flightarena/iris.launch.xml"]
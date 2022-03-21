MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

CONTAINERNAME?=starling-sim-iris-px4-flightarena
NETWORK?=host
ENV?=

all: flightarena

flightarena:
	$(BAKE) $(CONTAINERNAME)

local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) --push $(CONTAINERNAME)

run: flightarena
	docker run -it --rm --net=$(NETWORK) $(ENV) uobflightlabstarling/$(CONTAINERNAME):latest

run_bash: flightarena
	docker run -it --rm --net=$(NETWORK) uobflightlabstarling/$(CONTAINERNAME):latest bash

.PHONY: all flightarena local-build-push run run_bash

IMG_TAG = loam_family:noetic
CONTAINER_NAME = sp_a_loam
VOLUME_LOCAL = /home/bhbhchoi/docker_ws
VOLUME_REMOTE = /root/docker_ws

all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""


build:
	@docker build -t $(IMG_TAG) .

run:
	@docker run --gpus all -it -v /tmp/.X11-unix:/tmp/.X11-unix -v $(VOLUME_LOCAL):$(VOLUME_REMOTE) -e DISPLAY=unix$DISPLAY -e="QT_X11_NO_MITSHM=1" --name $(CONTAINER_NAME) $(IMG_TAG)

clean:
	@docker rmi -f $(IMG_TAG)

remove:
	@docker rm -f $(CONTAINER_NAME)
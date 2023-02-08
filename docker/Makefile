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
	@docker build --tag loam_family:noetic -f ./Dockerfile ..

run:
	@docker run --gpus all -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/bhbhchoi/docker_ws:/root/docker_ws -e DISPLAY=unix$DISPLAY -e="QT_X11_NO_MITSHM=1" -p 8080:8080 -p 8022:22 --name sp_a_loam ros:aloam

clean:
	@docker rmi -f ros:aloam

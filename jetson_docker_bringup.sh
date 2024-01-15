xhost +local:
docker run --runtime nvidia -it --rm --network host --volume ./:/home/scout_ros2_ws --volume /tmp/argus_socket:/tmp/argus_socket --volume /etc/enctune.conf:/etc/enctune.conf --volume /etc/nv_tegra_release:/etc/nv_tegra_release --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model --device /dev/snd --device /dev/bus/usb --device /dev/video0:/dev/video0:mwr -e DISPLAY=:1 -v /tmp/.X11-unix/:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth -e XAUTHORITY=/tmp/.docker.xauth -w /home/scout_ros2_ws takumanakao/ros:humble-desktop-l4t-r35.3.1
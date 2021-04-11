xhost +local:root
docker run --rm -it \
    --name="lara" \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev/serial/:/dev/serial/:rw" \
    --volume="$HOME/wr_home/navigation2/warthog-scorpius.navigation/:/root/navigation:rw" \
    gnardari/athome_lara:latest \
    bash

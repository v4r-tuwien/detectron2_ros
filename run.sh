docker run --gpus all \
           --network host \
           --privileged \
           -e QT_X11_NO_MITSHM=1 \
           -e DISPLAY=$DISPLAY \
           -e XAUTHORITY=$XAUTH \
           -v $XAUTH:$XAUTH \
           -v /tmp/.X11-unix/:/tmp/.X11-unix \
           -v $HOME/.Xauthority:/root/.Xauthority \
           -it --name detectron2_ros detectron2_ros /bin/bash

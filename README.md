# detectron2_ros

This code provides ROS wrappers for detectron2 framework.

## Dependencies
1. Install [detectron2](https://github.com/facebookresearch/detectron2). 
   Refer to the install information on their website.
2. Install [object_detector_msgs](https://github.com/v4r-tuwien/object_detector_msgs) into your catkin workspace.
3. Buld ROS packages and do not forget to source your setup.bash file

## Examples
### Docker:
To start the serivice, run 

```
docker-compose up
```

This starts the docker container and runs the service. Make sure the parameters `ROS_IP` and `ROS_MASTER_URI` in the `docker-compose.yml` file are correct. 
Calling the service [`/detectron2_service/start`](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/srv/start.srv) starts the detector. The detections can be read from the topic [`/detectron2_service/detections`](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/msg/Detections.msg). 
Calling [`/detectron2_service/stop`](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/srv/stop.srv) stops the detector. 

### Service:
Detections with Image subscription as input.
```
rosrun detectron2_ros service.py -ct 0.5 \
       -c /detectron2_repo/configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml \
       -o MODEL.WEIGHTS detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl \
       -t /camera/rgb/image_rect_color \
       -v 
```  
**-ct**  confidence threshold of detections

**-c**   config file for detectron2 model

**-o**   options for detectron2 model (see their website for more infos)
 
**-t**   topic to listen for new image messages 

**-v**   publish visualization on topic /detectron2_service/image

Detections result messages will be published on '/detectron2_service/detections'


### ActionServer & ActionClient:
One shot detections using actionlib and image file.
1. Start server
```
rosrun detectron2_ros action_server.py -ct 0.2
```
**-ct**  confidence threshold of detections

**-c**   config file for detectron2 model

**-o**   options for detectron2 model (see their website for more infos))

2. Start client
```
rosrun detectron2_ros action_client.py -i data/ocid.png
```
**-i**  path to input image

You should see visualizations of the detections.

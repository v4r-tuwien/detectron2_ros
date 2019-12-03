#! /usr/bin/env python3

from __future__ import print_function

import argparse
from cv_bridge import CvBridgeError

import rospy
from sensor_msgs.msg import Image

from object_detector_msgs.srv import start, stop, detectron2_service_server, detectron2_service_serverResponse
from object_detector_msgs.msg import Detections
from detectron2_ros.predictor import Detectron2Predictor
from detectron2_ros.detectron2_ros import Detectron2Ros

# Only used to get path to detectron2 package for default values
import os
import detectron2

class Detectron2Service(object):
    def __init__(self, name, args):
        self.name = name
        rospy.loginfo('%s:  Start object detection.', self.name)
        self.running = 0
        self.detector = Detectron2Predictor(args)
        rospy.Service(self.name+'/detect', detectron2_service_server, self.detect)

    def detect(self, image):
        try:
            detectron2ros = Detectron2Ros(self.detector)
            detector_message = detectron2ros.run_on_image(image)
            return detectron2_service_serverResponse(detector_message)
        except CvBridgeError as e:
            rospy.logerr('%s: Error during callback:\%s', self.name, e)


if __name__ == '__main__':
    rospy.init_node('detectron2_service')
    default_config_file = os.path.join(detectron2.__path__[0], "../configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-ct',
        '--confidence-threshold',
        dest='confidence_threshold',
        type=float,
        default=0.5,
        help='Score confidence threshold of detections')
    parser.add_argument(
        '-c',
        '--config-file',
        dest='config_file',
        type=str,
        default=default_config_file,
        help='Path to detectron2 configuration file')
    parser.add_argument(
        '-o',
        '--opts',
        dest='opts',
        type=str,
        nargs='+',
        default=['MODEL.WEIGHTS',
                 'detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl'],
        help='Additional options to detectron model')
    parser.add_argument(
        '-q-size',
        '--queue-size',
        dest='queue_size',
        type=int,
        default=1,
        help='Size of publisher queue')
    parser.add_argument(
        '-t',
        '--camera-topic',
        dest='topic',
        type=str,
        default='/camera/rgb/image_rect_color',
        help='Camera topic to subscribe to')
    parser.add_argument(
        '-v',
        '--visualize',
        dest='visualize',
        default=False,
        action='store_true',
        help='Visualize detection')

    args = parser.parse_args()
    service = Detectron2Service(rospy.get_name(), args)
    rospy.spin()


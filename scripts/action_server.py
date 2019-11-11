#! /usr/bin/env python3

from __future__ import print_function

import argparse
import rospy
import actionlib
from cv_bridge import CvBridgeError
import os
import rospkg
import object_detector_msgs
from detectron2_ros.predictor import Detectron2Predictor
from detectron2_ros.detectron2_ros import Detectron2Ros

# Only used to get path to detectron2 package for default values
import detectron2

class Detectron2Action(object):
    # create messages that are used to publish feedback/result
    _result = object_detector_msgs.msg.CheckForDetectionsResult()

    def __init__(self, name, args):
        self.detector = Detectron2Predictor(args)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, object_detector_msgs.msg.CheckForDetectionsAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing detection.' % (self._action_name))
        try:
            detectron2ros = Detectron2Ros(self.detector)
            self._result.detections = detectron2ros.run_on_image(goal.image)
            self._as.set_succeeded(self._result)
        except CvBridgeError as e:
            rospy.logerr('%s:  Error during callback:\%s', self.name, e)


if __name__ == '__main__':
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
    args = parser.parse_args()
    rospy.init_node('detectron2_action_server')
    server = Detectron2Action(rospy.get_name(), args)
    rospy.spin()

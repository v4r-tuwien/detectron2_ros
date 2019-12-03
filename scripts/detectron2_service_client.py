#!/usr/bin/env python

import rospy
import argparse

from object_detector_msgs.srv import detectron2_service_server
from sensor_msgs.msg import Image
from object_detector_msgs.msg import Detections, Detection 

import time
import sys

class Service:
    __img_message = None
    __name = None

    def image_cb(self, data):
        self.__img_message = data


    def run(self, name, args):

	self.__name = name
        image_sub = rospy.Subscriber(args.topic, Image, self.image_cb, buff_size=2**24, queue_size=1)

        #wait the service to be advertised, otherwise the service use will fail
        rospy.wait_for_service('/detectron2_service/detect')

        #setup a local proxy for the service
        srv=rospy.ServiceProxy('/detectron2_service/detect',detectron2_service_server)

        while self.__img_message is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        start = time.time()
        #use the service and send it the image
        result = srv(self.__img_message)

        end = time.time()
        #print the result from the service
        print result

        print('{:5.3f}s'.format(end-start))


if __name__ == '__main__':
    rospy.init_node('detectron2_detection_service')

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-t',
        '--camera-topic',
        dest='topic',
        type=str,
        default='/hsrb/head_rgbd_sensor/rgb/image_raw',
        help='Camera topic to subscribe to')
    args = parser.parse_args()

    service = Service()
    service.run(rospy.get_name(), args)


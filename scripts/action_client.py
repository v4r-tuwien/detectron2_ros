#! /usr/bin/env python3

from __future__ import print_function

import argparse
import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import actionlib
import time
import object_detector_msgs.msg
import numpy as np

client = None


def action_client(image):
    client.wait_for_server()
    goal = object_detector_msgs.msg.CheckForDetectionsGoal(image)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


def vis_results(image, result):
    detections = result.detections.detections
    overlay_rgba = cv2.cvtColor(
        np.zeros(image.shape, np.uint8), cv2.COLOR_RGB2RGBA)
    overlay_rgba_flatten = overlay_rgba.reshape(-1, overlay_rgba.shape[-1])

    # Sort by max. areas to minimize overlap for visualization
    areas = np.asarray([len(x.mask) for x in detections])
    if areas is not None:
        sorted_idxs = np.argsort(-areas).tolist()
        detections = [detections[k] for k in sorted_idxs]

    for item in detections:
        cv2.rectangle(image, (item.bbox.xmin, item.bbox.ymin),
                      (item.bbox.xmax, item.bbox.ymax), (255, 0, 0), 2)
        color = np.random.randint(0, high=255, size=3)
        overlay_rgba_flatten[list(item.mask)] = np.append(color, 255)

    overlay_rgba = overlay_rgba_flatten.reshape(overlay_rgba.shape)
    image_rgba = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)
    dst = cv2.addWeighted(image_rgba, 0.4, overlay_rgba, 0.6, 0)
    cv2.imshow("detection", dst)
    cv2.waitKey(0)


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-i',
            '--image_file',
            dest='image_file',
            type=str,
            default=None,
            help='Path to image file.')
        args = parser.parse_args()
        img_cv = cv2.imread(args.image_file)
        img_message = CvBridge().cv2_to_imgmsg(cv2.cvtColor(
            img_cv, cv2.COLOR_RGB2BGR), encoding="rgb8")
        rospy.init_node('detectron2_action_client')

        # Setup action client and call with image from file
        client = actionlib.SimpleActionClient(
            'detectron2_action_server', object_detector_msgs.msg.CheckForDetectionsAction)
        rospy.loginfo('Client is up, calling server.')
        start_time = time.time()
        result = action_client(img_message)
        end_time = time.time() - start_time
        rospy.loginfo('Detected %d objects in %.3f sec ',
                      len(result.detections.detections), end_time)
        for item in result.detections.detections:
            rospy.loginfo('%s, %.3f' % (item.name, item.score))

        vis_results(img_cv, result)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion", file=sys.stderr)
    except CvBridgeError as e:
        rospy.logerr('Error during image conversion:\%s' % e)

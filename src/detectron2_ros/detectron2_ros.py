import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from object_detector_msgs.msg import Detections, Detection


class Detectron2Ros(object):
    """ This class provides methods for interfacing detectron2 with ros
    """

    def __init__(self, predictor):
        """
        Args:
            predictor (Detectron2Predictor): an instance of predictor wrapper class
        """
        self.predictor = predictor
        self.image_np = None
        self.header = None
        self.height = None
        self.width = None
        self.predictions = None
        self.detections = 0
        self.bridge = CvBridge()

    def run_on_image(self, image_msg):
        """
        Args:
            image_msg (Image): a sensor_msgs image
        Returns:
             An object_detector_msgs Detections message.
        """
        self.header = image_msg.header
        self.height = image_msg.height
        self.width = image_msg.width
        image_np = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        self.predictions = self.predictor.run_on_image(image_np)
        self.detections = self.__count_detections()
        return self.to_detector_message()

    def to_detector_message(self):
        boxes, classes, scores, masks = self.predictor.convert_from_tensor(
            self.predictions)
        message = Detections()
        message.header = self.header
        message.width = self.width
        message.height = self.height
        for i, box in enumerate(boxes):
            detection = Detection()
            detection.name = classes[i]
            detection.score = scores[i]
            detection.bbox.xmin = int(round(box[0]))
            detection.bbox.ymin = int(round(box[1]))
            detection.bbox.xmax = int(round(box[2]))
            detection.bbox.ymax = int(round(box[3]))
            detection.mask = [] if masks is None else np.nonzero(masks[i].flatten())[
                0]
            message.detections.append(detection)
        return message

    def to_vis_message(self):
        visualization = self.predictor.visualize(
            self.predictions, self.image_np)
        image_message = self.bridge.cv2_to_imgmsg(cv2.cvtColor(
            visualization, cv2.COLOR_RGB2BGR), encoding="rgb8")
        return image_message

    def __count_detections(self):
        boxes = self.predictions["instances"].pred_boxes if self.predictions["instances"].has(
            "pred_boxes") else None
        return len(boxes)

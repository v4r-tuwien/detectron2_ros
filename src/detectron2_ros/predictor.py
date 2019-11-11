import numpy as np
import torch

from detectron2.engine.defaults import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import ColorMode, Visualizer, GenericMask
from detectron2.data import MetadataCatalog


class Detectron2Predictor(object):
    def __init__(self, args):
        """
        Args:
            args (parser arg):
        """
        self.cfg = self.setup_cfg(args)
        self.cpu_device = torch.device("cpu")
        self.predictor = DefaultPredictor(self.cfg)
        self.metadata = MetadataCatalog.get(
            self.cfg.DATASETS.TEST[0] if len(self.cfg.DATASETS.TEST) else "__unused")

    def run_on_image(self, image):
        """
        Args:
            image (np.ndarray): an image of shape (H, W, C) (in BGR order).
                This is the format used by OpenCV.
        Returns:
            predictions (dict): the output of the detectron2 model.
        """

        predictions = self.predictor(image)
        return predictions

    def convert_from_tensor(self, predictions):
        """
        Args:
            predictions (dict):  result from detectron2 model
        Returns:
            tuples of np.ndarrays of bounding boxes, labels, scores, and masks
        """

        instances = boxes = masks = labels = None
        if "instances" in predictions:
            instances = predictions["instances"].to(self.cpu_device)
            boxes = instances.pred_boxes.tensor.numpy() if instances.has("pred_boxes") else None
            scores = instances.scores.numpy() if instances.has("scores") else None
            classes = instances.pred_classes.numpy() if instances.has("pred_classes") else None
            class_names = self.metadata.get("thing_classes", None)
            labels = None

            if classes is not None and class_names is not None and len(class_names) > 1:
                labels = [class_names[i] for i in classes]

            if instances.has("pred_masks"):
                masks = np.asarray(instances.pred_masks)
                # masks = [GenericMask(x, self.output.height, self.output.width) for x in masks]
            else:
                masks = None
        return boxes, labels, scores, masks

    def visualize(self, predictions, image):
        """
        Args:
            predictions (dict): predictions to be visualized
            image (np.ndarray): corresponding image of shape (H, W, C) (in BGR order).
                This is the format used by OpenCV.
        Returns:
            vis_image (np.ndarray): visualization image.
        """

        vis_image = image[:, :, ::-1]
        visualizer = Visualizer(vis_image, self.metadata,
                                instance_mode=ColorMode.IMAGE)
        if "instances" in predictions:
            instances = predictions["instances"].to(self.cpu_device)
            vis_out = visualizer.draw_instance_predictions(
                predictions=instances)
            vis_image = vis_out.get_image()
        return vis_image

    def setup_cfg(self, args):
        """
        Args:
            args (parser args): create detector2 config from parsed commandline arguments
        Returns:
            cfg (CfgNode): detectron2 configuration instance.
        """

        # load config from file and command-line arguments
        cfg = get_cfg()
        cfg.merge_from_file(args.config_file)
        cfg.merge_from_list(args.opts)
        # Set score_threshold for builtin models
        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = args.confidence_threshold
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence_threshold
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = args.confidence_threshold
        cfg.freeze()
        return cfg

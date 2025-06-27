# Some basic setup:
# Setup detectron2 logger
import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import some common libraries
import numpy as np
import os, json, cv2, random

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog

class MyDetectron2:
    def __init__(self):
        self.cfg = get_cfg()
        # add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
        # Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        self.predictor = DefaultPredictor(self.cfg)

    def predict(self, img, display=False):
        #im = cv2.imread(img_path)

        outputs = self.predictor(img)

        instances = outputs["instances"]
        #boxes = instances.pred_boxes if instances.has("pred_boxes") else None
        #scores = instances.scores if instances.has("scores") else None
        classes = instances.pred_classes if instances.has("pred_classes") else None
        #masks = instances.pred_masks if instances.has("pred_masks") else None
        class_names = []

        coco_metadata = MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0])
        for i in range(len(classes)):
            class_name = coco_metadata.thing_classes[classes[i]]
            class_names.append(class_name)
            #print(f"Detected object: {class_name}, Bounding box: {boxes[i]}")
            #mask = masks[i].cpu().numpy()  # Get the mask as a numpy array
            #pixels = np.where(mask == 1)   # Get the pixel coordinates corresponding to the mask
            #print(pixels)

        instances.set("class_names", class_names)
        
        if display:
            # We can use `Visualizer` to draw the predictions on the image.
            v = Visualizer(img[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1.2)
            out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
            cv2.imshow('pre', out.get_image()[:, :, ::-1])
            cv2.waitKey()

        return instances
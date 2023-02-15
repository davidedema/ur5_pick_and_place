from pathlib import Path
import sys
import os
import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
VISION_PATH = os.path.abspath(os.path.join(ROOT, ".."))

LEGO_NAMES = [  'X1-Y1-Z2',
                'X1-Y2-Z1',
                'X1-Y2-Z2',
                'X1-Y2-Z2-CHAMFER',
                'X1-Y2-Z2-TWINFILLET',
                'X1-Y3-Z2',
                'X1-Y3-Z2-FILLET',
                'X1-Y4-Z1',
                'X1-Y4-Z2',
                'X2-Y2-Z2',
                'X2-Y2-Z2-FILLET']

# -----------------------------------------------------------------------------

class LegoDetect:

    def __init__(self):
        '''
        yolov5 detection using custom trained weights: ../weights/best.pt
        '''
        self.weights_path = os.path.join(VISION_PATH, "weights/best.pt")
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', self.weights_path)
        self.model.conf = 0.5
        self.model.multi_label = True
        self.model.iou = 0.5
        self.lego_list = []

    def setConfidence(self, conf):
        '''
        Set confidence threshold. Call before detect()
        '''
        self.model.conf = conf

    def detect(self, img_path):
        '''
        Detect lego
        '''
        self.lego_list.clear()
        self.img_path = img_path
        self.results = self.model(self.img_path)
        self.results.show()

        self.img = Image.open(self.img_path)

        print(self.img_path)
        print('img size:', self.img.width, 'x', self.img.height)
        self.calculateBoundingBox()

    def calculateBoundingBox(self):
        '''
        Calculate bboxes in detail. Call after detect()
        '''
        bboxes = self.results.pandas().xyxy[0].to_dict(orient="records")
        # For each detected obj, add to lego_list
        for bbox in bboxes:
            name = bbox['name']
            conf = bbox['confidence']
            x1 = int(bbox['xmin'])
            y1 = int(bbox['ymin'])
            x2 = int(bbox['xmax'])
            y2 = int(bbox['ymax'])
            self.lego_list.append(Lego(name, conf, x1, y1, x2, y2, self.img_path))

# -----------------------------------------------------------------------------

class Lego:

    def __init__(self, name, conf, x1, y1, x2, y2, img_source_path):
        self.name = name
        self.class_id = LEGO_NAMES.index(name)
        self.confidence = conf
        self.xmin = x1
        self.ymin = y1
        self.xmax = x2
        self.ymax = y2
        self.img_source_path = img_source_path
        self.img_source = Image.open(self.img_source_path)
        self.center_point = (int((x1+x2)/2), int((y1+y2)/2))
        self.center_point_uv = (self.img_source.width - self.center_point[0], self.center_point[1])
        self.point_cloud = ()
        self.point_world = ()

    def show(self):
        self.img = self.img_source.crop((self.xmin, self.ymin, self.xmax, self.ymax))

        # Resize detected obj
        # Not neccessary. Useful when the obj is too small to see
        aspect_ratio = self.img.size[1] / self.img.size[0]
        new_width = 70  # resize width (pixel) for detected object to show
        new_size = (new_width, int(new_width * aspect_ratio))
        self.img = self.img.resize(new_size, Image.LANCZOS)

        # Obj details
        print()
        display(self.img)
        print('class =', self.name)
        print('id =', self.class_id)
        print('confidence =', '%.2f' %self.confidence)
        print('center_point =', self.center_point)
        print('center_point_uv =', self.center_point_uv)
        print('--> point cloud =', self.point_cloud)
        print('--> point world =', self.point_world)
        print('\n-----------------')

# -----------------------------------------------------------------------------

# To use in command:
# python3 LegoDetect.py /path/to/img...

if __name__ == '__main__':
    img_path = sys.argv[1]
    legoDetect = LegoDetect()
    legoDetect.detect(img_path)
    for lego in legoDetect.lego_list:
        lego.show()

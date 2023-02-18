"""!
@file LegoDetect.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the class Lego and LegoDetect.
@date 2023-02-17
"""
# ---------------------- IMPORT ----------------------
from pathlib import Path
import sys
import os
import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image
from RegionOfInterest import RegionOfInterest

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
VISION_PATH = os.path.abspath(os.path.join(ROOT, ".."))
IMG_ROI = os.path.abspath(os.path.join(ROOT, "log/img_ROI.png"))

WEIGHTS_PATH = os.path.join(VISION_PATH, "weights/best.pt")
CONFIDENCE = 0.7
MODEL = torch.hub.load('ultralytics/yolov5', 'custom', WEIGHTS_PATH)

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

# ---------------------- CLASS ----------------------

class LegoDetect:
    """
    @brief This class use custom trained weights and detect lego blocks with YOLOv5
    """

    def __init__(self, img_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
        """

        MODEL.conf = CONFIDENCE
        MODEL.multi_label = False
        # MODEL.iou = 0.5
    
        self.lego_list = []
        self.detect(img_path)

        # Let user choose detect method
        choice = '0'
        while True:
            while (choice != '1' and choice != '2' and choice != ''):
                ask =  ('\nContinue     (ENTER)'+
                        '\nDetect again (1)'+
                        '\nDetect ROI   (2)'+
                        '\nchoice ----> ')
                choice = input(ask)

            # Continue
            if choice == '':
                break

            # Detect again using original image
            if choice == '1':
                print('Detecting again...')
                self.detect(img_path)
                choice = '0'

            # Detect using ROI
            if choice == '2':
                self.detect_ROI(img_path)
                choice = '0'

    def detect_ROI(self, img_path):
        """ @brief Detect using Region Of Interest
            @param img_path (String): path of input image
        """

        print('Draw RegionOfInterest')
        roi = RegionOfInterest(img_path, IMG_ROI)
        roi.run_auto()
        print('Detecting RegionOfInterest...')
        self.detect(IMG_ROI)

    def detect(self, img_path):
        """ @brief This function pass the image path to the model and calculate bounding boxes for each object
            @param img_path (String): path of input image
        """
        self.lego_list.clear()

        # Detection model
        self.results = MODEL(img_path)
        self.results.show()
        img = Image.open(img_path)
        print(img_path)
        print('img size:', img.width, 'x', img.height)

        # Bounding boxes
        bboxes = self.results.pandas().xyxy[0].to_dict(orient="records")
        for bbox in bboxes:
            name = bbox['name']
            conf = bbox['confidence']
            x1 = int(bbox['xmin'])
            y1 = int(bbox['ymin'])
            x2 = int(bbox['xmax'])
            y2 = int(bbox['ymax'])
            # Add lego to list
            self.lego_list.append(Lego(name, conf, x1, y1, x2, y2, img_path))

        # Info
        print('Detected', len(self.lego_list), 'object(s)\n')
        self.show()

    def show(self):
        """ @brief This function show infos of detected legos
        """
        for index, lego in enumerate(self.lego_list, start=1):
            print(index)
            lego.show()

# ---------------------- CLASS ----------------------

class Lego:
    """
    @brief This class represents info of detected lego
    """

    def __init__(self, name, conf, x1, y1, x2, y2, img_source_path):
        """ @brief Class constructor
            @param name (String): lego name
            @param conf (float): confidence
            @param x1 (float): xmin of bounding box
            @param y1 (float): ymin of bounding box
            @param x2 (float): xmax of bounding box
            @param y2 (float): ymax of bounding box
            @param img_source_path (String): path to image
        """

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
        """ @brief Show lego info
        """

        self.img = self.img_source.crop((self.xmin, self.ymin, self.xmax, self.ymax))

        # Resize detected obj
        # Not neccessary. Useful when the obj is too small to see
        aspect_ratio = self.img.size[1] / self.img.size[0]
        new_width = 70  # resize width (pixel) for detected object to show
        new_size = (new_width, int(new_width * aspect_ratio))
        self.img = self.img.resize(new_size, Image.LANCZOS)

        # Lego details
        display(self.img)
        print('class =', self.name)
        print('id =', self.class_id)
        print('confidence =', '%.2f' %self.confidence)
        print('center_point =', self.center_point)
        print('center_point_uv =', self.center_point_uv)
        print('--> point cloud =', self.point_cloud)
        print('--> point world =', self.point_world)
        print()

# ---------------------- MAIN ----------------------
# To use in command:
# python3 LegoDetect.py /path/to/img...

if __name__ == '__main__':
    legoDetect = LegoDetect(img_origin_path=sys.argv[1])


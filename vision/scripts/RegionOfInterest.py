"""!
@file RegionOfInterest.py
@author Giulio
@brief Defines the class RegionOfInterest.py
@date 2023-02-17
"""
# ---------------------- IMPORT ----------------------
import cv2
import numpy as np
from pathlib import Path
import sys
import os

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
USING_REAL_CAM = True

# ---------------------- CLASS ----------------------

class RegionOfInterest:
    """
    @brief This class defines custom Region Of Interest
    """

    def __init__(self, image_path, output_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
            @param output_path (String): path of out image
        """

        self.img_path = image_path
        self.output_path = output_path
        self.img = cv2.imread(self.img_path)

    def run_auto(self):
        """ @brief Draw custom mask and set the outside of the mask black
        """
        mask = np.zeros(self.img.shape[0:2], dtype=np.uint8)

        if USING_REAL_CAM:
            points = np.array([[[457,557], [555,272], [779,267], [960,532]]])
        else:
            points = np.array([[[845,409], [1201,412], [1545,913], [658, 921]]])

        #method 1 smooth region
        cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)
        #method 2 not so smooth region
        # cv2.fillPoly(mask, points, (255))
        res = cv2.bitwise_and(self.img,self.img,mask = mask)
        rect = cv2.boundingRect(points) # returns (x,y,w,h) of the rect
        cropped = res[rect[1]: rect[1] + rect[3], rect[0]: rect[0] + rect[2]]
        ## crate the white background of the same size of original image
        wbg = np.ones_like(self.img, np.uint8)*255
        cv2.bitwise_not(wbg,wbg, mask=mask)
        # overlap the resulted cropped image on the white background
        dst = wbg+res
        # cv2.imshow('Original',self.img)
        # cv2.imshow("Mask",mask)
        # cv2.imshow("Cropped", cropped )
        # cv2.imshow("Samed Size Black Image", res)
        cv2.imwrite(self.output_path, res)
        # cv2.imshow("Samed Size White Image", dst)
        # cv2.waitKey(0)
        cv2.destroyAllWindows()

# ---------------------- MAIN ----------------------
# To use in command:
# python3 RegionOfInterest.py /path/to/input/img /path/to/output/img
if __name__ == '__main__':
    roi = RegionOfInterest(img_path=sys.argv[1], output_path=sys.argv[2])
    roi.run()











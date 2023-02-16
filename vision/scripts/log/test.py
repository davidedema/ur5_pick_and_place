from pathlib import Path
import sys
import os
import cv2
import numpy

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
IMG_ZED = os.path.abspath(os.path.join(ROOT, "img_ZED_cam.png"))


img = cv2.imread(IMG_ZED)
cv2.imshow('image', img)
cv2.waitKey(0)


import cv2
import numpy as np
from pathlib import Path
import sys
import os

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative


class RegionOfInterest:
    def __init__(self, image_path, output_path):
        self.img_path = image_path
        self.output_path = output_path
        self.img = cv2.imread(self.img_path)
        self.draw_img = self.img.copy()
        self.boxes = []
        self.drawing = False
        self.start = (-1, -1)
        self.end = (-1, -1)

        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.draw_box)

    def draw_box(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start = (x, y)
            self.end = (x, y)
            print('x =', self.start[0], 'y =', self.start[1])
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                self.end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end = (x, y)
            self.boxes.append((self.start, self.end))
            self.start = (-1, -1)
            self.end = (-1, -1)

    def run(self):
        while True:
            temp_img = self.draw_img.copy()
            for box in self.boxes:
                cv2.rectangle(temp_img, box[0], box[1], (0, 255, 0), 2)
            cv2.imshow('image', temp_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 13: # Check for "ENTER" button press
                mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
                for box in self.boxes:
                    cv2.rectangle(mask, box[0], box[1], 255, -1)
                self.img[mask == 0] = (0, 0, 0)
                cv2.imwrite(self.output_path, self.img)
                self.draw_img[mask == 0] = (0, 0, 0)
                cv2.imshow('image', self.draw_img)
                cv2.waitKey(0)
                break

        cv2.destroyAllWindows()

if __name__ == '__main__':

    roi = RegionOfInterest(img_path=sys.argv[1], output_path=sys.argv[2])
    roi.run()











import os
import cv2

import numpy as np
import matplotlib.pyplot as plt

from PIL import Image, ImageDraw

class StopSignDetector:
  def __init__(self, threshold=0.5):
    self.model = None#torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
    self.threshold = threshold
    self.results = None

  def predict(self, img):
    """
    Takes in a path or numpy array representing an image
    returns whether or not there is a stop sign
    """
    if type(img) == str:
      # Path has been passed in
      img_path = img
      img = read_image(img_path)

    # convert the image to the HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # define a range of red color in HSV
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # combine the masks
    mask = cv2.bitwise_or(mask1, mask2)

    # apply the mask to the original image
    red_img = cv2.bitwise_and(img, img, mask=mask)

    # convert the red image to grayscale
    gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)

    # apply a median blur to the grayscale image
    gray = cv2.medianBlur(gray, 5)

    # find contours in the image
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # find the contour with the largest area
    max_area = 400 # Require that the stop sign take up at least 400 pixels
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is None:
        return False, (0, 0, 0, 0)
    else:
      # draw a bounding box around the largest contour
      x, y, w, h = cv2.boundingRect(max_contour)
      return True, (x, y, x + w, y + h)


# Utilities

# Image
def read_image(path):
    rgb_im = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    return rgb_im

def draw_rect(im, xmin, ymin, xmax, ymax):
    box = xmin, ymin, xmax, ymax
    img = Image.fromarray(im)
    imgd = ImageDraw.Draw(img)
    imgd.rectangle(box, outline='red')
    return img

def draw_box(im, box):
    img = Image.fromarray(im)
    imgd = ImageDraw.Draw(img)
    imgd.rectangle(box, outline='red')
    return img

# Detecting Utils

THRESHOLD = 0.7

def is_stop_sign(df, label='stop sign', threshold=THRESHOLD):
    confidences = df[df['confidence'] > threshold]
    return len(confidences[confidences['name'] == label]) != 0 # If a stop sign has been detected

def get_bounding_box(df, label='stop sign', threshold=THRESHOLD):
    if not is_stop_sign(df, label=label, threshold=threshold): return (0, 0, 0, 0)
    confidences = df[df['confidence'] > threshold]
    stop_sign = confidences[confidences['name'] == label].head(1)
    coords = stop_sign.xmin, stop_sign.ymin, stop_sign.xmax, stop_sign.ymax
    return [coord.values[0] for coord in coords]

if __name__=="__main__":
    detector = StopSignDetector()

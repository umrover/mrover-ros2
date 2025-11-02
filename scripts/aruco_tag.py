#! /usr/bin/env python3

# imports
import cv2
import numpy as np

TAG_WIDTH = int(100)
TAG_ID = 2


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    markerImage = np.zeros((TAG_WIDTH, TAG_WIDTH), dtype=np.uint8)  # zeros
    markerImage = cv2.aruco.generateImageMarker(dictionary, TAG_ID, TAG_WIDTH, markerImage, 1)  # type: ignore

    cv2.imwrite("marker0.png", markerImage)


if __name__ == "__main__":
    main()

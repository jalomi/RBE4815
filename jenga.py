import cv2
import numpy as np
from PIL import Image
import zbar
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import BaseHTTPServer
import abb

"""
Author: John Lomi
"""

#setup camera
capture = None


def initializeCamera():
    camera = PiCamera()

    camera.framerate = 15
    camera.resolution = (1280, 720)

    rawCap = PiRGBArray(camera, size=(1280, 720))

    #sleep to allow camera to warm up
    sleep(.5)



def main():
    initializeCamera()

    for frame in camera.capture_continuous(rawCap, format="bgr", use_video_port=True):
        image = frame.array

        # Convert to greyscale
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Convert to PIL image
        pil = Image.fromarray(grey)

        # Setup scanner
        scanner = zbar.Scanner()
        codes = scanner.scan(pil)

        # Print results
        results = []
        for decoded in codes:
            #draw square on image
            TL, BL, BR, TR = decoded.position
            contour = np.array([np.array([TL, BL, BR, TR], dtype=np.int32)])
            cv2.drawContours(image, contour, 0, (0,0,255), 5)
            #draw dot at center
            m = cv2.moments(contour)
            cX = int(m["m10"] / m["m00"])
            cY = int(m["m01"] / m["m00"])
            cv2.circle(image, (cX,cY), 7, (0,255,0), -1)
            #draw label
            cv2.putText(image, decoded.data, BL, cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 5)

            results.append((cX, cY, decoded.data))

        # Display current frame
        cv2.imshow('Frame', image)
        key = cv2.waitKey(1) & 0xFF
        rawCap.truncate(0)


if __name__ == "__main__":
    main()

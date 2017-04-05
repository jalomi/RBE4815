import cv2
import numpy as np
from PIL import Image
import zbar
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import BaseHTTPServer

"""
Author: John Lomi
"""

#setup camera
capture = None


class ServerHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(self):
        print("CtlHandler: do_GET")
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        try:
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

                #return the position and value of every QR code
                self.wfile.write(results)

        except KeyboardInterrupt:
            return


def initializeCamera():
    camera = PiCamera()

    camera.framerate = 15
    camera.resolution = (1280, 720)

    rawCap = PiRGBArray(camera, size=(1280, 720))

    #sleep to allow camera to warm up
    sleep(.5)


def serverInit():
    #start control server
    try:
        server = HTTPServer(('', 5001), ServerHandler)
        print("5001 started")
        server.serve_forever()
    except KeyboardInterrupt:
        server.socket.close()


def main():
    initializeCamera()

    #start server process
    server = Process(target=serverInit)
    server.start()

    #wait for Ctrl-C
    try:
        while(1):
            pass
    except KeyboardInterrupt:
        pass


    server.join()


if __name__ == "__main__":
    main()

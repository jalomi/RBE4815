import cv2
import numpy as np
from PIL import Image
import zbar
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import socket

"""
Author: John Lomi
"""

#setup camera
rawCap = None
gameOver = False

def initializeServers():
    """Set up TCP/IP Servers for arm and computer"""
    global arm, comp

    #Connect to arm
    s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    print("Socket created")

    try:
        s1.bind(('',5515))
    except socket.error as msg:
        print(str(msg[0]) + ":" + msg[1])

    print("Socket binded")

    s1.listen(10)

    print("Socket listening")


    arm, addr1 = s1.accept()
    print("Arm Connected: " + addr1[0] + ":" + str(addr1[1]))

    #connect to computer
    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    print("Socket created")

    try:
        s2.bind(('',5001))
    except socket.error as msg:
        print(str(msg[0]) + ":" + msg[1])

    print("Socket binded")

    s2.listen(10)

    print("Socket listening")


    comp, addr2 = s2.accept()
    print("Comp Connected: " + addr2[0] + ":" + str(addr2[1]))



def initializeCamera():
    """Setup camera"""
    global rawCap
    camera = PiCamera()

    camera.framerate = 15
    camera.resolution = (1280, 720)

    rawCap = PiRGBArray(camera, size=(1280, 720))

    #sleep to allow camera to warm up
    sleep(.5)


def takePictureQR():
    """Return a list of QR code positions and values"""
    global rawCap
    camera.capture(rawCap, format="bgr", use_video_port=True)
    image = rawCap.array

    # Convert to greyscale
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Convert to PIL image
    pil = Image.fromarray(grey)

    # Setup scanner
    scanner = zbar.Scanner()
    codes = scanner.scan(pil)


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

    return results


def moveToCode(val, blocks):
    """Move to where the QR code of value val is suspected to be

    Arguments:
    val -- the code value to move to
    blocks -- the locations of all the blocks in the tower
    """
    #find QR code in main array

    #move to selected position

    pass


def scanTower():
    """Use camera to generate matrix to represent block posistions"""
    #this is just a placeholder.  It should move the arm up the tower and
    #use takePictureQR to generate this matrix of codes.  This is supposed
    #to be like a map of the blocks
    tower = [[1, 2, 3],
            [4, 5, 6],
            [7, 8, 9],
            [10, 11, 12],
            [13, 14, 15],
            [16, 17, 18],
            [19, 20, 21],
            [22, 23, 24],
            [25, 26, 27],
            [28, 29, 30],
            [31, 32, 33],
            [34, 35, 36],
            [37, 38, 39],
            [40, 41, 42],
            [43, 44, 45],
            [46, 47, 48],
            [49, 50, 51],
            [52, 53, 54],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]]

    return tower




def main():
    """Algorithm to play Jenga using the ABB arm"""
    global arm, comp

    """
    Use arm.send() and comp.send() to send stuff
    Use arm.recv() and comp.recv() to recieve stuff
    """

    #start servers
    initializeServers()

    #define camera settings
    initializeCamera()

    #find tower in workspace
    towerlocation = [0, 0, 0]

    #scan tower
    blockPos = scanTower()

    #loop until game over
    while not gameOver:
        #wait for code number
        val = comp.recv(16)

        #move to block
        moveToCode(val, blockPos)

        #take picture
        codeList = takePictureQR()

        #if gameover, continue
        if not codeList:
            gameOver = True
            print("GAMEOVER: No codes found")
            continue 

        #find desired code
        for code in codeList:
            if code[2] == val:
                codeX = code[0]
                codeY = code[1]
                break
        else:
            gameOver = True
            print("GAMEOVER: Code not in image")
            continue

        #make adjustments
        diffX = codeX - 640
        diffY = codeY - 360

        #push

        #rotate

        #pull

        #drop

        #grab lengthwise

        #place on top of tower

        #return to start position
        arm.send("home% #")




if __name__ == "__main__":
    main()

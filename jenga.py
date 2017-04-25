import cv2
import numpy as np
from PIL import Image
import zbar
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import socket
import RPi.GPIO as GPIO

"""
Author: John Lomi
"""

#setup GPIO
GPIO.setmode(GPIO.BCM)
limit = 21
GPIO.setup(limit, GPIO.IN)

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
##    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
##
##    print("Socket created")
##
##    try:
##        s2.bind(('',5001))
##    except socket.error as msg:
##        print(str(msg[0]) + ":" + msg[1])
##
##    print("Socket binded")
##
##    s2.listen(10)
##
##    print("Socket listening")
##
##
##    comp, addr2 = s2.accept()
##    print("Comp Connected: " + addr2[0] + ":" + str(addr2[1]))



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

def runMultipleCommands(string):
    #run various movement messages
    #msg1 = "setPokeOffset%#"
    #poke takes like ColumnxRow
    
##    msg3 = ""
##    msg4 = ""
##    arm.send(msg1)
##    
##    data = arm.recv(1024)
##    print(data)

    arm.send(string)
    data = arm.recv(1024)
    print(data)

    if data == "trigger":
        arm.send("false")
        
    data = arm.recv(1024)
    print(data)
    if data == "trigger":
        arm.send("false")
    data = arm.recv(1024)
    print(data)
    if data == "trigger":
        arm.send("false")
    data = arm.recv(1024)
    print(data)

    msg3 = "moveAround% x,#"
    arm.send(msg3)
    data = arm.recv(1024)
    print(data)
    
    
##    arm.send(msg3)
##    data = arm.recv(1024)
##    print(data)
##    
##    arm.send(msg4)
##    data = arm.recv(1024)
##    print(data)

def runSingleCommand(string):
    arm.send(string)
    data = arm.recv(1024)
    print(data)

    
def triggered():
    #placeholder for limit switch check
    while arm.recv(1024) != "complete":
        #check limit
        arm.send("false")#placeholder for checking limit
    print("Poke Completed")
    



def main():
    """Algorithm to play Jenga using the ABB arm"""
    global arm, comp, limit

    """
    Use arm.send() and comp.send() to send stuff
    Use arm.recv() and comp.recv() to recieve stuff
    """

    #start servers
    initializeServers()

    #define camera settings
    initializeCamera()


    #loop until game over
    while not gameOver:
        #wait for code number
        val = comp.recv(1024)

        #send block value to arm
        arm.send("poke% {}, #".format(val))

        #check limit switch
        data = arm.recv(1024)
        print(data)

        if data == "trigger":
            if GPIO.input(limit):
                arm.send("true")
            else:
                arm.send("false")
        
        #check limit switch
        data = arm.recv(1024)
        print(data)
        if data == "trigger":
            if GPIO.input(limit):
                arm.send("true")
            else:
                arm.send("false")

        #check limit switch
        data = arm.recv(1024)
        print(data)
        if data == "trigger":
            if GPIO.input(limit):
                arm.send("true")
            else:
                arm.send("false")


        data = arm.recv(1024)
        print(data)
        

        # msg3 = "moveAround% x,#"
        # arm.send(msg3)
        # data = arm.recv(1024)
        # print(data)



        # #if gameover, continue
        # if not codeList:
        #     gameOver = True
        #     print("GAMEOVER: No codes found")
        #     continue 

        # #find desired code
        # for code in codeList:
        #     if code[2] == val:
        #         codeX = code[0]
        #         codeY = code[1]
        #         break
        # else:
        #     gameOver = True
        #     print("GAMEOVER: Code not in image")
        #     continue

        # #make adjustments
        # diffX = codeX - 640
        # diffY = codeY - 360

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

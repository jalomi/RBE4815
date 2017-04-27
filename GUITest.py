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
GPIO.setup(limit, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def initializeServers():
    """Set up TCP/IP Servers for arm and computer"""
    global comp

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


def main():
    global comp
    
    initializeServers()

    while True:
        val = comp.recv(1024)
        comp.send(val)
        place = comp.recv(1024)
        comp.send(place)
        print("Comp: {}".format(val))
        print("Comp: {}".format(place))
        #print("poke% {}, x, #".format(val))

        #data = GPIO.input(limit)
        #comp.send(data)
    

##    while True:
##        try:
##            pin = GPIO.input(limit)
##           print(pin)
##        except KeyboardInterrupt:
##            break





if __name__ == "__main__":
    main()

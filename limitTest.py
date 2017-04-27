import RPi.GPIO as GPIO

"""
Author: John Lomi
"""

#setup GPIO
GPIO.setmode(GPIO.BCM)
limit = 21
GPIO.setup(limit, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

while True:
    try:
        pin = GPIO.input(limit)
        print(pin)
    except KeyboardInterrupt:
        break

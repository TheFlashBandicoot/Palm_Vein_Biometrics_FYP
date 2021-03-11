

# --------------------------------------- Description --------------------------------------- #

# Python Vein Imager. This python file runs on a Raspberry-Pi based IoT device
# and performs contactless vein image capture using NIR IREDs, an NIR camera,
# an ultrasonic distance sensor, and various UI elements such as buttons and leds.
# It then sends to images to a server using the custom I2P application protocol.

# Author: Bryan McSweeney 08/01/21
# Version: 1.0

# ----------------------------------- Imports and variables --------------------------------- #

import socket
import i2p
from picamera import PiCamera
import time
import sys
from io import BytesIO
import RPi.GPIO as GPIO
import numpy

preview = 0     # variable to keep track of when the camera preview has been turned on
dist_counter = 0    # variable to count how many samples hand is at the right distance
pulse_start = 0
pulse_end = 0

# -------------------------------------- Camera Setup --------------------------------------- #

My_Buffer = BytesIO()   # Buffer to store byte-string produced by camera

camera = PiCamera()     # Camera object
# camera.resolution = (3280,2464) # setting resolution
camera.resolution = (1640,1232) # (2x2 binning)
camera.iso = 100        # setting 'iso' (part of exposure triangle)
camera.awb_mode="off"   # turning off auto-white-balance
camera.awb_gains = 1    # setting white balance gains manually

# --------------------------------------- GPIO Setup ---------------------------------------- #

GPIO.setmode(GPIO.BCM)  # setting GPIO pin numbering scheme (set to Broadcom)

TRIG = 21
ECHO = 16
HIGH = 25
JUST_RIGHT = 12
LOW = 20
IREDS = 4
FAIL = 23
PASS = 18
BUTTON = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(HIGH, GPIO.OUT)
GPIO.setup(JUST_RIGHT, GPIO.OUT)
GPIO.setup(LOW, GPIO.OUT)
GPIO.setup(IREDS, GPIO.OUT)
GPIO.setup(FAIL, GPIO.OUT)
GPIO.setup(PASS, GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(TRIG, 0)
GPIO.output(HIGH, 0)
GPIO.output(JUST_RIGHT, 0)
GPIO.output(LOW, 0)
GPIO.output(IREDS, 0)
GPIO.output(FAIL, 0)
GPIO.output(PASS, 0)

# --------------------------------------- Functions ------------------------------------------ #

def distance_feedback(range):   # function to turn on the correct distance feedback LED

    GPIO.output(HIGH, 0)
    GPIO.output(JUST_RIGHT, 0)
    GPIO.output(LOW, 0)

    if (range==HIGH or range==JUST_RIGHT or range==LOW ):
        GPIO.output(range, 1)


def image_capture(shutter_speed): # function to perform image capture

    camera.exposure_mode = 'off'    # tur off auto-adjusting of exposure
    camera.shutter_speed = shutter_speed     # manually set shutter-speed

    camera.capture(My_Buffer, 'yuv')
    camera.stop_preview()
    GPIO.output(IREDS, 0)   # turn off IREDs

def result_feedback(result):    # function to flash either 'PASS' or 'FAIL' LED for 2 sec

    for i in range(5):
        GPIO.output(result,1)
        time.sleep(0.2)
        GPIO.output(result,0)
        time.sleep(0.2)

# --------------------------------------- Main Loop ------------------------------------------ #

try:
    print('Running Vein Imager...')

    while True: # Main Program Loop

        # ---------------------------------------- I2P Setup --------------------------------- #
        try:
            My_Client = i2p.I2P_Client()    # I2P client object
        except socket.error as error:
            camera.close()
            My_Buffer.close()
            My_Client.shutdown()
            GPIO.cleanup()
            sys.exit()

        print('\nWaiting for hand...')

        # ------------------------------------------------------------------------------------ #
        while True: # Ultrasonic Distance Sensing Loop

            GPIO.output(TRIG, 1)    # sending trigger pulse to hc-sr04
            time.sleep(0.00001)
            GPIO.output(TRIG, 0)

            while GPIO.input(ECHO)==0:
                pulse_start = time.time()   # record time when echo pulse starts

            while GPIO.input(ECHO)==1:
                pulse_end = time.time()     # record time when echo pulse ends

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150   # object distance calculation (cm)

            if (distance>=19.5 and distance<=21.5): # This is the correct height for imaging

                dist_counter = dist_counter + 1 # increment counter
                distance_feedback(JUST_RIGHT)   # turn on appropriate LED

                if (preview==0):        # Object is within range so warm up the camera
                    camera.start_preview()
                    GPIO.output(IREDS, 1)   # turn on IREDs
                    preview = 1     # camera preview is now on

            else:

                if (dist_counter>0):
                    dist_counter = dist_counter - 1 # decrement counter of object drifts out of range

                elif (preview == 1):   # if no object in range, counter depleted, and preview on,
                    camera.stop_preview()   # turn off camera preview
                    GPIO.output(IREDS, 0)   # turn off IREDs
                    preview = 0     # preview is now off

                if (distance>21.5 and distance <30): # if hand too high
                    distance_feedback(HIGH)     # turn on appropriate LED
                elif (distance<19.5 and distance>2):    # if hand too low
                    distance_feedback(LOW)      # turn on appropriate LED
                else:   # if no hand detected
                    distance_feedback(0)       # turn on no LED

            if (dist_counter>=20):  # if hand held at correct height for 20 loops (~2 seconds)
                print ('\nHAND DETECTED!')
                if (GPIO.input(BUTTON)==0):
                    enrol = 1
                    print('(for enrollment)')
                else:
                    enrol = 0
                    print('(for identification)')
                dist_counter = 0    # reset variables
                preview = 0 # (preview is turned off in image_capture() )
                distance_feedback(0)
                break       # break out of distance sensing loop when hand detected at ~20cm

            print('distance = ', distance, '         ', end='\r')   # print distance

            time.sleep(0.1) # sample distance every 10th of a second
        # -------------------------------------------------------------------------------------------------- #


        image_capture(4000) # capture a vein image @4000us shutter-speed (saved to My_Buffer)
        #image_array = numpy.frombuffer(My_Buffer.getvalue(),dtype=numpy.uint8,count=8121344)
        #image_array = image_array.reshape(2464,3296)
        #image_array = numpy.delete(image_array, numpy.s_[3280:3296], axis=1)
        image_array = numpy.frombuffer(My_Buffer.getvalue(),dtype=numpy.uint8,count=2050048)
        image_array = image_array.reshape(1232,1664)
        image_array = numpy.delete(image_array, numpy.s_[1640:1664], axis=1)
        image_data = image_array.tobytes()


        try:
            My_Client.connect('192.168.1.2', 9916) # connect to Server socket
        except socket.error as error:
            print('ERROR CONNECTING TO SERVER')
            print(error)
            result_feedback(FAIL)   # flash fail LED
            continue    # restart main loop (waiting for hand)
        except socket.timeout as timeout:
            print('ERROR CONNECTING TO SERVER')
            print(timeout)
            result_feedback(FAIL)
            continue


        try:
            if (enrol==1):
                My_Client.transmit('enrol', None)    # transmit command
            else:
                My_Client.transmit('identify', None)
        except socket.error as error:
            print('ERROR TRANSMITTING DATA')
            print(error)
            result_feedback(FAIL)
            continue
        except TypeError as error:
            print('ERROR TRANSMITTING DATA')
            print(error)
            result_feedback(FAIL)
            continue
        except ValueError as error:
            print('ERROR TRANSMITTING DATA')
            print(error)
            result_feedback(FAIL)
            continue


        try:
            # My_Client.transmit(My_Buffer.tell(), My_Buffer.getvalue())    # transmit header and image
            My_Client.transmit(len(image_data), image_data)
        except socket.error as error:
            print('ERROR TRANSMITTING IMAGE')
            print(error)
            result_feedback(FAIL)
            continue
        except TypeError as error:
            print('ERROR TRANSMITTING IMAGE')
            print(error)
            result_feedback(FAIL)
            continue
        except ValueError as error:
            print('ERROR TRANSMITTING IMAGE')
            print(error)
            result_feedback(FAIL)
            continue


        try:
            data = My_Client.receive()  # wait to receive result
        except socket.error as error:
            print('ERROR RECEIVING DATA FROM SERVER')
            print(error)
            result_feedback(FAIL)
            continue
        except socket.timeout as error:
            print('ERROR RECEIVING DATA FROM SERVER')
            print(error)
            result_feedback(FAIL)
            continue


        if(data=='    PASS'):
            result_feedback(PASS)
        else:
            result_feedback(FAIL)

        My_Client.shutdown()
        My_Buffer.seek(0)


except KeyboardInterrupt:
    print('\n\n-- CLOSING VEIN IMAGER --')
    camera.close()
    My_Buffer.close()
    My_Client.shutdown()
    GPIO.cleanup()
    sys.exit()


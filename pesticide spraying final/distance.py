#!/usr/bin/python

import time
import RPi.GPIO as GPIO

"""
PIN NUMBERS ACCORDING TO RASPBERRY PI 3B:
VCC: 2
GND: 9
TRIG: 16
ECHO: 18
"""



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO_TRIGGER = 23
GPIO_ECHO = 24

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

GPIO.output(GPIO_TRIGGER, False)
print("Ultrasonic Measurement")

time.sleep(0.5)

def measure_distance():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2

    return distance

curr_dist = 0

def object_movement():
    dist = measure_distance()
    if dist - curr_dist + 2< 0:
        return True
    return False
    
def detect_object():
    dist = measure_distance()
    if dist >= 20:
        return False
    global curr_dist
    curr_dist = dist
    return True



try:
    flag = False
    while False:
        print(measure_distance())
        time.sleep(1)
        if(detect_object() and not flag): 
            print("Object detected, Distance: ", curr_dist)
            flag = True
        elif(object_movement()): 
            print("Object moving")


except KeyboardInterrupt:
    GPIO.cleanup()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import RPi.GPIO as GPIO

from LOBOROBOT import LOBOROBOT


servo = LOBOROBOT()

s1 = 15
s2 = 14
s3 = 13
s4 = 12

s1MIN = 0
s1MAX = 170
s1INIT = 0
s1Angle = 0

s2MIN = 0
s2MAX = 170
s2INIT = 0
s2Angle = 0

s3MIN = 0
s3MAX = 170
s3INIT = 0
s3Angle = 0

s4MIN = 0
s4MAX = 170
s4INIT = 0
s4Angle = 0


def setup():
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
 

def destroy():
    servo.t_stop(0)     #停止
    GPIO.cleanup()

if __name__ == "__main__":
        setup()  #停止
        try:
            servo.set_servo_pulse(s4,0)
        except KeyboardInterrupt:
                destroy()

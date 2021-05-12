#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO
from ROBOT import ROBOT

TRIG = 20
ECHO = 21

SensorRight = 18
SensorMid   = 26
SensorLeft  = 13

BtnPin  = 19
Gpin    = 5
Rpin    = 6

servo = ROBOT()

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

def keysacn():
    val = GPIO.input(BtnPin)
    while GPIO.input(BtnPin) == False:
        val = GPIO.input(BtnPin)
    while GPIO.input(BtnPin) == True:
        time.sleep(0.01)
        val = GPIO.input(BtnPin)
        if val == True:
            GPIO.output(Rpin,1)
            while GPIO.input(BtnPin) == False:
                GPIO.output(Rpin,0)
        else:
            GPIO.output(Rpin,0)

def setup():
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        GPIO.setup(SensorRight,GPIO.IN)
        GPIO.setup(SensorMid,GPIO.IN)
        GPIO.setup(SensorLeft,GPIO.IN)
        GPIO.setup(Gpin, GPIO.OUT)
        GPIO.setup(Rpin, GPIO.OUT)
        GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def Re_Servo():
  global  s1Angle
  global  s2Angle
  global  s3Angle
  global  s4Angle

  servo.set_servo_angle(s1,s1INIT)
  servo.set_servo_angle(s2,s2INIT)
  servo.set_servo_angle(s3,s3INIT)
  servo.set_servo_angle(s4,s4INIT)

  s1Angle = s1INIT
  s2Angle = s2INIT
  s3Angle = s3INIT
  s4Angle = s4INIT
  time.sleep(1)

def destroy():
    servo.t_stop(0)     #停止
    GPIO.cleanup()

def distance():
	GPIO.output(TRIG, 0)
	time.sleep(0.000002)
	GPIO.output(TRIG, 1)
	time.sleep(0.00001)
	GPIO.output(TRIG, 0)

	while GPIO.input(ECHO) == 0:
		a = 0
	time_start = time.time()
	while GPIO.input(ECHO) == 1:
		a = 1
	during = time.time() - time_start
	return during * 340 / 2 * 100

if __name__ == "__main__":
        setup()
        # keysacn()
        # Re_Servo()     #停止
        try:
            servo.set_servo_angle(s4,90)

        except KeyboardInterrupt:
                destroy()

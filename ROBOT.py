#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import math
import smbus
import RPi.GPIO as GPIO

Dir = [
    'forward',
    'backward',
]

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)

  def write(self, reg, value):
    self.bus.write_byte_data(self.address, reg, value)
    print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

  def read(self, reg):
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result

  def setPWMFreq(self, freq):
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1)
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H + 4*channel, on >> 8)
    self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H + 4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))

  def setDutycycle(self, channel, pulse):
    self.setPWM(channel, 0, int(pulse * (4096 / 100)))

  def setLevel(self, channel, value):
    if (value == 1):
      self.setPWM(channel, 0, 4095)
    else:
      self.setPWM(channel, 0, 0)

class ROBOT():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 2
        self.AIN2 = 1

        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

        self.PWMC = 6
        self.CIN2 = 7
        self.CIN1 = 8

        self.PWMD = 11
        self.DIN1 = 25
        self.DIN2 = 24
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIN1,GPIO.OUT)
        GPIO.setup(self.DIN2,GPIO.OUT)

    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            self.pwm.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
        elif(motor == 1):
            self.pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            else:
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
        elif(motor == 2):
            self.pwm.setDutycycle(self.PWMC,speed)
            if(index == Dir[0]):
                self.pwm.setLevel(self.CIN1,1)
                self.pwm.setLevel(self.CIN2,0)
            else:
                self.pwm.setLevel(self.CIN1,0)
                self.pwm.setLevel(self.CIN2,1)
        elif(motor == 3):
            self.pwm.setDutycycle(self.PWMD,speed)
            if (index == Dir[0]):
                GPIO.output(self.DIN1,0)
                GPIO.output(self.DIN2,1)
            else:
                GPIO.output(self.DIN1,1)
                GPIO.output(self.DIN2,0)

    def MotorStop(self, motor):
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, 0)
        elif(motor == 1):
            self.pwm.setDutycycle(self.PWMB, 0)
        elif(motor == 2):
            self.pwm.setDutycycle(self.PWMC, 0)
        elif(motor == 3):
            self.pwm.setDutycycle(self.PWMD, 0)

    def up(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)

    def down(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)

    def moveLeft(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)

    def moveRight(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)

    def turnLeft(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)

    def turnRight(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)

    def forwardLeft(self,speed,t_time):
        self.MotorStop(0)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorStop(0)
        time.sleep(t_time)

    def forwardRight(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)

    def backwardLeft(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)

    def backwardRight(self,speed,t_time):
        self.MotorStop(0)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorStop(3)
        time.sleep(t_time)

    def stop(self,t_time):
        self.MotorStop(0)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorStop(3)
        time.sleep(t_time)

    def set_servo_pulse(self,channel,pulse):
        pulse_length = 1000000
        pulse_length //= 60
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        self.pwm.setPWM(channel, 0, pulse)

    def set_servo_angle(self,channel,angle):
        angle=4096*((angle*11)+500)/20000
        self.pwm.setPWM(channel,0,int(angle))
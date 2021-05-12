import Adafruit_PCA9685


pwm=Adafruit_PCA9685.PCA9685()

pwm.set_pwm_freq(50)

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)

duty1=4096*((90*11)+500)/20000
pwm.set_pwm(13,0,int(duty1))
pwm.set_pwm(9,0,int(duty1))
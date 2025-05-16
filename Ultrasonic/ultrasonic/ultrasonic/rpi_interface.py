import time
import RPi.GPIO as GPIO


class RPi_HCS04(object):

    def __init__(self, trig, echo):
        self.TRIG = trig
        self.ECHO = echo

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

    def __del__(self):
        GPIO.cleanup([self.TRIG, self.ECHO])

    def measure_pulse_duration(self):
        GPIO.output(self.TRIG, GPIO.HIGH)
        # Setting TRIG high for 10 microseconds sends out ultrasonic sound pulse
        time.sleep(0.00001)
        GPIO.output(self.TRIG, GPIO.LOW)

        pulseStart = time.time()
        pulseStop = time.time()

        while GPIO.input(self.ECHO) == 0:
            pulseStart = time.time()

        while GPIO.input(self.ECHO) == 1:
            pulseStop = time.time()

        return pulseStop - pulseStart

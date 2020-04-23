from django.http import HttpResponse, Http404, HttpResponseRedirect, HttpResponseServerError, StreamingHttpResponse
from django.shortcuts import get_object_or_404, render
from django.urls import reverse

from django.views.decorators import gzip

#import roboclaw
from roboclaw_3 import Roboclaw
from time import sleep

#import cv2
import threading, time
#import serial, serial.tools.list_ports
#from pyfirmata import ArduinoMega, util
from time import sleep
from threading import Timer
import re
import RPi.GPIO as GPIO
import smbus
from smbus import SMBus
from PCA9685 import PWM
import serial

measWaitTime = 0
motorWaitTime = 0

ser=serial.Serial()
batteryVoltage = ""
roboclaw = Roboclaw("/dev/ttyS0", 38400)

def connectMotorSerial():
    global roboclaw
    
    motor_baudrate=38400
    print("connecting motors")
    roboclaw.Open()
        

def connectSerial():
    connectMotorSerial()
    global ser
    global batteryVoltage
    try:
        ser=serial.Serial("/dev/ttyACM0",9600) #change ACM number as found from ls /dev/tty/ACM*
        ser.baudrate=9600
    except:
        ser.close()
        batteryVoltage = -1

count = 0

print("Connecting to Serial")
connectSerial()


#roboclaw init
MCaddr1 = 0x80
MCaddr2 = 0x81

class Lidar_Lite():
    def __init__(self, address):
        self.address = address
        self.distWriteReg = 0x00
        self.statReadReg = 0x11
        self.distWriteVal = 0x04
        self.distReadReg1 = 0x8f
        self.distReadReg2 = 0x10
        self.velWriteReg = 0x04
        self.velWriteVal = 0x08
        self.velReadReg = 0x09
        self.quickTermReg = 0xE5
        self.quickTermVal = 0x08

    def connect(self, bus):
        try:
            self.bus = smbus.SMBus(bus)
            time.sleep(0.5)
            self.bus.write_byte_data(self.address, self.quickTermReg, self.quickTermVal);
            return 0
        except:
            return -1

    def writeAndWait(self, register, value):
        self.bus.write_byte_data(self.address, register, value);

    def readAndWait(self, register):
        res = self.bus.read_byte_data(self.address, register)
    #time.sleep(0.005)
        return res

    #removed line time.sleep(0.02) before return
    def readDistAndWait(self, register):
        return (self.readAndWait(register)|self.readAndWait(register+1)<<8)

    def getDistance(self):
        self.writeAndWait(self.distWriteReg, self.distWriteVal)
        dist = self.readDistAndWait(self.distReadReg2)
        return dist

lidar5 = Lidar_Lite(0x62)
connected5 = lidar5.connect(6)
lidar4 = Lidar_Lite(0x62)
connected4 = lidar4.connect(5)
lidar3 = Lidar_Lite(0x62)
connected3 = lidar3.connect(4)
lidar2 = Lidar_Lite(0x62)
connected2 = lidar2.connect(3)
lidar1 = Lidar_Lite(0x62)
connected1 = lidar1.connect(1)

meas = [0,0,0,0,0,0]


def getMeasurements():
    global batteryVoltage
    global rightSpeed
    global leftSpeed
    while True:
        try:
            ser.flushInput()
            batteryVoltage = ser.readline()
            batteryVoltage = ser.readline()
            batteryVoltage = float(re.findall('\d*\.?\d+', str(batteryVoltage, 'ascii'))[0])
            batteryVoltage = 0
        except:
            ser.close()
            batteryVoltage = -1
            connectSerial()
        try:
            meas[0] = lidar1.getDistance()
        except:
            meas[0] = -1
        try:
            meas[1] = lidar2.getDistance()
        except:
            meas[1] = -1
        try:
            meas[2] = lidar3.getDistance()
        except:
            meas[2] = -1
        try:
            meas[3] = lidar4.getDistance()
        except:
            meas[3] = -1
        try:
            meas[4] = lidar5.getDistance()
        except:
            meas[4] = -1

        try:
            #print("rightSpeed: ", rightSpeed)
            #print("leftSpeed: ", leftSpeed)
            rightwheelswrite(rightSpeed)
            leftwheelswrite(leftSpeed)
        except:
            connectMotorSerial()

        meas[5] = batteryVoltage*100
        time.sleep(measWaitTime)
        
leftSpeed = 0
rightSpeed = 0

def controlMotors():
    global rightSpeed
    global leftSpeed
    while True:
        try:
            #print("rightSpeed: ", rightSpeed)
            #print("leftSpeed: ", leftSpeed)
            rightwheelswrite(rightSpeed)
            leftwheelswrite(leftSpeed)
        except:
            connectMotorSerial()
        time.sleep(motorWaitTime)
        
PWMenabled = False

def enablePWM():
    global PWMenabled
    print("Enabling PWM")
    PWMenabled = True
    GPIO.output(PWM_OEpin, 1)


def disablePWM():
    global PWMenabled
    print("Disabling PWM")
    PWMenabled = False
    GPIO.output(PWM_OEpin, 0)

def zoomIn():
    GPIO.output(zoomOutPin, 0)
    GPIO.output(zoomInPin, 1)

def zoomOut():
    GPIO.output(zoomInPin, 0)
    GPIO.output(zoomOutPin, 1)

def zoomStop():
    GPIO.output(zoomInPin, 0)
    GPIO.output(zoomOutPin, 0)



motorToDutyRatio = 4
motorToDutyOffset = 4
leftFront = 4
rightFront = 5
leftBack = 6
rightBack = 7

tiltUpperLimit = 95
tiltLowerLimit= -15
tiltToDutyRatio = 8.1
tiltToDutyOffset = 5.6
tiltRange = tiltUpperLimit - tiltLowerLimit
tiltPin = 1

panUpperLimit = 360
panLowerLimit= -360
panToDutyRatio = 1.29
panToDutyOffset = 7.05
panRange = panUpperLimit - panLowerLimit
panPin = 0

flirToDutyRatio = 4
flirToDutyOffset = 4
flirColor= 3
flirZoom= 2

GPIO.setmode(GPIO.BCM)
PWM_OEpin = 4
GPIO.setup(PWM_OEpin, GPIO.OUT)

zoomInPin = 11
zoomOutPin = 9
GPIO.setup(zoomInPin, GPIO.OUT)
GPIO.setup(zoomOutPin, GPIO.OUT)
GPIO.output(zoomInPin, 0)
GPIO.output(zoomOutPin, 0)

pwm_frequency = 50
pwm_i2c_address = 0x40
pwm = PWM(SMBus(1), pwm_i2c_address)
pwm.setFreq(pwm_frequency)

print("Starting Lidar and Battery")
measThread = threading.Thread(target=getMeasurements)
measThread.daemon = True
measThread.start()

print("Starting Motor Control")
motorThread = threading.Thread(target=controlMotors)
motorThread.daemon = True
#motorThread.start()


def stopmotors():
    
    print('Stopping Motors')
    leftwheelswrite(0)
    rightwheelswrite(0)
    
    disablePWM()
    return

def leftwheelswrite(speed):
    if speed >= 0:
        roboclaw.ForwardM2(MCaddr1,speed)
        #roboclaw.ForwardM2(MCaddr2,speed)
    else:
        roboclaw.BackwardM2(MCaddr1,abs(speed))
        #roboclaw.BackwardM2(MCaddr2,abs(speed))
    return

def rightwheelswrite(speed):
    if speed >= 0:
        roboclaw.ForwardM1(MCaddr1,speed)
        #roboclaw.ForwardM1(MCaddr2,speed)
    else:
        roboclaw.BackwardM1(MCaddr1,abs(speed))
        #roboclaw.BackwardM1(MCaddr2,abs(speed))
    return

def setPanAngle(angle):
    global panToDutyRatio
    global panToDutyOffset
    pwm.setDuty(panPin, panToDutyRatio / 180 * -angle + panToDutyOffset)

def setTiltAngle(angle):
    global tiltToDutyRatio
    global tiltToDutyOffset
    pwm.setDuty(tiltPin, tiltToDutyRatio / 180 * angle + tiltToDutyOffset)

def setFlirRange(pin, value):
    global flirToDutyRatio
    global flirToDutyOffset
    pwm.setDuty(pin, flirToDutyRatio / 180 * value + flirToDutyOffset)


stopMotorTimer = threading.Timer(0.1, stopmotors, [])
stopMotorTimer.start()

def stoprecord(request):
    print('Stopping Rec')
    return HttpResponse('stopped')

def startrecord(request):
    print('Starting Rec')
    return HttpResponse('started')


stopMotorTimer = threading.Timer(0.1, stopmotors, [])
stopMotorTimer.start()

def command(request):
    if not(PWMenabled):
        enablePWM()
    global stopMotorTimer
    stopMotorTimer.cancel()
    global count
    global leftSpeed
    global rightSpeed
    count = count + 1
    text = request.POST['text']
    nums = [float(s) for s in re.findall(r"[+-]?\d+(?:\.\d+)?", text)]
    
    rightSpeed = int(100*nums[1])
    leftSpeed = int(100*nums[0])
    print(leftSpeed, rightSpeed)

    setPanAngle(nums[2])
    setTiltAngle(nums[3])

    if (nums[4]>0):
        zoomIn()
    elif (nums[4]<0):
        zoomOut()
    else:
        zoomStop()

    print(nums)

    stopMotorTimer = threading.Timer(1, stopmotors, [])
    stopMotorTimer.start()
    response = str(count)+'\n'+str(meas)
    print(response)
    return HttpResponse(response)


def index(request):
    #global activeStream
    #activeStream = False
    return render(request, 'control/index.html')

def random(request):
    #global activeStream
    #activeStream = False
    return render(request, 'control/random.html')


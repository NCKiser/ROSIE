from django.http import HttpResponse, Http404, HttpResponseRedirect, HttpResponseServerError, StreamingHttpResponse
from django.shortcuts import get_object_or_404, render
from django.urls import reverse

from django.views.decorators import gzip

#import cv2
import threading, time
import serial, serial.tools.list_ports
from pyfirmata import ArduinoMega, util
from time import sleep
from threading import Timer
import RPi.GPIO as GPIO

count = 0

portList = [port.device for port in serial.tools.list_ports.comports()]
print("port "+portList[0])


def stopmotors():
    global activeStream
    print('stopping motors')
    leftwheelswrite(0)
    rightwheelswrite(0)
    activeStream = False


board = ArduinoMega(portList[0])
print("connected to arduino")
board.digital[13].write(1)
tiltServo = board.get_pin('d:9:s')
# 120+-60
tiltServo.write(180)
panServo = board.get_pin('d:11:s')
# 84+-80
panServo.write(68)
frontLeftForward = board.get_pin('d:22:o')
frontLeftBackward = board.get_pin('d:23:o')
frontLeftPWM = board.get_pin('d:2:p')
frontRightForward = board.get_pin('d:24:o')
frontRightBackward = board.get_pin('d:25:o')
frontRightPWM = board.get_pin('d:3:p')
rearLeftForward = board.get_pin('d:26:o')
rearLeftBackward = board.get_pin('d:27:o')
rearLeftPWM = board.get_pin('d:4:p')
rearRightForward = board.get_pin('d:28:o')
rearRightBackward = board.get_pin('d:29:o')
rearRightPWM = board.get_pin('d:5:p')
stopMotorTimer = threading.Timer(0.1, stopmotors, [])
stopMotorTimer.start()

def leftwheelswrite(speed):
    mag = abs(speed)
    if (mag>0):
        sign = speed/mag
    else:
        sign = 0
    if (sign>0):
        frontLeftBackward.write(0)
        frontLeftForward.write(1)
        rearLeftBackward.write(0)
        rearLeftForward.write(1)
    elif (sign==0):
        frontLeftBackward.write(0)
        frontLeftForward.write(0)
        rearLeftBackward.write(0)
        rearLeftForward.write(0)
    else:
        frontLeftForward.write(0)
        frontLeftBackward.write(1)
        rearLeftForward.write(0)
        rearLeftBackward.write(1)
    frontLeftPWM.write(mag)
    rearLeftPWM.write(mag)


def rightwheelswrite(speed):
    mag = abs(speed)
    if (mag>0):
        sign = speed/mag
    else:
        sign = 0
    if (sign>0):
        frontRightBackward.write(0)
        frontRightForward.write(1)
        rearRightBackward.write(0)
        rearRightForward.write(1)
    elif (sign==0):
        frontRightBackward.write(0)
        frontRightForward.write(0)
        rearRightBackward.write(0)
        rearRightForward.write(0)
    else:
        frontRightForward.write(0)
        frontRightBackward.write(1)
        rearRightForward.write(0)
        rearRightBackward.write(1)
    frontRightPWM.write(mag)
    rearRightPWM.write(mag)



def stoprecord(request):
    print('Stopping Rec')
    return HttpResponse('stopped')

def startrecord(request):
    print('Starting Rec')
    return HttpResponse('started')

def command(request):
    global stopMotorTimer
    stopMotorTimer.cancel()
    global count
    count = count + 1
    #Get the variable text
    text = request.POST['text']
  #  print(text)
    nums = [float(s) for s in re.findall(r"[+-]?\d+(?:\.\d+)?", text)]
  #  print(nums)
    panAngle = nums[2]/-255*70+84
  #  print('pan: '+str(panAngle))
    panServo.write(panAngle)

    tiltAngle = nums[3]/-255*60+120
  #  print('tilt: '+str(tiltAngle))
    tiltServo.write(tiltAngle)

    leftSpeed = nums[0]/255
    leftwheelswrite(leftSpeed)
    
    rightSpeed = nums[1]/255
    rightwheelswrite(rightSpeed)
    

    stopMotorTimer = threading.Timer(1, stopmotors, [])
    stopMotorTimer.start()

    response = str(count)+ '\nLeft: '+str(round(leftSpeed, 2))+ '\nRight: ' +str(round(rightSpeed, 2))
    #Send the response
    return HttpResponse(response)


def index(request):
    #global activeStream
    #activeStream = False
    return render(request, 'control/index.html', {'quality': 30})

def indexQ(request, quality):
    #global activeStream
    #activeStream = False
    return render(request, 'control/index.html', {'quality': quality})

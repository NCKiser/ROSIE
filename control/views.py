from django.http import HttpResponse, Http404, HttpResponseRedirect, HttpResponseServerError, StreamingHttpResponse
from django.shortcuts import get_object_or_404, render
from django.urls import reverse

from django.views.decorators import gzip

import cv2, threading, time
import serial, serial.tools.list_ports
from pyfirmata import ArduinoMega, util
from time import sleep
from threading import Timer
import re

#TODO Video streaming _not_ with Django, use a cdn or red5 or....something. Django not made for video.
#TODO re-write the whole thing in NodeJS??...loose support for Firmata
#TODO Apache-red5-ffmpeg-django stack. I know, its a lot, but its what we need to do.

streamLocation = 'http://root:pass@192.168.1.150/mjpg/video.mjpg'
#streamLocation = "/dev/video0"
#videoLocation = '/home/rosie/Videos/Large.mp4'

count = 0
streamCount = 0

activeStream = False
videoQuality = 30

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


"""
def comconnect():
    global havePort, ser
    havePort = 0;
    portList = [port.device for port in serial.tools.list_ports.comports()]
    print(portList)
    if (portList):
        print("have port" + portList[0])
        havePort = 1;
        ser = serial.Serial(portList[0], 9600, timeout=1)
"""


class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture(streamLocation)
        (self.grabbed, self.frame) = self.video.read()
        threading.Thread(target=self.update, args=()).start()

    def __del__(self):
        self.video.release()

    def get_frame(self,quality):
        image = self.frame
        ret, jpeg = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return jpeg.tobytes()

    def update(self):
        while True:
            (self.grabbed, self.frame) = self.video.read()

cam = VideoCamera()


def gen(camera, quality):
    global streamCount
    streamCount = streamCount+1
    print('startStream+++++++++++++++++++++++++++++++++++++++++'+str(streamCount))
    global videoQuality
    videoQuality = quality
    while activeStream:
        frame = cam.get_frame(quality)
        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
    streamCount = streamCount-1
    print('stopStream------------------------------------------'+str(streamCount))

@gzip.gzip_page
def stream(request):
    global activeStream
    activeStream = True
    try:
        return StreamingHttpResponse(gen(VideoCamera(),30), content_type="multipart/x-mixed-replace;boundary=frame")
    except:  # This is bad! replace it with proper handling
        pass

def custom(request, quality):
    global activeStream
    activeStream = True
    try:
        return StreamingHttpResponse(gen(VideoCamera(),quality), content_type="multipart/x-mixed-replace;boundary=frame")
    except:  # This is bad! replace it with proper handling
        pass

def index(request):
    global activeStream
    activeStream = False
    return render(request, 'control/index.html', {'quality': 30})

def indexQ(request, quality):
    global activeStream
    activeStream = False
    return render(request, 'control/index.html', {'quality': quality})

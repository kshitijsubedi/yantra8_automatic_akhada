import cv2
import numpy as np 
import argparse
from time import sleep
import serial
import os
from imutils.video import VideoStream
from pyzbar import pyzbar
import datetime
import kimutils
import csv
from gpiozero import Buzzer

bz=Buzzer(3)

qrfile=open('qr.csv','a')

tt=["fire.png","left.png","right.png"]


try:
  ser = serial.Serial('/dev/ttyUSB0', 9600)
except:
  ser = serial.Serial('/dev/ttyUSB1', 9600)

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv")
args = vars(ap.parse_args())

csv = open(args["output"], "w")
qrfound = set()

print("Lets Start Camera")

vs=VideoStream(usePiCamera=True).start()
sleep(2.0)

w=200
h=200


def ctemplate(image):

  template = cv2.resize(image, (200,200), interpolation = cv2.INTER_AREA)
  #template= cv2.Canny(template,50,200)
  return template

def barcode(frame):
  barcodes = pyzbar.decode(frame)
  for barcode in barcodes:
  (x, y, w, h) = barcode.rect
  cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
  barcodeData = barcode.data.decode("utf-8")
  barcodeType = barcode.type
  text = "{} ({})".format(barcodeData, barcodeType)
  cv2.putText(frame, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
  print("QR Found :: \n")
  print(text)
  qrfile.write(text)
  qrfile.write("\n")

while True:
  frame = vs.read()
  frame = cv2.flip(frame, 0)
  frame = cv2.resize(frame, (400,400), interpolation = cv2.INTER_AREA)
  barcode(frame)
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  found = None

  for i in tt:
    timage=cv2.imread(i)
    template = ctemplate(timage)
    #frame=cv2.Canny(frame,50,200)
    #cv2.imshow("",frame)
    cv2.imshow("..",template)
    
    res = cv2.matchTemplate(frame,template,cv2.TM_CCOEFF_NORMED)
    threshold = 0.6
    loc = np.where( res >= threshold)
    print(loc)
    
    for pt in zip(*loc[::-1]):
      print(i)
      cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
      if (pt[0] > 1 and c==0) :
      c=1
      if (i == 'fire.png'):
        bz.on()
        ser.write(b'H')
      if (i == 'left.png'):
        ser.write(b'L')
      if (i == 'right.png'):
        ser.write(b'R')

	cv2.imshow("",frame)
	c=0
  k = cv2.waitKey(10) & 0xff
  if k == 27:
	break

qrfile.close()
cv2.destroyAllWindows()
vs.stop()
ser.close()
#ok

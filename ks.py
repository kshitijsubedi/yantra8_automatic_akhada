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

tt=["fire.png","left.png","right.png"]


# try:
# 	ser = serial.Serial('/dev/ttyACM0', 9600)
# except:
# 	ser = serial.Serial('/dev/ttyACM1', 9600)

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv")
args = vars(ap.parse_args())

csv = open(args["output"], "w")
qrfound = set()

print("Lets Start Camera")

vs=VideoStream(usePiCamera=True).start()
sleep(2.0)

w=100
h=100
# def temp(templates):

#     template = cv2.imread(args["template"])
#     template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
#     template = cv2.Canny(template, 50, 200)
#     cv2.imshow("Template", template)

def ctemplate(image):

  template = cv2.imread(image,0)
  template = cv2.resize(template, (100,100), interpolation = cv2.INTER_AREA)
  print(template.shape)
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
    with open(r'qr', 'a') as f:
      writer = csv.writer(f)
      writer.writerow(text)

    # if barcodeData not in qrfound:
    # 	csv.write("{},{}\n".format(datetime.datetime.now(),
    # 		barcodeData))
    # 	csv.flush()
    # 	qrfound.add(barcodeData)

while True:
    frame = vs.read()
    frame = imutils.resize(frame,width=400)
    barcode(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = None

    for i in tt:
      timage=cv2.imread(i)
      template = ctemplate(timage)

      res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
      threshold = 0.7
      loc = np.where( res >= threshold)
      for pt in zip(*loc[::-1]):
        cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)



    

    # for scale in np.linspace(0.2, 1.0, 20)[::-1]:
    #     resized = kimutils.resize(gray, width = int(gray.shape[1] * scale))
    #     r = gray.shape[1] / float(resized.shape[1])
    #     if resized.shape[0] < tH or resized.shape[1] < tW:
    #         break
    #     edged = cv2.Canny(resized, 50, 200)
    #     result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
    #     (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
    #     if found is None or maxVal > found[0]:
    #         found = (maxVal, maxLoc, r)

    # (_, maxLoc, r) = found
    # (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    # (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))

    # cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
    # cv2.imshow("Image", frame)


    cv2.imshow("",frame)
    k = cv2.waitKey(10) & 0xff
    if k == 27:
      break

csv.close()
cv2.destroyAllWindows()
vs.stop()
#ok
import cv2
import numpy as np
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy, Circle, PCMD
from olympe.messages.move import extended_move_by,extended_move_to
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged


fbRange=[6200,6800]
pid=[0.4,0.4,0]
w,h=360,240
fb=0

def findFace(img):
    faceCascade=cv2.CascadeClassifier('')
    imgGray=cv2.cvtColor(img,cv2.COLOR_BayerGBR2GRAY)
    faces=faceCascade.detectMultiScale(imgGray,1.2,8)
    myFaceListC=[]
    myFaceListArea=[]
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,225),2)
        cx=x+w//2
        cy=y+h//2
        area=w*h
        myFaceListC.append([cx,cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) !=0:
        i=myFaceListArea.index(max(myFaceListArea))
        return img,[myFaceListC[i],myFaceListArea[i]]
    else:return img,[[0,0],0]
def trackFace(drone,info,w,pid,pError):
    x,y=info[0]
    area=info[1]
    fb=0

    error=x-w//2
    speed=pid[0]*error+pid[1]*(error-pError)
    speed=int(np.clip(speed,-100,100))
    if area>fbRange[0] and area<fbRange[1]:
        fb=0
    elif area>fbRange[1]:
        fb=-0.2
    elif area<fbRange[0] and area !=0:
        fb=0.2

    if x==0:
        speed=0
        error=0

    drone(extended_move_by(fb, 0, 0, 0, 1, 0.5, 0.5)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    return error

def tracking(img):
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    pError=0
    while True:
        img=cv2.resize(img,(w,h))
        img,info=findFace(img)
        pError=trackFace(drone,info,w,pid,pError)
        print('cneter',info[0],'area',info[1])
        cv2.imshow('Output',img)
        cv2.waitKey(1)




    
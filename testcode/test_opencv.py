import cv2
import numpy as np
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy, Circle, PCMD
from olympe.messages.move import extended_move_by,extended_move_to
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
import argparse
from olympe import Pdraw, PDRAW_YUV_FORMAT_I420, PDRAW_YUV_FORMAT_NV12, PdrawState
import time
import sys
import os

"""
ドローンに映し出される仕組みを考える
"""

fbRange=[6200,6800]
pid=[0.4,0.4,0]
w,h=360,240
fb=0
DRONE_IP='192.168.42.1'

def findFace(img):
    faceCascade=cv2.CascadeClassifier('haarcascade_frontalface_alt')

    # カスケードファイルが存在するか
    if os.path.isfile(faceCascade) is False:
        print('ファイルが存在しない')
        return
    # imgGray=cv2.cvtColor(img,cv2.COLOR_BayerGBR2GRAY)
    faces=faceCascade.detectMultiScale(img,1.2,8)
    myFaceListC=[]
    myFaceListArea=[]
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,225),2)
        cx=x+w//2
        cy=y+h//2
        area=w*h
        myFaceListC.append([cx,cy])
        myFaceListArea.append(area)

    # img = cv2.cvtColor(img, cv2.CV_GRAY2BGR)
    if len(myFaceListArea) !=0:
        i=myFaceListArea.index(max(myFaceListArea))
        return img,[myFaceListC[i],myFaceListArea[i]]
    else:return img,[[0,0],0]



def yuv_frame_cb(yuv_frame):
    """
    This function will be called by Olympe for each decoded YUV frame.
        :type yuv_frame: olympe.VideoFrame
    """
    # the VideoFrame.info() dictionary contains some useful information
    # such as the video resolution
    info = yuv_frame.info()
    height, width = info["yuv"]["height"], info["yuv"]["width"]

    # yuv_frame.vmeta() returns a dictionary that contains additional
    # metadata from the drone (GPS coordinates, battery percentage, ...)

    # convert pdraw YUV flag to OpenCV YUV flag
    cv2_cvt_color_flag = {
        PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
        PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
    }[info["yuv"]["format"]]

    # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
    # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

    # Use OpenCV to convert the yuv frame to RGB
    cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
    img = cv2.cvtColor(cv2frame, cv2.COLOR_RGB2GRAY)
    img,info=findFace(img)

    # Use OpenCV to show this frame
    cv2.imshow("Olympe Pdraw Example", cv2frame)
    cv2.waitKey(1)  # please OpenCV for 1 ms...


def main(argv):
    parser = argparse.ArgumentParser(description="Olympe Pdraw Example")
    parser.add_argument(
        "-u",
        "--url",
        default="rtsp://192.168.42.1/live",
        help=(
            "Media resource (rtsp:// or file://) URL.\n"
            "See olympe.Pdraw.play documentation"
        ),
    )
    parser.add_argument("-m", "--media-name", default="DefaultVideo")
    args = parser.parse_args(argv)
    pdraw = Pdraw()
    pdraw.set_callbacks(raw_cb=yuv_frame_cb)
    pdraw.play(url=args.url, media_name=args.media_name)
    assert pdraw.wait(PdrawState.Playing, timeout=5)
    if args.url.endswith("/live"):
        # Let's see the live video streaming for 10 seconds
        time.sleep(100)
        pdraw.close()
        timeout = 5
    else:
        # When replaying a video, the pdraw stream will be closed automatically
        # at the end of the video
        # For this is example, this is the replayed video maximal duration:
        timeout = 90
    assert pdraw.wait(PdrawState.Closed, timeout=timeout)
    pdraw.dispose()

if __name__ == '__main__':
    main(sys.argv[1:])


    
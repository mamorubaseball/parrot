import math
import sys

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, NavigateHome
import threading
import time
import queue
import cv2
import logging


from olympe.messages.move import extended_move_by, extended_move_to
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages import gimbal
import numpy as np
import pandas as pd
import os
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, MaxAltitude




class OlympeStreaming(threading.Thread):
    def __init__(self, drone):
        self.drone = drone
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.frame_num = 0
        self.renderer = None
        self.w = 1280
        self.h = 720
        self.go_ahead=0
        self.CX=self.w//2
        self.CY=self.h//2
        self.max_altitude = 0.5
        self.drone(MaxAltitude(self.max_altitude)).wait()
        self.sita=0
        self.flag=False
        self.log_df = pd.DataFrame(columns=['sita','slide','go'])
        # self.drone(moveBy(0,0,0.5,0))
        super().__init__()
        super().start()

    def set_gimbal(self):
        self.drone(gimbal.set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="none",  # None instead of absolute
            yaw=0.0,
            pitch_frame_of_reference="absolute",
            pitch=-90.0,
            roll_frame_of_reference="none",  # None instead of absolute
            roll=0.0,
        )).wait()

    def start(self):
        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming もしかしたらここでスレッドを立てているのかもしれない
        self.drone.streaming.start()
        # self.renderer = PdrawRenderer(pdraw=self.drone.streaming)

    def stop(self):
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        self.drone.streaming.stop()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        pass

    def left(self,x):
        os.system('python3 right.py -x {}'.format(x))


    def right(self,x):
        os.system('python3 right.py -x {}'.format(x))

    def move_slide(self,x):
        os.system('python3 move_slide.py -x {}'.format(x))

    def rotation(self,sita):
        os.system('python3 rotation.py -r {}'.format(sita))

    def go(self,distance):
        os.system('python3 go.py -m {}'.format(distance))

    def make_sita(self,x1,y1,x2,y2):
        sita = math.acos(abs(y2-y1)/((x1-x2)**2+(y1-y2)**2)**(1/2))
        return sita




    def land(self):
        os.system('python3 landing.py')


    def display_frame(self, yuv_frame, x=None):
        print('display_funcが呼び出されました')
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()

        height, width = (  # noqa
            info["raw"]["frame"]["info"]["height"],
            info["raw"]["frame"]["info"]["width"],
        )

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)
        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[yuv_frame.format()]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # face_cascade_path = 'cascade.xml'
        # faceCascade = cv2.CascadeClassifier(face_cascade_path)
        # ここの処理がものすごくCPUを消費しているのでは？？動画が遅い理由はなんだ？
        # faces = faceCascade.detectMultiScale(cv2frame, scaleFactor=1.2, minNeighbors=2)



        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        ###########緑色に変換する###################
        #BGR色空間からHSV空間への変換
        hsv = cv2.cvtColor(cv2frame,cv2.COLOR_BGR2HSV)

        #色検出しきい値(緑の色相範囲30~90)
        lower = np.array([30, 64, 0])
        upper = np.array([90, 255, 255])

        fram_msk=cv2.inRange(hsv,lower,upper)
        # 論理演算で色検出
        cv2frame = cv2.bitwise_and(cv2frame, cv2frame, mask=fram_msk)

        ###########################################
        # cv2frame = cv2.cvtColor(cv2frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2frame, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges,
                                rho=1,
                                theta=np.pi / 360,
                                threshold=100,
                                minLineLength=200,
                                maxLineGap=6)
        myFaceListC = []
        myFaceListArea = []
        mylineList=[]
        
        # cx,cy は顔の中心
        if lines is None:
            self.flag=False
            pass
        else:
            print('kenshutu')
            print(lines[0][0])
            x1,y1,x2,y2=lines[0][0][0],lines[0][0][1],lines[0][0][2],lines[0][0][3]
            mylineList.append([x1,y1,x2,y2])
            cv2.line(cv2frame,(x1,y1),(x2,y2),(0,0,225),2)
            cx=(x1+x2)//2
            cy=(y1+y2)//2

            #########回転###########
            if y1<=y2:
                self.sita=self.make_sita(x1,y1,x2,y2)
            else:
                self.sita=self.make_sita(x2,y2,x1,y1)
            #15度以上傾くと回転する
            if self.sita>=math.pi/12:
                self.rotation(self.sita)
                self.log_df.append({'sita' : self.sita ,'slide' : 'None','go':0} , ignore_index=True)

            #########並行移動##########
            distance=cx-self.CX

            if distance>30 and distance<-30:
                pass
            else:

                if distance>60:
                    self.move_slide(0.3)
                    self.log_df.append({'sita': 'None', 'slide': 0.3,'go':0}, ignore_index=True)
                    time.sleep(3)
                elif distance>40:
                    self.move_slide(0.2)
                    self.log_df.append({'sita': 'None', 'slide': 0.2,'go':0}, ignore_index=True)

                    time.sleep(3)
                elif distance>-40:
                    self.move_slide(-0.2)
                    self.log_df.append({'sita': 'None', 'slide': -0.2,'go':0}, ignore_index=True)

                    time.sleep(3)
                else:
                    self.move_slide(-0.3)
                    self.log_df.append({'sita': 'None', 'slide': -0.3,'go':0}, ignore_index=True)

                    time.sleep(3)

            if self.sita>=math.pi/12 and (distance>30 and distance<-30):
                self.go_ahead=0.5
                self.go(self.go_ahead)
                self.log_df.append({'sita' : self.sita ,'slide' : 'None','go':self.go_ahead} , ignore_index=True)

        # # img⇛カラーに変換
        # img = cv2.cvtColor(cv2frame, cv2.CV_GRAY2BGR)
        # if len(myFaceListArea) != 0:
        #     i = myFaceListArea.index(max(myFaceListArea))
        #     info = [myFaceListC[i], myFaceListArea[i]]
        # else:
        #     info = [[0, 0], 0]
        # cx = info[0][0]
        # cy = info[0][1]
        # error = self.w // 2 - cx
        # 
        # if error > 0:
        #     rotation = - math.pi // 4
        # else:
        #     rotation = math.pi // 4

        #self.drone(extended_move_by(0, 0, 0, rotation, 1, 0.5, 0.5)
        #          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

        cv2.imshow("cv2_show", cv2frame)
        cv2.waitKey(1)
        self.log_df.to_csv('log.csv')
        print('='*10+'log'+'='*10)
        print(self.log_df)
        print('=='*10)

    def run(self):
        print(threading.enumerate())
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.display_frame(yuv_frame)
                except Exception as e:
                    print(e)
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
        self.log_df.to_csv('log.csv')


logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # eventually IP will be specified depending on what drone is chosen
    IP = "192.168.42.1"
    drone = olympe.Drone(IP)
    drone.connect()

    streamer = OlympeStreaming(drone)
    streamer.set_gimbal()
    time.sleep(4)

    streamer.start()
    drone(TakeOff()).wait().success()
    ### Flight commands here ###
    time.sleep(300)

    streamer.stop()

    drone(Landing()).wait().success()
    drone.disconnect()

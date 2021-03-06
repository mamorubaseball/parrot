import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, NavigateHome
import threading
import time
import queue
import cv2
import logging
import csv
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import time
import cv2
import numpy as np
import numpy
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, MaxAltitude
from olympe.messages.move import extended_move_by,extended_move_to
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged



'''
min_threadingを使って、飛行しながら画像認識
'''

class OlympeStreaming(threading.Thread):
    def __init__(self, drone):
        self.drone = drone
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.frame_num = 0
        self.renderer = None
        self.max_altitude = 2.0
        self.drone(MaxAltitude(self.max_altitude)).wait()
        self.w = 360
        self.h = 240
        self.pid = [0.4, 0.4, 0]
        self.pError = 0

        super().__init__()
        super().start()

    def start(self):
        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
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

    def display_frame(self, yuv_frame):
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

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        img = cv2.cvtColor(cv2frame, cv2.COLOR_RGB2GRAY)
        img, info = self.Find_Detection(img)
        cv2.imshow("cv2_show", img)
        cv2.waitKey(1)

    def tracking(self, info, pError):
        cx, cy = info[0]
        area = info[1]
        print('=======面積{}========'.format(area))
        W, H = self.w, self.h
        pid = self.pid
        fbRange = [6200, 6800]
        fb = 0

        ###この辺のコードの理解がまだ少し足りていない###
        error = cx - W // 2
        speed = pid[0] * error + pid[1] * (error - pError)
        speed = int(np.clip(speed, -100, 100))

        # move baack and forward
        if area > fbRange[0] and area < fbRange[1]:
            fb = 0
        elif area > fbRange[1]:
            fb = -0.05
        elif area < fbRange[0] and area != 0:
            fb = 0.05

        # move rotation
        '''
        面積からドローンと物体までの距離を求め、回転させる角度の計算を行う
        '''
        if error:
            if speed > 0:
                rotation = math.pi // 4
            else:
                rotation = -math.pi // 4

        self.drone(extended_move_by(fb, 0, 0, rotation, 1, 0.5, 0.5)
                   >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        return error

    def Find_Detection(self, img):
        # shift+右クリックでパスのコピーwinとlinux
        # face_cascade_path='C:\Users\manak\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu18.04onWindows_79rhkp1fndgsc\LocalState\rootfs\home\manaki\awesome\mamoru\parrot2\haarcascade_frontalface_alt.xml'
        face_cascade_path = 'haarcascade_frontalface_alt.xml'
        wall_lack_cascade_path = ''

        # カスケードファイルが存在するか
        if os.path.isfile(face_cascade_path) is False:
            print('ファイルが存在しない')
            return

        faceCascade = cv2.CascadeClassifier(face_cascade_path)
        imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # imgGray = img
        faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)
        myFaceListC = []
        myFaceListArea = []
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 225), 2)
            cx = x + w // 2
            cy = y + h // 2
            area = w * h
            myFaceListC.append([cx, cy])
            myFaceListArea.append(area)
        # img⇛カラーに変換
        # img = cv2.cvtColor(img, cv2.CV_GRAY2BGR)
        if len(myFaceListArea) != 0:
            i = myFaceListArea.index(max(myFaceListArea))
            return img, [myFaceListC[i], myFaceListArea[i]]
        else:
            return img, [[0, 0], 0]

    def run(self):
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


logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # eventually IP will be specified depending on what drone is chosen
    IP = "192.168.42.1"
    drone = olympe.Drone(IP)
    drone.connect()
    drone(TakeOff()).wait().success()
    streamer = OlympeStreaming(drone)
    streamer.start()
    ### Flight commands here ###
    time.sleep(300)

    streamer.stop()

    drone(Landing()).wait().success()
    drone.disconnect()

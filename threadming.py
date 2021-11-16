#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.

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

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt, MaxAltitude
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.video.renderer import PdrawRenderer
from olympe.messages.move import extended_move_by,extended_move_to


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")


class StreamingExample:
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.tempd, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ["fps", "bitrate"]
        )
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.renderer = None
        self.max_altitude = 2.0
        self.drone(MaxAltitude(self.max_altitude)).wait()
        self.w=360
        self.h=240
        self.pid=[0.4,0.4,0]
        self.pError=0



    def start(self):
        # Connect the the drone
        self.drone.connect()

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.streaming.set_output_files(
            video=os.path.join(self.tempd, "streaming.mp4"),
            metadata=os.path.join(self.tempd, "streaming_metadata.json"),
        )

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
        self.renderer = PdrawRenderer(pdraw=self.drone.streaming)

    def stop(self):
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        self.drone.streaming.stop()
        self.drone.disconnect()
        self.h264_stats_file.close()

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
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["is_sync"]):
            while len(self.h264_frame_stats) > 0:
                start_ts, _ = self.h264_frame_stats[0]
                if (start_ts + 1e6) < frame_ts:
                    self.h264_frame_stats.pop(0)
                else:
                    break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = 8 * sum(map(lambda t: t[1], self.h264_frame_stats))
            self.h264_stats_writer.writerow({"fps": h264_fps, "bitrate": h264_bitrate})

    def show_yuv_frame(self, window_name, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        windowSize = (640,480)

        height, width = (  # noqa
            info["raw"]["frame"]["info"]["height"],
            info["raw"]["frame"]["info"]["width"],
        )

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        # import cv2
        cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[yuv_frame.format()]
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        img = cv2.cvtColor(cv2frame, cv2.COLOR_RGB2GRAY)
        img, info = self.Find_Detection(img)
        
        sdl2.ext.init()
        window = sdl2.ext.Window("test", size=windowSize)
        window.show()
        windowSurf = sdl2.SDL_GetWindowSurface(window.window)
        windowArray = sdl2.ext.pixels3d(windowSurf.contents)

        while True:
            #keep reading to have a live feed from the cam
            junk,image = vc.read()
            image = numpy.insert(image,3,255,axis=2) #add alpha
            image = numpy.rot90(image) #rotate dims
            numpy.copyto(windowArray, image)
            window.refresh()
        
        
        
        
        self.pError = self.tracking(info, pError=self.pError)
        cv2.imshow(window_name, img)
        cv2.waitKey(1)  # please OpenCV for 1 ms...
        
    def tracking(self,info,pError):
        cx, cy = info[0]
        area = info[1]
        print('=======面積{}========'.format(area))
        W,H=self.w,self.h
        pid=self.pid
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
            if speed>0:
                rotation = math.pi//4
            else:rotation = -math.pi//4

        self.drone(extended_move_by(fb, 0, 0, rotation, 1, 0.5, 0.5)
              >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        return error

    def Find_Detection(self,img):
        # shift+右クリックでパスのコピーwinとlinux
        # face_cascade_path='C:\Users\manak\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu18.04onWindows_79rhkp1fndgsc\LocalState\rootfs\home\manaki\awesome\mamoru\parrot2\haarcascade_frontalface_alt.xml'
        face_cascade_path = 'haarcascade_frontalface_alt.xml'
        wall_lack_cascade_path = ''

        # カスケードファイルが存在するか
        if os.path.isfile(face_cascade_path) is False:
            print('ファイルが存在しない')
            return

        faceCascade = cv2.CascadeClassifier(face_cascade_path)
        # imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        imgGray = img
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
        #img = cv2.cvtColor(img, cv2.CV_GRAY2BGR)
        if len(myFaceListArea) != 0:
            i = myFaceListArea.index(max(myFaceListArea))
            return img, [myFaceListC[i], myFaceListArea[i]]
        else:
            return img, [[0, 0], 0]

    def fly(self):
        # Takeoff, fly, land, ...
        print("Takeoff if necessary...")
        self.drone(TakeOff()).wait().success()
        self.drone(MaxTilt(40)).wait().success()
        print("Landing...")
        time.sleep(5)
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("Landed\n")

    def replay_with_vlc(self):
        # Replay this MP4 video file using VLC
        mp4_filepath = os.path.join(self.tempd, "streaming.mp4")
        subprocess.run(shlex.split(f"vlc --play-and-exit {mp4_filepath}"), check=True)


def test_streaming():
    streaming_example = StreamingExample()
    # Start the video stream
    streaming_example.start()
    # Perform some live video processing while the drone is flying
    streaming_example.fly()
    # Stop the video stream
    streaming_example.stop()
    # Recorded video stream postprocessing
    # streaming_example.replay_with_vlc()


if __name__ == "__main__":
    test_streaming()


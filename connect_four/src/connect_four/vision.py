#!/usr/bin/python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import numpy as np
import threading

from copy import deepcopy
from os import system

import rospy

import cv2

from cv2 import cv
from cv_bridge import CvBridge

from geometry_msgs.msg import (
  PolygonStamped,
)

from sensor_msgs.msg import (
  Image,
)

from std_msgs.msg import (
  String,
)

import baxter_interface


class ConnectFourVision(object):
    def __init__(self, limb):
        # Start camera
        self._side = limb
        self._camera_name = self._side + "_hand_camera"
        print ("Opening " + self._camera_name + "...")
        try:
            self._head_camera = baxter_interface.CameraController("head_camera")
            print ("Attempting to turn off the head camera...")
            self._head_camera.close()
        except Exception:
            pass
        self._camera = baxter_interface.CameraController(self._camera_name)
        self._camera.open()
        self._camera.resolution = [1280, 800]
        self._camera.gain = 40

        self.grid = [[0 for _i in range(7)] for _j in range(6)]
        self.cv_image = None
        self._bridge = CvBridge()

        self.yellow_sample = ()
        self.red_sample = ()
        self.blue_sample = ()

        self.yellows = None
        self.reds = None
        self.blues = None

        self._roi_points = [[100, 100], [200, 100], [200, 200], [100, 200]]
        self._roi_move = False
        self._point_selected = -1
        self._gain_slider = 40
        self._red_thresh = 100
        self._yellow_thresh = 100
        self._slider_time = rospy.Time.now()
        self._gain_set = False
        self._text = ['X', 'Y', 'R', 'G', 'B']
        self._pixel = dict()
        for label in self._text:
            self._pixel[label] = 0.0
        self._vector = dict()
        self._grid = [[0 for _i in range(7)] for _j in range(6)]
        self._pnts = [[0 for i in range(8)] for j in range(7)]
        self._user_cnt = 0
        self._baxter_cnt = 0

        # initialize images
        self._np_image = np.zeros((300, 300, 3), np.uint8)
        self._image_grid = np.zeros((300, 300, 3), np.uint8)
        self._yellow = np.zeros((300, 300), np.uint8)
        self._red = np.zeros((300, 300), np.uint8)
        self._projected = np.zeros((300, 300, 3), np.uint8)

        self.subLock = threading.Lock()

        camera_topic = '/cameras/' + self._camera_name + '/image'
        _camera_sub = rospy.Subscriber(
            camera_topic,
            Image,
            self._on_camera)

        roi_topic = '/connect_four/localize/grid_pixels'
        _roi_sub = rospy.Subscriber(
            roi_topic,
            PolygonStamped,
            self._on_roi)

        board_state_topic = '/vision/connect_four_state'
        self._board_state_pub = rospy.Publisher(
            board_state_topic,
            String, queue_size=10)

        print 'All set! Starting to process images!'
        self._process_images()

    def _show_image(self):
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()

        for idx, points in enumerate(self._roi_points):
            cv2.circle(local_image, (points[0], points[1]), 5, (255, 0, 0), 2)

        cv2.polylines(local_image, np.int32([np.array(self._roi_points)]),
                      1, (0, 255, 0), 2)

        cv2.imshow("Connect Four RGB", local_image)

        cv.SetMouseCallback("Connect Four RGB", self._on_mouse_click, 0)
        cv.CreateTrackbar("Gain", "Connect Four RGB", self._gain_slider,
                          100, self._on_gain_slider)
        cv.CreateTrackbar("Red Threshold", "Connect Four RGB",
                          self._red_thresh, 500, self._on_red_slider)
        cv.CreateTrackbar("Yellow Threshold", "Connect Four RGB",
                          self._yellow_thresh, 500, self._on_yellow_slider)
        cv.WaitKey(3)

    def _process_images(self):
        while not rospy.is_shutdown():
            # gain changed from slider settled
            if (rospy.Time.now() - self._slider_time > rospy.Duration(3.0)
                and self._gain_set == True):
                self._gain_set = False
                print 'Setting GAIN!'
                self._camera.gain = self._gain_slider
            # process red/yellow image
            self._show_image()
            self._project_roi()
            self._filter_yellow()
            self._filter_red()
            self._process_colors(deepcopy(self._red), deepcopy(self._yellow))
            self._update_image_grid()

#             # publish state
            self._pub_state()
            rospy.sleep(0.1)

    def _process_colors(self, red, yellow):
        # look down each column building up from bottom
        self._grid = [[0 for _i in range(7)] for _j in range(6)]
        self._image_grid = deepcopy(self._projected)
        self._user_cnt = 0
        self._baxter_cnt = 0
        for col in xrange(7):
            cur_row = True
            x_offset = 42 * col
            # Look from the bottom up checking if piece is there
            for row in xrange(5, -1, -1):
                if cur_row == True:
                    y_offset = 50 * row
                    red_cnt = 0
                    yellow_cnt = 0
                    # look though each pixel in current grid location
                    if len(yellow) != 300 or len(red) != 300:
                        print 'BAILING - IMAGE SIZE IS UNEXPECTED'
                        return

                    for y in xrange(50):
                        for x in xrange(42):
                            if yellow[y + y_offset, x + x_offset] == 255:
                                yellow_cnt += 1
                            elif red[y + y_offset, x + x_offset] == 255:
                                red_cnt += 1

                    if yellow_cnt > self._yellow_thresh:
                        cv2.putText(self._image_grid,
                                    '2',
                                    (x_offset + 15, y_offset + 30),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                    1, (0, 255, 255)
                        )
                        self._grid[row][col] = 2
                        self._user_cnt += 1
                    elif red_cnt > self._red_thresh:
                        cv2.putText(self._image_grid,
                                    '1',
                                    (x_offset + 15, y_offset + 30),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                    1,
                                    (0, 0, 255)
                        )
                        self._grid[row][col] = 1
                        self._baxter_cnt += 1
                    else:
                        cur_row = False

    def _update_image_grid(self):
        for idx in xrange(1, 6):
            cv2.line(self._image_grid, (42 * idx, 0), (42 * idx, 300),
                     (0, 255, 0), 1)
            cv2.line(self._image_grid, (0, 50 * idx), (300, 50 * idx),
                     (0, 255, 0), 1)
            cv2.line(self._image_grid, (42 * 6, 0), (42 * 6, 300),
                     (0, 255, 0), 1)
            cv2.imshow('Board State', self._image_grid)

    def _project_roi(self):
        warped_in = np.float32([np.array(self._roi_points)])
        project_out = np.float32([[0, 0], [300, 0], [300, 300], [0, 300]])
        M = cv2.getPerspectiveTransform(warped_in, project_out)
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()
        self._projected = cv2.warpPerspective(local_image, M, (300, 300))

    def _filter_yellow(self):
        # Finds yellow colors in HSV space
        hsv = cv2.cvtColor(self._projected, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 60, 60])
        upper_yellow = np.array([45, 255, 255])
        self._yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.imshow('Yellow', self._yellow)

    def _filter_red(self):
        # Finds red colors in HSV space
        hsv = cv2.cvtColor(self._projected, cv2.COLOR_BGR2HSV)
        lower_red = np.array([165, 60, 60])
        upper_red = np.array([180, 255, 255])
        self._red = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow('Red', self._red)

    def _pub_state(self):
        state = dict()
        state['baxter_count'] = self._baxter_cnt
        state['user_count'] = self._user_cnt
        state['board'] = self._grid
        self._board_state_pub.publish(str(state))

    def _on_roi(self, data):
        if data.polygon.points:
            for idx, point in enumerate(data.polygon.points):
                self._roi_points[3 - idx] = [int(point.x), int(point.y)]

    def _on_camera(self, data):
        try:
            self.cv_image = self._bridge.imgmsg_to_cv2(data,
                                                       desired_encoding="bgr8")
            local_image = np.asarray(self.cv_image)
        except Exception:
            print 'OH NO - IMAGE WENT WRONG!!'

        self.subLock.acquire(True)
        self._np_image = deepcopy(local_image)
        self.subLock.release()

    def _on_gain_slider(self, pos):
        self._gain_slider = pos
        self._gain_set = True
        self._slider_time = rospy.Time.now()

    def _on_red_slider(self, pos):
        self._red_thresh = pos

    def _on_yellow_slider(self, pos):
        self._yellow_thresh = pos

    def _on_mouse_click(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            width = self.cv_image.shape[0]
            height = self.cv_image.shape[1]
            for idx, points in enumerate(self._roi_points):
                if (x <= points[0] + 5 and x >= points[0] - 5
                    and y <= points[1] + 5 and y >= points[1] - 5):
                    self._roi_move = True
                    self._point_selected = idx

        elif event == cv.CV_EVENT_MOUSEMOVE and self._roi_move:
            self._roi_points[self._point_selected] = [x, y]

        elif event == cv.CV_EVENT_LBUTTONUP and self._roi_move:
            self._roi_move = False

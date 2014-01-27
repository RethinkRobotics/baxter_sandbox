#!/usr/bin/env python

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

import cv
import cv_bridge
import rospy
import rospkg
import os
from PIL import Image, ImageDraw, ImageFont, ImageOps
from signal import SIGINT
import time
import copy
import json

from baxter_interface import Navigator, RobotEnable, CameraController, Gripper
from brr_ui import (
    mk_process,
    rostopic_handler,
    ros_process,
    gen_msg,
    rgb_to_bgr,
    PIL_to_cv,
    cv_to_msg,
    msg_to_cv,
    overlay,
    BrrButton,
    BrrWindow,
    exit_with_return_code,
    kill_python_procs,
    python_proc_ids,
)
from sensor_msgs.msg import Image as ImageMsg
from baxter_core_msgs.srv import ListCameras

class BrrUi():
    def __init__(self, windows, btn_context):
        self.topic = rostopic_handler('/robot/xdisplay', latch=True)
        self.status = RobotEnable()
        self._rp = rospkg.RosPack()
        self.font = ImageFont.truetype(self._rp.get_path('brr_ui') + '/share/FreeSerif.ttf', 25)
        self.windows = windows
        self.btn_context = btn_context

        self.textHeight = self.font.getsize('W')[1]
        self.img = Image.new('RGB', (1024, 600), 'white')
        
        self.frames = {}

        self.selected_btn_index = 0
        self.active_window = 'demo_1'
        self.active_example = False
        self.current_frame=None
        

        self.navigators = {'left': Navigator('left'), 'right': Navigator('right')}

        self.navigators['left'].button0_changed.connect(self.left_ok_pressed)
        self.navigators['right'].button0_changed.connect(self.right_ok_pressed)

        self.navigators['left'].wheel_changed.connect(self.left_wheel_moved)
        self.navigators['right'].wheel_changed.connect(self.right_wheel_moved)

        self.navigators['left'].button2_changed.connect(self.enable)
        self.navigators['right'].button2_changed.connect(self.enable)

        self.navigators['left'].button1_changed.connect(self.back)
        self.navigators['right'].button1_changed.connect(self.back)


        self.recent_wheel = False
        self.wheel_time = 0
        self.wheel_states = {'left': self.navigators['left'].wheel, 
                             'right': self.navigators['right'].wheel}
        
        self.cameras = {'left_hand': CameraController('left_hand_camera'), 
                        'right_hand': CameraController('right_hand_camera'),
                        'head': CameraController('head_camera')}
        self.cam_sub = ''

        self.l_grip = {'interface': Gripper('left'), 'type': 'custom'}
        self.r_grip = {'interface': Gripper('right'), 'type': 'custom'}
        rospy.Timer(rospy.Duration(.5), self.update_grippers)
    
        self.enable()
        mk_process('rosrun baxter_tools tuck_arms.py -u')

    def enable_cuff(self):
        if len(python_proc_ids('gripper_cuff_control')) == 0:
            ros_process('rosrun baxter_examples gripper_cuff_control.py')

    def update_grippers(self, event):
        new_l = self.l_grip['interface'].type()
        new_r = self.r_grip['interface'].type()
        if new_l != self.l_grip['type']:
            self.l_grip['type'] = new_l
            if new_l == 'electric':
                self.l_grip['interface'].calibrate()
        if new_r != self.r_grip['type']:
            self.r_grip['type'] = new_r
            if new_r == 'electric':
                self.r_grip['interface'].calibrate()
        self.enable_cuff()

    def selected(self):
        return self.windows[self.active_window].selected_btn()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Main Draw function.
    # Converts the appropriate frame to a ros message and sends 
    #     it to the screen.
    # Also sets the current_frame parameter, in expectation of 
    #     future hooks to merge images into the current view
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def draw(self):
        img = Image.new('RGB', (1024, 600), 'white')
        d = ImageDraw.Draw(img)
        print self.active_window
        img = gen_msg(self.draw_window(img, d, self.active_window))
        self.img = img
        msg = cv_to_msg(img)
        self.topic.pub(msg)
        rospy.sleep(.1)

    def draw_window(self, img, draw, window, selected=True):
        if self.windows[window].parent:
            print '--@draw_window: drawing parent: %s' % self.windows[window].parent
            img = self.draw_window(img, draw, window=self.windows[window].parent, selected=False)
        return self.windows[window].draw(img, draw, selected)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Functions linking wheel turns with scrolling in the UI
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def left_wheel_moved(self, v):
        self.wheel_moved(v, 'left')

    def right_wheel_moved(self, v):
        self.wheel_moved(v, 'right')

    def wheel_moved(self, v, side):
        print '--@wheel_moved():  v=%s    side=%s   last=%s ok=%s' % (v, side, self.wheel_states[side], self.wheel_ok())
        print 'active?: %s' % self.active_example
        if not self.active_example and self.wheel_ok():
            print "?"
            wheel = self.wheel_states[side]
            if v > wheel and v-wheel < 100:
                self.scroll(1)
            else:
                self.scroll(-1)
            self.wheel_states[side] = v 
            self.recent_wheel = True
            self.wheel_time = time.time()

    def wheel_ok(self):
        return (self.recent_wheel==False or time.time()-self.wheel_time > .01)

    def scroll(self, direction):
        print '--@scroll():  direction=%s' % direction
        if not self.windows[self.active_window].no_scroll:
            self.inc_scroll(direction)
            i=0
            while True:
                i+=1
                if self.selected().selectable == False:
                    self.inc_scroll(direction)
                else:
                    break
                if i > len(self.windows[self.active_window].buttons):
                    break
            self.draw()

    def inc_scroll(self, inc):
        win_name = self.active_window
        win = self.windows[self.active_window]
        btn_index = win.selected_btn_index
        print "--@inc_scroll: btn_index = %s" % btn_index
        if inc==1:
            if btn_index < len(win.buttons)-1:
                self.windows[win_name].selected_btn_index += 1
        else:
            if btn_index > 0:
                self.windows[win_name].selected_btn_index -= 1

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Functions linking pressing the OK button on either arm with 
    #     the currently selected example
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def left_ok_pressed(self, v):
        self.ok_pressed(v, 'left')

    def right_ok_pressed(self, v):
        self.ok_pressed(v, 'right')
    
    def ok_pressed(self, v, side):
        if v == True:
            context = self.btn_context[self.windows[self.active_window].selected_btn().name]
            func = self.btn_context[self.selected().name]['function']
            if func=="Back":
                self.kill_examples()
            self.active_window = context['nextWindow']
            self.draw()
            if func and func != "Back":
                globals()[func](side)

    def back(self, v):
        if v == True:
            if self.windows[self.active_window].parent:
                self.kill_examples()
                self.active_window = self.windows[self.active_window].parent
                self.draw()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Commands to enable the robot (if it is disabled when the demo
    #     starts) and to kill all currently running examples.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def kill_examples(self, v=1):
        print '--@kill_examples'
        self.active_example = False
        self.selected().status = 'selected'
        commands = ['joint_torque', 'wobbler', 'puppet', 'joint', 'baxter_interface', 'baxter_examples']
        for cmd in commands:
            kill_python_procs(cmd)
        for camera in self.cameras:
            self.cameras[camera].close()
        if self.cam_sub != '':
            self.cam_sub.unregister()
        self.draw()
        self.enable()
    

    def enable(self, v=1):
        if v==1:
            try:
                self.status.enable()
            except:
                self.error_screen('stopped')
                return false
            if not self.status.state().enabled:
                self.error_screen('no_enable')
        self.enable_cuff()

    def error_screen(self, error):
        self.windows[error].parent = self.active_window
        self.active_window = error
        self.draw()

def cam_right(side):
    camera_disp('right_hand')
def cam_left(side):
    camera_disp('left_hand')
def cam_head(side):
    camera_disp('head')
def camera_disp(side):
    def _display(camera, name):
        camera.close()
        camera.resolution = (640, 400)
        camera.open()

    def _cam_to_screen(msg):
        newMsg = overlay(ui.img, msg, (1024, 600), (205, 140, 640, 400))
        ui.topic.pub(newMsg)
        
    ui.cam_sub = rospy.Subscriber(
        'cameras/%s_camera/image' % side,
        ImageMsg,
        _cam_to_screen)

    camera = ui.cameras[side] 
    _display(camera, '%s_camera' % side)

def springs(side):
    proc = ros_process('rosrun baxter_examples joint_torque_springs.py -l %s' % side)

def puppet(side):
    proc = ros_process('rosrun baxter_examples joint_velocity_puppet.py -l %s' % side)

def wobbler(side):
    proc = ros_process('rosrun baxter_examples joint_velocity_wobbler.py')
    proc.process.stdin.close()

def record(side):
    proc = ros_process('rosrun baxter_examples joint_recorder.py -f recording')
    ui.windows['record_submenu'].buttons[2].selectable=True

def play(side):
    proc1 = ros_process('rosrun baxter_interface joint_trajectory_action_server.py &')
    rospy.sleep(1)
    proc2 = ros_process('rosrun baxter_examples joint_trajectory_file_playback.py -f recording -l 0')

def calib(stage=0):
    print stage
    if stage==0 or stage == 1:
        run_calibs(stage)
    else:
        mk_process('rm -rf /var/tmp/hlr/calib.txt')

def run_calibs(stage):
    f = open('/var/tmp/hlr/calib.txt', 'w')
    f.write('stage %s' % (stage+1))
    for side in ['left', 'right']:
        if run_calib(routine, side)==0:
            ui.error_screen('calib_error')
            return 0
    exit_with_return_code('EXIT_REBOOT')

def run_calib(stage, side):
    if stage == 0:
        return mk_process('rosrun baxter_tools calibrate_arm.py -l %s' % side)
    elif stage == 1:
        return mk_process('rosrun baxter_tools tare.py -l %s' % side)
    
def check_calib():
    try:
        f=open('/var/tmp/hlr/calib.txt', 'r')
        print "?"
        stage = f.read()
        calib(int(stage.split()[1]))
    except IOError:
        pass

if __name__=='__main__':
    rospy.init_node('rsdk_demo')
    rp = rospkg.RosPack()
    pack_path = rp.get_path('brr_ui') + '/share'
    
    f = open('%s/config.json' % pack_path).read()
    conf_data = json.loads(f)

         
    buttons = {}
    windows = {}
    btn_context = {}
    for window in conf_data['Windows']:
        if window['back']:
            name = '%s_back' % window['name']
            size = window['back']['size']
            offset = window['back']['offset']
            if window['parent']:
                img_pref = 'Back'
                inner = True
            else: 
                img_pref = 'MainBack'
                inner = False
            buttons[name] = BrrButton(name, size, offset, 0, img_pref, inner, window['name'], '', True)
            btn_context[name] = {'nextWindow': window['parent'], 'function': 'Back'}
    for btn in conf_data['Buttons']:
        buttons[btn['name']] = BrrButton(btn['name'], btn['size'], btn['offset'], btn['index'], btn['image_prefix'], btn['inner'], btn['window'], '', btn['selectable'])
        btn_context[btn['name']] = {'nextWindow': btn['nextWindow'], 'function': btn['function']}
    for window in conf_data['Windows']:
        windows[window['name']] = BrrWindow(window, buttons)

    ui = BrrUi(windows, btn_context)
    '''
    ui.draw()
    check_calib()
    time.sleep(.5)
    ui.ok_pressed(True, 'left')
    time.sleep(.5)
    ui.ok_pressed(True, 'left')
    ui.press('left')
    time.sleep(1)
    ui.press('left')
    time.sleep(1)
    ui.scroll(1)
    time.sleep(1)
    ui.press('left')
    time.sleep(1)
    ui.press('left')
    time.sleep(1)
    '''

    while not rospy.is_shutdown():
        rospy.spin()

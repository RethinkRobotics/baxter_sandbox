#!/usr/bin/env python

# Copyright (c) 2014, Rethink Robotics
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

import copy
import json
import os
from PIL import Image, ImageFont

import rospy
import rospkg

from baxter_interface import Navigator, RobotEnable, CameraController, Gripper
from baxter_demo_ui import (
    BrrButton,
    BrrWindow,
    cv_to_msg,
    exit_with_return_code,
    gen_cv,
    kill_python_procs,
    mk_process,
    overlay,
    python_proc_ids,
    RosProcess,
)
from sensor_msgs.msg import Image as ImageMsg


class BrrUi(object):
    def __init__(self, windows, btn_context, commands, share_path):
        self.xdisp = rospy.Publisher('/robot/xdisplay', ImageMsg, latch=True)
        self.status = RobotEnable()
        self.commands = commands
        self.share_path = share_path
        self.rp = rospkg.RosPack()
        self.font = ImageFont.truetype(
                '%s/FreeSerif.ttf' % self.share_path, 25
        )
        self.windows = windows
        self.btn_context = btn_context

        self.textHeight = self.font.getsize('W')[1]
        self.img = Image.new('RGB', (1024, 600), 'white')

        self.frames = {}
        self.selected_btn_index = 0
        self.active_window = self.windows['demo_1']
        self.active_example = False
        self.current_frame = None


        self.navigators = {'left': Navigator('left'), 
                           'right': Navigator('right')}

        # Navigator OK Button
        self.navigators['left'].button0_changed.connect(self.left_ok_pressed)
        self.navigators['right'].button0_changed.connect(self.right_ok_pressed)

        # Navigator Wheel
        self.navigators['left'].wheel_changed.connect(self.left_wheel_moved)
        self.navigators['right'].wheel_changed.connect(self.right_wheel_moved)

        # Navigator Baxter Button
        self.navigators['left'].button2_changed.connect(self.enable)
        self.navigators['right'].button2_changed.connect(self.enable)

        # Navigator Back Button
        self.navigators['left'].button1_changed.connect(self.back)
        self.navigators['right'].button1_changed.connect(self.back)

        self.wheel_ok = True
        self.wheel_states = {'left': self.navigators['left'].wheel,
                             'right': self.navigators['right'].wheel}

        self.cameras = {'left_hand': CameraController('left_hand_camera'),
                        'right_hand': CameraController('right_hand_camera'),
                        'head': CameraController('head_camera')}
        self.cam_sub = None

        self.l_grip = {'interface': Gripper('left'), 'type': 'custom'}
        self.r_grip = {'interface': Gripper('right'), 'type': 'custom'}
        rospy.Timer(rospy.Duration(.5), self.update_grippers)

        self.enable()
        mk_process('rosrun baxter_tools tuck_arms.py -u')

    

    def selected(self):
        return self.active_window.selected_btn()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Main Draw function.
    # Converts the appropriate frame to a ros message and sends
    #     it to the screen.
    # Also sets the current_frame parameter, in expectation of
    #     future hooks to merge images into the current view
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def draw(self):
        img = Image.new('RGB', (1024, 600), 'white')
        print '--@UI.draw():  window = %s' % self.active_window.name
        img = gen_cv(self.draw_window(img, self.active_window.name))
        self.img = img
        msg = cv_to_msg(img)
        self.xdisp.publish(msg)
        rospy.sleep(.1)

    def draw_window(self, img, window, selected=True):
        if self.windows[window].parent:
            img = self.draw_window(img,
                                   window=self.windows[window].parent, 
                                   selected=False)
        return self.windows[window].draw(img, selected)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Functions linking wheel turns with scrolling in the UI
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def left_wheel_moved(self, v):
        self.wheel_moved(v, 'left')

    def right_wheel_moved(self, v):
        self.wheel_moved(v, 'right')

    def wheel_moved(self, v, side):
        print self.active_example, self.wheel_ok
        if not self.active_example and self.wheel_ok:
            wheel = self.wheel_states[side]
            if v > wheel and v - wheel < 100:
                self.scroll(1)
            else:
                self.scroll(-1)
            self.wheel_states[side] = v
            self.wheel_ok = False
            rospy.Timer(rospy.Duration(.01), self.set_wheel_ok, oneshot=True)

    def set_wheel_ok(self, event):
        print "?"
        self.wheel_ok = True

    def scroll(self, direction):
        print '--@scroll():  direction=%s' % direction
        if not self.active_window.no_scroll:
            win = self.active_window
            i = win.selected_btn_index + direction
            while (i >= 0 and i < len(win.buttons)):
                if win.buttons[i].selectable:
                    self.active_window.selected_btn_index = i
                    break
                i += direction
            self.draw()


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
            context = self.btn_context[self.selected().name]
            func = self.btn_context[self.selected().name]['function']
            if func == "Back":
                self.kill_examples()
            self.active_window = self.windows[context['nextWindow']]
            self.draw()
            if func and func != "Back":
                globals()[func](self, side)

    def back(self, v):
        if v == True:
            if self.active_window.parent:
                self.kill_examples()
                self.active_window = self.windows[self.active_window.parent]
                self.draw()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Commands to enable the robot (if it is disabled when the demo
    #     starts) and to kill all currently running examples.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def kill_examples(self, v=1):
        print '--@kill_examples'
        self.active_example = False
        self.selected().status = 'selected'
        for cmd in self.commands:
            kill_python_procs(cmd)
        for camera in self.cameras:
            self.cameras[camera].close()
        if self.cam_sub != None:
            self.cam_sub.unregister()
        self.draw()
        self.enable()

    def enable(self, v=1):
        if v == 1:
            try:
                self.status.enable()
            except:
                self.error_screen('stopped')
                return False
            if not self.status.state().enabled:
                self.error_screen('no_enable')
        self.enable_cuff()

    def error_screen(self, error):
        self.windows[error].parent = self.active_window.name
        self.active_window = self.windows(error)
        self.draw()

    def enable_cuff(self):
        if len(python_proc_ids('gripper_cuff_control')) == 0:
            RosProcess('rosrun baxter_examples gripper_cuff_control.py')

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


def cam_right(ui, side):
    camera_disp(ui, 'right_hand')

def cam_left(ui, side):
    camera_disp(ui, 'left_hand')

def cam_head(ui, side):
    camera_disp(ui, 'head')

def camera_disp(ui, side):
    def _display(camera, name):
        camera.close()
        camera.resolution = (640, 400)
        camera.open()

    def _cam_to_screen(msg):
        newMsg = overlay(ui.img, msg, (1024, 600), (205, 140, 640, 400))
        ui.xdisp.publish(newMsg)

    ui.cam_sub = rospy.Subscriber(
        'cameras/%s_camera/image' % side,
        ImageMsg,
        _cam_to_screen)

    camera = ui.cameras[side]
    _display(camera, '%s_camera' % side)

def springs(ui, side):
    proc = RosProcess('rosrun baxter_examples '
                       'joint_torque_springs.py -l %s' % side)

def puppet(ui, side):
    proc = RosProcess('rosrun baxter_examples '
                       'joint_velocity_puppet.py -l %s' % side)

def wobbler(ui, side):
    proc = RosProcess('rosrun baxter_examples '
                       'joint_velocity_wobbler.py')
    proc.process.stdin.close()

def record(ui, side):
    proc = RosProcess('rosrun baxter_examples' 
                       'joint_recorder.py -f recording')
    ui.windows['record_submenu'].buttons[2].selectable = True

def play(ui, side):
    proc1 = RosProcess('rosrun baxter_interface '
                        'joint_trajectory_action_server.py &')
    rospy.sleep(1)
    proc2 = RosProcess('rosrun baxter_examples' 
                        'joint_trajectory_file_playback.py -f recording -l 0')

def tare(ui, side):
    calib()

def reboot(ui, side):
    exit_with_return_code('EXIT_REBOOT')

def shutdown(ui, side):
    exit_with_return_code('EXIT_SHUTDOWN')

def calib(ui, stage=0):
    print stage
    if stage == 0 or stage == 1:
        run_calibs(stage)
    else:
        mk_process('rm -rf /var/tmp/hlr/calib.txt')

def run_calibs(stage):
    f = open('/var/tmp/hlr/calib.txt', 'w')
    f.write('stage %s' % (stage + 1))
    for side in ['left', 'right']:
        if run_calib(stage, side) == 0:
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
        f = open('/var/tmp/hlr/calib.txt', 'r')
        stage = f.read()
        calib(int(stage.split()[1]))
    except IOError:
        pass

def main():
    rospy.init_node('rsdk_demo_ui')
    rp = rospkg.RosPack()
    pack_path = rp.get_path('baxter_demo_ui') + '/share'

    f = open('%s/config.json' % pack_path).read()
    conf_data = json.loads(f)


    windows = {}
    btn_context = {}
    for window in conf_data['Windows']:
        buttons = {}
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
            buttons[name] = BrrButton(name, size, offset, 0,
                                      img_pref, inner,
                                      '', True, pack_path)
            btn_context[name] = {'nextWindow': window['parent'],
                                     'function': 'Back'}
        try:
            for btn in window['Buttons']:
                buttons[btn['name']] = BrrButton(btn['name'], btn['size'],
                                                 btn['offset'], btn['index'],
                                                 btn['image_prefix'], btn['inner'],
                                                 '', btn['selectable'], pack_path)
                btn_context[btn['name']] = {'nextWindow': btn['nextWindow'],
                                            'function': btn['function']}
        except:
            pass

        windows[window['name']] = BrrWindow(window, buttons, pack_path)

    commands = ['joint_torque', 'wobbler', 
                'puppet', 'joint', 
                'baxter_interface', 'baxter_examples']
    ui = BrrUi(windows, btn_context, commands, pack_path)
    ui.draw()
    check_calib()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()

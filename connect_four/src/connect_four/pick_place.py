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

import collections

from copy import deepcopy

import rospy

import tf
import cv2
import cv_bridge
import rospkg
import tf

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from sensor_msgs.msg import (
    Image,
    JointState,
)

import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class PickPlace(object):
    def __init__(self, limb):
        self._rp = rospkg.RosPack()
        self._images = (self._rp.get_path('connect_four') +
                          '/share/images')
        self._path = self._rp.get_path('connect_four') + '/config/'
        self._side = limb
        self._limb = baxter_interface.Limb(limb)

        dash_io = baxter_interface.DigitalIO(limb + '_upper_button')
        circle_io = baxter_interface.DigitalIO(limb + '_lower_button')

        self.pick_location = dict()
        self.pick_approach = dict()

        self.place_jp = dict()
        self.camera_jp = dict()
        self.neutral_jp = dict()
        self._place_pose = dict()
        self.place_approach = dict()
        self._slots = ('one', 'two', 'three', 'four', 'five', 'six', 'seven')
        self._connect_pose_vector = dict()

        self._gripper = baxter_interface.Gripper(limb)

        self._gripper.calibrate()
        self._gripper.set_holding_force(100.0)

        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        circle_io.state_changed.connect(self._pick)
        dash_io.state_changed.connect(self._place)

    def _find_jp(self, pose):
        ikreq = SolvePositionIKRequest()

        goal_pose = Pose()
        goal_pose.position = pose['position']
        goal_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=goal_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _find_approach(self, pose, offset):
        ikreq = SolvePositionIKRequest()
        # Add 5 cm offset in Z direction
        try:
            pose['position'] = Point(x=pose['position'][0],
                                     y=pose['position'][1],
                                     z=pose['position'][2] + offset
                                     )
        except Exception:
            pose['position'] = Point(x=pose['position'].x,
                                     y=pose['position'].y,
                                     z=pose['position'].z + offset
                                     )
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _pick(self, value):
        if value:
            if len(self.camera_jp) == 0:
                # Record Camera Location
                print 'Recording camera position'
                self.camera_jp = self._limb.joint_angles()
            elif len(self.pick_approach) == 0:
                # Record Pick Location
                print 'Recording pick location'
                self.pick_location = self._limb.joint_angles()
                self.pick_approach = self._find_approach(
                                         self._limb.endpoint_pose(),
                                         0.05)
                self._gripper.close()

    def _place(self, value):
        if value:
            if len(self.place_jp) == 0:
                print 'Recording left most'
                self.place_jp[self._slots[0]] = self._limb.joint_angles()
                self._place_pose[self._slots[0]] = self._limb.endpoint_pose()
            elif len(self.place_jp) == 1:
                print 'Recording right most'
                self.place_jp[self._slots[6]] = self._limb.joint_angles()
                self._place_pose[self._slots[6]] = self._limb.endpoint_pose()

                euler = dict()
                for key in self._place_pose.keys():
                    quaternion = (self._place_pose[key]['orientation'][0],
                                  self._place_pose[key]['orientation'][1],
                                  self._place_pose[key]['orientation'][2],
                                  self._place_pose[key]['orientation'][3],)
                    rot = tf.transformations.euler_from_quaternion(quaternion)
                    euler[key] = rot

                print 'Getting that vector for ya'
                x = (self._place_pose['seven']['position'][0] -
                     self._place_pose['one']['position'][0])
                y = (self._place_pose['seven']['position'][1] -
                      self._place_pose['one']['position'][1])
                z = (self._place_pose['seven']['position'][2] -
                     self._place_pose['one']['position'][2])
                roll = euler['seven'][0] - euler['one'][0]
                pitch = euler['seven'][1] - euler['one'][1]
                yaw = euler['seven'][2] - euler['one'][2]
                self._connect_pose_vector['x'] = x
                self._connect_pose_vector['y'] = y
                self._connect_pose_vector['z'] = z
                self._connect_pose_vector['roll'] = roll
                self._connect_pose_vector['pitch'] = pitch
                self._connect_pose_vector['yaw'] = yaw

                # Get all those joint moves along 6 steps
                move_pose = collections.defaultdict(dict)
                pre_offset = tf.transformations.quaternion_from_euler(
                                 euler['one'][0],
                                 euler['one'][1],
                                 euler['one'][2]
                             )
                for idx, move in enumerate(self._slots):
                    move_pose[move]['position'] = Point(
                        x=(self._place_pose['one']['position'][0] +
                           self._connect_pose_vector['x'] / 6.0 * idx),
                        y=(self._place_pose['one']['position'][1] +
                           self._connect_pose_vector['y'] / 6.0 * idx),
                        z=(self._place_pose['one']['position'][2] +
                           self._connect_pose_vector['z'] / 6.0 * idx),
                    )

                    roll = (euler['one'][0] +
                            self._connect_pose_vector['roll'] / 6.0 * idx)
                    pitch = (euler['one'][1] +
                             self._connect_pose_vector['pitch'] / 6.0 * idx)
                    yaw = (euler['one'][2] +
                           self._connect_pose_vector['yaw'] / 6.0 * idx)
                    quaternion = tf.transformations.quaternion_from_euler(
                                     roll, pitch, yaw)
                    move_pose[move]['orientation'] = Quaternion(
                        x=quaternion[0],
                        y=quaternion[1],
                        z=quaternion[2],
                        w=quaternion[3]
                    )
                    jp = self._find_jp(deepcopy(move_pose[move]))
                    approach = self._find_approach(deepcopy(move_pose[move]),
                                                   0.03)
                    self.place_jp[move] = jp
                    self.place_approach[move] = approach
                if self._side == 'right':
                    self.neutral_jp = self._find_approach(
                                          deepcopy(move_pose['seven']),
                                          0.1
                                      )
                else:
                    self.neutral_jp = self._find_approach(
                                          deepcopy(move_pose['one']),
                                          0.1
                                      )
                filename = self._path + self._side + '_poses.config'
                self._save_file(filename)

    def _read_file(self, file):
        with open(file, 'r') as f:
            for line in f:
                split = line.split('=')
                location = split[0]
                prefix = location.split("_")[0]
                position = split[1]
                if location == 'camera':
                    self.camera_jp = eval(position)
                elif location == 'pick':
                    self.pick_location = eval(position)
                elif location == 'pick_approach':
                    self.pick_approach = eval(position)
                elif '_approach' in location:
                    self.place_approach[prefix] = eval(position)
                elif '_place' in location:
                    self.place_jp[prefix] = eval(position)
                elif location == 'neutral':
                    self.neutral_jp = eval(position)

    def _save_file(self, file):
        print "Saving your positions to file!"
        f = open(file, 'w')
        f.write('camera=' + str(self.camera_jp) + '\n')
        f.write('pick=' + str(self.pick_location) + '\n')
        f.write('pick_approach=' + str(self.pick_approach) + '\n')
        f.write('neutral=' + str(self.neutral_jp) + '\n')
        for prefix in self._slots:
            f.write(prefix + '_place=' + str(self.place_jp[prefix]) + '\n')
            f.write(prefix + '_approach=' +
                    str(self.place_approach[prefix]) + '\n')
        f.close()

    def get_locations(self):
        good_input = False
        while not good_input:
            self.read_file = raw_input("Would you like to use the previously "
                                       "found pick and place locations (y/n)?")
            if self.read_file != 'y' and self.read_file != 'n':
                print "You must answer 'y' or 'n'"
            elif self.read_file == 'y':
                filename = self._path + self._side + '_poses.config'
                self._read_file(filename)
                good_input = True
            else:
                print ("Move %s arm into a location allowing the camera to "
                       "see the board - press Circle button to confirm"
                       % (self._side,))
                while(len(self.camera_jp) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Nice choice!")

                print ("Move Gripper into pick location - press Circle button "
                       " to grasp piece")
                while(len(self.pick_location) == 0 and
                      not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Cool - Got it!")

                print ("Move Gripper into left most drop slot - press Dash "
                       "button to record")
                while(len(self.place_jp) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Awesome Got the Left Spot!\n\n")

                print ("Move Gripper into right most drop slot - press Dash "
                       "button to record")
                while(len(self.place_jp) != 7 and not rospy.is_shutdown()):
                    rospy.sleep(0.5)
                print ("Awesome Got the Right Spot!\n\n")
                good_input = True

    def move_neutral(self):
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(self.neutral_jp,
                                           threshold=0.08727)  # 5 degrees

    def get_piece(self):
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(self.pick_approach,
                                           threshold=0.01745)  # 1 degree
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(self.pick_location,
                                           threshold=0.003491)  # 0.2 degrees
        self._gripper.command_position(0.0)
        self._limb.move_to_joint_positions(self.pick_approach,
                                           threshold=0.01745)  # 1 degree

    def place_piece(self, slot):
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(
            self.place_approach[self._slots[slot]],
            threshold=0.01745  # 1 degree
        )
        self._limb.set_joint_position_speed(0.3)
        self._limb.move_to_joint_positions(
            self.place_jp[self._slots[slot]],
            threshold=0.003491  # 0.2 degrees
        )
        self._gripper.command_position(100.0)
        self._limb.move_to_joint_positions(
            self.place_approach[self._slots[slot]],
            threshold=0.034906  # 2 degrees
        )

    def move_camera(self):
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(self.camera_jp)

    def pick_pieces(self):
        print('My Turn!')

        for move in self._slots:
            print 'moving to position ' + move
            if (move != 'one' or self.read_file == 'y'):
                self._limb.set_joint_position_speed(0.8)
                self._limb.move_to_joint_positions(self.neutral_jp)
                self._limb.move_to_joint_positions(self.pick_approach)
                self._limb.set_joint_position_speed(0.2)
                self._limb.move_to_joint_positions(self.pick_location)
                self._gripper.command_position(0.0)
                self._limb.move_to_joint_positions(self.pick_approach)
                self._limb.set_joint_position_speed(0.8)
                self._limb.move_to_joint_positions(self.neutral_jp)
            self._limb.set_joint_position_speed(0.8)
            self._limb.move_to_joint_positions(self.place_approach[move])
            self._limb.set_joint_position_speed(0.2)
            self._limb.move_to_joint_positions(self.place_jp[move])
            self._gripper.command_position(100.0)
            self._limb.move_to_joint_positions(self.place_approach[move])

    def nod(self):
        head = baxter_interface.Head()
        for _ in xrange(2):
            head.command_nod()

    def celebrate(self):
        head = baxter_interface.Head()
        for _ in xrange(3):
            head.command_nod()
            self._gripper.command_position(100.0, block=True)
            self._gripper.command_position(0.0, block=True)
        head.set_pan(1.0, 5.0)
        head.set_pan(-1.0, 5.0)
        head.set_pan(0.0, 5.0)
        img = cv2.imread(self._images + '/baxterwins.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
        pub = rospy.Publisher('/robot/xdisplay', Image,
                              latch=True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(5.0)
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
        pub.publish(msg)

    def disappoint(self):
        head = baxter_interface.Head()
        for _ in xrange(3):
            self._gripper.command_position(100.0, block=True)
            self._gripper.command_position(0.0, block=True)
        head.set_pan(1.0, 5.0)
        head.set_pan(-1.0, 5.0)
        head.set_pan(0.0, 5.0)
        img = cv2.imread(self._images + '/congrats.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
        pub = rospy.Publisher('/robot/xdisplay', Image,
                              latch=True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(5.0)
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
        pub.publish(msg)

    def tie(self):
        head = baxter_interface.Head()
        for _ in xrange(3):
            self._gripper.command_position(100.0, block=True)
            self._gripper.command_position(0.0, block=True)
        head.set_pan(1.0, 5.0)
        head.set_pan(-1.0, 5.0)
        head.set_pan(0.0, 5.0)
        img = cv2.imread(self._images + '/tie.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
        pub = rospy.Publisher('/robot/xdisplay', Image,
                              latch=True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(5.0)
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
        pub.publish(msg)


def main():
    rospy.init_node("rsdk_connect_four_test_pick")

    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    pp = PickPlace('right')
    pp.get_locations()
    pp.pick_pieces()

if __name__ == "__main__":
    main()

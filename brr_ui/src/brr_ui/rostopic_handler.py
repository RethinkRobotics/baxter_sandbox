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

from MassLib import mk_process

import roslib
roslib.load_manifest('baxter_interface')
import rospy
import baxter_dataflow as dataflow

class rostopic_handler():
    def __init__(self, name, latch=None):
        self.name = name
        self.get_info()
        self.import_msg()
        self.data = None
        if latch==True:
            self.publisher = rospy.Publisher(self.name, self.msg_type, latch=True)
        self.subscriber = rospy.Subscriber(self.name, self.msg_type, self.data_callback) 
        rospy.sleep(1)
    def get_info(self):
        self.info = mk_process('rostopic info %s' % self.name, get_output=True, quiet=True).split('\n')
        self.type = self.info[0].split(': ')[1]

        if 'Subscribers: ' in self.info:
            self.sub_index = self.info.index('Subscribers: ')
            self.subscribers = [sub.split('* ')[1] for sub in self.info[self.sub_index+1:] if sub != '']
        else:
            self.sub_index = self.info.index('Subscribers: None')
            self.subscribers = []
        if 'Publishers: ' in self.info:
            self.pub_index = self.info.index('Publishers: ')
            self.publishers = [pub.split('* ')[1] for pub in self.info[self.pub_index+1:self.sub_index-1] if pub != '']
        else:
            self.pub_index = self.info.index('Publishers: None')
            self.publishers = []

    def import_msg(self):
        msg_group = self.type.split('/')[0]
        msg_type = self.type.split('/')[1]
        if msg_type not in locals():
            exec('from %s.msg import %s' % (msg_group, msg_type))
        self.msg_type = locals()[msg_type]

    def data_callback(self, data):
        self.data = data

    def pub(self, data):
        self.publisher.publish(data)

    def wait(self, timer=5.0):
        dataflow.wait_for(
            lambda: self.data != None,
            timeout=timer
        )

if __name__=='__main__':
    rospy.init_node('rostopic_tester')
    halo_g = rostopic_handler('/robot/sonar/head_sonar/lights/set_green_level')
    halo_g.pub(100.0)
    rospy.sleep(1)

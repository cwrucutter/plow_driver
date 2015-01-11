#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Matthew Klein
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the names of the authors nor the names of their
# affiliated organizations may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import UInt8
# struct needed to convert an UInt8.data to a byte array to be sent with serial
import serial, struct

class plow_driver(object):
  def __init__(self):
    # ROS init node and subscriber
    rospy.init_node('plow_driver')
    # Listen for a /std_msgs/UInt8 on the topic /plow/angle 
    rospy.Subscriber('/plow/angle', UInt8, self.plowAngleSubCB)

    # Set some parameters for the serial port (port and baudrate)
    plowPort = rospy.get_param('~port','/dev/ttyACM0')
    plowBaudrate = rospy.get_param('~baud',9600)
    # and initialize the serial port (we should add close() somewhere in here)
    self.plowSerial = serial.Serial(port=plowPort,baudrate=plowBaudrate, timeout = 0.01)

  # Callback function called when a message is published to /plow/angle
  def plowAngleSubCB(self, msg):
    # Coerce the message to lie between 0 and 45, convert it to a byte array, and send it over serial to the arduino
    self.plowSerial.write(struct.pack("B",min(45,max(0,msg.data))))

  # This function just runs rospy.spin(), which is essentially a blank while loop that keeps the program running while the subscriber keeps an eye on the topic /plow/angle, and runs the callback function when a message arrives.
  def run(self):
    rospy.spin()

# If this script is executed, we will instaniate a plow_driver object and run the run function.
if __name__ == "__main__":
  pd = plow_driver()
  pd.run()

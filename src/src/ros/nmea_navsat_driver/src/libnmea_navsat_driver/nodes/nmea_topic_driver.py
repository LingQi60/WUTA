# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

"""Defines the main method for the nmea_topic_driver executable."""


from nmea_msgs.msg import Sentence
import rospy

from libnmea_navsat_driver.driver import RosNMEADriver


def nmea_sentence_callback(nmea_sentence, driver):
    """Process a NMEA sentence message with a RosNMEADriver.

    Args:
        nmea_sentence (nmea_msgs.msg.Sentence): NMEA sentence message to feed to the driver.
        driver (RosNMEADriver): Driver to feed the sentence.
    """
    try:
        driver.add_sentence(

            nmea_sentence.sentence,
            frame_id=nmea_sentence.header.frame_id,
            timestamp=nmea_sentence.header.stamp)     #add_sentence()接收frame_id和时间戳，接收sentence并解析，发布解析后的时间，位置，速度和航向角信息。
    except ValueError as e:
        rospy.logwarn(
            "Value error, likely due to missing fields in the NMEA message. "
            "Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, "
            "including a bag file with the NMEA sentences that caused it." %
            e)


def main():
    """Create and run the nmea_topic_driver ROS node.

    Creates a NMEA Driver and feeds it NMEA sentence strings from a ROS subscriber.

    ROS subscribers:
        mea_sentence (nmea_msgs.msg.Sentence): NMEA sentence messages to feed to the driver.
    """
    rospy.init_node('nmea_topic_driver')

    driver = RosNMEADriver()           #实例化RosNMEADriver()类

    rospy.Subscriber("nmea_sentence", Sentence, nmea_sentence_callback,
                     driver)             #订阅话题nmea_sentence，话题类型Sentence，回调函数nmea_sentence_callback，回调函数第二个参数driver。

    rospy.spin()

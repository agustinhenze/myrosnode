#!/usr/bin/env python
# Copyright 2017 Agustin Henze <tin@aayy.com.ar>

# Permission is hereby granted, free of charge, to any
# person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the
# Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the
# Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice
# shall be included in all copies or substantial portions of
# the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
# KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
# OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
import csv
import threading
import time
import tempfile
import rospy
from nav_msgs.msg import Odometry
from mynode.srv import StartRecording, StopRecording


class AlreadyRecording(Exception):
    pass


class PathLogger(object):
    SERVICE_NAME = 'path_logger'

    def __init__(self, topic='odom'):
        self.csvfile = None
        self.csvwriter = None
        self.enable_recording = False
        self.topic = topic
        self.counter = 0
        # log callback and stop_recording can be called from different threads
        self.logger_lock = threading.RLock()

    def start_recording(self, filename):
        if self.enable_recording:
            raise AlreadyRecording()
        self.csvfile = open(filename, 'wb')
        self.csvwriter = csv.writer(self.csvfile, delimiter=' ', quotechar='|',
                                    quoting=csv.QUOTE_MINIMAL)
        self.enable_recording = True
        rospy.loginfo('The recording has been started at %s', filename)

    def stop_recording(self):
        with self.logger_lock:
            self.enable_recording = False
            if self.csvfile and not self.csvfile.closed:
                self.csvfile.close()
                rospy.loginfo('Recorded %d entry points of odometry at %s',
                              self.counter, self.csvfile.name)
            self.counter = 0

    def setup(self):
        rospy.init_node(self.SERVICE_NAME)
        rospy.Subscriber(self.topic, Odometry, self.log)
        rospy.on_shutdown(self.stop_recording)
        rospy.Service('start_recording', StartRecording, self.start_recording_handler)
        rospy.Service('stop_recording', StopRecording, self.stop_recording_handler)
        rospy.loginfo('Path logger ready')

    def start_recording_handler(self, req):
        if not self.enable_recording:
            prefix = time.strftime("%Y%m%d-%H%M%S")
            path = '{}.csv'.format(tempfile.mktemp(prefix=prefix))
            self.start_recording(path)
            return "Recording has been started at {}".format(path)
        else:
            return 'Recording is already started'

    def stop_recording_handler(self, req):
        if self.enable_recording:
            self.stop_recording()
            return 'Recording stopped', self.csvfile.name
        else:
            return 'Recording is not started yet', None

    def log(self, data):
        with self.logger_lock:
            if not self.enable_recording:
                return
            self.counter += 1
            position = data.pose.pose.position
            orientation = data.pose.pose.orientation
            self.csvwriter.writerow(
                [position.x, position.y, position.z, orientation.x, orientation.y,
                 orientation.z, orientation.w, data.header.stamp]
            )


def main():
    path_logger = PathLogger()
    path_logger.setup()
    rospy.spin()


if __name__ == '__main__':
    main()

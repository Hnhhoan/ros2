# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import cv2
import os
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        video='/root/camera-video.mp4'
        assert os.path.exists(video)
        self.i = 0
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        timer_period = 0.5  # seconds
        self.vidcap = cv2.VideoCapture(video)   
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.publish_mgsimg()

    def timer_callback(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        success, cv_image = self.vidcap.read()
        if success:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            self.publisher_.publish(image_message)
            self.get_logger().info('Publishing: image "%s"' % self.i)
            self.i =self.i+ 1

    def publish_mgsimg(self):
        #print(cv2.getBuildInformation())
        while self.vidcap.isOpened():
            success, cv_image = self.vidcap.read()
            if success:
                image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
                self.publisher_.publish(image_message)
                self.get_logger().info('Publishing: image "%s"' % self.i)
                self.i =self.i+ 1
            else:
                break
        cv2.destroyAllWindows()
        self.vidcap.release()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

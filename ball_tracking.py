#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class BallTrackingNode(Node):
    def _init_(self):
        super()._init_('ball_tracking_node')
        self.get_logger().info("Ball Tracking Node Initialized")
        self.cv_bridge = CvBridge()

        self.capture = cv2.VideoCapture(0)  # Open the default camera (usually webcam)

        if not self.capture.isOpened():
            raise Exception("Could not open video device")

        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.ball_position_publisher = self.create_publisher(Point, 'ball_position', 10)
        
        self.timer = self.create_timer(1.0 / 30, self.track_ball)

    def track_ball(self):
        ret, frame = self.capture.read()
        if ret:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur_frame = cv2.GaussianBlur(gray_frame, (17,17), 0)
            
            try:
                # Apply Hough Circle Transform to detect circles (balls) with a timeout
                circles = cv2.HoughCircles(
                    blur_frame, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
                    param1=100, param2=30, minRadius=100, maxRadius=400)

                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for circle in circles[0, :]:
                        x, y, radius = circle
                        # Publish the ball position as a Point message
                        ball_position_msg = Point()
                        ball_position_msg.x = float(x)  # Convert to float
                        ball_position_msg.y = float(y)  # Convert to float
                        ball_position_msg.z = float(radius)  # Convert to float
                        self.ball_position_publisher.publish(ball_position_msg)
                        # Draw the detected circle
                        cv2.circle(frame, (x, y), radius, (0, 255, 0), 2)

            except cv2.error as e:
                self.get_logger().error(f'Error detecting circles: {e}')

            # Publish the frame with the detected circles
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(img_msg)

            # Display the video frame in a window
            cv2.imshow("Video Stream", frame)
            cv2.waitKey(1)  # Adjust the argument to control the display rate


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()

#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/lane_detect.py
# WeGo LIMO Pro를 이용한 차선 인식 코드

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, Bool
from dynamic_reconfigure.server import Server
from limo_application.cfg import image_processingConfig

import cv2
import numpy as np

import matplotlib.pyplot as plt

class RedLineDetection(object):
    '''
        Detecting Red Line representing the stop sign
    '''
    def __init__(self):
        # ROS Part
        rospy.init_node("lane_detect")
        srv = Server(image_processingConfig, self.reconfigure_callback)
        self.cvbridge = CvBridge()
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.image_topic_callback)
	#self.redline_distance_pub = rospy.Publisher("/limo/redline_distance_y", Int32, queue_size=5)
	self.redline_detection_pub = rospy.Publisher("/limo/redline_detection", Bool, queue_size=1)

	self.viz = rospy.get_param("~visualization", False)

    def visResult(self):
        if self.stop_line_y is not None:
            cv2.line(self.frame, (0, self.stop_line_y), (480, self.stop_line_y), (0, 255, 0), 2)
        
        # cv2.imshow("lane_original", self.frame)
        cv2.imshow('Red Mask', self.red_mask)
        
        cv2.waitKey(1)
        

    def colorDetect(self, _img):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask1 = cv2.inRange(hls, self.RED_LINE_LOW_TH1, self.RED_LINE_HIGH_TH1)
        mask2 = cv2.inRange(hls, self.RED_LINE_LOW_TH2, self.RED_LINE_HIGH_TH2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        return red_mask

    def detect_stop_line(self, _mask, y_bin_size=20, min_pixel_threshold=2000):
        height, width = _mask.shape

        histogram = []
        for y in range(0, height, y_bin_size):
            pixel_count = np.sum(_mask[y:y + y_bin_size, :] > 0)
            histogram.append(pixel_count)

        max_pixels = max(histogram)
        stop_line_y = histogram.index(max_pixels) * y_bin_size if max_pixels > min_pixel_threshold else None

        return stop_line_y, histogram

    def reconfigure_callback(self, config, level):
        self.RED_LINE_LOW_TH1 = np.array([0, 50, 150])
        self.RED_LINE_HIGH_TH1 = np.array([10, 200, 255])
        self.RED_LINE_LOW_TH2 = np.array([170, 50, 150])
        self.RED_LINE_HIGH_TH2 = np.array([180, 200, 255])
        return config

    def image_topic_callback(self, img):
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.red_mask = self.colorDetect(self.frame)
        self.stop_line_y, self.histogram = self.detect_stop_line(self.red_mask)
	#self.redline_distance_pub.publish(self.stop_line_y)

	if 50 < self.stop_line_y < 330:
	    self.redline_detection_pub.publish(True)
	else:
	    self.redline_detection_pub.publish(False)


	if self.viz:
            self.visResult()

def run():
    new_class = RedLineDetection()
    rospy.spin() 
    
if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")



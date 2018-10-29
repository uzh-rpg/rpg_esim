#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Node that listens to esim_msgs/OpticFlow messages, and publishes it
    as a color-coded image, and as a vector field.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from esim_msgs.msg import OpticFlow
from cv_bridge import CvBridge

class FlowConverterNode:
    _p0 = None

    def __init__(self):
        self._bridge = CvBridge()

        rospy.Subscriber("flow", OpticFlow, self._OpticFlowCallback)
        self.pub_color = rospy.Publisher("flow_color", Image, queue_size=0)
        self.pub_arrows = rospy.Publisher("flow_arrows", Image, queue_size=0)

        self.arrows_step = rospy.get_param('~arrows_step', 12)
        self.arrows_scale = rospy.get_param('~arrows_scale', 0.1)
        self.arrows_upsample_factor = rospy.get_param('~arrows_upsample_factor', 2)
        self.publish_rate = rospy.get_param('~publish_rate', 20)

        rospy.loginfo('Started flow converter node')
        rospy.loginfo('Step size between arrows: {}'.format(self.arrows_step))
        rospy.loginfo('Scale factor: {:.2f}'.format(self.arrows_scale))
        rospy.loginfo('Upsample factor: x{}'.format(self.arrows_upsample_factor))
        rospy.loginfo('Publish rate: {} Hz'.format(self.publish_rate))

        self.arrows_color = (0,0,255) # red
        self.first_msg_received = False

    def reset(self):
        self.first_msg_received = False
        self.last_msg_stamp = None

    def _OpticFlowCallback(self, msg):
        if not self.first_msg_received:
            rospy.logdebug('Received first message at stamp: {}'.format(msg.header.stamp.to_sec()))
            self.last_msg_stamp = msg.header.stamp
            self.first_msg_received = True
            self.convertAndPublishFlow(msg)

        if msg.header.stamp.to_sec() < self.last_msg_stamp.to_sec():
            rospy.loginfo('Optic flow converter reset because new stamp is older than the latest stamp')
            self.reset()
            return

        time_since_last_published_msg = (msg.header.stamp - self.last_msg_stamp).to_sec()
        rospy.logdebug('Time since last published message: {}'.format(time_since_last_published_msg))
        if time_since_last_published_msg >= 1./float(self.publish_rate):
            self.last_msg_stamp = msg.header.stamp
            self.convertAndPublishFlow(msg)

    def convertAndPublishFlow(self, msg):
        height, width = msg.height, msg.width
        flow_x = np.array(msg.flow_x).reshape((height, width))
        flow_y = np.array(msg.flow_y).reshape((height, width))
        self.publishColorCodedFlow(flow_x, flow_y, msg.header.stamp)
        self.publishArrowFlow(flow_x, flow_y, msg.header.stamp)

    def publishColorCodedFlow(self, flow_x, flow_y, stamp):
        assert(flow_x.shape == flow_y.shape)
        height, width = flow_x.shape
        magnitude, angle = cv2.cartToPolar(flow_x, flow_y)
        magnitude = cv2.normalize(src=magnitude, dst=magnitude, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        hsv = np.zeros((height,width,3), dtype=np.uint8)
        self.hsv = hsv.copy()
        hsv[...,1] = 255
        hsv[...,0] = 0.5 * angle * 180 / np.pi
        hsv[...,2] = cv2.normalize(magnitude,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        img_msg = self._bridge.cv2_to_imgmsg(bgr, 'bgr8')
        img_msg.header.stamp = stamp
        self.pub_color.publish(img_msg)

    def publishArrowFlow(self, flow_x, flow_y, stamp):
        assert(flow_x.shape == flow_y.shape)
        height, width = flow_x.shape

        step = self.arrows_step
        scale = self.arrows_scale
        ss = self.arrows_upsample_factor
        arrow_field = np.zeros((ss * height, ss * width,3), dtype=np.uint8)

        for x in np.arange(0, width, step):
            for y in np.arange(0, height, step):
                vx = flow_x[y,x]
                vy = flow_y[y,x]
                cv2.arrowedLine(arrow_field, (ss * x, ss * y),
                                (int(ss * (x + scale * vx)), int(ss * (y + scale * vy))), color=self.arrows_color, thickness=1)

        img_msg = self._bridge.cv2_to_imgmsg(arrow_field, 'bgr8')
        img_msg.header.stamp = stamp
        self.pub_arrows.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node('flow_converter_node')
    node = FlowConverterNode()
    rospy.spin()

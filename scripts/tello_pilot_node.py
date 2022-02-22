#!/usr/bin/env python3

from time import sleep

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, Imu
from tello_pilot.msg import CameraDirection

import cv2 as cv
from RospyLogger import RospyLogger
from TelloParameterParser import TelloParameterParser

from cv_bridge import CvBridge
from threading import Thread, Lock

from DJITelloPy.djitellopy.tello import Tello, TelloException

# Locks
port_update_lock = Lock()
video_stream_port = 11111

class TelloNode:

    def __init__(self):
        (single_mode, tello_list) = TelloParameterParser.param_tello_list(rospy.get_param('~tello_list', ''))
        
        self.threads = []
        for prefix in tello_list.keys():
            thread = Thread(target=TelloSwarmMember, daemon=True, args=(prefix,))
            thread.start()
            self.threads.append(thread)
        # ---- AP ----
        #if(rospy.get_param('~ap_mode', False)):
        #    self.tello = Tello()
        #    self.tello.connect()
        #    self.tello.connect_to_wifi(rospy.get_param('~ap_ssid'), rospy.get_param('~ap_password'))

        #Tello.LOGGER = RospyLogger()

class TelloSwarmMember:

    def __init__(self, prefix: str) -> None:
        self.prefix = prefix
        self.eth_interface = self.pn('eth_interface')
        self.ssid = self.pn('ssid')

        self.imu_msg = Imu()
        self.imu_msg.header.seq = -1

        self.tello = Tello(host=self.pn('ip')) # eth_interface=self.eth_interface, ssid=self.ssid

        self.bridge = CvBridge()
        
        self.normal_setup()

    def ap_setup(self, prefix: str):
        pass

    def normal_setup(self):
        self.tello.connect()      

        with port_update_lock:
            global video_stream_port
            self.tello.set_network_ports(Tello.STATE_UDP_PORT, video_stream_port)
            video_stream_port = video_stream_port + 1

        #self.tello.connect_to_wifi(rospy.get_param('~ap_ssid'), rospy.get_param('~ap_password'))

        self.takeoff_subscriber = rospy.Subscriber(self.tn('takeoff'), Empty, self.cmd_takeoff)
        self.land_subscriber = rospy.Subscriber(self.tn('land'), Empty, self.cmd_land)

        self.imu_publisher = rospy.Publisher(self.tn('imu'), Imu, queue_size=1)

        # ---- Settings ----
        self.camera_direction = Tello.CAMERA_FORWARD
        self.tello.set_video_direction(self.camera_direction)
        self.tello.set_video_fps(
            TelloParameterParser.param_camera_fps(self.pn('camera_fps', rospy.get_param('~camera_fps', 30))))
        self.tello.set_video_bitrate(
            TelloParameterParser.param_camera_bitrate(self.pn('camera_bitrate', rospy.get_param('~camera_bitrate'))))

        # ---- Camera ----
        if True:
            self.tello.streamon()
            self.camera_direction_subscriber = rospy.Subscriber(self.tn('camera/direction'), CameraDirection, self.cmd_camera_direction),
            self.image_raw_publisher = rospy.Publisher(self.tn('camera/image_raw'), Image, queue_size=1)
            self.video_thread = Thread(target=self.pub_image_raw)
            self.frame_read = self.tello.get_frame_read()
            self.video_thread.start()

    def pub_image_raw(self):
        while True:
            img_msg = self.bridge.cv2_to_imgmsg(self.frame_read.frame, 'rgb8')
            img_msg.header.stamp = rospy.Time.now()
            self.image_raw_publisher.publish(img_msg)
            sleep(1 / 25.)

    def cmd_takeoff(self, msg):
        self.tello.takeoff()

    def cmd_land(self, msg):
        self.tello.land()

    def cmd_camera_direction(self, msg):
        if msg.forward:
            if self.camera_direction is not Tello.CAMERA_FORWARD:
                self.tello.set_video_direction(Tello.CAMERA_FORWARD)
                self.camera_direction = Tello.CAMERA_FORWARD
        elif self.camera_direction is not Tello.CAMERA_DOWNWARD: 
            self.tello.set_video_direction(Tello.CAMERA_DOWNWARD)
            self.camera_direction = Tello.CAMERA_DOWNWARD

    def __del__(self):
        if self.video_thread is not None:
            self.video_thread.join()

    def tn(self, topic:str):
        return "%s/%s" % (self.prefix, topic)

    def pn(self, parameter:str, default=None):
        return rospy.get_param("~%s_%s" % (self.prefix, parameter), default=default)


def main():
    rospy.init_node('tello_pilot_node')
    TelloNode()
    rospy.spin()


if __name__ == '__main__':
    main()
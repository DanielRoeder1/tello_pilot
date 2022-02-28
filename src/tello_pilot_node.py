#!/usr/bin/env python3

import time
import math

from utils.GetTelloIP import get_tello_ip

from anyio import sleep

import rospy
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Empty
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import TwistStamped
from tello_pilot.msg import CameraDirection

import cv2 as cv
import numpy as np
from utils.RospyLogger import RospyLogger
from TelloParameterParser import TelloParameterParser

from cv_bridge import CvBridge
from threading import Thread, Lock

from DJITelloPy.djitellopy.tello import Tello, TelloException

# Locks
port_update_lock = Lock()
av_open_lock = Lock()

video_stream_port = 11111

class TelloNode:

    def __init__(self):
        (single_mode, tello_list) = TelloParameterParser.param_tello_list(rospy.get_param('~tello_list', ''))

        self.automatic_recovery = rospy.get_param('~automatic_recovery', False)
        self.recovery_lock = Lock()
        self.recovery_queue = []   
        self.threads = {}
        for prefix in tello_list.keys():
            self.add_drone(prefix)

        if self.automatic_recovery:
            self.recovery_timer = rospy.Timer(rospy.Duration(1), self.process_recovery_queue)

        # ---- AP ----
        #if(rospy.get_param('~ap_mode', False)):
        #    self.tello = Tello()
        #    self.tello.connect()
        #    self.tello.connect_to_wifi(rospy.get_param('~ap_ssid'), rospy.get_param('~ap_password'))

        #Tello.LOGGER = RospyLogger()

    def add_drone(self, prefix):
        thread = Thread(target=TelloSwarmMember, daemon=True, args=(prefix,self,))
        thread.start()
        self.threads[prefix] = thread

    def process_recovery_queue(self, event):
        with self.recovery_lock:
            while len(self.recovery_queue) > 0:
                prefix = self.recovery_queue.pop()
                self.threads[prefix].join()
                self.add_drone(prefix)


class TelloSwarmMember:

    def __init__(self, prefix: str, node) -> None:
        self.node = node
        self.prefix = prefix
        self.eth_interface = self.pn('eth_interface')
        self.ssid = self.pn('ssid')

        # ---- IMU ----
        self.imu_msg = Imu()
        self.imu_msg.header.seq = -1

        self.old_state = None
        self.old_state_time = None 

        self.imu_publisher = rospy.Publisher(self.tn('imu'), Imu, queue_size=1)
        # ---- IMU End ----

        # ---- Settings ----
        self.video_frontend = TelloParameterParser.param_video_frontend(rospy.get_param('~video_frontend', 'av'))


        tello_ip = get_tello_ip(self.pn('mac'))
        print(tello_ip)

        self.tello = Tello(host=tello_ip,
            state_update_callback=self.imu_odometry_callback,
            av_open_lock=av_open_lock,
            video_frontend=self.video_frontend)

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
        self.cmd_vel_subscriber = rospy.Subscriber(self.tn('cmd_vel'), TwistStamped, self.cmd_vel)
        self.emergency_subscriber = rospy.Subscriber(self.tn('emergency'), Empty, self.cmd_emergency)

        # ---- Settings ----
        self.camera_direction = Tello.CAMERA_FORWARD
        self.tello.set_video_direction(self.camera_direction)

        self.camera_fps = rospy.get_param('~camera_fps', 30)
        self.tello.set_video_fps(TelloParameterParser.param_camera_fps(self.camera_fps))
        self.camera_fps = int(self.camera_fps)

        self.tello.set_video_bitrate(
            TelloParameterParser.param_camera_bitrate(self.pn('camera_bitrate', rospy.get_param('~camera_bitrate'))))
        self.tello.set_video_resolution(
            TelloParameterParser.param_camera_resolution(self.pn('camera_resolution', rospy.get_param('~camera_resolution'))))

        # ---- Camera ----
        self.tello.streamon()
        self.camera_direction_subscriber = rospy.Subscriber(self.tn('camera/direction'), CameraDirection, self.cmd_camera_direction),
        self.image_raw_publisher = rospy.Publisher(self.tn('camera/image_raw'), Image, queue_size=1)
        self.frame_read = self.tello.get_frame_read(callback=self.pub_image_raw)

        # ---- Recovery ----
        if self.node.automatic_recovery:
            self.recover_timer = rospy.Timer(rospy.Duration(1), self.recovery_callback)

        # ---- Keep-Alive Signal ----
        self.keep_alive_timer = rospy.Timer(rospy.Duration(3), self.keep_alive_callback)
    
    def keep_alive_callback(self, event):
        rospy.loginfo("keepalive")
        self.tello.send_keepalive()
    
    def recovery_callback(self, event):
        if self.tello.is_alive:
            if time.time() - self.tello.last_packet_received > 3:
                self.tello.is_alive = False
                print("Full drone disconnect detected! Queing drone for recovery process!")
                with self.node.recovery_lock:
                    self.node.recovery_queue.append(self.prefix)
            elif time.time() - self.tello.last_video_frame_received > 3:
                print("Video stream dropped out! Trying to recover video stream!")
                self.tello.streamoff()
                self.tello.streamon()
                self.frame_read.stop()
                self.frame_read = self.tello.get_frame_read()


    def imu_odometry_callback(self, state):

        now = rospy.Time.now()
        if self.old_state is not None:
            # ---- IMU ----
            self.imu_msg.header.seq = self.imu_msg.header.seq + 1
            self.imu_msg.header.stamp = now

            self.imu_msg.linear_acceleration.x = state['agx'] / 100.
            self.imu_msg.linear_acceleration.y = state['agy'] / 100.
            self.imu_msg.linear_acceleration.z = state['agz'] / 100.

            dt =  (now - self.old_state_time).to_sec()
            self.imu_msg.angular_velocity.x = (state['roll'] - self.old_state['roll']) / dt * (math.pi / 180.)
            self.imu_msg.angular_velocity.y = -(state['pitch'] - self.old_state['pitch']) / dt * (math.pi / 180.)
            self.imu_msg.angular_velocity.z = -(state['yaw'] - self.old_state['yaw']) / dt * (math.pi / 180.)
            
            q = quaternion_from_euler(state['roll'] * (math.pi / 180.), -state['pitch'] * (math.pi / 180.),
                                        -state['yaw'] * (math.pi / 180.))
            self.imu_msg.orientation.x = q[0]
            self.imu_msg.orientation.y = q[1]
            self.imu_msg.orientation.z = q[2]
            self.imu_msg.orientation.w = q[3]

            self.imu_publisher.publish(self.imu_msg)

        self.old_state = state
        self.old_state_time = now

    def pub_image_raw(self, frame):
        img_msg = self.bridge.cv2_to_imgmsg(self.frame_read.frame, 'rgb8')
        img_msg.header.stamp = rospy.Time.now()
        self.image_raw_publisher.publish(img_msg)

    def cmd_emergency(self, msg):
        self.tello.emergency()

    def cmd_takeoff(self, msg):
        self.tello.takeoff()

    def cmd_land(self, msg):
        self.tello.land()

    def cmd_vel(self, msg:TwistStamped):
        linear_multiplier = 1.25
        vel_vector = (np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]) * 100. * linear_multiplier).astype(np.int32)
        
        yaw_velocity = msg.twist.angular.z
        if abs(yaw_velocity) > (math.pi / 2.):
            rospy.logwarn_throttle(2, "The yaw-velocity provided is too high. The Tello can rotate at a maximum rate of pi/2 rad/s.")
        yaw_velocity = round((-yaw_velocity / (math.pi / 2.)) * 100)

        # NOTE: right-hand coordinate system
        self.tello.send_rc_control(-vel_vector[1].item(), vel_vector[0].item(), vel_vector[2].item(), yaw_velocity)

    def cmd_camera_direction(self, msg):
        if msg.forward:
            if self.camera_direction is not Tello.CAMERA_FORWARD:
                self.tello.set_video_direction(Tello.CAMERA_FORWARD)
                self.camera_direction = Tello.CAMERA_FORWARD
        elif self.camera_direction is not Tello.CAMERA_DOWNWARD: 
            self.tello.set_video_direction(Tello.CAMERA_DOWNWARD)
            self.camera_direction = Tello.CAMERA_DOWNWARD

    def __del__(self):
        #if self.video_thread is not None:
        #    self.video_thread.join()
        pass

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
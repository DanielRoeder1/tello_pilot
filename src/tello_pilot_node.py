#!/usr/bin/env python3

import time
import math

from utils.GetTelloIP import get_tello_ip

from anyio import sleep

import rospy
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, Imu
from sensor_msgs.msg import Joy
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
        if not tello_ip:
            raise ValueError('The tello drone could not be found in the local network')
        
        print(tello_ip)

        self.tello = Tello(host=tello_ip,
            state_update_callback=self.imu_odometry_callback,
            av_open_lock=av_open_lock,
            video_frontend=self.video_frontend)
        
        # ---- Teleop Control ----
        self.gamepad_controller = GamePadOperator(self.tello)

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
        self.mission_subscriber = rospy.Subscriber(self.tn("mission"),String, self.run_mission)
        self.teleop_subscriber = rospy.Subscriber("joy", Joy, self.gamepad_controller.control)

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
        #rospy.loginfo("keepalive")
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

    def run_mission(self,path):
        import datetime
        self.tello.enable_mission_pads()
        with open(path.data, "r") as file:
            self.tello.takeoff()
            pad = self.tello.get_mission_pad_id()
            rospy.loginfo(f"mission pad num {pad}")

            for line in file:
                if not line: break
                command_split = line.split()
                # Go
                if (command_split[0] == "go"):
                    rospy.loginfo("Running Go")
                    inputs = [int(l) for l in command_split[1:]]
                    if len(inputs) == 5:
                        self.tello.go_xyz_speed_mid(*inputs, timeout = 100)
                    elif len(inputs) == 4:
                        self.tello.go_xyz_speed(*inputs, timeout = 100)
                # Curve
                elif (command_split[0] == "curve"):
                    self.tello.curve_xyz_speed_mid(*command_split[1:])
                # Jump
                elif (command_split[0] == "jump"):
                    pass
                # Rotate
                elif (command_split[0] == "r"):
                    param = int(command_split[1])
                    if param > 0:
                        self.tello.rotate_clockwise(param)
                    else:
                        self.tello.rotate_counter_clockwise(abs(param))


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



# ----- Teleop Classes & Methods  ----- #

class GamepadState:
    def __init__(self):
        self.A = False
        self.B = False
        self.X = False
        self.Y = False
        self.LB = False
        self.RB = False
        self.back = False
        self.start = False
        self.power = False
        self.b_stick_l = False
        self.b_stick_r = False
        self.cross_left = False
        self.cross_right = False
        self.cross_top = False
        self.cross_bottom = False
        self.l_stick_up = 0
        self.l_stick_left = 0
        self.r_stick_up = 0
        self.r_stick_left = 0

class GamePadOperator():
    def __init__(self, tello):
        self.prev_state = GamepadState()
        self.unlock_stick_control = False
        self.tello = tello

    def control(self, msg):
        state = self.parse_msg(msg)

        if self.tello.is_flying:
            if state.back and not self.prev_state.back:
                self.tello.land()
            elif state.Y and not self.prev_state.Y:
                self.tello.move_up(20)
            elif state.A and not self.prev_state.A:
                self.tello.move_down(20)
            elif state.cross_top and not self.prev_state.cross_top:
                self.tello.move_forward(20)
            elif state.cross_bottom and not self.prev_state.cross_bottom:
                self.tello.move_back(20)
            elif state.cross_left and not self.prev_state.cross_left:
                self.tello.move_left(20)
            elif state.cross_right and not self.prev_state.cross_right:
                self.tello.move_right(20)
            elif state.power and not self.prev_state.power:
                self.switch_stick_control()

            if self.unlock_stick_control:
                cmd = self.parse_stick(state)
                self.tello.send_rc_control(*cmd)


        else:
            if state.start and not self.prev_state.start:
                self.tello.takeoff()
                
        self.prev_state = state

    def parse_msg(self, msg):
        state = GamepadState()

        state.A = msg.buttons[0] == 1
        state.B = msg.buttons[1] == 1
        state.X = msg.buttons[2] == 1
        state.Y = msg.buttons[3] == 1
        state.LB = msg.buttons[4] == 1
        state.RB = msg.buttons[5] == 1
        state.back = msg.buttons[6] == 1
        state.start = msg.buttons[7] == 1
        state.power = msg.buttons[8] == 1
        state.b_stick_l = msg.buttons[9] == 1
        state.b_stick_r = msg.buttons[10] == 1
        state.cross_left = msg.axes[6] == 1
        state.cross_right = msg.axes[6] == -1
        state.cross_top = msg.axes[7] == 1
        state.cross_bottom = msg.axes[7] == -1
        state.l_stick_up = msg.axes[1]
        state.l_stick_left = msg.axes[0]
        state.r_stick_up = msg.axes[4]
        state.r_stick_left = msg.axes[3]

        return state

    def switch_stick_control(self):
        self.unlock_stick_control = not self.unlock_stick_control

        if not self.unlock_stick_control:
            self.tello.send_rc_control(0,0,0,0)
            rospy.loginfo(f"Locked stick control")
            return

        rospy.loginfo(f"Unlocked stick control")

    def parse_stick(self, state):
        safeguard = 50
        left_right = -int(state.l_stick_left * safeguard)
        forward_backward = int(state.l_stick_up * safeguard)
        up_down = int(state.r_stick_up * safeguard)
        yaw_velocity = -int(state.r_stick_left * safeguard)

        return (left_right, forward_backward, up_down, yaw_velocity)




# ------------------------------------ #

def main():
    rospy.init_node('tello_pilot_node')
    TelloNode()
    rospy.spin()


if __name__ == '__main__':
    main()
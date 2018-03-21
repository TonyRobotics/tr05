#!/usr/bin/python2

import sys
import time
import math
import struct

import serial
import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

class TRDSerialDriver():

    def __init__(self, serialport_name, baudrate):
        self.serialport_name = serialport_name
        self.baudrate = baudrate
        self.conn = serial.Serial(self.serialport_name, self.baudrate)
        self.encoder1_offset = 0
        self.encoder2_offset = 0
        self.first_time_flag = True

    def reconnect(self, side):
        print(side, 'reconnecting...')
        while True:
            try:
                self.conn = serial.Serial(self.serialport_name, self.baudrate)
            except:
                time.sleep(1)
                print(side, 'reconnecting...')
            else:
                print(side, 'reconnected!')
                break

    def send_cmd(self, cmd):
        for i in range(len(cmd)-2):
            cmd[-2] ^= cmd[i]
        while True:
            try:
                self.conn.write(cmd)
                break
            except serial.serialutil.SerialException:
                self.reconnect('write')

    def get_response(self, n):
        try:
            res = self.conn.read(n)
        except serial.serialutil.SerialException:
            self.reconnect('read')
            return None
        else:
            # using bytearray for Python 2.x
            res = bytearray(res)
            return res

    def reset_base(self):
        print('Resetting Base...')
        cmd = [0xea, 0x03, 0x50, 0x00, 0x0d]
        self.send_cmd(cmd)
        time.sleep(3)
        print('Reset Base:', self.get_response(9))

    def reset_encoder(self):
        cmd = [0xea, 0x03, 0x35, 0x00, 0x0d]
        self.send_cmd(cmd)
        print('Reset Encoder:', self.get_response(5))

    def enable_timeout(self):
        cmd = [0xea, 0x02, 0x39, 0x00, 0x0d]
        self.send_cmd(cmd)
        print('Enable Timeout:', self.get_response(5))

    def set_speed(self, v1, v2):
        cmd = [0xea, 0x04, 0x31, v1, 0x00, 0x0d]
        self.send_cmd(cmd)
        cmd = [0xea, 0x04, 0x32, v2, 0x00, 0x0d]
        self.send_cmd(cmd)

    def get_mode(self):
        cmd = [0xea, 0x03, 0x2b, 0x00, 0x0d]
        self.send_cmd(cmd)
        res = self.get_response(6)
        return res

    def get_encoders(self):
        cmd = [0xea, 0x03, 0x25, 0x00, 0x0d]
        self.send_cmd(cmd)
        res = self.get_response(13)
        if not res:
            return (None, None)
        encoder1, = struct.unpack('>i', res[3:7])
        encoder2, = struct.unpack('>i', res[7:11])
        if self.first_time_flag:
            self.encoder1_offset = encoder1
            self.encoder2_offset = encoder2
            self.first_time_flag = False
        encoder1 -= self.encoder1_offset
        encoder2 -= self.encoder2_offset
        #print('{}: {} {}'.format(time.time(), encoder1, encoder2))
        return (encoder1, encoder2)

    def get_version(self):
        cmd = [0xea, 0x03, 0x51, 0x00, 0x0d]
        self.send_cmd(cmd)
        res = self.get_response(13)
        return res[2:11]

class DriverNode():

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.serialport_name = rospy.get_param('~serialport_name', \
                                               default='/dev/motor_trd')
        self.baudrate = rospy.get_param('~baudrate', default=38400)
        self.linear_coef = rospy.get_param('~linear_coef', default=82)
        self.angular_coef = rospy.get_param('~angular_coef', default=14.64)
        self.left_coef = rospy.get_param('~left_coef', default=1)
        self.right_coef = rospy.get_param('~right_coef', default =1)
        self.encoder_ticks_per_rev = rospy.get_param('~encoder_ticks_per_rev', \
                                                     default=1600)
        self.base_width = rospy.get_param('~base_width', default=0.39)
        self.wheel_diameter = rospy.get_param('~wheel_diameter', default=0.125)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.sub_vel = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.encoder1_prev = 0
        self.encoder2_prev = 0
        self.time_prev = rospy.Time.now()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.serial_driver = TRDSerialDriver(self.serialport_name, self.baudrate)
        self.serial_driver.reset_encoder()
        self.serial_driver.reset_base()
        self.serial_driver.enable_timeout()

    def vel_callback(self, vel_msg):
        v1 = self.linear_coef * vel_msg.linear.x
        v2 = self.linear_coef * vel_msg.linear.x
        v1 -= self.angular_coef * vel_msg.angular.z
        v2 += self.angular_coef * vel_msg.angular.z
        v1 *= self.left_coef
        v2 *= self.right_coef
        v1 += 128
        v2 += 128
        v1 = int(v1) if v1<255 else 255
        v2 = int(v2) if v2<255 else 255
        v1 = int(v1) if v1>0 else 0
        v2 = int(v2) if v2>0 else 0
        self.serial_driver.set_speed(v1, v2)

    def update_odom(self):
        (encoder1, encoder2) = self.serial_driver.get_encoders()
        if None==encoder1 or None==encoder2:
            print('Encoder(None, None)')
            return
        time_current = rospy.Time.now()
        time_elapsed = (time_current - self.time_prev).to_sec()
        self.time_prev = time_current
        dleft = self.left_coef * math.pi * self.wheel_diameter * \
                (encoder1 - self.encoder1_prev) / self.encoder_ticks_per_rev
        dright = self.right_coef * math.pi * self.wheel_diameter * \
                (encoder2 - self.encoder2_prev) / self.encoder_ticks_per_rev
        d = (dleft + dright) / 2
        dtheta = (dright - dleft) / self.base_width
        if d != 0:
            dx = math.cos(dtheta) * d
            dy = -math.sin(dtheta) * d
            self.x += dx*math.cos(self.theta)-dy*math.sin(self.theta)
            self.y += dx*math.sin(self.theta)+dy*math.cos(self.theta)
        self.theta += dtheta

        self.odom.header.stamp = time_current
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0,0,self.theta)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        self.odom.twist.twist.linear.x = d / time_elapsed
        self.odom.twist.twist.angular.z = dtheta / time_elapsed

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.update_odom()
                self.pub_odom.publish(self.odom)
                self.tf_broadcaster.sendTransform(
                    (self.x,self.y,0),
                    tf.transformations.quaternion_from_euler(0, 0, self.theta),
                    rospy.Time.now(),
                    'base_link',
                    'odom')
                rate.sleep()
            except KeyboardInterrupt:
                print('exit.')
                break

if __name__=='__main__':
    driver_node = DriverNode('trd_driver')
    driver_node.run()


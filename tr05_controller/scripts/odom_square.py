#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from tr10_controller.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

class OdomSquare():
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_square', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        rate = 20

        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

        # Set the parameters for the target square
        goal_distance = rospy.get_param("~goal_distance", 1.0)
        goal_angle = radians(rospy.get_param("~goal_angle", 90))
        linear_speed = rospy.get_param("~linear_speed", 0.2)
        angular_speed = rospy.get_param("~angular_speed", 0.7)
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2))

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_link for TR10
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # INitialize the tf litener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Set the base frame
        self.base_frame = '/base_link'

        # Initialize the position variable as a Point type
        position = Point()

        # Cycle through the four sides of the square
        for i in range(4):
            # Initialize the movement command
            move_cmd = Twist()

            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed

            # Get the starting position values
            (position, rotation) = self.get_odom()

            x_start = position.x
            y_start = position.y

            # Keep track of the distance traveled
            distance = 0

            # Enter the loop to move along a side
            while distance < goal_distance and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)

                r.sleep()

                # Get the current position
                (position, rotation) = self.get_odom()

                # Compute the Euclidean distance from the start
                distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

            # Stop the robot before rotating
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)

            # Set the movement command to a rotation
            move_cmd.angular.z = angular_speed

            # Track the las angle measured
            last_angle = rotation 

            # Track how far we have turned
            turn_angle = 0

            # Begin the rotation
            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)

                r.sleep()

                # Get the current rotation
                (position, rotation) = self.get_odom()

                # Compute teh amount of rotation since the last loop
                delta_angle = normalize_angle(rotation - last_angle)

                turn_angle += delta_angle
                last_angle = rotation

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)

        # Stop the robot when we are done
        self.cmd_vel.publish(Twist())

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        OdomSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")

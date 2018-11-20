#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan
from sensor_msgs.msg import LaserScan
import sys

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('bug2', anonymous=False)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # Connect the node with larse sensor
        scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)


        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # rate = 10
        rate = 5
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)

        # speed of the robote
        self.linear_speed = 0.4

        # Set the rotation speed in radians per second
        self.angular_speed = 0.5

        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = radians(5.0)

        # anything to start, initialize the goal with 1
        self.g_range_ahead = 1

        # Set the odom frame
        self.odom_frame = '/odom'

        # list of position the robot have visited with m-line
        self.visited_mline_position = []

        # time of first n times hit the m-line, first hit the m-line, the robot still around the m-linear
        self.time_of_tolerance_mline = 6

        # indicate the status of the robot: driving forward means follow the m line, false means bypass the obstacle
        # initial value is true,
        self.driving_forward = True

        self.self_rotation = False

        # params for the robot to work
        self.MAX_LENGTH_LARSE = sys.float_info.max

        # detection range
        self.FORWARD_RANGE_DISTANCE = 1.35
        self.TRUNING_RIGHT_DISTANCE = 2.05
        self.ROTATE_DEGREE = 3

        self.goal = Point(10,0,0)
        # Give tf some time to fill its buffer
        # rospy.sleep(2)
        print 'initialize the robot'
        rospy.sleep(2)

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    def scan_callback(self, msg):
        # callback function for LaserScan
        # get the closest obstacle

        larse_min = self.MAX_LENGTH_LARSE
        for i in msg.ranges:
            # should conside the value NaN
            if not isnan(i):
                larse_min = min(larse_min, i)
        # update the global value
        self.g_range_ahead = larse_min


    def move_forward(self, goal_distance):
    # control the robot to move forward {meter}
        print 'move forward '

        # Initialize the movement command
        move_cmd = Twist()

        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed

        for i in range(goal_distance):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()


    def rotate(self, goal_angle):
        # rotate the robot
        print 'rotate the robot'

        # movement pass to the robot
        move_cmd = Twist()

        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed

        # negative rotation
        if goal_angle < 0:
            move_cmd.angular.z = -self.angular_speed

        for i in range(abs(goal_angle)):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()


    def distance_to_goal(self, position, goal):
        # distance to the goal
        # Compute the Euclidean distance from the start
        return sqrt(pow((position.x - goal.x), 2) +
                        pow((position.y - goal.y), 2))

    def achive_goal(self, position):
        # whether the robot arrive the goal
        if self.distance_to_goal(position, self.goal) <= 0.3 :
            return True
        else:
            return False


    def hit_mline(self, position):
        # check hit the m-line
        print 'test hit m -liine '

        # we already know the target is (10,0,0), then we just need to consider the position.y
        if abs(position.y) < 0.2:
            print 'hit m line'

            visited = False
            closer = True
            current_position_dis = self.distance_to_goal(position,self.goal)

            # consider if the hit position already been visited;
            if len(self.visited_mline_position) > 0:
                for index in range(len(self.visited_mline_position)):
                    # test whether the robot is closer

                    distance_to_former_hit_position = self.distance_to_goal(position, self.visited_mline_position[index])
                    # tolerance of hit position, distance less than 0.5 consider the same hit position
                    if distance_to_former_hit_position < 0.5:
                        # has been this position, there is no solution
                        print 'hit the position again, no solution'
                        if i == 0:
                            self.shutdown()
                        return False

            # add the position into the visited m-line current_position
            print 'never been there, save the position into the visited list'

            self.visited_mline_position.append(position)
            self.driving_forward = True
        # if hit the m-line , then rotate the robot
            (position, rotation) = self.get_odom()
            while abs(rotation) > self.angular_tolerance:
                self.rotate(1)
                (position, rotation) = self.get_odom()
            return True
        else:
            # don't hit the
            print 'not hit m -line'
            return False



    def follow_mline(self):
        # follow the m_line
        position = Point()
        # get the initial position
        (position, rotation) = self.get_odom()

        print 'start running'
        # count = 0

        # if hit the goal destination, the tolerance is 0.35 meters
        while (not self.achive_goal(position)):
            # the robot have different status,
            # the robot driving forward
            if self.driving_forward:
                print 'driving forward'
                # first go forward
                self.move_forward(1)
                if self.g_range_ahead < self.FORWARD_RANGE_DISTANCE:
                    self.driving_forward = False
                    # save the position of leaving m-line
                    # update the position of robote
                    (position, rotation) = self.get_odom()
                    self.visited_mline_position.append(position)

            elif self.g_range_ahead < self.FORWARD_RANGE_DISTANCE:
                # find some obstacle
                print 'trun left'
                self.self_rotation = True
                self.rotate(self.ROTATE_DEGREE)
                (position, rotation) = self.get_odom()
            elif (not self.self_rotation) and self.g_range_ahead > self.TRUNING_RIGHT_DISTANCE:
                # after turning left then the robot forward a distance,
                print 'turn right'
                self.rotate(-self.ROTATE_DEGREE)
                (position, rotation) = self.get_odom()
            else:
                print 'move farwrad around the obstacle'
                self.move_forward(5)

                self.self_rotation = False

                (position, rotation) = self.get_odom()

                # whether hit the m line
                self.hit_mline(position)


    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    #try:
    #pdb.set_trace()
    rob = OutAndBack()
    rob.follow_mline()

    #except:
     #   rospy.loginfo("bug2 node terminated.")

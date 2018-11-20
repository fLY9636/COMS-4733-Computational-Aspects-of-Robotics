#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
from scipy.spatial import ConvexHull

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from math import radians, pi


class Lab():
    def __init__(self):
        # Give the node a name
        rospy.init_node('calibrate_angular', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', 360.0))

        self.speed = rospy.get_param('~speed', 0.5) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians

        # global store
        self.obstacles = []

        # two point form a edge
        self.obstacles_edge = set()

        self.grow_obstacles = []

        # all the edges, including the obstacles edges
        self.edges = set()

        self.marker_id = 0; # the marker id should be identical

        self.start = [0,0]
        self.goal = [0,0]
        self.count = 0
        
        #self.markers = []; # list of markers, every line is a marker
 
        # Initialize the visualization markers for RViz
        self.init_markers()

         # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")


        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def load_obstacles(self, object_path):
        obstacles = []
        obstacle = []
        with open(object_path) as f:
            numObstacles = int(f.readline())
            coordinates = int(f.readline())
            for i in range(coordinates):
                line = f.readline()
                obstacle.append(list(map(int, line.strip().split(' '))))
            for line in f:
                coordinates = list(map(int, line.strip().split(' ')))
                if len(coordinates) == 1:
                    obstacles.append(obstacle)
                    obstacle = []
                else:
                    obstacle.append(coordinates)
        obstacles.append(obstacle)
        assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
        return obstacles        


    def load_goal(self, goal_path):
        with open(goal_path) as f:
            line = f.readline()
            goal = list(map(int, line.strip().split(' ')))
        return goal

    def draw_Obstacles(self):
        obstacles = self.load_obstacles("../data/world_obstacles.txt")
        self.goal = self.load_goal("../data/goal.txt")
        start = [0, 0]

        # draw obstacles
        for ob in obstacles:
            self.obstacles.append(ob)
    

    def convex_hull(self):
        for ob in self.obstacles:
            #print(ob)

            new_ob = list()
            for point in ob:
                # grow point to 4 point based on the robot
                new_ob.append([(point[0]-18)/100.0,(point[1]-18)/100.0])
                new_ob.append([(point[0]-18)/100.0,(point[1]+18)/100.0])
                new_ob.append([(point[0]+18)/100.0,(point[1]-18)/100.0])
                new_ob.append([(point[0]+18)/100.0,(point[1]+18)/100.0])
           
            #print('new obj')
            #print(new_ob)
            hull = ConvexHull(new_ob)

            # get a new marker for a obstacle
            marker = self.publish_markers()
            
            # store all the point 
            pointList = []  

            for point in hull.vertices:
                pointList.append(Point(new_ob[point][0],new_ob[point][1],0))
                #self.grow_obstacles.append(Point(new_ob[point][0],new_ob[point][1],0))
                #pointList.append(Point(new_ob[point][0],new_ob[point][1],0))
                #waypoints.append(Pose(Point(3, 0.0, 0.0), quaternions[0]))
                marker.points.append(Point(new_ob[point][0],new_ob[point][1],0))

            # connect the first point with the last point
            marker.points.append(pointList[0]) 

            self.grow_obstacles.append(pointList)
            # print(self.grow_obstacles)
            # add the first element to fill a convex shape
            pointList.append(pointList[0])

            
            # get all the obstacle edges
            for i in range(len(pointList)-1):
                # obstacle edges
                self.obstacles_edge.add(((pointList[i].x,pointList[i].y),(pointList[i+1].x,pointList[i+1].y)))
                self.edges.add(((pointList[i].x,pointList[i].y),(pointList[i+1].x,pointList[i+1].y)))
                # print(self.obstacles_edge)

            # generate obstacle edges
            self.marker_arr.markers.append(marker)

            # draw the last added
            self.marker_pub.publish(self.marker_arr.markers[-1])


    def init_markers(self):
        # Set up our waypoint markers
        self.marker_scale = 0.015
        self.marker_lifetime = 0 # 0 is forever
        self.marker_ns = 'waypoints'
        self.marker_arr = MarkerArray()
        # self.marker_id += 1
        self.marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=15)

    def publish_markers(self):
        # Initialize the marker points list.
        markers = Marker()
        markers.ns = self.marker_ns
        
        self.marker_id = self.marker_id + 1
        markers.id = self.marker_id
        #markers.type = Marker.LINE_LIST
        markers.type = Marker.LINE_STRIP
        markers.action = Marker.ADD
        markers.lifetime = rospy.Duration(self.marker_lifetime)
   

        markers.scale.x = self.marker_scale
        markers.scale.y = self.marker_scale
        markers.color.r = self.marker_color['r']
        markers.color.g = self.marker_color['g']
        markers.color.b = self.marker_color['b']
        markers.color.a = self.marker_color['a']

        markers.header.frame_id = 'odom'
        markers.header.stamp = rospy.Time.now()
        markers.points = list()
        return markers

    def draw_one_edge(self, p1, p2):
        # draw an edge according to two points
        # Initialize the marker points list.
        markers = Marker()
        markers.ns = self.marker_ns
        
        self.marker_id = self.marker_id + 1
        markers.id = self.marker_id
        #markers.type = Marker.LINE_LIST
        markers.type = Marker.LINE_STRIP
        markers.action = Marker.ADD
        markers.lifetime = rospy.Duration(self.marker_lifetime)
   

        markers.scale.x = self.marker_scale
        markers.scale.y = self.marker_scale
        markers.color.r = self.marker_color['r']
        markers.color.g = self.marker_color['g']
        markers.color.b = self.marker_color['b']
        markers.color.a = self.marker_color['a']

        markers.header.frame_id = 'odom'
        markers.header.stamp = rospy.Time.now()
        markers.points = list()
        markers.points.append(Point(p1.x, p1.y, 0))
        markers.points.append(Point(p2.x, p2.y, 0))

        self.marker_arr.markers.append(markers)
        # update the last one
        self.marker_pub.publish(self.marker_arr.markers[-1])
        # return markers



    def onSegment(self, p1,p2,p3,p4):
        # Given three colinear points p, q, r, the function checks if 
        # point q lies on line segment 'pr' 
        if (max(p1[0], p2[0]) > min(p3[0], p4[0])
        and max(p3[0], p4[0]) > min(p1[0], p2[0])
        and max(p1[1], p2[1]) > min(p3[1], p4[1])
        and max(p3[1], p4[1]) > min(p1[1], p2[1])):
            return True
        return False

    # To find orientation of ordered triplet (p, q, r). 
    #  The function returns following values 
    # 0 --> p, q and r are colinear 
    # 1 --> Clockwise 
    # 2 --> Counterclockwise 
    def orientation(self,p1,p2,p3):

        return (p2[0]-p1[0]) *(p3[1] -p1[1]) - (p3[0]-p1[0]) *(p2[1]-p1[1])
        #33return x1*y2-x2*y1

    # The main function that returns true if line segment 'p1q1' 
    # and 'p2q2' intersect. 
    def doIntersect(self, p1, p2, p3, p4):
    #def doIntersect(self, p1, q1, p2, q2):
        # Main function to check whether the closed line segments p1 - q1 and p2 - q2 intersect 
        # Find the four orientations needed for general and 
        isIntersected = False
        if self.onSegment(p1,p2,p3,p4):
            if (self.orientation(p1, p2, p3) * self.orientation(p1, p2, p4) < 0 
                and self.orientation(p3, p4, p1) * self.orientation(p3, p4, p2) < 0):
                isIntersected = True
            else:
                isIntersected = False
        else:
            isIntersected = False
        return isIntersected
    

    def valid_edge(self, p1,p2 ):
        # check an edge with all the existing edges
        m1 = (p1.x, p1.y)
        m2 = (p2.x, p2.y)
        
        for edge in self.obstacles_edge:
            # ((q1.x, q1.y),(q2.x,q2.y))
            q1,q2 = edge
            #if self.hasSamePoint(m1,m2,q1,q2):
             #   continue
            print('m1:', q1, 'm2: ', q2)
            #print('m1:', q1, 'm2: ', q2)
            # check if intesect
            if self.doIntersect(m1,m2,q1,q2):
                # print 
                return False
        #self.count = self.count + 1
        #print(self.count)
        return True        
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        # self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def visibility_graph(self):
    # generate visibility graph , draw line one by one
    # add start end goal to the vertex set
        self.grow_obstacles.append([Point(0,0,0)])
        self.grow_obstacles.append([Point(self.goal[0]/100,self.goal[1]/100,0)])
        # visited every point in the vertex set
        
        # visited every point in the vertex set
        for i in range(len(self.grow_obstacles)):
            cur_obstacle = self.grow_obstacles[i]
            #print('p1', p1)
           
            for j in range(len(cur_obstacle)):
                p1 = cur_obstacle[j]

                 # combine with other vertex to an edge
                for k in range(len(self.grow_obstacles)):
                    # not in the same obstacle
                    if i != k:
                        for m in range(len(self.grow_obstacles[k])):
                            p2 = self.grow_obstacles[k][m]
                            # check is valid edge
                            if self.valid_edge(p1,p2):
                                #draw an edge
                                print('p1: ',p1, '; p2: ', p2)
                                self.draw_one_edge(p1,p2)
                                # add to edge set
                                self.edges.add(((p1.x,p1.y),(p2.x,p2.y)))

      

if __name__ == "__main__":
    lab = Lab()
    # drwa obstacles
    lab.draw_Obstacles()
    lab.convex_hull()
    lab.visibility_graph()
    # generate hull

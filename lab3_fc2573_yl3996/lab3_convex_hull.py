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
        self.grow_obstacles = []
        self.marker_id = 0; # the marker id should be identical

        self.start = (0,0)
        
        #self.markers = []; # list of markers, every line is a marker

        # two point form a edge
        self.obstacles_edge = set()

        # points = np.random.rand(30, 2) 
        #print('points: ')
        #print(points)

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)

        # Then convert the angles to quaternions
        #for angle in euler_angles:
         #   q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
          #  q = Quaternion(*q_angle)
           # quaternions.append(q)

        # Create a list to hold the waypoint poses
        #waypoints = list()

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        #waypoints.append(Pose(Point(3, 0.0, 0.0), quaternions[0]))
        #waypoints.append(Pose(Point(3, 3, 0.0), quaternions[1]))
        #waypoints.append(Pose(Point(0.0, 3, 0.0), quaternions[2]))
        #waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))

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

        # Set a visualization marker at each waypoint

        #for waypoint in waypoints:
         #   p = Point()
          #  p = waypoint.position
            #self.markers.points.append(p)

        # Update the marker display
        #for i in range[4]:
        #self.marker_pub.publish(self.markers)
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

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

    """
    ------------------------------------------
    (0, 0) is at the center of the map;
    (0, 0) is at the top left of the image

    Thus, we need some transformation
    ------------------------------------------
    """

    def map2img(self, ob):
        """ transform an obstacle in map frame to img frame """
        ob_tranformed = []
        t = np.array([[1, 0, 0, 300],
                        [0, -1, 0, 300],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
        for p in ob:
            p = np.array(p) # in case it is not already numpy array
            p = np.hstack((p, [0, 1]))
            p = t.dot(p).astype(int)
            ob_tranformed.append(p[:2])
        return np.array(ob_tranformed)

    def img2map(self, ob):
        """ transform an obstacle in img frame to map frame """
        ob_tranformed = []
        t = np.array([[1, 0, 0, -300],
                        [0, -1, 0, 300],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
        for p in ob:
            p = np.array(p) # in case it is not already numpy array
            p = np.hstack((p, [0, 1]))
            p = t.dot(p).astype(int)
            ob_tranformed.append(p[:2])
        return np.array(ob_tranformed)


    def draw_Obstacles(self):
        obstacles = self.load_obstacles("../data/world_obstacles.txt")
        goal = self.load_goal("../data/goal.txt")
        start = [0, 0]

        # draw obstacles
        for ob in obstacles:
            self.obstacles.append(ob)
            #print(self.obstacles)

        # cv2.fillConvexPoly(img, ob.reshape(-1, 1, 2), (255,255,0))

        # draw start and goal point
        # goal = tuple(self.map2img([goal])[0])
        # start = tuple(self.map2img([start])[0])
        # cv2.circle(img, goal, 7, (100, 0, 0), -1)
        # cv2.circle(img, start, 7, (0, 0, 100), -1)

        # import pdb; pdb.set_trace()
        # cv2.polylines(img,[pts],True,(0,255,255), thickness=1)

        # cv2.imshow( "Display window", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imwrite('../maps/map.png',img)


    def convex_hull(self):
        for ob in self.obstacles:
            print(ob)

            new_ob = list()
            for point in ob:
                # grow point to 4 point based on the robot
                new_ob.append([(point[0]-18)/100.0,(point[1]-18)/100.0])
                new_ob.append([(point[0]-18)/100.0,(point[1]+18)/100.0])
                new_ob.append([(point[0]+18)/100.0,(point[1]-18)/100.0])
                new_ob.append([(point[0]+18)/100.0,(point[1]+18)/100.0])

            #
            self.grow_obstacles.append(new_ob)
            #print('new obj')
            #print(new_ob)
            hull = ConvexHull(new_ob)

            # get a new marker for a obstacle
            marker = self.publish_markers()
            
            # store all the point 
            pointList = []  

            for point in hull.vertices:
                pointList.append(Point(new_ob[point][0],new_ob[point][1],0))
                #pointList.append(Point(new_ob[point][0],new_ob[point][1],0))
                #waypoints.append(Pose(Point(3, 0.0, 0.0), quaternions[0]))
                marker.points.append(Point(new_ob[point][0],new_ob[point][1],0))

            # connect the first point with the last point
            marker.points.append(pointList[0]) 

            # add the first element to fill a convex shape
            pointList.append(pointList[0])

            print(len(pointList))

            #for i in range(len(pointList)-2):
               # print(i)
                #self.obstacles_edge.add(((pointList[i].x,pointList[i].y),(pointList[i+1].x,pointList[i+1].y))
                #print(self.obstacles_edge)

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
        self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=25)

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
        # marker.pose.position.x = 1;
        # marker.pose.position.y = 1;
        # marker.pose.position.z = 1;
        # self.marker.pose.orientation.x = 0.0;
        # self.marker.pose.orientation.y = 0.0;
        #self.marker.pose.orientation.z = 0.0;
        #self.marker.pose.orientation.w = 1.0;

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

    def draw_edge(self, p1, p2):
        # draw an edge according to two points
        pass

    def generate_edge(self, p1, p2):
        # generate edge accodring to two points
        pass

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        # self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    # Create a black image
    # img = np.full((600,1200,3), 255, np.uint8)
    lab = Lab()
    # drwa obstacles
    lab.draw_Obstacles()
    lab.convex_hull()

    # generate hull

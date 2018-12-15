#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        rate = 5
        self.step = 0;
        self.stop = False
        self.r = rospy.Rate(rate)
    
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 29,  156,  120])
        upper_yellow = numpy.array([29, 255, 179])

        lower_red = numpy.array([ 150,  39,  40])
        upper_red = numpy.array([178, 255, 248])

        lower_green = numpy.array([ 60,  100,  50])
        upper_green = numpy.array([80, 255, 255])

        lower_blue = numpy.array([ 99,  39,  43])
        upper_blue = numpy.array([128, 255, 248])

        
        mask_green = cv2.inRange(hsv, lower_green, upper_green);
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue);
        mask_red = cv2.inRange(hsv, lower_red, upper_red);
       
        h, w, d = image.shape
        search_top = 7*h/8 - 18
        search_bot = 7*h/8 
    
       # yellow
        #if not self.detected:
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        mask_green[0:search_top, 0:w] = 0
        mask_green[search_bot:h, 0:w] = 0
        M_green = cv2.moments(mask_green)

        mask_blue[0:search_top, 0:w] = 0
        mask_blue[search_bot:h, 0:w] = 0
        M_blue = cv2.moments(mask_blue)

        mask_red[0:search_top, 0:w] = 0
        mask_red[search_bot:h, 0:w] = 0
        M_red = cv2.moments(mask_red)

        if not self.stop: 
            if M_red['m00'] > 0:
                print('detected red!!! stop')
                self.step = 51
                self.stop = True
                self.move_forward(0.3)

            elif M_green['m00'] > 0:
                print('detected Green!!! turn left')
                self.move_forward(0.4)
                #self.detected = True
            elif M_blue['m00'] > 0:
                print('detected blue!!! turn right')
                self.move_forward(-0.4)
                
            elif M['m00'] > 0:
                cx = int(M['m10']/M['m00'])    
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1) 
                err=cx-w/2
            
                self.twist.linear.x = 0.8
                self.twist.angular.z = -float(err)/100 
                self.cmd_vel_pub.publish(self.twist)
        else:
            self.move_forward(-0.3)

        cv2.imshow("window", image)
        cv2.waitKey(3)

    def move_forward(self, rotate):
    # control the robot to move forward {meter}
        #print 'move forward '
        self.twist.linear.x = 0.5
        if self.step == 1:

            self.shutdown()
            return
        elif self.step > 1:
            self.step = self.step -1
            self.twist.linear.x = 0.5
            
        # Set the movement command to forward motion
        
        self.twist.angular.z = rotate
        self.cmd_vel_pub.publish(self.twist)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        #rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        self.r.sleep()

rospy.init_node('follower')
follower = Follower()
rospy.spin()


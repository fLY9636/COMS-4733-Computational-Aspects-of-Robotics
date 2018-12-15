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
        self.step = 0
        self.star = 0
        self.direction = 0
        self.stop = False
        self.r = rospy.Rate(rate)
        self.count = 0
        self.leftArea = 0
        self.rightArea = 0
    
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 29,  156,  120])
        upper_yellow = numpy.array([29, 255, 179])

        lower_red = numpy.array([ 0,  40,  50])
        upper_red = numpy.array([20, 255, 248])
        
        
        l_y = numpy.array([ 30,  156,  120])
        u_y = numpy.array([30, 255, 179])
        
        mask_y = cv2.inRange(hsv, l_y, u_y)
        mask_y_left = cv2.inRange(hsv, l_y, u_y)
        mask_y_right = cv2.inRange(hsv, l_y, u_y)
        mask_y_left_middle = cv2.inRange(hsv, l_y, u_y)
        
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_red_left = cv2.inRange(hsv, lower_red, upper_red)
        mask_red_right = cv2.inRange(hsv, lower_red, upper_red);

        h, w, d = image.shape
        search_top = 7*h/8 - 15
        search_bot = 7*h/8 
        search_left = w/4
        search_right = 3*w/4
       # yellow
        #if not self.detected:
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask[0:(3*h/4+40), 0:w] = 0
        mask[(3*h/4+60):h, 0:w] = 0
        M = cv2.moments(mask)

        mask_red[0:search_top, 0:w] = 0
        mask_red[search_bot:h, 0:w] = 0
        mask_red[search_top: search_bot, search_left:search_right] = 0
        
        # the mask for triangle yellow
        mask_y[0:3*h/4, 0:w] = 0
        mask_y[(3*h/4+40):h, 0:w] = 0
        mask_y[3*h/4:(3*h/4+40), 0:w/7] = 0
        mask_y[3*h/4:(3*h/4+40), 6*w/7:w] = 0
        
#        mask_y_left_middle[0:h/5, 0:w] = 0
#        mask_y_left_middle[(3*h/4+40):h, 0:w] = 0
#        mask_y_left_middle[h/5:h, 0:w/3] = 0
#        mask_y_left_middle[h/5:h, w/7:w] = 0
        
        # check if big area appears in the mask_y
        S_y = self.bulkReport(mask_y)
        S_y_1 = self.bulkReport1(mask_y_left_middle, self.star)
        
        M_red = cv2.moments(mask_red)

        if S_y:
            self.star = self.star + 1

        if not self.stop: 
            if S_y and S_y_1:
                print('detected mark!!! stop and wait for order')

                self.count +=1
                if self.count >= 4:
#                    cv2.imshow("red image", image)
                    mask_y_left[0:search_top-80, 0:w] = 0
                    mask_y_left[search_bot+10:h, 0:w] = 0
                    mask_y_left[search_top-80:search_bot+10, w/2:w] = 0
                    mask_y_left[search_top-80:search_bot+10, 0:200] = 0

                    mask_y_right[0:search_top-80, 0:w] = 0
                    mask_y_right[search_bot+10:h, 0:w] = 0
                    mask_y_right[search_top-80:search_bot+10, 0:w/2] = 0

                    mask_y_right[search_top-80:search_bot+10, w-200:w] = 0
                 
                    self.leftArea = self.countArea(mask_y_left)
                    self.rightArea = self.countArea(mask_y_right)

                    print self.leftArea
                    print 'ddd'
                    print self.rightArea

                    result = self.leftArea - self.rightArea
               
                    if self.leftArea < 3000:
                        # stop  
                        #self.step = 51
                        print 'a star, stop and adjust to the center'
                        self.direction = 3
                        # turn right
                    elif result > 0:
                        # turn right
                        #self.step = 51
                        print 'a triangle, turn right'
                        self.direction = 2
                        
                    else:
                        # turn left
                        print 'a triangle, turn left'
                        self.direction = 1
                        
                        
                    self.step = 45
                    self.stop = True
                    self.count = 0
                   
                #cv2.cou
                #self.step = 51
                #self.direction = 1
                
                self.move_forward(0.3)
                # which direction, compare the are

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
        self.twist.angular.z = rotate
        # stop
        if self.step == 12 and self.direction == 3:
            self.shutdown()
            return
        elif self.step == 1:
            self.stop = False
        elif self.step > 1:
            
            # left
            if self.direction == 1 : 
                self.twist.angular.z = 0.28
                self.step = self.step -1
            #right
            elif self.direction == 2 :
                self.twist.angular.z = -0.3
                self.step = self.step -1
            elif self.direction == 3: 
                self.twist.angular.z = -0.4
                self.twist.linear.x = 0.3
                self.step = self.step -0.5
        
            
        # Set the movement command to forward motion
        
        
        self.cmd_vel_pub.publish(self.twist)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        #rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        self.r.sleep()

    def countArea(self, mask):
        area = 0
        for i in range(len(mask)):
            for j in range(len(mask[i])):
                if mask[i, j] != 0:
                    area +=1
        return area

    def bulkReport(self, mask):
        P = False
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 7000:
                P = True
                break
                
        return P
        
     
        
    def bulkReport1(self, mask1, mask2):
        P = False
        if (mask2 > 13) and (mask2 < 31):
            star = False
        else:
            star = True

        _, contours, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 5000:
                P = True
                break
        return star
                
         
rospy.init_node('follower')
follower = Follower()
rospy.spin() 


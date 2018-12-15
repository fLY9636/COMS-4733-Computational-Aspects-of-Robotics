## Group 15
Fujunku Chen 
UNI: fc2573

Yuxiang Liu    
UNI: yl3996


## Info
##### Operation System:
Ubuntu 14.04.5

##### Platform:
ROS Indigo
Python 2.7
OpenCV 3.1.0

## Usage
​1. Open the Ubuntu terminal 


2. Set the ROS 

```
roscore
```

​3. Open a new terminal and launch the TurtleBot in Gazebo(an example of running Part 2)
```
source ~/catkin_ws/devel/setup.bash

ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

​4. Open a new terminal and run the program(an example of running Part 2)
```
cd ~/the directory of follower_2.py file

python follower_2.py    
```

## Method
For the basic following yellow line, we use the yellow filter and apply the proportional adjustment as the reference code instructs.

For that markers are in the same shape but different colors, we design another three filters for the green, blue and red, respectively, with the uniform coverage. In this case, movement will be completely different over different ```if``` cases. Function ```move_forward(self, rotate)``` decides actions, *turn left*, *turn right*, and *stop*, depending on the value of ```rotate```.

For that markers are in the same color but different shapes, we design two filters for the triangle and the star, respectively, with the different coverage. We also design another two filters in order to distinguish between the triangle pointing left and right. Specifically, we focus on the square covered in each filter, compare them using ```countArea(self, mask)```, and finally divide cases into three. This time, due to the high complexity of ```countArea(self, mask)```, the turtlebot will first stop to calculate necessary parameters and then make the decision.

For that markers are all in yellow. We add another two filters to reinforce our measurement of square value. Functions ```bulkReport(self, mask)``` and ```bulkReport1(self, mask)``` are implemented. We also use the ```cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)``` and ```cv2.contourArea(contour)``` to calculate some squares in real time.

## Video

HW5: [https://www.youtube.com/playlist?list=PL1bJy6Gjm88_jCrzkThv1JFSB3dENtP7D](https://www.youtube.com/playlist?list=PL1bJy6Gjm88_jCrzkThv1JFSB3dENtP7D)

(Best under 720p or higher)


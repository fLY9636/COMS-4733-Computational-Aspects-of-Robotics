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

## Usage
​1. Open the Ubuntu terminal 


2.Set the ROS 

```
roscore
```

​3. Launch the simulated TurtleBot in Gazebo(open a new terminal)
```
cd /the directory of your world files/

roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/world file
```

​4. Run the program (open a new terminal)
```
source ~/catkin_ws/devel/setup.bash

cd /the directory of bug2.py file/

python bug2.py      
```

## Method
Function ```follow_mline(self)``` loops the course that the robot goes along the m-line or adjusts itself
to the outline of obstacles. When calling function ```hit_mline(self, position)```, it enables the robot 
to check whether it reaches a correct point to make another m-line following or just reaches the last hit point 
again. We use the y coordinate of the reached point to represent its distance to the m-line just because 
m-line coincides with x axis in our problem set. Funtion ```move_forward(self, goal_distance)``` and ```rotate(self, goal_angle)``` make it possible for the robot to adjust itself properly with customized input parameters.

## Video

bug2_0: [https://youtu.be/CbclWUBKW3I](https://youtu.be/CbclWUBKW3I)

bug2_1: [https://youtu.be/mXQ8LEfn42w](https://youtu.be/mXQ8LEfn42w)

bug2_2: [https://youtu.be/RF4NTP_CenQ](youtu.be/RF4NTP_CenQ)

bug2_3: [https://youtu.be/SQkTwPWIntk](https://youtu.be/SQkTwPWIntk)

bug2_5: [https://youtu.be/Eyl5fe7-k5g](https://youtu.be/Eyl5fe7-k5g)

bug2_extra: [https://youtu.be/UQc2w9hlqFc](https://youtu.be/UQc2w9hlqFc)

(Best under 720p or higher)


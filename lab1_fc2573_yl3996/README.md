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

​3. Launch the simulated TurtleBot (open a new terminal)
```
source ~/catkin_ws/devel/setup.bash

roslaunch rbx1_bringup fake_turtlebot.launch
```

​4. Bring up RViz    (open a new terminal)
```
source ~/catkin_ws/devel/setup.bash

rosrun rviz rviz -d `rospack find rbx1_nav` /sim.rviz
```

​5. Run the program (open a new terminal)
```
rosrun rbx1_nav timed_out_and_back.py      
```

## Method

Inputs are received by ```raw_input()``` in ```handleIput()```, and assigned to ```move_cmd``` by calling ```translation()``` or ```rotation()```.
Backward motion
is made by setting corresponding speed negative. ```handleInput()```
internally calls itself to loop the course, except that ```shutdown()``` is
triggered to terminate the course when receiving ‘q’ or ‘Q’ .



## Video

[https://youtu.be/yQQc8_ptcrA](https://youtu.be/yQQc8_ptcrA) (Please
use at least 720p)

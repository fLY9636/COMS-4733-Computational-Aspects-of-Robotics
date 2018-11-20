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

​3. Bring up ArbotiX turtlebot simulator and RViz
```
roslaunch vgraph launch.launch
```

​4. Run the program (an example)
```
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws/src/vgraph/src

python lab3_follow_path.py       
```

## Method
To follow the given instruction, our program is divided into four parts, calling functions ```draw_Obstacles()```, ```convex_hull()```, ```visibility_graph()```, ```shortest_path()``` and ```follow_path()``` respectively to let the turtlebot navigate itself to the goal spot.

Specifically, ```img2map(ob)``` is used to transform a map to an image, and we import package ```cv2``` to draw obstacles in the image. We use ```ConvexHull``` from package ```scipy.spatial``` to form each obstacle's convex hull and exhaustively generate edges among vertices. Then, ```onSegment(p1,p2,p3,p4)```, ```doIntersect(p1, p2, p3, p4)``` and ```valid_edge(p1,p2 )``` are used to exclude those edges which intersect with some other existing edges or have existing vertices lying on them. Accordingly, ```visibility_graph()``` generates a graph with all valid edges. Finally, we implemented the *Floyd Warshall Algorithm* to find the shortest path, and thus the turtlebot can navigate itself to the goal point by heading to every identified coordinate.

## Video

HW3: [https://www.youtube.com/watch?v=1MtARJ50AVM&list=PL1bJy6Gjm88_3sWjmnYdh7oSni6hCXzd_](https://www.youtube.com/watch?v=1MtARJ50AVM&list=PL1bJy6Gjm88_3sWjmnYdh7oSni6hCXzd_)

(Best under 720p or higher)


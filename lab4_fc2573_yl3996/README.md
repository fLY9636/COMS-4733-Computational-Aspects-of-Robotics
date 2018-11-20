## Group 15
Fujunku Chen 
UNI: fc2573

Yuxiang Liu    
UNI: yl3996


## Info
##### Operation System:
MAC OS/Windows/Linux

##### Platform:
Python 3.6.5


## Usage
​1. Open the PC terminal 

2. Rewrite the directory of .txt files
In both .py files, edit:
```
if __name__ == "__main__":

    ...

    path = build_obstacle_course('/*your directory of world_obstacles.txt*', ax)

    start, goal = add_start_and_goal('/*your directory of start_goal.txt*', ax)
    
    ...
    
    E_obs = E_obs('/*your directory of world_obstacles.txt*')
    
    ...
```

3. Run the program (an example of running single-directional RRT)
```
cd ~/*your directory of singledirectional_rrt.py*

python singledirectional_rrt.py world_obstacles.txt start_goal.txt
```

​4. Set the parameters
```
value of stepsize (should be bigger than density)

value of density (value less than 20 is recommended)
```

(In the video examples, we tested two RRT algorithms with: stepsize=30, density=15; stepsize=50, density=15; stepsize=70, density=20.)

NOTE: 
1. ```stepsize``` is the distance the point robot extends towards the random configuration each time.

2. ```density``` is the **minimum distance** between **a new figured configuration** and **those already in the vertex set** of RRT, which ensures the new configuration won't coincide with any of the existing ones. Of course, that *density* is 0 won't' affect the exploration, but will postpone the finish since many iterations will be wasted in seating new configurations where existing ones sit. *BUT make sure the stepsize is always bigger than the density you input, or the point robot is unable to extend at the first iteration!*


## Method
We store all segments that outline every obstacle. Function ```segment_intersection(A, B, CD)``` is used to check if segment AB intersects with CD. To be specific, we calculate the product of the cross product AC×AD and BC×BD, as well as the product of CA×CB and DA×DB, respectively. If both the above products are nonpositive, then AB must intersect with CD. Under this principle, we treat the last new configuration (given which is already in free space) as A, the current new configuration as B, and any edge of obstacles as CD. Thus, we can figure out if a new configuration falls into the free space during each iteration (see ```no_stride()``` function in the program).

We also notice that it sometimes takes a large number of iterations before finding the path. Actually this is caused by the overlap of the new configurations and the old ones. So we set a parameter ```density``` to control the reasonable distance between a new configuration and those already in the vertex set of RRT. Thus, every iteration ensures an authentically new configuration (see ```repetition_new()``` function in the program).

The main algorithms of single directional and bidirectional RRT are consistent with those covered in the class notes.

## Video

HW4: [https://www.youtube.com/playlist?list=PL1bJy6Gjm88_E7IXiPPHLXHTi39bRv6JO](https://www.youtube.com/playlist?list=PL1bJy6Gjm88_E7IXiPPHLXHTi39bRv6JO)

(Best under 720p or higher)


from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

# Generate all segments that outlining obstacles
def E_obs(obstacle_path):
    coordinates = list()
    with open(obstacle_path) as f:
        for line in f:
            x = tuple(map(int, line.strip().split(' ')))
            if len(x) > 1:
                coordinates.append(x)
            else:
                coordinates.append((0))
    coordinates.append((0))
    E_obs = list()
    new_obs = True
    for v in range(len(coordinates)-1):
        if coordinates[v] != 0:
            if coordinates[v+1] != 0:
                if new_obs == True:
                    w = v
                    new_obs = False
                E_obs.append((coordinates[v],coordinates[v+1]))
            else:
                E_obs.append((coordinates[v],coordinates[w]))
                new_obs = True
    E_obs.append(((1,1),(1,599)))
    E_obs.append(((1,599),(599,599)))
    E_obs.append(((599,599),(599,1)))
    E_obs.append(((599,1),(1,1)))

    
    return E_obs

# Convert segment to vector
def vector(v1, v2):
    vector = (v2[0]-v1[0],v2[1]-v1[1])
    
    return vector

# Calculate the cross product of two vectors
def cross_product(vector1, vector2):
    sin = vector1[0]*vector2[1]-vector2[0]*vector1[1]
    
    return sin

# Check if the segment of point v and reference point intersects with any edge of obstacles
def segment_intersection(r_point, v, e_obs):
    vector1 = vector(r_point, e_obs[0])
    vector2 = vector(r_point, e_obs[1])
    vector3 = vector(v, e_obs[0])
    vector4 = vector(v, e_obs[1])
    n_vector1 = vector(e_obs[0], r_point)
    n_vector3 = vector(e_obs[0], v)
    n_vector2 = vector(e_obs[1], r_point)
    n_vector4 = vector(e_obs[1], v)
    condition1 = cross_product(vector1, vector2)*cross_product(vector3, vector4)
    condition2 = cross_product(n_vector1, n_vector3)*cross_product(n_vector2, n_vector4)
   
    return condition1<=0 and condition2<=0

# Give out the minimum distance from a new configuration v_new to all configurations in V_RRT   
def repetition_new(v_new, V_RRT):
    d = list()
    for n in range(len(V_RRT)):
        d.append(math.sqrt(math.pow(v_new[0]-V_RRT[n][0],2)+math.pow(v_new[1]-V_RRT[n][1],2)))
        v_RRT = V_RRT[d.index(min(d))]
        min_d = min(d)
        
    return min_d, v_RRT

# Make sure each time the random configuration v_rand doesn't coincide with any of the configuration in V_RRT
def repetition_rand(v_rand, V_RRT):
    repetition = False
    for n in range(len(V_RRT)):
        if v_rand == V_RRT[n]:
            repetition = True
    
    return repetition

# Make sure each time the new configuration v_new falls in the free space
def no_stride(e0, e1, E_obs):
    no_stride = True
    for e in range(len(E_obs)):
        if segment_intersection(e0, e1, E_obs[e]) == True:
            no_stride = False
    
    return no_stride

# Generate a random configuration v_rand
def rand_vertex(start, goal, E_obs, V_RRT):
    while True:
        x_coordinate = random.randint(0,600)
        y_coordinate = random.randint(0,600)
        v_rand = (x_coordinate,y_coordinate)
        if repetition_rand(v_rand, V_RRT) == False:
            return v_rand
            break

# Find the nearest neighbor of v_rand in V_RRT           
def nearest_vertex(v_rand, V_RRT):
    d = list()
    for i in range(len(V_RRT)):
        d.append(math.sqrt(math.pow(v_rand[0]-V_RRT[i][0],2)+math.pow(v_rand[1]-V_RRT[i][1],2)))
        v_nearest = V_RRT[d.index(min(d))]
    
    return v_nearest

# Generate a new configuration one stepsize away from v_nearest in the direction of v_rand
def new_config(v_nearest, v_rand, stepsize):
    x_coordinate = int(v_nearest[0]+(v_rand[0]-v_nearest[0])/math.sqrt(math.pow(v_rand[0]-v_nearest[0],2)+math.pow(v_rand[1]-v_nearest[1],2))*stepsize)
    y_coordinate = int(v_nearest[1]+(v_rand[1]-v_nearest[1])/math.sqrt(math.pow(v_rand[0]-v_nearest[0],2)+math.pow(v_rand[1]-v_nearest[1],2))*stepsize)
    v_new = (x_coordinate,y_coordinate)
    e_new = (v_nearest,v_new)
    
    return v_new, e_new

# Add v_new to V_RRT
def add_vertex(v_new, V_RRT):
    V_RRT.append(v_new)
    
    return V_RRT

# Connect v_new and v_nearest and add e_new to E_RRT
def add_edge(v_nearest, v_new, E_RRT):
    E_RRT.append((v_nearest,v_new))
            
    return E_RRT

# Get the path connecting start and goal
def get_path(E_RRT, e_RRT, rootnode):
    path_RRT = list()
    path_RRT.append(e_RRT[1])
    path_RRT.append(e_RRT[0])
    ref = E_RRT.index(e_RRT)
    while path_RRT[len(path_RRT)-1] != rootnode:
        edge = ref-1
        while True:
            if E_RRT[edge][1] == E_RRT[ref][0]:
                path_RRT.append(E_RRT[edge][0])
                ref = edge
                break
            else:
                edge = edge-1
    
    return path_RRT

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                         help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                         help="File path for obstacle set")
    args = parser.parse_args()

    plt.ion()
    fig, ax = plt.subplots()
    plt.show()
    path = build_obstacle_course('/Users/mac/Desktop/world_obstacles.txt', ax)
    start, goal = add_start_and_goal('/Users/mac/Desktop/start_goal.txt', ax)

    stepsize = int(input("please type the stepsize:"))
    density = int(input("please type the density:"))
    V_RRT_1 = [start]
    V_RRT_2 = [goal]
    E_RRT_1 = list()
    E_RRT_2 = list()
    E_obs = E_obs('/Users/mac/Desktop/world_obstacles.txt')
    find_path = False
    for t in range(500):
        print(t)
        new_settle_1 = False
        new_settle_2 = False

        # Dominant round for the upper RRT
        if t%2==0:
            while new_settle_1 == False:
                v_rand = rand_vertex(start, goal, E_obs, V_RRT_1)
                v_nearest = nearest_vertex(v_rand, V_RRT_1)
                v_new, e_new = new_config(v_nearest, v_rand, stepsize)

                # Check if v_new is in free space and also if it coincides with former trials
                min_d, v_RRT_1 = repetition_new(v_new, V_RRT_1)
                if (no_stride(e_new[0], e_new[1], E_obs) == True and min_d > density):

                    ax.add_patch(patches.Circle(v_new, facecolor='xkcd:lightblue'))
                    codes = [Path.MOVETO, Path.LINETO]
                    vertices = np.array([v_nearest, v_new], float)
                    path = Path(vertices, codes)
                    pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
                    ax.add_patch(pathpatch)
                    plt.draw()
                    plt.pause(0.005)
                    V_RRT_1 = add_vertex(v_new, V_RRT_1)
                    E_RRT_1 = add_edge(v_nearest, v_new, E_RRT_1)

                    # Check if the upper RRT encounters the lower RRT
                    min_d, v_RRT_2 = repetition_new(v_new, V_RRT_2)
                    if min_d<=25:
                        find_path = True
                        finale_1 = v_new
                        finale_2 = v_RRT_2
                        break

                    v_nearest = nearest_vertex(v_new, V_RRT_2)
                    v_new, e_new = new_config(v_nearest, v_new, stepsize)
                    min_d, v_RRT_2 = repetition_new(v_new, V_RRT_2)

                    # Check if v_new is in free space and also if it coincides with former trials
                    if (no_stride(e_new[0], e_new[1], E_obs) == True and min_d > density):

                        new_settle_1 = True
                        ax.add_patch(patches.Circle(v_new, facecolor='xkcd:darkgreen'))
                        codes = [Path.MOVETO, Path.LINETO]
                        vertices = np.array([v_nearest, v_new], float)
                        path = Path(vertices, codes)
                        pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
                        ax.add_patch(pathpatch)
                        plt.draw()
                        plt.pause(0.005)
                        V_RRT_2 = add_vertex(v_new, V_RRT_2)
                        E_RRT_2 = add_edge(v_nearest, v_new, E_RRT_2)

                    # If v_new isn't available, do a random walk
                    else:
                        while True:
                            v_rand = rand_vertex(start, goal, E_obs, V_RRT_2)
                            v_nearest = nearest_vertex(v_rand, V_RRT_2)
                            v_new, e_new = new_config(v_nearest, v_rand, stepsize)
                            min_d, v_RRT_2 = repetition_new(v_new, V_RRT_2)
                            if (no_stride(e_new[0], e_new[1], E_obs) == True and min_d > density):
                                new_settle_1 = True
                                ax.add_patch(patches.Circle(v_new, facecolor='xkcd:darkgreen'))
                                codes = [Path.MOVETO, Path.LINETO]
                                vertices = np.array([v_nearest, v_new], float)
                                path = Path(vertices, codes)
                                pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
                                ax.add_patch(pathpatch)
                                plt.draw()
                                plt.pause(0.005)
                                V_RRT_2 = add_vertex(v_new, V_RRT_2)
                                E_RRT_2 = add_edge(v_nearest, v_new, E_RRT_2)
                                break

                    # Check if the lower RRT encounters the upper RRT       
                    min_d, v_RRT_1 = repetition_new(v_new, V_RRT_1)
                    if min_d<=25:
                        find_path = True
                        finale_2 = v_new
                        finale_1 = v_RRT_1
                        break

        # Dominant round for the lower RRT           
        else:
            while new_settle_2 == False:
                v_rand = rand_vertex(start, goal, E_obs, V_RRT_2)
                v_nearest = nearest_vertex(v_rand, V_RRT_2)
                v_new, e_new = new_config(v_nearest, v_rand, stepsize)

                # Check if v_new is in free space and also if it coincides with former trials
                min_d, v_RRT_2 = repetition_new(v_new, V_RRT_2)
                if (no_stride(e_new[0], e_new[1], E_obs) == True and min_d > density):

                    ax.add_patch(patches.Circle(v_new, facecolor='xkcd:darkgreen'))
                    codes = [Path.MOVETO, Path.LINETO]
                    vertices = np.array([v_nearest, v_new], float)
                    path = Path(vertices, codes)
                    pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
                    ax.add_patch(pathpatch)
                    plt.draw()
                    plt.pause(0.005)
                    V_RRT_2 = add_vertex(v_new, V_RRT_2)
                    E_RRT_2 = add_edge(v_nearest, v_new, E_RRT_2)

                    # Check if the lower RRT encounters the upper RRT
                    min_d, v_RRT_1 = repetition_new(v_new, V_RRT_1)
                    if min_d<=25:
                        find_path = True
                        finale_2 = v_new
                        finale_1 = v_RRT_1
                        break
                    
                    v_nearest = nearest_vertex(v_new, V_RRT_1)
                    v_new, e_new = new_config(v_nearest, v_new, stepsize)

                    # Check if v_new is in free space and also if it coincides with former trials
                    min_d, v_RRT_1 = repetition_new(v_new, V_RRT_1)
                    if (no_stride(e_new[0], e_new[1], E_obs) == True and min_d > density):

                        new_settle_2 = True
                        ax.add_patch(patches.Circle(v_new, facecolor='xkcd:lightblue'))
                        codes = [Path.MOVETO, Path.LINETO]
                        vertices = np.array([v_nearest, v_new], float)
                        path = Path(vertices, codes)
                        pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
                        ax.add_patch(pathpatch)
                        plt.draw()
                        plt.pause(0.005)
                        V_RRT_1 = add_vertex(v_new, V_RRT_1)
                        E_RRT_1 = add_edge(v_nearest, v_new, E_RRT_1)

                    # If v_new isn't available, do a random walk   
                    else:
                        while True:
                            v_rand = rand_vertex(start, goal, E_obs, V_RRT_1)
                            v_nearest = nearest_vertex(v_rand, V_RRT_1)
                            v_new, e_new = new_config(v_nearest, v_rand, stepsize)
                            min_d, v_RRT_1 = repetition_new(v_new, V_RRT_1)
                            if (no_stride(e_new[0], e_new[1], E_obs) == True and min_d > density):
                                new_settle_2 = True
                                ax.add_patch(patches.Circle(v_new, facecolor='xkcd:lightblue'))
                                codes = [Path.MOVETO, Path.LINETO]
                                vertices = np.array([v_nearest, v_new], float)
                                path = Path(vertices, codes)
                                pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
                                ax.add_patch(pathpatch)
                                plt.draw()
                                plt.pause(0.005)
                                V_RRT_1 = add_vertex(v_new, V_RRT_1)
                                E_RRT_1 = add_edge(v_nearest, v_new, E_RRT_1)
                                break
                            
                    # Check if the upper RRT encounters the lower RRT
                    min_d, v_RRT_2 = repetition_new(v_new, V_RRT_2)
                    if min_d<=25:
                        find_path = True
                        finale_1 = v_new
                        finale_2 = v_RRT_2
                        break
        if find_path == True:
            break
        
    codes = [Path.MOVETO, Path.LINETO]
    vertices = np.array([finale_1, finale_2], float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, edgecolor='xkcd:grey', linewidth=1)
    ax.add_patch(pathpatch)

    path_doubleRRT_1 = get_path(E_RRT_1, E_RRT_1[V_RRT_1.index(finale_1)-1], start)
    path_doubleRRT_2 = get_path(E_RRT_2, E_RRT_2[V_RRT_2.index(finale_2)-1], goal)

    for node_1 in range(len(path_doubleRRT_1)-1):
        ax.add_patch(patches.Circle(path_doubleRRT_1[node_1], facecolor='xkcd:orange'))
        codes = [Path.MOVETO, Path.LINETO]
        vertices = np.array([path_doubleRRT_1[node_1], path_doubleRRT_1[node_1+1]], float)
        path = Path(vertices, codes)
        pathpatch = patches.PathPatch(path, edgecolor='xkcd:yellow', linewidth=1.2)
        ax.add_patch(pathpatch)
    vertices = np.array([path_doubleRRT_1[0], path_doubleRRT_2[0]], float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, edgecolor='xkcd:yellow', linewidth=1.2)
    ax.add_patch(pathpatch)
    for node_2 in range(len(path_doubleRRT_2)-1):
        ax.add_patch(patches.Circle(path_doubleRRT_2[node_2], facecolor='xkcd:orange'))
        codes = [Path.MOVETO, Path.LINETO]
        vertices = np.array([path_doubleRRT_2[node_2], path_doubleRRT_2[node_2+1]], float)
        path = Path(vertices, codes)
        pathpatch = patches.PathPatch(path, edgecolor='xkcd:yellow', linewidth=1.2)
        ax.add_patch(pathpatch)

    print('Find path in %d extensions'%(t))
    plt.ioff()
    plt.show()

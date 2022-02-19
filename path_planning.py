import numpy as np
import pylab as pl
import sys
sys.path.append('osr_examples/scripts/')
import environment_2d
import math
import pickle

pl.ion()
np.random.seed(4)
env = environment_2d.Environment(10, 6, 5)
pl.clf()
env.plot()
q = env.random_query()
if q is not None:
  x_start, y_start, x_goal, y_goal = q
  env.plot_query(x_start, y_start, x_goal, y_goal)

print(env.size_x, env.size_y)



###### start planning with PRM

### data structure
class Point:
    x = 0.0
    y = 0.0

    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    node_id = None
    point = None
    

    def __init__(self, x, y, node_id):
        self.point = Point(x,y)
        self.node_id = node_id
        self.neighbors = []

### Auxiliary functions
def getEuclideanDistance(p1, p2):
    return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))

#checking collision to add edges, checking  3 points in the middle of the two nodes
def checkLineCollision(p1, p2):
    middle = not (env.check_collision((p1.x + p2.x)/2, (p1.y + p2.y)/2))
    middle_left = not (env.check_collision(p1.x/4 + 3*p2.x/4, p1.y/4 + 3*p2.y/4))
    middle_right = not (env.check_collision(p2.x/4 + 3*p1.x/4, p2.y/4 + 3*p1.y/4))
    if middle and middle_left and middle_right:
        return True
    else:
        return False

### 1. constuct nodes are randomly generated in C-free space
nodes = []

def generateRandomPoint(env, num_samples):
    count = 0 

    while(count < num_samples):
        new_node = False
        while new_node == False:
            x = np.random.rand()*env.size_x
            y = np.random.rand()*env.size_y
            if not env.check_collision(x, y):
                p = Node(x, y, count) ## create node and node_id          
                nodes.append(p)      
                new_node = True
                break
        count+=1
    
#plot the samples
generateRandomPoint(env, 1000)
x_coordinates = []
y_coordinates = []
for i in nodes:
    x_coordinates.append(i.point.x)
    y_coordinates.append(i.point.y)
pl.plot(x_coordinates, y_coordinates, "bs", markersize = 1)
pl.savefig("sampling")


### 2. Construct the edges - write it into NeighborGraph
def computeNeighborGraph(env, nodes):
    for i in nodes:
        distanceMap = []
        for j in nodes:
            if (i.node_id != j.node_id):
                if (checkLineCollision(i.point, j.point) and (getEuclideanDistance(i.point, j.point) < 0.5)):
                    distanceMap.append((getEuclideanDistance(i.point, j.point), j)) #append the distance and the node 
        
        #distanceMap = sorted(distanceMap, key=lambda x: x[0]) #sort the distance
        count_neighbor  = 0
        for pair in distanceMap:
                if (count_neighbor >=10): # get max 10 neighbors
                    break
                i.neighbors.append(pair[1])
                count_neighbor+=1

### draw the edges
computeNeighborGraph(env, nodes)
count_edges = 0
for i in nodes:
    for j in i.neighbors:
        pl.plot([i.point.x,j.point.x], [i.point.y,j.point.y], "g" , linewidth = 0.5)
        count_edges += 1
pl.savefig("edges")
print("number of edges " + str(count_edges/2))


### save  graph 
with open("graph.txt", "w") as f:
    for s in nodes:
        f.write(str(s) +"\n")
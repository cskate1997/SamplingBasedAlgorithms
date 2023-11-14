# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        p1 = [node1.row,node1.col]
        p2 = [node2.row,node2.col]
        d = math.dist(p1,p2)

        return d

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        p1 = [node1.row,node1.col]
        p2 = [node2.row,node2.col]
        ptx = []
        pty = []
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])
        # print(dx,dy,p1,p2)
        N = int(max((dx),(dy)))
        if (N==0):
            divN = 0.0
        else:
            divN = float(1/N)
        xstep = dx * divN
        ystep = dy * divN
        # x = p1[0]
        # y = p1[1]
        # for i in range (N-1):
        #     x+=xstep
        #     y+=ystep
        #     z = (round(x),round(y))
        #     pt.append(z)
        # print("xstep",xstep)
        # print("ystep",ystep)
        # print("N",N)
        if xstep ==0:
            ptx = np.array([p1[0] for i in range(N)])
            # print("halaluya")
        else:
            ptx = np.arange(min(p1[0],p2[0]), max(p1[0],p2[0]), xstep)
            # print(ptx, min(p1[0],p2[0]), max(p1[0],p2[0]), xstep)
            # print("j")

        if ystep ==0:
            pty = np.array([p1[1] for i in range(N)])
        else:
            pty= np.arange(min(p1[1],p2[1]),max(p1[1],p2[1]),ystep)
        # print("This ss", ptx,pty)  
        ptx =np.round(ptx).astype(int)
        pty =np.round(pty).astype(int)

        # print("This sssssss", ptx,pty)
        for pts in range(len(ptx)-1):
            g,l = ptx[pts], pty[pts]
            if self.map_array[g][l] ==0:
                return True
            

        return False


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        d = np.random.rand(2)* 299
        d = d.astype(int)
        node =Node(d[0],d[1])
        if self.map_array[d[0]][d[1]] == 0:
            while self.map_array[d[0]][d[1]] == 0:
                d = np.random.rand(2)* 299
                d = d.astype(int)
                node =Node(d[0],d[1])
        new_point=np.random.choice([self.goal,node],p=[goal_bias,1-goal_bias])

                
            
        return new_point
        # return self.goal
        # return new_point

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        min = 10000000000000
        l =None
        for pt in self.vertices: 
            # print(pt,point)  
            # if self.check_collision(pt,point)==False:
            # print("gggggg")
            dist = self.dis(pt,point)  
            # print(dist)  
            if min > dist:
                min = dist
                l = pt      
                
        return l 

        # return self.vertices[0]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbours=[]
        for i in self.vertices:
                d = self.dis(new_node,i)
                if d < neighbor_size:
                    neighbours.append(i)
        
        return neighbours
        # return [self.vertices[0]]


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        
        for n in neighbors:
            dist = self.dis(new_node,n)
            total_cost = dist + n.cost
            if total_cost < new_node.cost:
                new_node.cost = total_cost
                new_node.parent = n             

        for n in neighbors:
            dist = self.dis(new_node,n)
            original = n.cost
            new = dist + new_node.cost
            if new < original:
                n.parent = new_node
                n.cost = new_node.cost + dist
    
    def steps(self,t1,t2,step_size):
        x1,x2,y1,y2 = t1.row,t2.row,t1.col,t2.col
        dist= self.dis(t1,t2)
        val1 = x2-x1
        val2 = y2 -y1
        val1 = val1/dist
        val2 = val2/dist
        x3 = x1 + step_size * val1
        y3 = y1 + step_size * val2
        # print("x3",x3,y3)
        nd = Node(int(x3), int(y3))

        return nd
    def extend(self,nearest_node,new_node):
        '''Step Length- How Far to sample'''
        stepsize = 10
        if self.dis(nearest_node,new_node)<stepsize:
            node = new_node
            
        else:
            node = self.steps(nearest_node,new_node,stepsize)
        
        return node
            

            
            
            


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###
        for epochs in range(n_pts):
            new_point= self.get_new_point(0.1)
            nearest_node = self.get_nearest_node(new_point)
            # print("lllllllsdnbks",nearest_node,new_point)
            node=self.extend(nearest_node,new_point)
            if not self.check_collision(nearest_node,node):
                self.vertices.append(node)
                node.parent=nearest_node
                node.cost = nearest_node.cost+self.dis(nearest_node,node)
            if self.dis(self.vertices[-1],self.goal)<10:
                if not self.check_collision(self.vertices[-1],self.goal):
                    self.goal.parent = self.vertices[-1]
                    self.goal.cost = self.dis(self.vertices[-1],self.goal)+self.vertices[-1].cost
                    self.found = True
                    break
       
        # print("Goal Found for RRT")
        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.


        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()
        np.random.seed(20)
        ### YOUR CODE HERE ###
        for epochs in range(n_pts):
            new_point= self.get_new_point(0.1)
            nearest_node = self.get_nearest_node(new_point)
            # print("lllllllsdnbks",nearest_node,new_point)
            node = self.extend(nearest_node,new_point)
            if not self.check_collision(nearest_node,node):
                node.parent=nearest_node
                node.cost = nearest_node.cost+self.dis(nearest_node,node)
                neighbors = self.get_neighbors(node,neighbor_size)
                self.rewire (node,neighbors)
                self.vertices.append(node)
            # neighbors.remove(node)
            
                if self.dis(node,self.goal)<20:
                    if not self.check_collision(node,self.goal):
                        self.goal.parent = node
                        self.goal.cost = self.dis(node,self.goal)+node.cost
                        self.found = True
                    
        # for epochs in range(n_pts):
        #     while self.vertices[len(self.vertices)-1] != self.goal:
        #         new_point= self.get_new_point(0.1)
        #         nearest_node = self.get_nearest_node(new_point)
        #         ext = self.extend(nearest_node,new_point)
        #         neighbours = self.get_neighbors(ext,neighbour_size=20)
        #         self.rewire(ext,neighbours)

            # self.found = True
            # print("Goal Found for RRT***")
        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()

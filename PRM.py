# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import math
import numpy as np
import networkx as nx
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
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
        


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        d = math.dist(point1,point2)
        return d


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
       
        for j in range(0,self.size_row,10):
            for k in range(0,self.size_col,10):
                if j < 300:
                    if k < 300:
                        if self.map_array[j][k]==1:
                            self.samples.append((j,k))
        
        
        # self.samples.append((0, 0))
        return self.samples

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        pts = []

        for j in range(n_pts):
            x = np.random.randint(0,299)
            y = np.random.randint(0,299)
            pts.append((x,y))
            
        for h in pts:
            a,b = h[0],h[1]
            if self.map_array[a][b]==1:
                self.samples.append(h)

#        self.samples.append((0, 0))
        return self.samples


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        pts = []

        for j in range(n_pts):
            x = np.random.randint(0,(self.size_row-1))
            y = np.random.randint(0,(self.size_col-1))
            # x = np.random.rand(1) * (self.size_row - 1)
            # y = np.random.rand(1) * (self.size_col - 1)
            # x = int(x[0])
            # y = int(y[0])
            val = (x, y)
            # print(val)
            pts.append(val)

        for pt in pts:
            x = pt[0]
            y = pt[1]
            zx = int(np.random.normal(pt[0],30))
            zy = int(np.random.normal(pt[1],30))
            # print(zx, zy)
            if zx < 300 and zy < 300 and zx >= 0 and zy >= 0:
                # print( (x, y) , (zx, zy))
                if self.map_array[x][y] == 0 and self.map_array[zx][zy] == 1:
                    val = (zx, zy)
                    if val not in self.samples:
                        self.samples.append(val)
                if self.map_array[x][y] == 1 and self.map_array[zx][zy] == 0:
                    val = (x, y)
                    if val not in self.samples:
                        self.samples.append(val)

        # self.samples.append((0, 0))
        # print(self.samples)
        return self.samples


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        pts = []
        np.random.seed(0)
        for j in range(n_pts):
            pt = np.random.rand(2) * (self.size_row - 1)
            pt = pt.astype(int)
            a,b = pt[0],pt[1]
            if self.map_array[a][b]==0:
                z = np.random.normal(pt,40).astype(int)
                zx = z[0]
                zy = z[1]
                # zy = int(np.random.normal(pt[1],25))
                if (zx < 300) and (zy < 300) and zx >= 0 and zy >= 0:
                    if self.map_array[zx][zy]==0:
                        x3 = int((zx + a)/2.0)
                        y3 = int((zy + b)/2.0)
                        if self.map_array[x3][y3] == 1:
                            self.samples.append((x3, y3))
        # self.samples.append((0, 0))
        # print(self.samples)
        return self.samples


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        r = 19
        # print(self.samples)
        kdtree = spatial.KDTree(np.array(self.samples))
        pairss = list(kdtree.query_pairs(r))
        pairs=[]
        for k in range(len(pairss)):
            pt1, pt2 = self.samples[pairss[k][0]], self.samples[pairss[k][1]]
            # print(pt1,pt2)
            if pt1[0] < 300 and pt1[1] < 300:
                if pt2[0] < 300 and pt2[1] < 300:
                    if not self.check_collision(pt1,pt2):
                        d = self.dis(pt1,pt2)
                        pairs.append((pairss[k][0],pairss[k][1],d))
                        pairs.append((pairss[k][1],pairss[k][0],d))
        
        index_s=[]
        for inx in range(len(self.samples)):
            index_s.append(inx)

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from(index_s)
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        r = 75
        kdtree = spatial.KDTree(np.array(self.samples))
        pairss = list(kdtree.query_pairs(r))
        for t in range(len(pairss)):
            if self.samples.index(start) in pairss[t]:
                pt1,pt2 = pairss[t][0],pairss[t][1]
                if not self.check_collision(self.samples[pt1],self.samples[pt2]):
                    node1 = ("start", pt1, self.dis(self.samples[pt1], self.samples[pt2]))
                    node2 = (pt1, "start", self.dis(self.samples[pt1], self.samples[pt2]))
                    if node1 not in start_pairs:
                        start_pairs.append(node1)
                    if node2 not in start_pairs:
                        start_pairs.append(node2)
            if self.samples.index(goal) in pairss[t]:
                pt1,pt2 = pairss[t][0],pairss[t][1]
                if not self.check_collision(self.samples[pt1],self.samples[pt2]):
                    node1 = ("goal", pt1, self.dis(self.samples[pt1], self.samples[pt2]))
                    node2 = (pt1, "goal", self.dis(self.samples[pt1], self.samples[pt2]))
                    if node1 not in goal_pairs:
                        goal_pairs.append(node1)
                    if node2 not in goal_pairs:
                        goal_pairs.append(node2)


        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        
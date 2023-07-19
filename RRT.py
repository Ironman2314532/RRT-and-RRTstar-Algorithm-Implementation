# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial


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
        e_dist = math.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)
        return e_dist

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        rows_bet = np.linspace(node1.row, node2.row, dtype=int)
        col_bet = np.linspace(node1.col, node2.col, dtype=int)

        for p in zip(rows_bet, col_bet):
            if self.map_array[p[0]][p[1]] == 0:
                return True
        return False


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        if np.random.random() < goal_bias:
            point = [self.goal.row, self.goal.col] 
        else:
            point = [np.random.randint(0, self.size_row - 1), np.random.randint(0, self.size_col - 1)]
        return point

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        #print(self.vertices)    
        samp_list = [[n.row, n.col] for n in self.vertices]
        # print(samples)
        kdtree = spatial.KDTree(samp_list)
        p, num = kdtree.query(point)
        return self.vertices[num]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        
        samp_list = [[n.row, n.col] for n in self.vertices]
        kdtree = spatial.KDTree(samp_list)
        num = kdtree.query_ball_point([new_node.row, new_node.col], neighbor_size)
        neighbors = [self.vertices[i] for i in num]
        neighbors.remove(new_node)
        
        return neighbors

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        if not neighbors:
            return
        
        dist = [self.dis(new_node, n) for n in neighbors]

        for num, d in enumerate(dist):
            cost = 0
            current = neighbors[num]
            
            while self.start.row != current.row or self.start.col != current.col:
                p_node = current.parent
                if p_node is None:
                    return 0
                cost = cost + current.cost
                current = p_node
            
            t_cost = d + cost
        nums = np.argsort(np.array(t_cost))

        for n in nums:
            if self.check_collision(new_node, neighbors[n]) == False:
                new_node.parent = neighbors[n]
                new_node.cost = dist[n]
                break

        for i, nei in enumerate(neighbors):
            #########
            cost = 0
            current = new_node
            
            while self.start.row != current.row or self.start.col != current.col:
                p_node = current.parent
                if p_node is None:
                    return 0
                cost = cost + current.cost
                current = p_node    
            
            new = cost + dist[i]

            cost1 = 0
            current = nei
            
            while self.start.row != current.row or self.start.col != current.col:
                p_node = current.parent
                if p_node is None:
                    return 0
                cost1 = cost1 + current.cost
                current = p_node  

            if cost1 > new and self.check_collision(nei, new_node) == False:
                nei.parent = new_node
                nei.cost = dist[i]
     
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

        
        # In each step,
        for n in range(n_pts):
        # get a new point, 
            new_point = self.get_new_point(goal_bias=0)
        # get its nearest node,
            near_node = self.get_nearest_node(new_point)
        
        # extend the node and check collision to decide whether to add or drop,
            extend = 10
            slope = np.arctan2(new_point[1]-near_node.col, new_point[0]-near_node.row)
            new_r = near_node.row + extend*np.cos(slope)
            new_c = near_node.col + extend*np.sin(slope)
            new_node = Node(int(new_r), int(new_c))

            if (0 <= new_r < self.size_row) and (0 <= new_c < self.size_col) and (self.check_collision(near_node, new_node) == False):
                new_node.parent = near_node
                new_node.cost = extend
                self.vertices.append(new_node)
        
        # if added, check if reach the neighbor region of the goal.
                if not self.found:
                    d = self.dis(new_node, self.goal)
                    if d < extend:
                        self.goal.cost = d
                        self.goal.parent = new_node
                        self.vertices.append(self.goal)
                        self.found = True    
            
            if self.found:
                break

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

        # In each step,
        for n in range(n_pts):
        # get a new point,
            new_point = self.get_new_point(goal_bias=0) 
        # get its nearest node,
            near_node = self.get_nearest_node(new_point) 
        
        # extend the node and check collision to decide whether to add or drop,
            extend = 10
            slope = np.arctan2(new_point[1]-near_node.col, new_point[0]-near_node.row)
            new_r = near_node.row + extend*np.cos(slope)
            new_c = near_node.col + extend*np.sin(slope)
            new_node = Node(int(new_r), int(new_c))

            if (0 <= new_r < self.size_row) and (0 <= new_c < self.size_col) and (self.check_collision(near_node, new_node) == False):
                new_node.parent = near_node
                new_node.cost = extend
                self.vertices.append(new_node)
        
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
                if not self.found:
                    d = self.dis(new_node, self.goal)
                    if d < extend:
                        self.goal.cost = d
                        self.goal.parent = new_node
                        self.vertices.append(self.goal)
                        self.found = True
            
            else:
                new_node = None
        
            if new_node is not None:
                neighbors = self.get_neighbors(new_node, neighbor_size)
                self.rewire(new_node, neighbors)

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

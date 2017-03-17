import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# Represents a motion planning problem to be solved using A*
class AStar(object):

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo    # state space lower bound (e.g., (-5, -5))
        self.statespace_hi = statespace_hi    # state space upper bound (e.g., (5, 5))
        self.x_init = x_init                  # initial state
        self.x_goal = x_goal                  # goal state
        self.occupancy = occupancy            # occupancy grid
        self.resolution = resolution          # resolution of the discretization of state space (cell/m)

        self.closed_set = []    # the set containing the states that have been visited
        self.open_set = []      # the set containing the states that are condidate for future expension

        self.f_score = {}       # dictionary of the f score (estimated cost from start to goal passing through state)
        self.g_score = {}       # dictionary of the g score (cost-to-go from start to state)
        self.came_from = {}     # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.append(x_init)
        self.g_score[x_init] = 0
        self.f_score[x_init] = self.distance(x_init,x_goal)

        self.tb_radius = 0.17

        self.path = None        # the final path as a list of states

    # Checks if a give state is free, meaning it is inside the bounds of the map and
    # is not inside any obstacle
    # INPUT: (x)
    #          x - tuple state
    # OUTPUT: Boolean True/False
    def is_free(self, x):
        if x==self.x_init or x==self.x_goal:
            return True
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim]:
                return False
            if x[dim] >= self.statespace_hi[dim]:
                return False
        if not self.occupancy.is_free(x):
            return False
        return True

    # computes the euclidean distance between two states
    # INPUT: (x1, x2)
    #          x1 - first state tuple
    #          x2 - second state tuple
    # OUTPUT: Float euclidean distance
    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1)-np.array(x2))

    # gets the FREE neighbor states of a given state. Assumes a motion model
    # where we can move up, down, left, right, or along the diagonals
    # by an amount equal to self.resolution (this is important for problem 3!). Uses
    # the function self.is_free in order to check if any given state is indeed free.
    # INPUT: (x)
    #           x - tuple state
    # OUTPUT: List of neighbors that are free, as a list of TUPLES
    def get_neighbors(self, x):
        # TODO: fill me in!
        neighbor_list = []

        for dx in [-1., 0., 1.]:
            for dy in [-1., 0., 1.]:
                if dx != 0 or dy != 0:
                    nn = (x[0] + dx*self.resolution, x[1] + dy*self.resolution)
                    nn = self.snap_to_grid(nn, self.resolution)
                    dir = np.array([dx, dy])/np.linalg.norm(np.array([dx, dy]))
                    nn_left = (nn[0] - dir[1]*self.tb_radius, nn[1] + dir[0]*self.tb_radius)
                    nn_left = self.snap_to_grid(nn_left, self.resolution)

                    nn_top = (nn[0] + dir[0]*self.tb_radius, nn[1] + dir[1]*self.tb_radius)
                    nn_top = self.snap_to_grid(nn_top, self.resolution)

                    nn_right = (nn[0] + dir[1]*self.tb_radius, nn[1] - dir[0]*self.tb_radius)
                    nn_right = self.snap_to_grid(nn_right, self.resolution)

                    if self.is_free(nn) and self.is_free(nn_left) and self.is_free(nn_top) and self.is_free(nn_right):
                        neighbor_list.append(nn)

        return neighbor_list

    def snap_to_grid(self,xy, resolution):
        return (resolution*round(xy[0]/resolution), resolution*round(xy[1]/resolution))




        #Checks if a given path is still collision-free
    def check_path(self, newpath):
        for i in range(len(newpath)-1):
            myneighs = self.get_neighbors(newpath[i])
            if newpath[i+1] not in myneighs:
                return False
        return True

    # Gets the state in open_set that has the lowest f_score
    # INPUT: None
    # OUTPUT: A tuple, the state found in open_set that has the lowest f_score
    def find_best_f_score(self):
        return min(self.open_set, key=lambda x: self.f_score[x])

    # Use the came_from map to reconstruct a path from the initial location
    # to the goal location
    # INPUT: None
    # OUTPUT: A list of tuples, which is a list of the states that go from start to goal
    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    # Plots the path found in self.path and the obstacles
    # INPUT: None
    # OUTPUT: None
    def plot_path(self):
        if not self.path:
            return

        fig = plt.figure()

        self.occupancy.plot(fig.number)

        solution_path = np.array(self.path) * self.resolution
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="solution path", zorder=10)
        plt.scatter([self.x_init[0]*self.resolution, self.x_goal[0]*self.resolution], [self.x_init[1]*self.resolution, self.x_goal[1]*self.resolution], color="green", s=30, zorder=10)
        plt.annotate(r"$x_{init}$", np.array(self.x_init)*self.resolution + np.array([.2, 0]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal)*self.resolution + np.array([.2, 0]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis('equal')
        plt.show()
        self.fig  = fig

    # Solves the planning problem using the A* search algorithm. It places
    # the solution as a list of of tuples (each representing a state) that go
    # from self.x_init to self.x_goal inside the variable self.path
    # INPUT: None
    # OUTPUT: Boolean, True if a solution from x_init to x_goal was found
    def solve(self):
        while len(self.open_set)>0:
            # TODO: fill me in!

            # The current node is the one from the open set with the lowest f score
            x_current = self.find_best_f_score()

            # If the current node is the goal, return the path
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True

            # Remove the current node from the open set and add to closed set
            self.open_set.remove(x_current)
            self.closed_set.append(x_current)

            # For each neighbor of the current node
            for x_neighbor in self.get_neighbors(x_current):

                # Skip loop if the neighbor is in the closed set
                if x_neighbor in self.closed_set:
                    continue

                # Calculate the tentative g score
                tentative_g_score = self.g_score[x_current] + self.distance(x_current, x_neighbor)

                # If the neighbor is not in the open set, add it or skip if tentative g score is higher
                if x_neighbor not in self.open_set:
                    self.open_set.append(x_neighbor)
                elif tentative_g_score > self.g_score[x_neighbor]:
                    continue

                # Update the came_from and score dictionaries
                self.came_from[x_neighbor] = x_current
                self.g_score[x_neighbor] = tentative_g_score
                self.f_score[x_neighbor] = tentative_g_score + self.distance(x_neighbor, self.x_goal)

        return False

# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.5):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = probs
        self.window_size = window_size
        self.thresh = thresh

    def is_free(self, state):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        p_total = 1.0
        lower = -int(round((self.window_size-1)/2))
        upper = int(round((self.window_size-1)/2))
        for dx in range(lower,upper+1):
            for dy in range(lower,upper+1):
                x = state[0] + dx * self.resolution
                y = state[1] + dy * self.resolution
                grid_x = int((x - self.origin_x) / self.resolution)
                grid_y = int((y - self.origin_y) / self.resolution)
                if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
                    p_total *= (1.0-max(0.0,float(self.probs[grid_y * self.width + grid_x])/100.0))
        return (1.0-p_total) < self.thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(len(self.probs)):
            # convert i to (x,y)
            gy = int(i/self.width)
            gx = i % self.width
            x = gx * self.resolution + self.origin_x
            y = gy * self.resolution + self.origin_y
            if not self.is_free((x,y)):
                pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')

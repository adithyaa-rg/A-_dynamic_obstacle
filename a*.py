import random
import numpy as np
from sklearn.neighbors import KDTree
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


class Node:
    def __init__(self, state, parent = None):
        self.state = state
        self.parent = parent
        self.cost = 0
        self.children = []
        self.heur = 0

class A_star:
    def __init__(self, start, goal, obstacles, bounds, road_center_line, velocity, time, dt, plot = True):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.bounds = bounds
        self.road_center_line = road_center_line
        self.plot = plot
        self.velocity = velocity
        self.time = time
        self.dt = dt
        self.expnded = []
    
    def dynamics(self, state, theta_new):
        x = state[0]
        y = state[1]
        theta = state[2]

        x_dot = self.velocity * np.cos(theta_new)
        y_dot = self.velocity * np.sin(theta_new)
        theta_dot = 0

        x = x + x_dot * self.time
        y = y + y_dot * self.time
        theta = theta + theta_dot * self.time

        return (x, y, theta)
    
    def steer(self, start_node, goal_node):
        x1, y1, theta1 = start_node
        x2, y2, theta2 = goal_node
        theta = theta2 - theta1

        path = []

        while np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2])) > 5:
            # print(np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2])))
            x1 = x1 + self.dt * np.cos(theta)
            y1 = y1 + self.dt * np.sin(theta)
            path.append((x1, y1))
        return path

    def check_collision(self, state):
        """
            Returns True if the given state is in collision with the obstacles, False otherwise.
        """
        x, y, theta = state
        for obstacle in self.obstacles:
            x1, y1, x2, y2 = obstacle
            if x1 <= x <= x2 and y1 <= y <= y2:
                return True
        return False
    
    def cross_track_error(self, state):
        x, y, theta = state
        distances = []
        for point in self.road_center_line:
            x1, y1 = point
            distances.append(np.linalg.norm(np.array([x, y]) - np.array([x1, y1])))
        return min(distances)

    def getSuccessors(self, state):
        x, y, theta = state
        successors = []
        angles = np.linspace(-np.pi/2, np.pi/2, 10)
        # print(state)
        for new_theta in angles:
            # print(new_theta)
            new_state = self.dynamics(state, new_theta)
            if not self.check_collision(new_state):
                # print("No collision")
                cost = self.cross_track_error(new_state)
                # print(cost)
                successors.append((new_state, new_theta, cost))
                # print(self.plot)
                # print("Successor added")
                new_state_node = Node(new_state)
                new_state_node.cost = cost
                new_state_node.heur = self.heuristic(new_state)
                new_state_node.parent = Node(state)
                self.expnded.append(new_state_node)
                path = self.steer(state, new_state)
                if self.plot:
                    self.plot_graph(new_state, state, path)
        return successors

    def plot_graph(self, new_state, state, path):
        for i in range(len(self.expnded)):
            # print(self.expnded[i].state)
            if self.expnded[i].parent:
                plt.plot([self.expnded[i].state[0], self.expnded[i].parent.state[0]], [self.expnded[i].state[1], self.expnded[i].parent.state[1]], 'b-')

        for obs in self.obstacles:
            x1, y1, x2, y2 = obs
            plt.plot([x1, x1, x2, x2, x1], [y1, y2, y2, y1, y1], 'k-')

        plt.plot(self.start[0], self.start[1], 'go')
        plt.plot(self.goal[0], self.goal[1], 'ro')
        plt.plot(new_state[0], new_state[1], 'bo')
        plt.plot(state[0], state[1], 'bo')

        for point in path:
            plt.plot(point[0], point[1], 'ro')
        plt.pause(0.001)
        plt.draw()
        plt.clf()

    def heuristic(self, state):
        x, y, theta = state
        return np.linalg.norm(np.array([x, y]) - np.array([self.goal[0], self.goal[1]]))
    
    def isGoalNode(self, state):
        x, y, theta = state
        if np.linalg.norm(np.array([x, y]) - np.array([self.goal[0], self.goal[1]])) < 5:
            return True

    def planner(self):
        START_NODE = Node(self.start)
        CLOSED = []
        FRINGE = []
        FRINGE += [START_NODE]

        while True:
            if len(FRINGE) == 0:
                return "Failure to find a solution"
            FRINGE_NODE = FRINGE.pop(0)
            CLOSED += [FRINGE_NODE.state]
            if self.isGoalNode(FRINGE_NODE.state):
                return FRINGE_NODE
            successors = self.getSuccessors(FRINGE_NODE.state)
            for successor in successors:
                NEW_FRINGE_NODE = Node(successor[0])
                NEW_FRINGE_NODE.parent = FRINGE_NODE
                NEW_FRINGE_NODE.cost = FRINGE_NODE.cost + successor[2]
                NEW_FRINGE_NODE.heur = 5 * self.heuristic(NEW_FRINGE_NODE.state)

                if NEW_FRINGE_NODE.state not in CLOSED:
                    state_array = [node.state for node in FRINGE]
                    cost_array = [node.cost for node in FRINGE]
                    heur_array = [node.heur for node in FRINGE]
                    new_state_cost = NEW_FRINGE_NODE.cost
                    new_state_heur = NEW_FRINGE_NODE.heur
                    if NEW_FRINGE_NODE.state in state_array:
                        index_state = state_array.index(NEW_FRINGE_NODE.state)
                        if FRINGE[index_state].cost > new_state_cost:
                            # print("Index to be removed", index_state)
                            FRINGE.pop(index_state)
                            index_to_be_added = 0
                            for i in range(len(cost_array) - 1):
                                if cost_array[i] + heur_array[i] > new_state_cost + new_state_heur:
                                    break
                                index_to_be_added += 1

                            FRINGE.insert(index_to_be_added, NEW_FRINGE_NODE)
                    else:
                        index_to_be_added = 0
                        for i in range(len(cost_array)):
                            if cost_array[i] + heur_array[i] > new_state_cost + new_state_heur:
                                break
                            index_to_be_added += 1

                        FRINGE.insert(index_to_be_added, NEW_FRINGE_NODE)
    
    def plot_path(self, path):
        for i in range(len(self.expnded)):
            # print(self.expnded[i].state)
            if self.expnded[i].parent:
                plt.plot([self.expnded[i].state[0], self.expnded[i].parent.state[0]], [self.expnded[i].state[1], self.expnded[i].parent.state[1]], 'b-')

        for obs in self.obstacles:
            x1, y1, x2, y2 = obs
            plt.plot([x1, x1, x2, x2, x1], [y1, y2, y2, y1, y1], 'k-')

        plt.plot(self.start[0], self.start[1], 'go')
        plt.plot(self.goal[0], self.goal[1], 'ro')
        
        for i in range(len(path)-1):
            plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'g-')
        plt.pause(0.1)
        plt.draw()
        plt.clf()
        # plt.show()


    def plan(self):
        node = self.planner()
        path = []
        while node.parent is not None:
            path.append(node.state)
            node = node.parent
        self.plot_path(path)
        return path    
    
    

if __name__ == "__main__":
    
    # Define the four points
    x = np.linspace(-10,40,100)
    y = np.linspace(-10,40,100)

    road_center_line = np.array([[x, y] for x, y in zip(x, y)])

    start = (-10, -10, np.pi/4)
    goal = [road_center_line[-10][0], road_center_line[-10][1], np.pi/4]
    obstacles = [[10, 10, 20, 20]]
    bounds = (-50, 60, -50, 60)
    velocity = 3
    time = 1
    dt = 0.2
    # plt.plot(road_center_line[:,0], road_center_line[:,1], 'r-')
    # plt.show()
    for i in range(40):
        a_star = A_star(start, goal, obstacles, bounds, road_center_line, velocity, time, dt, plot = False)
        path = a_star.plan()
        for j in range(len(obstacles[0])):
            obstacles[0][j] += -.4
        for j in range(len(goal)-1):
            goal[j] = road_center_line[-39 + i][j]
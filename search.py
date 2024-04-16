# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 11:45:50 2023

@author: Bijo Sebastian
"""

"""
Implement your search algorithms here
"""

import operator
import math

class fringe_node:
  current_state = []
  path = []
  cost = 0
  heur = 0

def heuristic_1(problem, state):
    """
    Euclidean distance
    """
    "*** YOUR CODE HERE ***"
    current_state = state
    goal_state = problem.getGoalState()
    distance = (current_state[0] - goal_state[0])**2 + (current_state[1] - goal_state[1])**2
    return math.sqrt(distance)

def heuristic_2(problem, state):
    """
    Manhattan distance
    """
    "*** YOUR CODE HERE ***"
    current_state = state
    goal_state = problem.getGoalState()
    distance = abs(current_state[0] - goal_state[0]) + abs(current_state[1] - goal_state[1])
    return distance

def weighted_AStarSearch(problem, heuristic_ip):
    """
    Pop the node that having the lowest combined cost plus heuristic
    heuristic_ip can be M, E, or a number 
    if heuristic_ip is M use Manhattan distance as the heuristic function
    if heuristic_ip is E use Euclidean distance as the heuristic function
    if heuristic_ip is a number, use weighted A* Search with Euclidean distance as the heuristic function and the integer being the weight
    """
    "*** YOUR CODE HERE ***"
    if heuristic_ip == 'M':
      return aStarSearch(problem, heuristic_2)
    elif heuristic_ip == 'E':
      return aStarSearch(problem, heuristic_1)
    elif heuristic_ip.isdigit():
      return aStarSearch(problem, heuristic_1, weight=int(heuristic_ip))
   
def aStarSearch(problem, heuristic, weight=1):
  """
  Perform A* search algorithm to find a solution to the given problem.

  Args:
    problem: The problem instance to solve.
    heuristic: The heuristic function to estimate the cost from a state to the goal.
    weight: The weight to apply to the heuristic function (default is 1).

  Returns:
    If a solution is found, returns the path from the start state to the goal state.
    If no solution is found, returns "Failure to find a solution".
  """

  START_NODE = fringe_node()
  START_NODE.current_state = problem.getStartState()
  CLOSED = []
  FRINGE = []
  FRINGE += [START_NODE]

  while True:
    if len(FRINGE) == 0:
      return "Failure to find a solution"
    FRINGE_NODE = FRINGE.pop(0)
    CLOSED += [FRINGE_NODE.current_state]
    if problem.isGoalState(FRINGE_NODE.current_state):
      return FRINGE_NODE.path
    successors = problem.getSuccessors(FRINGE_NODE.current_state)
    for successor in successors:
      NEW_FRINGE_NODE = fringe_node()
      NEW_FRINGE_NODE.current_state = successor[0]
      NEW_FRINGE_NODE.path = FRINGE_NODE.path + [successor[1]]
      NEW_FRINGE_NODE.cost = FRINGE_NODE.cost + successor[2]
      NEW_FRINGE_NODE.heur = weight * heuristic(problem, NEW_FRINGE_NODE.current_state)

      if NEW_FRINGE_NODE.current_state not in CLOSED:
        state_array = [node.current_state for node in FRINGE]
        cost_array = [node.cost for node in FRINGE]
        heur_array = [node.heur for node in FRINGE]
        new_state_cost = NEW_FRINGE_NODE.cost
        new_state_heur = NEW_FRINGE_NODE.heur
        if NEW_FRINGE_NODE.current_state in state_array:
          index_state = state_array.index(NEW_FRINGE_NODE.current_state)
          if FRINGE[index_state].cost > new_state_cost:
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
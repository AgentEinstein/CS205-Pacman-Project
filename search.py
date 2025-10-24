# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions
from typing import List

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    "Initialize list of directions, nodes travelled, and node stack"
    directions = []
    travelled = []

    "Get the start node's neighbors"
    neighbors = problem.getSuccessors(problem.getStartState())

    "Run recursive function of Depth-First Search"
    RecursiveDFS(problem, directions, travelled, problem.getStartState(), neighbors)
    return directions


def RecursiveDFS(problem, directions, travelled, current, nodes):
    "Add current node to travelled list"
    travelled.append(current)
    "Look through neighbors of current node"
    for node in nodes:
        "if node is goal state add node to direction and return true"
        if problem.isGoalState(node[0]):
            directions.append(node[1])
            return True
        "if neighbor node is not a goal state and hasn't been travelled, go deeper into recursive DFS on it"
        if node[0] not in travelled:
            "get neighbors of current neighbor"
            neighbors = problem.getSuccessors(node[0])
            "if the recursive function is returning true, add current node to directions and return true"
            if RecursiveDFS(problem, directions, travelled, node[0], neighbors):
                directions.insert(0, node[1])
                return True
    "if the node is a dead end, return false"
    return False

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    "Initialize:"
    "path      - list of directions"
    path = []
    "travelled - nodes travelled"
    travelled = []
    "frontier  - node queue"
    frontier = util.PriorityQueue()
    "cost      - a dictionary for the cost of each node with the start node set to 0"
    cost = {problem.getStartState(): 0}
    "start     - the start state"
    start = problem.getStartState()

    "If current state is a goal, return empty list"
    if problem.isGoalState(start):
        return []

    "Add start state to queue"
    frontier.push(problem.getStartState(), heuristic(problem.getStartState(), problem))

    "While there are nodes in the priority queue, loop"
    while not frontier.isEmpty():
        "Get the current node from the top of the priority queue"
        current = frontier.pop()

        "If current node is goal state exit loop"
        if problem.isGoalState(current):
            directions = trace_destination(path, current)
            return directions

        "Add current node to the nodes travelled"
        travelled.append(current)

        "Iterate through node's neighbors"
        neighbors = problem.getSuccessors(current)
        for neighbor in neighbors:
            "generate neighbor's total cost from start"
            neighbor_cost = cost[current] + neighbor[2]
            "if neighbor is unexplored and not in the queue, add to queue and add update cost of node"
            if neighbor[0] not in travelled and neighbor[0] not in cost:
                frontier.update(neighbor[0], neighbor_cost + heuristic(neighbor[0], problem))
                cost[neighbor[0]] = neighbor_cost

                "add to path"
                path.append((current, neighbor[0], neighbor[1]))
            "if neighbor's cost is less than the current cost of the node, update and remove old path"
            if neighbor_cost < cost[neighbor[0]]:
                frontier.update(neighbor[0], neighbor_cost + heuristic(neighbor[0], problem))
                cost[neighbor[0]] = neighbor_cost

                "remove old path and add to path"
                path = list(filter(lambda tup: tup[1] != neighbor[0], path))
                path.append((current, neighbor[0], neighbor[1]))
    return []


def trace_destination(path, parent):
    """function to retrace the path to the destination"""
    directions = []
    "starting from the goal state"
    path.reverse()
    "iterate on the path and add direction to directions until the start is reached"
    for i in range(0, len(path)):
        if parent == path[i][1]:
            parent = path[i][0]
            directions.append(path[i][2])
    "reverse the list of directions to get the directions from start to goal state"
    directions.reverse()
    return directions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
from collections import deque

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

class Node():
    def __init__(self, state, pathCost, action=None, parent=None):
        self.action = action
        self.pathCost = pathCost
        self.parent = parent
        self.state = state

    def __eq__(self, other):
        return self.action == other.action and self.pathCost == other.pathCost \
             and self.parent == other.parent and self.state == other.state

    def __hash__(self):
        return hash(self.parent)

    def __str__(self):
        return "Node{parent: " + self.parent + ", action: " + self.action + ", state: " + self.state + ", action: " + self.action + "}"


def aNode(state, parent, action, stepCost):
    return Node(state=state,
                parent=parent,
                action=action,
                pathCost=parent.pathCost + stepCost)

def solution(node):
    return list(accumulateActions(node, deque()))

def accumulateActions(node, actions):
    if node.parent is None:
        return actions
    else:
        actions.appendleft(node.action)
        return accumulateActions(node.parent, actions)

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  state = problem.getStartState()
  pathCost = 0
  node = Node(state, pathCost)
  
  if problem.isGoalState(state):
      return solution(node)
  
  frontier = util.Queue()
  frontier.push(node)
  explored = set()

  while True:
      assert not frontier.isEmpty(), "failure"
  
      #choose the shallowest node in frontier
      node = frontier.pop()
      state = node.state
      explored.add(state)

      successors = problem.getSuccessors(state)
      for (state, action, stepCost) in successors:
          child = aNode(state, node, action, stepCost)
          if child not in explored or child not in frontier.list:
              if problem.isGoalState(child.state):
                  return solution(child)
              frontier.push(child)

def uniformCostSearch(problem):
  "Search the frontierNode of least total cost first. "
  state = problem.getStartState()
  pathCost = 0
  frontierNode = Node(state, pathCost)

  # a priority queue ordered by PATH-COST, with frontierNode as the only element
  frontier = util.PriorityQueueWithFunction(lambda node: node.pathCost)
  frontier.push(frontierNode)
  explored = list()

  i = 0
  while not frontier.isEmpty():
      assert not frontier.isEmpty(), "failure"
      # select the lowest cost frontierNode in frontier
      frontierNode = frontier.pop()
      state = frontierNode.state

      if problem.isGoalState(state):
          return solution(frontierNode)

      explored.append(state)

      successors = problem.getSuccessors(state)
      for (state, action, stepCost) in successors:
          child = aNode(state, frontierNode, action, stepCost)

          if state not in explored:
              frontier.push(child)
  return None


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

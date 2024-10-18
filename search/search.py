# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# # Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in search_agents.py).
"""
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in obj-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def get_start_state(self):
        """
        Returns the start state for the search problem.
        """
        util.raise_not_defined()

    def is_goal_state(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raise_not_defined()

    def get_successors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raise_not_defined()

    def get_cost_of_actions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raise_not_defined()


def tiny_maze_search(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

# def addSuccessors(problem, addCost=True):

class SearchNode:
    def __init__(self, parent, node_info):
        """
            parent: parent SearchNode.

            node_info: tuple with three elements => (coord, action, cost)

            coord: (x,y) coordinates of the node position

            action: Direction of movement required to reach node from
            parent node. Possible values are defined by class Directions from
            game.py

            cost: cost of reaching this node from the starting node.
        """

        self.__state = node_info[0]
        self.action = node_info[1]
        self.cost = node_info[2] if parent is None else node_info[2] + parent.cost
        self.parent = parent

    # The coordinates of a node cannot be modified, se we just define a getter.
    # This allows the class to be hashable.
    @property
    def state(self):
        return self.__state

    def get_path(self):
        path = []
        current_node = self
        while current_node.parent is not None:
            path.append(current_node.action)
            current_node = current_node.parent
        path.reverse()
        return path
    
    # Consider 2 nodes to be equal if their coordinates are equal (regardless of everything else)
    # def __eq__(self, __o: obj) -> bool:
    #     if (type(__o) is SearchNode):
    #         return self.__state == __o.__state
    #     return False

    # # def __hash__(self) -> int:
    # #     return hash(self.__state)

def depth_first_search(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.get_start_state())
    print("Is the start a goal?", problem.is_goal_state(problem.get_start_state()))
    print("Start's successors:", problem.get_successors(problem.get_start_state()))
    """
    "*** YOUR CODE HERE ***"
    #print("Start:", problem.get_start_state())
    #print("Is the start a goal?", problem.is_goal_state(problem.get_start_state()))
    #print("Start's successors:", problem.get_successors(problem.get_start_state()))
    
    # i use the expanded_nodes and the frontier as described in the lectures
    # expanded_nodes is a set from the class SearchNode
    expanded_nodes = set()
    frontier = util.Stack()
    starting_node_info = (problem.get_start_state(),None,0)
    starting_node = SearchNode(None,starting_node_info)
    frontier.push(starting_node)
    # until the frontier is not empty i perform the bfs
    while frontier.is_empty()==False :
        parent_node = frontier.pop()
        expanded_nodes.add(parent_node.state)
        if problem.is_goal_state(parent_node.state):
            return parent_node.get_path()
        for child_node_info in problem.get_successors(parent_node.state):
            child_node = SearchNode(parent_node,child_node_info)
            if frontier.contains(child_node)==False and child_node.state not in expanded_nodes:
                frontier.push(child_node)
    util.raise_not_defined()



def breadth_first_search(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    expanded_nodes = set()
    frontier = util.Queue()
    # compared to dfs here i have an additional set frontier_set of the class SearchNode
    # where i keep track which nodes are in the frontier
    # i use this set because the Queue does not have functions defined functions to tell me if an element is in there
    frontier_set = set()
    starting_node_info = (problem.get_start_state(),None,0)
    starting_node = SearchNode(None,starting_node_info)
    frontier.push(starting_node)
    frontier_set.add(starting_node)
    # a classic bfs is performed 
    while frontier.is_empty()==False :
        parent_node = frontier.pop()
        frontier_set.remove(parent_node)
        expanded_nodes.add(parent_node.state)
        if problem.is_goal_state(parent_node.state): 
            return parent_node.get_path()
        for child_node_info in problem.get_successors(parent_node.state):
            child_node = SearchNode(parent_node,child_node_info)
            if child_node not in frontier_set and child_node.state not in expanded_nodes:
                frontier.push(child_node)
                frontier_set.add(child_node)

    util.raise_not_defined()

def uniform_cost_search(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raise_not_defined()

def null_heuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def a_star_search(problem, heuristic=null_heuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raise_not_defined()

# Abbreviations
bfs = breadth_first_search
dfs = depth_first_search
astar = a_star_search
ucs = uniform_cost_search

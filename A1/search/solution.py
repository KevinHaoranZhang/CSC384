#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import math
import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    manhattan_distance = 0
    for box in state.boxes:
      min_dis = float("inf")
      for storage in state.storage:
        cur_dis = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        if cur_dis < min_dis:
          min_dis = cur_dis
      manhattan_distance += min_dis
    return manhattan_distance


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
  '''a better heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  #heur_manhattan_distance has flaws.
  #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
  #Your function should return a numeric value for the estimate of the distance to the goal.
  unstored_box = set(state.boxes - state.storage)
  unboxed_storage = set(state.storage - state.boxes)
  hval = 0
  for box in unstored_box:
    # Deadlock Check
    # Corner: formed by wall or obstacles
    if ((box[0] - 1, box[1]) in state.obstacles or (box[0] + 1, box[1]) in state.obstacles \
      or box[0] == 0 or box[0] == state.width - 1) and \
      ((box[0], box[1] - 1) in state.obstacles or (box[0], box[1] + 1) in state.obstacles \
      or box[1] == 0 or box[1] == state.height - 1):
      return float("inf")
    # Edge: no storage along the chosen edge
    if ((box[0] == 0 or box[0] == state.width - 1) and (not any(box[0] == storage[0] for storage in unboxed_storage))):
      return float("inf")
    if (box[1] == 0 or box[1] == state.height - 1) and (not any(box[1] == storage[1] for storage in unboxed_storage)):
      return float("inf")
    # Edge: two consecutive boxes along a edge
    if (box[0] == 0 or box[0] == state.width - 1) and ((box[0], box[1] + 1) in unstored_box or (box[0], box[1] - 1) in unstored_box):
      return float("inf")
    if (box[1] == 0 or box[1] == state.height - 1) and ((box[0] + 1, box[1]) in unstored_box or (box[0] - 1, box[1]) in unstored_box):
      return float("inf")

    # Compute the minimum distance between a box to an occupied storage
    min_dist_bs = float("inf")
    for storage in unboxed_storage:
      dist_bs = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
      if dist_bs < min_dist_bs:
        min_dist_bs = dist_bs
    
    # Compute the minimum distance between a robot to a box
    min_dist_rb = float("inf")
    for robot in state.robots:
      dist_rb = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
      if dist_rb < min_dist_rb:
        min_dist_rb = dist_rb

    # Compute total numbers of obstacles around a box
    obstacle_count = 0
    box_surround = [(box[0] + 1, box[1]), (box[0] - 1, box[1]), (box[0], box[1] + 1), (box[0], box[1] - 1) \
      , (box[0] + 1, box[1] + 1), (box[0] + 1, box[1] - 1), (box[0] - 1, box[1] + 1), (box[0] - 1, box[1] + 1)]
    for spot in box_surround:
      if spot in state.obstacles:
        obstacle_count += 1
    
    hval += min_dist_bs + min_dist_rb + obstacle_count
  return hval

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime weighted astar algorithm'''
  # Set function wrapper
  wrapped_fval_function = (lambda sN : fval_function(sN,weight))
  # Initialize search engine
  search_engine = SearchEngine("custom", "default");
  search_engine.init_search(initState=initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=wrapped_fval_function)
  # Set return value
  result_state = None
  cost_bound = None
  end_time = os.times()[0] + timebound
  # Continue searching before time limit is reached
  while os.times()[0] < end_time:
    goal_state = search_engine.search(timebound=(end_time - os.times()[0]), costbound=cost_bound)[0]
    if goal_state == False:
      break
    result_state = goal_state
    # Update cost bound
    cost_bound = (float("inf"), float("inf"), goal_state.gval + heur_fn(goal_state))
    # Update weight
    if weight > 1:
      weight -= 1
  return result_state

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime greedy best-first search'''
  # Initialize search engine
  search_engine = SearchEngine("best_first", "default");
  search_engine.init_search(initState=initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)
  # Set return value
  result_state = None
  cost_bound = None
  end_time = os.times()[0] + timebound
  # Continue searching before time limit is reached
  while os.times()[0] < end_time:
    goal_state = search_engine.search(timebound=(end_time - os.times()[0]), costbound=cost_bound)[0]
    if goal_state == False:
      break
    result_state = goal_state
    # Update cost bound
    cost_bound = (goal_state.gval, float("inf"), float("inf"))
  return result_state

# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        minFoodDistance = min([manhattanDistance(newPos, curFoodPos) for curFoodPos in currentGameState.getFood().asList()], default=0)
        minGhostDistance = min([manhattanDistance(newPos, CurGhost.getPosition()) for CurGhost in newGhostStates], default=0)
        if minGhostDistance < 1:
            return float("-inf")
        return -minFoodDistance

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        def DFMiniMax(state, agent_index, depth):
            best_move = None
            if state.isWin() or state.isLose() or (depth >= self.depth and agent_index % state.getNumAgents() == 0):
                return best_move, self.evaluationFunction(state)
            if agent_index % state.getNumAgents() == 0:
                depth += 1
                value = float("-inf")
            else:
                value = float("inf")
            for move in state.getLegalActions(agent_index):
                nxt_pos = state.generateSuccessor(agent_index, move)
                nxt_val = DFMiniMax(nxt_pos, (agent_index + 1) % state.getNumAgents(), depth)[1]
                if agent_index % state.getNumAgents() == 0 and value < nxt_val:
                    value, best_move = nxt_val, move
                if agent_index % state.getNumAgents() > 0 and value > nxt_val:
                    value, best_move = nxt_val, move
            return best_move, value

        best_move = DFMiniMax(gameState, self.index, 0)[0]
        return best_move

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        def AlphaBeta(state, agent_index, depth, alpha, beta):
            best_move = None
            if state.isWin() or state.isLose() or (depth >= self.depth and agent_index % state.getNumAgents() == 0):
                return best_move, self.evaluationFunction(state)
            if agent_index % state.getNumAgents() == 0:
                depth += 1
                value = float("-inf")
            else:
                value = float("inf")
            for move in state.getLegalActions(agent_index):
                nxt_pos = state.generateSuccessor(agent_index, move)
                nxt_val = AlphaBeta(nxt_pos, (agent_index + 1) % state.getNumAgents(), depth, alpha, beta)[1]
                if agent_index % state.getNumAgents() == 0:
                    if value < nxt_val:
                        value, best_move = nxt_val, move
                    if value >= beta:
                        return best_move, value
                    alpha = max(alpha, value)
                if agent_index % state.getNumAgents() > 0:
                    if value > nxt_val:
                        value, best_move = nxt_val, move
                    if value <= alpha:
                        return best_move, value
                    beta = min(beta, value)
            return best_move, value

        best_move = AlphaBeta(gameState, self.index, 0, float("-inf"), float("inf"))[0]
        return best_move

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        def Expectimax(state, agent_index, depth):
            best_move = None
            if state.isWin() or state.isLose() or (depth >= self.depth and agent_index % state.getNumAgents() == 0):
                return best_move, self.evaluationFunction(state)
            if agent_index % state.getNumAgents() == 0:
                depth += 1
                value = float("-inf")
            else:
                value = 0
            for move in state.getLegalActions(agent_index):
                nxt_pos = state.generateSuccessor(agent_index, move)
                nxt_val = Expectimax(nxt_pos, (agent_index + 1) % state.getNumAgents(), depth)[1]
                if agent_index % state.getNumAgents() == 0 and value < nxt_val:
                    value, best_move = nxt_val, move
                if agent_index % state.getNumAgents() > 0:
                    value = value + float(nxt_val / len(state.getLegalActions(agent_index)))
            return best_move, value

        best_move = Expectimax(gameState, self.index, 0)[0]
        return best_move

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION:
    Factors that impact the score:
    1. food distance -> negative impact
    2. scrad time -> positive impact
    3. ghost distance -> negative impact
    The strategy is to compute all min values for these factors, and sum them up with original score.
    The weight for scrad time is larger since it is the best time to score.
    """
    "*** YOUR CODE HERE ***"

    curPos = currentGameState.getPacmanPosition()
    minFoodDistance = min([manhattanDistance(curPos, curFoodPos) for curFoodPos in currentGameState.getFood().asList()], default=0)
    minGhostDistance = min([manhattanDistance(curPos, CurGhost.getPosition()) for CurGhost in currentGameState.getGhostStates()], default=0)
    minScaredTime =  min([CurGhost.scaredTimer for CurGhost in currentGameState.getGhostStates()], default=0)
    return - minFoodDistance + minScaredTime * 5 - 1 / (minGhostDistance + 1) + currentGameState.getScore()
# Abbreviation
better = betterEvaluationFunction

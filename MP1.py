import time
from enum import Enum

class Strategy(Enum):
	BFS = 0
	DFS = 1
	Greedy = 2
	Astar = 3

class State:
	agentPosition = ()
	dots = []
	parent = None
	pathCostSoFar = 0
	heuristic = 0

	def __init__(self, agentPosition=None, dots=[], state=None, pathCostSoFar=0, heuristic=0):
		self.agentPosition = agentPosition
		self.dots = list(dots)
		self.parent = state
		self.pathCostSoFar = pathCostSoFar
		self.heuristic = heuristic

visited = []
maze = []
start = State()
def parseFiles():
	with open('1.2_Mazes/smallSearch.txt') as input_file:
		for i, line in enumerate(input_file):
			row = []
			for j in range(len(line)):
				if line[j] == " ":
					row.append(True)
				elif line[j]=="%":
					row.append(False)
				elif line[j]==".":
					row.append(True)
					start.dots.append((i,j))
				elif line[j]=="P":
					row.append(True)
					start.agentPosition = (i,j)
					visited.append(start)
			maze.append(row)

def removeDots(state, x, y):
	for i in range(len(state.dots)):
		if(state.dots[i] == (x,y)):
			state.dots.remove((x,y))
			return True
	return False

def Equals(list1, list2):
	for val in list1:
		if val not in list2:
			return False
	return True

def manhattanHeuristic(state):
	heuristic = 0
	coords = state.agentPosition
	dot = state.dots[0]
	if(len(dot)==0):
		heuristic = 0
	else:
		goal_x = dot[0]
		goal_y = dot[1]
		state_x = coords[0]
		state_y = coords[1]
		heuristic = abs(goal_y-state_y)+abs(goal_x-state_x)
	return heuristic

#sets heuristic as the manhattan distance to the center of the 
#average dots coordinate
def multipleDotAverage(state):
	heuristic = 0
	coords = state.agentPosition
	state_x = coords[0]
	state_y = coords[1]
	goal_x = 0
	goal_y = 0
	numDots = len(state.dots)
	if(numDots==0):
		heuristic = 0
	else:
		for dot in state.dots:
			goal_x+=dot[0]
			goal_y+=dot[1]
		goal_x/=numDots
		goal_y/=numDots
		heuristic = abs(goal_y-state_y)+abs(goal_x-state_x)
	return heuristic

def multipleDotClosestDot(state):
	heuristic = 0
	coords = state.agentPosition
	state_x = coords[0]
	state_y = coords[1]
	goal_x = 0
	goal_y = 0
	numDots = len(state.dots)
	if(numDots==0):
		heuristic = 0
	else:
		minDistance = abs(state.dots[0][1]-state_y)+abs(state.dots[0][0]-state_x)
		for dot in state.dots:
			goal_x = dot[0]
			goal_y = dot[1]
			distance = abs(goal_y-state_y)+abs(goal_x-state_x)
			if(distance < minDistance):
				minDistance = distance
		heuristic = minDistance
	return heuristic

#Takes in a State, creates all of the reachable neighbor States and 
#assigns s as their parent. Returns a list of these states
def transition(state,frontier):
	frontier.remove(state)
	visited.append(state)

	coords = state.agentPosition
	x = coords[0]
	y = coords[1]

	#always inside walls so dont need to check id in bounds
	if(maze[x+1][y]):
		newState = State((x+1,y),state.dots,state,state.pathCostSoFar+1)
		newState.heuristic = multipleDotClosestDot(newState) + multipleDotAverage(newState)
		removeDots(newState,x+1,y)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and Equals(visitedState.dots, newState.dots):
				shouldAdd = False
		for i in range(len(frontier)):
			frontierState = frontier[i]
			if shouldAdd and frontierState.agentPosition == newState.agentPosition and Equals(frontierState.dots, newState.dots):
				if newState.pathCostSoFar < frontierState.pathCostSoFar:
					frontier[i]=newState
				else:
					shouldAdd = False
		if shouldAdd:
			frontier.append(newState)

	if(maze[x-1][y]):
		newState = State((x-1,y),state.dots,state,state.pathCostSoFar+1)
		newState.heuristic = multipleDotClosestDot(newState) + multipleDotAverage(newState)
		removeDots(newState,x-1,y)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and Equals(visitedState.dots, newState.dots):
				shouldAdd = False
		for i in range(len(frontier)):
			frontierState = frontier[i]
			if shouldAdd and frontierState.agentPosition == newState.agentPosition and Equals(frontierState.dots, newState.dots):
				if newState.pathCostSoFar < frontierState.pathCostSoFar:
					frontier[i]=newState
				else:
					shouldAdd = False
		if shouldAdd:
			frontier.append(newState)

	if(maze[x][y+1]):
		newState = State((x,y+1),state.dots,state,state.pathCostSoFar+1)
		newState.heuristic = multipleDotClosestDot(newState) + multipleDotAverage(newState)
		removeDots(newState,x,y+1)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and Equals(visitedState.dots, newState.dots):
				shouldAdd = False
		for i in range(len(frontier)):
			frontierState = frontier[i]
			if shouldAdd and frontierState.agentPosition == newState.agentPosition and Equals(frontierState.dots, newState.dots):
				if newState.pathCostSoFar < frontierState.pathCostSoFar:
					frontier[i]=newState
				else:
					shouldAdd = False
		if shouldAdd:
			frontier.append(newState)

	if(maze[x][y-1]):
		newState = State((x,y-1),state.dots,state,state.pathCostSoFar+1)
		newState.heuristic = multipleDotClosestDot(newState) + multipleDotAverage(newState)
		removeDots(newState,x,y-1)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and Equals(visitedState.dots, newState.dots):
				shouldAdd = False
		for i in range(len(frontier)):
			frontierState = frontier[i]
			if shouldAdd and frontierState.agentPosition == newState.agentPosition and Equals(frontierState.dots, newState.dots):
				if newState.pathCostSoFar < frontierState.pathCostSoFar:
					frontier[i]=newState
				else:
					shouldAdd = False
		if shouldAdd:
			frontier.append(newState)

	return frontier

#Takes in a list of states and a strategy, returns the next state to 
#explore based on that strategy
def searchStrategy(frontier, strategy):
	if strategy == Strategy.BFS:
		return frontier[0]
	elif strategy == Strategy.DFS:
		return frontier[len(frontier)-1]
	elif strategy == Strategy.Greedy:
		minNode = frontier[0]
		for f in frontier:
			if f.heuristic < minNode.heuristic:
				minNode = f
		return minNode
	elif strategy == Strategy.Astar:
		minNode = frontier[0]
		for f in frontier:
			if f.heuristic + f.pathCostSoFar < minNode.heuristic + minNode.pathCostSoFar:
				minNode = f
		return minNode
	return frontier[0]

def printSolutionMaze(endNode):
	currentNode = endNode
	while(currentNode!=start):
		position = currentNode.agentPosition
		x = position[0]
		y = position[1]
		maze[x][y] = "."
		currentNode = currentNode.parent

	for row in maze:
		for value in row:
			if value == True:
				print " ",
			elif value == False:
				print "%",
			else:
				print ".",
		print("")

def printSolutionSearch(endNode):
	pathCounter = ['0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']

	#get how many dots there are
	#work way back and pace dots
	#???
	#profit

	numdots = len(start.dots)

	currentNode = endNode
	while(currentNode!=start):
		position = currentNode.agentPosition
		x = position[0]
		y = position[1]
		if(len(currentNode.parent.dots)!=len(currentNode.dots)):
			maze[x][y] = pathCounter[numdots]
			numdots-=1
		currentNode = currentNode.parent

	for row in maze:
		for value in row:
			if value == True:
				print " ",
			elif value == False:
				print "%",
			else:
				print value,
		print("")

def treeSearch(strategy):
	start.heuristic = multipleDotClosestDot(start) + multipleDotAverage(start)
	frontier = [start]
	frontier = transition(start,frontier)
	while(len(frontier)>0):
		node = searchStrategy(frontier,strategy)
		if(len(node.dots)==0):
			printSolutionSearch(node)
			break
		else:
			frontier = transition(node,frontier)

parseFiles()
start_time = time.time()
treeSearch(Strategy.Astar)
print("--- %s seconds ---" % (time.time() - start_time))
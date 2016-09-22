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
		self.dots = dots
		self.parent = state
		self.pathCostSoFar = pathCostSoFar
		self.heuristic = heuristic

visited = []
maze = []
start = State()
def parseFiles():
	with open('mediumMaze.txt') as input_file:
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

#Takes in a State, creates all of the reachable neighbor States and 
#assigns s as their parent. Returns a list of these states
def transition(state,frontier):
	#find and add all unvisited neighbors
	#return updated list
	frontier.remove(state)
	visited.append(state)
	coords = state.agentPosition
	x = coords[0]
	y = coords[1]

	#always inside walls so dont need to check id in bounds
	if(maze[x+1][y]):
		newState = State((x+1,y),state.dots,state,state.pathCostSoFar+1)
		removeDots(newState,x+1,y)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and visitedState.dots == newState.dots:
				shouldAdd = False
		if shouldAdd:
			frontier.append(newState)
		for frontierState in frontier:
			if frontierState.agentPosition == newState.agentPosition and frontierState.dots == newState.dots and newState.pathCostSoFar < frontierState.pathCostSoFar:
				frontier.remove(frontierState)
				frontier.append(newState)
			


	if(maze[x-1][y]):
		newState = State((x-1,y),state.dots,state,state.pathCostSoFar+1)
		removeDots(newState,x-1,y)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and visitedState.dots == newState.dots:
				shouldAdd = False
		if shouldAdd:
			frontier.append(newState)
		for frontierState in frontier:
			if frontierState.agentPosition == newState.agentPosition and frontierState.dots == newState.dots and newState.pathCostSoFar < frontierState.pathCostSoFar:
				frontier.remove(frontierState)
				frontier.append(newState)


	if(maze[x][y+1]):
		newState = State((x,y+1),state.dots,state,state.pathCostSoFar+1)
		removeDots(newState,x,y+1)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and visitedState.dots == newState.dots:
				shouldAdd = False
		if shouldAdd:
			frontier.append(newState)
		for frontierState in frontier:
			if frontierState.agentPosition == newState.agentPosition and frontierState.dots == newState.dots and newState.pathCostSoFar < frontierState.pathCostSoFar:
				frontier.remove(frontierState)
				frontier.append(newState)


	if(maze[x][y-1]):
		newState = State((x,y-1),state.dots,state,state.pathCostSoFar+1)
		removeDots(newState,x,y-1)
		shouldAdd = True
		for visitedState in visited:
			if visitedState.agentPosition == newState.agentPosition and visitedState.dots == newState.dots:
				shouldAdd = False
		if shouldAdd:
			frontier.append(newState)
		for frontierState in frontier:
			if frontierState.agentPosition == newState.agentPosition and frontierState.dots == newState.dots and newState.pathCostSoFar < frontierState.pathCostSoFar:
				frontier.remove(frontierState)
				frontier.append(newState)
		


	return frontier

#Takes in a list of states and a strategy, returns the next state to 
#explore based on that strategy
def searchStrategy(states,strategy):
	return states[0]

def printSolution(endNode):
	return None

def treeSearch(strategy):
	frontier = [start]
	frontier = transition(start,frontier)
	while(len(frontier)>0):
		node = searchStrategy(frontier,strategy)
		if(len(node.dots)==0):
			printSolution(node)
			break
		else:
			frontier = transition(node,frontier)

parseFiles()
treeSearch(Strategy.BFS)

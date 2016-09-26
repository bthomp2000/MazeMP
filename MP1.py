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
PathCosts = {} #(state_x,state_y): [{(goal_x1,goal_y1):pathcost1, (goal_x2,goal_y2):pathcost2},(minGoal_x,minGoal_y)]
start = State()
def parseFiles():
	with open('1.2_Mazes/mediumSearch.txt') as input_file:
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



def populateGoalCostMap():
	global visited
	PathCosts
	for i in range(len(maze)):
		for j in range(len(maze[i])):
			if maze[i][j]:
				minPathCost = 9999999
				PathCosts[(i,j)]=[None]*2
				PathCosts[(i,j)][0]={}
				PathCosts[(i,j)][1]=(0,0)
				for goal in start.dots:
					visited = []
					if (i,j)==goal:
						PathCosts[(i,j)][1]=(i,j)
						PathCosts[(i,j)][0][goal]=0
					else:
						state = State((i,j),[goal],0,0)
						frontier = [state]
						frontier = transition(state,frontier,Strategy.BFS)
						while(len(frontier)>0):

							node = searchStrategy(frontier,Strategy.BFS)
							if len(node.dots)==0:
								# print i,j,"hi"
								cost = node.pathCostSoFar
								if cost < minPathCost:
									PathCosts[(i,j)][1]=node.agentPosition
									# print PathCosts[(i,j)]
									minPathCost=cost
								PathCosts[(i,j)][0][goal]=cost
								break
							else:
								frontier = transition(node,frontier,Strategy.BFS)
				# print i,j,PathCosts[(i,j)]
	visited = []



#sets heuristic as the manhattan distance to the center of the 
#average dots coordinate. Attempt 1
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



#Attempt 2
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


#Attempt 4: Take the average path length from each dot to every other dot * number of dots
def averagePathLength(state):
	numDots = len(state.dots)
	averagePathLength = 0
	for dot1 in state.dots:
		for dot2 in state.dots:
			# averagePathLength += PathCosts[dot1][0][dot2]
			xdiff = abs(dot2[0]-dot1[1])
			ydiff = abs(dot2[1]-dot1[1])
			averagePathLength+=xdiff+ydiff
	averagePathLength = averagePathLength / (numDots)
	return averagePathLength

#Returns the path length to the nearest goal of passed in state
def closestPathDot(state):
	coords = state.agentPosition
	if coords in PathCosts:
		bestGoalCoord = PathCosts[coords][1]
		if bestGoalCoord in state.dots:
			return PathCosts[coords][0][bestGoalCoord]
		goalCoords = PathCosts[coords][0].keys()
		minPath = PathCosts[coords][0][goalCoords[0]]
		for goalCoord in goalCoords:
			currPath = PathCosts[coords][0][goalCoord]
			if currPath < minPath:
				minPath = currPath
				PathCosts[coords][1]=goalCoord
		# print "start: ",coords, "end: " ,PathCosts[coords][1], "path cost: ", minPath
		return minPath
	else:
		print "wtf"
		return 0

# Attempt 3
def finalHeuristic(state):
	heuristic = 0
	coords = state.agentPosition
	numDots = len(state.dots)
	if(numDots==0 or coords == state.dots[0]):
		heuristic = 0
	else:
		heuristic = closestPathDot(state) + averagePathLength(state)
	return heuristic



#Takes in a State, creates all of the reachable neighbor States and 
#assigns s as their parent. Returns a list of these states
def transition(state,frontier,strategy):
	frontier.remove(state)
	visited.append(state)
	coords = state.agentPosition
	x = coords[0]
	y = coords[1]
	removeDots(state,x,y)
	#always inside walls so dont need to check id in bounds
	if(maze[x+1][y]):
		newState = State((x+1,y),state.dots,state,state.pathCostSoFar+1)
		if(strategy==Strategy.Greedy or strategy==Strategy.Astar):
			newState.heuristic = finalHeuristic(newState)
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
		if(strategy==Strategy.Greedy or strategy==Strategy.Astar):
			newState.heuristic = finalHeuristic(newState)
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
		if(strategy==Strategy.Greedy or strategy==Strategy.Astar):
			newState.heuristic = finalHeuristic(newState)
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
		if(strategy==Strategy.Greedy or strategy==Strategy.Astar):
			newState.heuristic = finalHeuristic(newState)
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
	global start
	if strategy==Strategy.Astar or strategy==Strategy.Greedy:
		populateGoalCostMap()
		start.heuristic = finalHeuristic(start)
	frontier = [start]
	frontier = transition(start,frontier,strategy)
	while(len(frontier)>0):
		node = searchStrategy(frontier,strategy)
		if(len(node.dots)==0):
			printSolutionSearch(node)
			break
		else:
			frontier = transition(node,frontier,strategy)

parseFiles()
start_time = time.time()
treeSearch(Strategy.Greedy)
print("--- %s seconds ---" % (time.time() - start_time))
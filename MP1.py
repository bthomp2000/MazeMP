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

	def __init__(self):
		self.dots = []

visited = []
maze = []

def parseFiles():
	with open('mediumMaze.txt') as input_file:
		start = State()
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
					visited.append((i,j))
			maze.append(row)

#Takes in a State, creates all of the reachable neighbor States and 
#assigns s as their parent. Returns a list of these states
def transition(state):
	return [] #List of states

#Takes in a list of states and a strategy, returns the next state to 
#explore based on that strategy
def searchStrategy(states,strategy):
	return states[0]


parseFiles()
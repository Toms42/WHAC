import heapq
import numpy
import copy

inflationAmt = 2


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


possibleMoves = [(1, 0), (1, 1), (1, -1), (0, 1), (0, -1), (-1, 0), (-1, -1), (-1, 1)]


def expandNode(grid, state):
    states = []
    for action in possibleMoves:
        nextState = (state[0] + action[0], state[1] + action[1])
        if isIn(grid, nextState[0], nextState[1]) and grid[nextState[0]][nextState[1]] != 1:
            states += [(nextState, action)]
    return states


def heuristic(cur, final):
    (x, y) = cur
    (x1, y1) = final
    return abs(x - x1) + abs(y - y1)


def isIn(grid, x, y):
    xL = len(grid)
    yL = len(grid[0])
    return 0 <= x and x < xL and 0 <= y and y < yL


def map_inflation(grid):
    return grid
    xL = len(grid)
    yL = len(grid[0])
    newGrid = copy.deepcopy(grid)
    for i in range(xL):
        for j in range(yL):
            if grid[i][j] == 1:
                for di in range(-inflationAmt, inflationAmt + 1):
                    for dj in range(-inflationAmt, inflationAmt + 1):
                        if isIn(grid, i + di, j + dj):
                            newGrid[i + di][j + dj] = 1
    return newGrid


def a_star_search(grid, start, goal):
    grid = map_inflation(grid)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            return make_path(came_from, goal, start)
        for (next, action) in expandNode(grid, current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = (current, action)
    print("No path found")
    return None


def make_path(came_from, goal, start):
    path = []
    curr = goal
    while came_from[curr] != None:
        (prev, action) = came_from[curr]
        path.insert(0, curr)
        curr = prev
    path.insert(0, start)
    return path

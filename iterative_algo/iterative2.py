class Node:
    def __init__(self, state, parent, action, cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

class Maze:
    def __init__(self, maze):
        self.maze = maze
        self.start = None
        self.goal = None
        self.width = len(maze[0])
        self.height = len(maze)
        for i in range(self.height):
            for j in range(self.width):
                if maze[i][j] == 'S':
                    self.start = (i, j)
                if maze[i][j] == 'G':
                    self.goal = (i, j)

    def is_valid(self, state):
        i, j = state
        return 0 <= i < self.height and 0 <= j < self.width and self.maze[i][j] != '%'

    def children(self, state):
        i, j = state
        children = [(i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)]
        children = [child for child in children if self.is_valid(child)]
        return children

    def iterative_deepening_search(self, max_depth):
        for depth in range(max_depth + 1):
            result = self.depth_limited_search(depth)
            if result is not None:
                return result

    def depth_limited_search(self, depth_limit):
        frontier = [Node(self.start, None, None, 0)]
        explored = set()
        while frontier:
            node = frontier.pop(0)
            if node.state == self.goal:
                return node
            if node.cost < depth_limit:
                for child in self.children(node.state):
                    if child not in explored:
                        frontier.append(Node(child, node, child, node.cost + 1))
                        explored.add(child)
        return None

    def solution(self, node):
        path = []
        while node is not None:
            path.append(node.state)
            node = node.parent
        path = path[::-1]
        solution = [[' ' for j in range(self.width)] for i in range(self.height)]
        for i, j in path:
            solution[i][j] = 'x'
        return solution

maze = [['%', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', '%', '%', ' ', '%', '%'], 
        [' ', '%', '%', '%', '%', '%', ' ', '%', ' ', '%', '%', ' ', '%', ' ', ' ', ' ', ' '], 
        ['%', 'S', ' ', ' ', ' ', '%', ' ', ' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' '], 
        [' ', '%', ' ', '%', ' ', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', ' '], 
        ['%', '%', '%', '%', ' ', '%', '%', '%', ' ', '%', ' ', '%', '%', 'G', ' ', ' ', ' '], 
        [' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%', ' ', ' '], 
        ['%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%']]



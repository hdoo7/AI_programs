def get_successors(node):
    x, y = node
    directions = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
    successors = [(x_, y_) for x_, y_ in directions if 0 <= x_ < len(maze) and 0 <= y_ < len(maze[0]) and maze[x_][y_] != '%']
    return successors

def iterative_deepening_search(maze):  
    start = None
    goal = None
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 'S':
                start = (i, j)
            if maze[i][j] == 'G':
                goal = (i, j)
    
    for depth in range(1000000):
        path, cost, expanded = depth_limited_search(start, goal, depth)
        if path is not None:
            return path, cost, expanded
        

def depth_limited_search(node, goal, depth_limit):
    stack = [(node, [], 0)]
    expanded = 0
    while stack:
        node, path, cost = stack.pop()
        if node == goal:
            return path + [node], cost, expanded
        if cost < depth_limit:
            for successor in get_successors(node):
                if successor not in path:
                    stack.append((successor, path + [node], cost + 1))
                    expanded += 1
    return None, None, expanded


maze = [['%', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', '%', '%', ' ', '%', '%'], 
        [' ', '%', '%', '%', '%', '%', ' ', '%', ' ', '%', '%', ' ', '%', ' ', ' ', ' ', ' '], 
        ['%', 'S', ' ', ' ', ' ', '%', ' ', ' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' '], 
        [' ', '%', ' ', '%', ' ', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', ' '], 
        ['%', '%', '%', '%', ' ', '%', '%', '%', ' ', '%', ' ', '%', '%', 'G', ' ', ' ', ' '], 
        [' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%', ' ', ' '], 
        ['%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%']]

path, cost, expanded = iterative_deepening_search(maze)
if path:
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if (i, j) in path:
                print("x", end="")
            else:
                print(' ', end="")
        print()
    print("Solution cost:", cost)
    print("Nodes expanded:", expanded)
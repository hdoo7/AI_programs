import heapq

def print_maze(maze, path):
    for i, row in enumerate(maze):
        for j, col in enumerate(row):
            if (i, j) in path:
                print("x", end="")
            else:
                print(' ', end="")
        print()

def is_valid(maze, x, y):
    if x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0]):
        return False
    if maze[x][y] == "%":
        return False
    return True

def get_heuristic(x, y, goal_x, goal_y):
    return abs(x - goal_x) + abs(y - goal_y)

def astar(maze, start, goal):
    start_x, start_y = start
    goal_x, goal_y = goal
    heap = []
    heapq.heappush(heap, (0, start_x, start_y))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    nodes_expanded = 0

    while heap:
        _, x, y = heapq.heappop(heap)
        nodes_expanded += 1
        if (x, y) == goal:
            break

        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        for nx, ny in neighbors:
            if is_valid(maze, nx, ny):
                new_cost = cost_so_far[(x, y)] + 1
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    priority = new_cost + get_heuristic(nx, ny, goal_x, goal_y)
                    heapq.heappush(heap, (priority, nx, ny))
                    came_from[(nx, ny)] = (x, y)

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path, cost_so_far[goal], nodes_expanded


if __name__ == "__main__":
    maze = [['%', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', '%', '%', ' ', '%', '%'], 
            [' ', '%', '%', '%', '%', '%', ' ', '%', ' ', '%', '%', ' ', '%', ' ', ' ', ' ', ' '], 
            ['%', 'S', ' ', ' ', ' ', '%', ' ', ' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' '], 
            [' ', '%', ' ', '%', ' ', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', ' '], 
            ['%', '%', '%', '%', ' ', '%', '%', '%', ' ', '%', ' ', '%', '%', 'G', ' ', ' ', ' '], 
            [' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%', ' ', ' '], 
            ['%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%']]

    for x in range(len(maze)):
        for y in range(len(maze[0])):
            if maze[x][y] == 'S':
                start = (x, y)
            if maze[x][y] == 'G':
                goal = (x, y)
 
    path, cost, nodes_expanded = astar(maze, start, goal)
    print_maze(maze, path)
    print("Solution cost: ", cost)
    print("Nodes expanded: ", nodes_expanded)

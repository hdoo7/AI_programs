import heapq

def best_first_search(maze):
    start = None
    goal = None
    n = len(maze)
    m = len(maze[0])

    # Find start and goal positions
    for i in range(n):
        for j in range(m):
            if maze[i][j] == 'S':
                start = (i, j)
            if maze[i][j] == 'G':
                goal = (i, j)

    # The cost function is the Manhattan distance to the goal
    def cost(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    heap = [(0, start)]
    visited = set()
    parent = {}
    cost_so_far = {}
    cost_so_far[start] = 0
    expanded = 0
    
    while heap:
        _, current = heapq.heappop(heap)
        if current == goal:
            break
        
        visited.add(current)
        expanded += 1
        neighbors = [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]
        for next in neighbors:
            if next[0] < 0 or next[0] >= n or next[1] < 0 or next[1] >= m or maze[next[0]][next[1]] == '%' or next in visited:
                continue
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + cost(next)
                heapq.heappush(heap, (priority, next))
                parent[next] = current

    path = []
    node = goal
    while node != start:
        path.append(node)
        node = parent[node]
    path.append(start)
    path.reverse()

    print('Solution:')
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if (i, j) in path:
                print("x", end="")
            else:
                print(' ', end="")
        print()

    print('Cost:', cost_so_far[goal])
    print('Nodes expanded:', expanded)

# Example usage
maze = [['%', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', '%', '%', ' ', '%', '%'], 
        [' ', '%', '%', '%', '%', '%', ' ', '%', ' ', '%', '%', ' ', '%', ' ', ' ', ' ', ' '], 
        ['%', 'S', ' ', ' ', ' ', '%', ' ', ' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' '], 
        [' ', '%', ' ', '%', ' ', '%', '%', '%', '%', '%', ' ', '%', '%', '%', '%', '%', ' '], 
        ['%', '%', '%', '%', ' ', '%', '%', '%', ' ', '%', ' ', '%', '%', 'G', ' ', ' ', ' '], 
        [' ', ' ', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%', ' ', ' '], 
        ['%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%']]

best_first_search(maze)
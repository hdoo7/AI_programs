import sys
import math
import heapq
import time
import signal


def euclidian(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def manhattan(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def is_valid(maze, x, y):
    rows = len(maze)
    cols = len(maze[0])
    return 0 <= x < rows and 0 <= y < cols and maze[x][y] != "%"

class TimeoutError(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutError("The function took too long to execute")

def run_with_timeout(func, args, timeout):
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(timeout)
    try:
        result = func(*args)
    except TimeoutError as e:
        return None
    finally:
        signal.alarm(0)
    return result

def run_with_timeout(func, args, timeout):
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(timeout)
    try:
        result = func(*args)
    except TimeoutError as e:
        return None
    finally:
        signal.alarm(0)
    return result

def depth_first():
    def dfs(state, visited, path):
        if state in visited:
            return False
        visited.add(state)
        if state == goal:
            return True
        moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in moves:
            new_x, new_y = state[0] + dx, state[1] + dy
            if is_valid(maze, new_x, new_y):
                path.append((new_x, new_y))
                if dfs((new_x, new_y), visited, path):
                    return True
                path.pop()
        return False

    visited = set()
    path = [(start)]
    if dfs(start, visited, path):
        return path, len(path)-1, len(visited)


def best_first(heuristic):
    heap = [(0, start)]
    visited = set()
    parent = {}
    curr_cost = {}
    curr_cost[start], expanded, priority = 0, 0, 0
    
    while heap:
        _, curr = heapq.heappop(heap)
        if curr == goal:
            break
        
        visited.add(curr)
        expanded += 1
        nxt_states = [(curr[0]-1, curr[1]), (curr[0]+1, curr[1]), (curr[0], curr[1]-1), (curr[0], curr[1]+1)]
        for nxt in nxt_states:
            if nxt[0] < 0 or nxt[0] >= n or nxt[1] < 0 or nxt[1] >= m or maze[nxt[0]][nxt[1]] == '%' or nxt in visited:
                continue
            new_cost = curr_cost[curr] + 1
            if nxt not in curr_cost or new_cost < curr_cost[nxt]:
                curr_cost[nxt] = new_cost
                if heuristic == 'euclidian':
                    priority = new_cost + euclidian(nxt, goal)
                elif heuristic == 'manhattan':
                    priority = new_cost + manhattan(nxt, goal)
                heapq.heappush(heap, (priority, nxt))
                parent[nxt] = curr

    path = []
    state = goal
    while state != start:
        path.append(state)
        state = parent[state]
    path.append(start)
    path.reverse()

    return path, curr_cost[goal], expanded

def iterative():
    def get_nxt_states(state):
        x, y = state
        moves = [(x-1, y), (x, y-1), (x+1, y), (x, y+1)]
        nxt_states = [(x_, y_) for x_, y_ in moves if 0 <= x_ < n and 0 <= y_ < m and maze[x_][y_] != '%']
        return nxt_states
    
    def depth_limited_search(state, goal, depth_limit):
        stack = [(state, [], 0)]
        visited = 0
        while stack:
            state, path, cost = stack.pop()
            if state == goal:
                return path + [state], cost, visited
            if cost < depth_limit:
                for nxt in get_nxt_states(state):
                    if nxt not in path:
                        stack.append((nxt, path + [state], cost + 1))
                        visited += 1
        return None, None, visited

    for depth in range(1000000):
        path, cost, visited = depth_limited_search(start, goal, depth)
        if path is not None:
            return path, cost, visited
        

def astar(heuristic):
    start_x, start_y = start
    goal_x, goal_y = goal
    heap = []
    heapq.heappush(heap, (0, start_x, start_y))
    came_from = {}
    curr_cost = {}
    came_from[start] = None
    curr_cost[start], states_expanded, priority = 0, 0, 0

    while heap:
        _, x, y = heapq.heappop(heap)
        states_expanded += 1
        if (x, y) == goal:
            break

        nxt_states = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        for nx, ny in nxt_states:
            if is_valid(maze, nx, ny):
                new_cost = curr_cost[(x, y)] + 1
                if (nx, ny) not in curr_cost or new_cost < curr_cost[(nx, ny)]:
                    curr_cost[(nx, ny)] = new_cost
                    if heuristic == 'euclidian':
                        priority = new_cost + euclidian((nx, ny), (goal_x, goal_y))
                    elif heuristic == 'manhattan':
                        priority = new_cost + manhattan((nx, ny), (goal_x, goal_y))
                    heapq.heappush(heap, (priority, nx, ny))
                    came_from[(nx, ny)] = (x, y)

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path, curr_cost[goal], states_expanded


def print_maze(path, cost, states):
    if path:
        for i in range(n):
            for j in range(m):
                if (i, j) in path and (i, j) == start:
                    print("S", end="")
                elif (i, j) in path and (i, j) == goal:
                    print("G", end="")
                elif (i, j) in path:
                    print("x", end="")
                else:
                    print(' ', end="")
            print()
        print("Solution cost:", cost)
        print("Nodes expanded:", states)
    else:
        print("No solution found")


def switch(method):
    path, cost, nodes = '', 0, 0
    if method == 'depth-first':
        path, cost, nodes = depth_first()
    elif method == 'greedy':
        path, cost, nodes = best_first(heuristic)
    elif method == 'iterative':
        path, cost, nodes = iterative()
    elif method == 'astar':
        path, cost, nodes = astar(heuristic)
    print_maze(path, cost, nodes)


def compare():
    h_vals = ['euclidian', 'manhattan']
    for h in h_vals:
        print('========================================================================================')
        print('Heuristic:', h, '        Maze:', file_name)
        print('________________________________________________________________________________________')
        table = [['Name', 'Cost', 'Nodes Expanded', 'Time Spend']]
        start_time = time.time()
        _, cost, nodes = depth_first()
        end_time = time.time()
        table.append(['Depth-first', str(cost), str(nodes), end_time-start_time])

        start_time = time.time()
        _, cost, nodes = best_first(h)
        end_time = time.time()
        table.append(['Greedy', str(cost), str(nodes), end_time-start_time])

        result = run_with_timeout(iterative, [], 3)
        if result:
            start_time = time.time()
            _, cost, nodes = iterative()
            end_time = time.time()
            table.append(['Iterative', str(cost), str(nodes), end_time-start_time])
        else:
            table.append(['Iterative', 'Timeout', 'Timeout', '> 3s'])

        start_time = time.time()
        _, cost, nodes = astar(h)
        end_time = time.time()
        table.append(['Astar', str(cost), str(nodes), end_time-start_time])
        for name, cost, nodes, t in table:
            print ("{:<21} {:<21} {:<21} {:<21}".format(name, cost, nodes, t))
        print('========================================================================================')
        print()

def get_start_goal(maze):
    for x in range(len(maze)):
        for y in range(len(maze[x])):
            if maze[x][y] == 'S':
                start = (x, y)
            if maze[x][y] == 'G':
                goal = (x, y)
    return start, goal



if __name__ == "__main__":
    method = ''
    heuristic = ''
    maze = []
    to_compare = False

    for i, arg in enumerate(sys.argv):
        if arg == '-method':
            method = sys.argv[i+1]
        if arg == '-heuristic':
            heuristic = sys.argv[i+1]
        if arg == 'compare':
            to_compare = True

    file_name = sys.argv[-1]
    
    try:
        with open(file_name) as file:
            for line in file:
                maze.append([c for c in line.strip()])
            
        start, goal = get_start_goal(maze)
        n = len(maze)
        m = len(maze[0])

        if to_compare:
            compare()
        else:
            switch(method)
    except FileNotFoundError:
        print('File not found!')
    except ValueError:
        print("Invalid input.")


        
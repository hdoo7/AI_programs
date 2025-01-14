## Maze Navigation Algorithms

This repository contains Python implementations of various algorithms designed to solve maze navigation problems. The repository is organized into three folders:

1. **iterative_algo**
2. **search_algo**
3. **shortest_path**

---

### 1. **iterative_algo**

This folder contains implementations of iterative algorithms to explore mazes. It includes:

- **`get_successors(node)`**: Generates valid successors for a given node within the maze.
- **`iterative_deepening_search(maze)`**: Implements Iterative Deepening Search (IDS) to find the shortest path to the goal.
- **`depth_limited_search(node, goal, depth_limit)`**: Performs a Depth-Limited Search with a specified depth.

**Example Use Case:**

```python
maze = [
    ['%', '%', '%', 'S', '%'],
    [' ', '%', '%', ' ', '%'],
    [' ', ' ', '%', ' ', 'G'],
    ['%', ' ', '%', ' ', '%'],
    ['%', '%', '%', '%', '%']
]

path, cost, expanded = iterative_deepening_search(maze)

if path:
    print("Path found with cost:", cost)
    print("Nodes expanded:", expanded)
else:
    print("No path found.")
```

---

### 2. **search_algo**

This folder includes algorithms to navigate mazes using various heuristic and classical approaches:

- **Depth-First Search (DFS)**: Explores paths by going deep before backtracking.
- **Best-First Search**: Uses heuristics like Manhattan and Euclidean distance.
- **A* Search (A-star)**: Combines cost-so-far and heuristic distance to find optimal paths.

**Features:**

- **Heuristics**: Manhattan and Euclidean distances.
- **Timeout Mechanism**: Stops execution if it exceeds a given time limit.

**Example Use Case:**

```python
path, cost, nodes = best_first('manhattan')
print_maze(path, cost, nodes)
```

---

### 3. **shortest_path**

This folder will be dedicated to algorithms specifically designed to compute the shortest paths in mazes, such as Dijkstra's algorithm and advanced variations of A*.

---

### How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/hdoo7/AI_programs.git
   cd AI_programs
   ```

2. Navigate to the desired folder and execute the algorithms:
   ```bash
   cd iterative_algo
   python iterative.py
   ```

3. Customize the maze and parameters in the respective scripts.

---

### Requirements

- Python 3.6+
- Libraries: `heapq`, `math`, `signal`


---

### Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

---

### License

This repository is licensed under the MIT License. See the `LICENSE` file for details.

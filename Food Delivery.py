import heapq
import math
import time

class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h

    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()

def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(position, grid):
    directions = [(-1,0), (1,0), (0,-1), (0,1)]
    neighbors = []
    for dx, dy in directions:
        nx, ny = position[0] + dx, position[1] + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != '#':
            neighbors.append((nx, ny))
    return neighbors

def cost(position, grid):
    val = grid[position[0]][position[1]]
    return int(val) if val.isdigit() else 1

def reconstruct_path(current):
    path = []
    while current:
        path.append(current.position)
        current = current.parent
    return path[::-1]

def a_star(grid, start, goal):
    open_set = []
    start_time = time.time()
    start_node = Node(start, g=0, h=heuristic(start, goal))
    heapq.heappush(open_set, start_node)
    visited = set()
    node_count = 0

    while open_set:
        current = heapq.heappop(open_set)
        node_count += 1

        if current.position == goal:
            end_time = time.time()
            return reconstruct_path(current), node_count, end_time - start_time

        visited.add(current.position)

        for neighbor in get_neighbors(current.position, grid):
            if neighbor in visited:
                continue
            g = current.g + cost(neighbor, grid)
            h = heuristic(neighbor, goal)
            heapq.heappush(open_set, Node(neighbor, current, g, h))

    return [], node_count, time.time() - start_time

def gbfs(grid, start, goal):
    open_set = []
    start_time = time.time()
    start_node = Node(start, g=0, h=heuristic(start, goal))
    heapq.heappush(open_set, start_node)
    visited = set()
    node_count = 0

    while open_set:
        current = heapq.heappop(open_set)
        node_count += 1

        if current.position == goal:
            end_time = time.time()
            return reconstruct_path(current), node_count, end_time - start_time

        visited.add(current.position)

        for neighbor in get_neighbors(current.position, grid):
            if neighbor in visited:
                continue
            h = heuristic(neighbor, goal)
            heapq.heappush(open_set, Node(neighbor, current, 0, h))

    return [], node_count, time.time() - start_time

def print_grid(grid, path):
    grid_copy = [row[:] for row in grid]
    for x, y in path:
        if grid_copy[x][y] not in ['S', 'G']:
            grid_copy[x][y] = '*'
    for row in grid_copy:
        print(' '.join(row))

# Map
city_map = [
    ['S', '1', '1', '1', 'G'],
    ['#', '#', '1', '#', '1'],
    ['1', '1', '1', '1', '1'],
    ['1', '#', '1', '#', '1'],
    ['1', '1', '1', '1', '1'],
]

start = (0, 0)
goal = (0, 4)

# A* Test
print("A* Search:")
a_path, a_nodes, a_time = a_star(city_map, start, goal)
print_grid(city_map, a_path)
print(f"Nodes Visited: {a_nodes}")
print(f"Execution Time: {a_time:.6f} seconds\n")

# GBFS Test
print("Greedy Best-First Search:")
g_path, g_nodes, g_time = gbfs(city_map, start, goal)
print_grid(city_map, g_path)
print(f"Nodes Visited: {g_nodes}")
print(f"Execution Time: {g_time:.6f} seconds")


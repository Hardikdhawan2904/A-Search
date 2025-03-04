import heapq
import random

class Node:
    def __init__(self, position, parent=None, g=0, h=0, name=None):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h
        self.name = name

    def __lt__(self, other):  # Corrected __lt_
        return self.f < other.f

def a_star_search(grid, start, goal, heuristic_func, node_names):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    closed_set = set()

    start_node = Node(start, None, 0, heuristic_func(start, goal), name=node_names[start[0]][start[1]])
    heapq.heappush(open_list, start_node)

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    while open_list:
        current_node = heapq.heappop(open_list)  # Get node with the lowest f-score

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node)
                current_node = current_node.parent
            return path[::-1]  # Return path from start to goal

        closed_set.add(current_node.position)

        for dx, dy in directions:
            neighbor_pos = (current_node.position[0] + dx, current_node.position[1] + dy)

            if (0 <= neighbor_pos[0] < rows and 0 <= neighbor_pos[1] < cols and
                    grid[neighbor_pos[0]][neighbor_pos[1]] == 0 and
                    neighbor_pos not in closed_set):
                
                g_cost = current_node.g + 1
                h_cost = heuristic_func(neighbor_pos, goal)
                neighbor_node = Node(neighbor_pos, current_node, g_cost, h_cost, name=node_names[neighbor_pos[0]][neighbor_pos[1]])

                print(f"Heuristic from {neighbor_node.name} to goal: {h_cost}")

                heapq.heappush(open_list, neighbor_node)

    return None  # No path found

def main():
    grid = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    start = (0, 0)
    goal = (4, 4)
    
    node_names = [
        ['A', '', 'C', 'D', 'E'],
        ['F', ' ', 'G', ' ', 'H'],
        ['I', 'J', 'K', ' ', 'L'],
        [' ', ' ', 'M', ' ', 'N'],
        ['O', 'P', 'Q', 'R', 'S']
    ]

    def random_heuristic(a, b):
        return random.randint(1, 10)  # Random heuristic for testing

    path_nodes = a_star_search(grid, start, goal, random_heuristic, node_names)

    if path_nodes:
        path_names = [node.name for node in path_nodes]
        path_output = ' -> '.join(path_names)
        print("Shortest Path:", path_output)
    else:
        print("No path found.")

if __name__ == "__main__":  # Corrected __name_
    main()
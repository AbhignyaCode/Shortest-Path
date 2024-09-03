import random
import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAX_INT = float('inf')

def dijkstra(graph, start):
    num_nodes = len(graph)
    distances = [MAX_INT] * num_nodes
    distances[start] = 0
    pq = [(0, start)]
    prev = {}

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_dist > distances[current_node]:
            continue

        for neighbor, weight in enumerate(graph[current_node]):
            if weight > 0:
                distance = current_dist + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    prev[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

    return prev, distances

def print_path(prev, start, end):
    path = []
    node = end
    while node != start:
        path.append(node)
        if node not in prev:  # Check if the node is reachable
            print("Destination node is unreachable.")
            return None
        node = prev[node]
    path.append(start)
    path.reverse()
    return path

def simulate_travel(path, graph, node_positions, source, destination, delay=0.5):
    if path is None:
        return  # Exit if the destination node is unreachable

    fig, ax = plt.subplots(figsize=(12, 12), dpi=200)  # Adjust DPI for higher resolution
    ax.set_xlim(0, 600)
    ax.set_ylim(0, 600)
    ax.set_aspect('equal')

    # Plot nodes
    for pos in node_positions:
        ax.plot(pos[0], pos[1], 'ko', markersize=8)

    # Plot edges with weights
    for i in range(len(graph)):
        for j in range(len(graph)):
            if graph[i][j] > 0:
                x1, y1 = node_positions[i]
                x2, y2 = node_positions[j]
                ax.plot([x1, x2], [y1, y2], 'k-', lw=1)  # Black lines representing edges
                offset = 10
                ax.text((x1 + x2) / 2 + offset, (y1 + y2) / 2 + offset, str(graph[i][j]), fontsize=10, ha='center', va='center', color='blue')

    # Plot path
    path_x = [node_positions[node][0] for node in path]
    path_y = [node_positions[node][1] for node in path]
    line, = ax.plot([], [], 'r-', lw=2)

    # Plot car
    car, = ax.plot([], [], 'ro', markersize=10)

    # Label source and destination
    ax.text(node_positions[source][0], node_positions[source][1], 'Source', fontsize=12, ha='center')
    ax.text(node_positions[destination][0], node_positions[destination][1], 'Destination', fontsize=12, ha='center')

    def animate(i):
        x = path_x[i]
        y = path_y[i]
        car.set_data(x, y)
        line.set_data(path_x[:i + 1], path_y[:i + 1])
        return line, car

    ani = animation.FuncAnimation(fig, animate, frames=len(path), interval=delay * 1000, blit=True, repeat=False)
    plt.show()

def build_graph(num_nodes, node_positions):
    # Initialize adjacency matrix with all zeros
    graph = np.zeros((num_nodes, num_nodes), dtype=int)

    # Connect each node to every other node (fully connected graph)
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                graph[i][j] = random.randint(1, 100)  # Random positive integer cost

    return graph

# Example usage
num_nodes = 3
# Increased spacing between nodes by setting a wider range for random positions
node_positions = [(random.randint(100, 500), random.randint(100, 500)) for _ in range(num_nodes)]

graph = build_graph(num_nodes, node_positions)

start = 0
end = num_nodes - 1

prev, distances = dijkstra(graph, start)
shortest_path = print_path(prev, start, end)

simulate_travel(shortest_path, graph, node_positions, start, end)

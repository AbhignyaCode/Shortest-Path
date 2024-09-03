import heapq
import random
import simpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAX_INT = float('inf')

def dijkstra(graph, start):
    distances = {node: MAX_INT for node in graph}
    distances[start] = 0
    pq = [(0, start)]
    prev = {}

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_dist > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
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

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, 400)
    ax.set_ylim(0, 400)
    ax.set_aspect('equal')

    # Plot nodes
    for pos in node_positions:
        ax.plot(pos[0], pos[1], 'ko', markersize=5)

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
    graph = {}
    for i in range(num_nodes):
        distances = []
        for j in range(num_nodes):
            if i != j:
                cost = ((node_positions[i][0] - node_positions[j][0])**2 + (node_positions[i][1] - node_positions[j][1])**2)**0.5
                distances.append((j, cost))
        distances.sort(key=lambda x: x[1])  # Sort by distance
        nearest_neighbors = distances[:int(num_nodes * 0.1)]  # Nearest 10% of nodes
        graph[i] = {neighbor: weight for neighbor, weight in nearest_neighbors}
    return graph

# Example usage
num_nodes = 20
node_positions = [(random.randint(50, 350), random.randint(50, 350)) for _ in range(num_nodes)]

graph = build_graph(num_nodes, node_positions)

start = 0
end = num_nodes - 1

prev, distances = dijkstra(graph, start)
shortest_path = print_path(prev, start, end)

simulate_travel(shortest_path, graph, node_positions, start, end)

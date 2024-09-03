# Shortest-Path
This project demonstrates the development of an algorithm that dynamically computes the next shortest path of vehicles taking into account real time traffic conditions 
# Shortest Path Simulation with Real-Time Traffic Visualization

This project provides a simulation for finding the shortest path between nodes in a graph using Dijkstra's algorithm and visualizing the traversal with `matplotlib`. It also demonstrates how real-time traffic conditions can be simulated using OpenStreetMap (OSM) data.

## Features

- **Graph Creation**: Generates a graph with nodes placed randomly on a 2D plane. Supports both adjacency list and adjacency matrix representations.
- **Shortest Path Calculation**: Implements Dijkstra's algorithm to find the shortest path between a source and a destination node.
- **Real-Time Traffic Integration**: Simulates real-time traffic conditions using OpenStreetMap (OSM) data.
- **Visualization**: Animates the traversal of the shortest path using `matplotlib`, with adjustable node positions and edge weights.

## Requirements

- Python 3.x
- Required Python libraries:
  - `heapq`: For implementing a priority queue in Dijkstra's algorithm.
  - `random`: To generate random node positions and weights.
  - `numpy`: For creating and manipulating an adjacency matrix.
  - `matplotlib`: For visualizing the graph and animation.
  - `animation`: To create animations of the car's journey along the path.
  - `OSMNX` and `NetworkX` (if integrating with OpenStreetMap data): For fetching and handling OSM data to simulate real-time traffic conditions.

## Installation

1. **Clone the repository**:

    ```bash
    git clone https://github.com/AbhignyaCode/shortest-path-simulation.git
    cd shortest-path-simulation
    ```

2. **Install the required Python libraries**:

    ```bash
    pip install matplotlib numpy osmnx networkx
    ```

## OpenStreetMap (OSM) Integration

To simulate real-time traffic conditions, this project can integrate data from OpenStreetMap (OSM) using libraries like `OSMNX` and `NetworkX`. OSM provides geographic data, including road networks, which can be used to create more realistic traffic simulations. 

### How to Use OSM Data

1. **Fetch OSM Data**:

   Use the `OSMNX` library to fetch road network data for a specific geographic location:

   ```python
   import osmnx as ox

   # Define the location and fetch the road network
   place_name = "Manhattan, New York, USA"
   G = ox.graph_from_place(place_name, network_type='drive')

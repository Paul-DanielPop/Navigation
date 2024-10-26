# Navigation Search Algorithms

This project is an implementation of various search algorithms to solve a navigation problem on a metro map. The goal is to find the optimal route between two points on the map, considering different criteria such as the number of stops, travel time, or distance.

## Project Description

The project consists of two main parts:

1. **Part 1: Uninformed Search Methods**
   - Implemented algorithms:
     - Depth-First Search (DFS)
     - Breadth-First Search (BFS)
   - These methods explore the map without considering costs or heuristics, simply searching for a path from the start to the destination.

2. **Part 2: Informed Search Methods**
   - Implemented algorithms:
     - Uniform Cost Search
     - A* Search
   - These methods incorporate cost calculations and heuristics to find optimal solutions more efficiently.

The algorithms are tested using a metro map of Lyon, which includes information about the stations, connections, travel times, and line speeds.

## Features

- **Uninformed Search Algorithms (DFS, BFS):** Explore the metro map by visiting neighboring nodes in different orders, without considering path costs.
- **Informed Search Algorithms (Uniform Cost, A*):** Use cost functions and heuristics to find the optimal path, taking into account travel time, distance, or the number of transfers.
- **Handling of cycles and redundant paths:** Implemented functions to avoid infinite loops and eliminate non-optimal partial paths.
- **Distance calculation from coordinates to the nearest stations:** Allows finding the optimal path even when the starting or destination point is not a metro station.

## Project Structure

- **cityInformation/**: Contains data files representing the metro map:
  - `InfoVelocity.txt`: Speed information for each metro line.
  - `Stations.txt`: Station details, including ID, name, line number, and coordinates.
  - `Time.txt`: Travel time between connected stations.
  - `Lyon-city.jpg`: Image of the metro map for reference.
- **Code/**: Contains Python files for the implementation:
  - `SearchAlgorithm.py`: The main file where the search algorithms are implemented.
  - `SubwayMap.py`: Defines the `Map` and `Path` classes used to represent the metro network and routes.
  - `TestCases.py`: Includes test cases for verifying the algorithms.

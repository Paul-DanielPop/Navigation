# This file contains all the required routines to make an A* search algorithm.
#
__author__ = '1607129'

# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2023 - 2024
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy
from collections import deque


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    path_list = []
    current_station = path.last
    connections_from_current_station = map.connections.get(current_station, {})

    for connected_station, cost in connections_from_current_station.items():
        new_route = path.route.copy()
        current_cost = path.g
        current_heuristic = path.h
        new_route.append(connected_station)

        new_path = Path(new_route)
        new_path.g = current_cost
        new_path.h = current_heuristic
        path_list.append(new_path)

    return path_list


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """

    unique_path_list = []

    for path in path_list:
        route_set = set(path.route)
        route_list = list(route_set)

        if len(route_list) == len(path.route):
            unique_path_list.append(path)

    return unique_path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """

    list_of_path[:0] = expand_paths
    return list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """

    list_of_path = [Path([origin_id])]

    while list_of_path:
        current_path = list_of_path.pop(0)
        current_station = current_path.last

        if current_station == destination_id:
            return current_path

        expanded_paths = expand(current_path, map)
        unique_expanded_paths = remove_cycles(expanded_paths)
        list_of_path = insert_depth_first_search(unique_expanded_paths, list_of_path)

    return "No solution"


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    list_of_path.extend(expand_paths)
    return list_of_path


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    queue = deque([Path([origin_id])])

    while queue:
        current_path = queue.popleft()
        current_station = current_path.last

        if current_station == destination_id:
            return current_path

        expanded_paths = expand(current_path, map)
        unique_expanded_paths = remove_cycles(expanded_paths)
        queue = insert_breadth_first_search(unique_expanded_paths, queue)

    return "No solution"


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    for path in expand_paths:
        penultimate_station = path.penultimate
        current_station = path.last
        cost_between_stations = map.connections[penultimate_station][current_station]

        if type_preference == 0:
            path.update_g(1)
        elif type_preference == 1:
            path.update_g(cost_between_stations)
        elif type_preference == 2:
            if map.stations[penultimate_station]['name'] == map.stations[current_station]['name']:
                path.update_g(0)
            else:
                speed = map.stations[current_station]['velocity']
                distance = cost_between_stations * speed
                path.update_g(distance)
        else:
            if map.stations[penultimate_station]['name'] == map.stations[current_station]['name']:
                path.update_g(1)

    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    combined_list = list_of_path + expand_paths
    sorted_list_path = sorted(combined_list, key=lambda path: (path.g, len(path.route)))

    return sorted_list_path


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    list_of_path = [Path([origin_id])]

    while list_of_path:
        current_path = list_of_path.pop(0)
        current_station = current_path.last

        if current_station == destination_id:
            return current_path

        expanded_paths = expand(current_path, map)
        unique_expanded_paths = remove_cycles(expanded_paths)
        updated_path = calculate_cost(unique_expanded_paths, map, type_preference)
        list_of_path = insert_cost(updated_path, list_of_path)

    return "No solution"


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """

    for path in expand_paths:
        current_station = path.last

        if type_preference == 0:
            if current_station == destination_id:
                path.update_h(0)
            else:
                path.update_h(1)
        elif type_preference == 1:
            velocities = [info["velocity"] for info in map.stations.values()]
            h = (euclidean_dist((map.stations[current_station]['x'], map.stations[current_station]['y']),
                                (map.stations[destination_id]['x'], map.stations[destination_id]['y']))
                 / max(velocities))
            path.update_h(h)
        elif type_preference == 2:
            dist = euclidean_dist((map.stations[current_station]['x'], map.stations[current_station]['y']),
                                  (map.stations[destination_id]['x'], map.stations[destination_id]['y']))
            path.update_h(dist)
        else:
            current_station_line = map.stations[current_station]['line']
            destination_line = map.stations[destination_id]['line']
            if current_station_line == destination_line:
                path.update_h(0)
            else:
                path.update_h(1)

    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """

    non_redundant_paths = []
    for path in expand_paths:
        last_station = path.last

        if last_station in visited_stations_cost:
            if path.g < visited_stations_cost[last_station]:
                visited_stations_cost[last_station] = path.g
                non_redundant_paths.append(path)
                for path2 in list_of_path:
                    last = path2.last
                    if last_station == path2.last:
                        list_of_path.remove(path2)
        else:
            visited_stations_cost[last_station] = path.g
            non_redundant_paths.append(path)

    return non_redundant_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    update_f(expand_paths)
    combined_list = list_of_path + expand_paths
    sorted_list_path = sorted(combined_list, key=lambda path: (path.f, len(path.route)))

    return sorted_list_path


def distance_to_stations(coord, map):
    """
        From coordinates, it computes the distance to all stations in map.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            (dict): Dictionary containing as keys, all the Indexes of all the stations in the map, and as values, the
            distance between each station and the coord point
    """
    distances = {}

    for station_id, info in map.stations.items():
        station_coord = [info['x'], info['y']]
        distance = euclidean_dist(coord, station_coord)
        distances[station_id] = distance

    distances = dict(sorted(distances.items(), key=lambda item: (item[1], item[0])))

    return distances


def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path([origin_id])]
    visited_stations_cost = {}

    while list_of_path:
        current_path = list_of_path.pop(0)
        current_station = current_path.last

        if current_station == destination_id:
            return current_path

        expanded_paths = expand(current_path, map)
        unique_expanded_paths = remove_cycles(expanded_paths)
        updated_path = calculate_cost(unique_expanded_paths, map, type_preference)
        non_redundant_paths, list_of_path, visited_stations_cost = remove_redundant_paths(
            updated_path, list_of_path, visited_stations_cost)
        updated_path = calculate_heuristics(non_redundant_paths, map, destination_id, type_preference)
        list_of_path = insert_cost_f(updated_path, list_of_path)

    return "No solution"


def Astar_improved(origin_coord, destination_coord, map):
    """
    in this version you give an initial location x,y and final location x,y and find the optimal path
    """
    walking_speed = 5

    list_of_solutions = []

    distances_to_origin_station = distance_to_stations(origin_coord, map)

    distances_to_destination_station = distance_to_stations(destination_coord, map)

    first_5_origin_stations = {key: distances_to_origin_station[key] for key in
                               list(distances_to_origin_station.keys())[:5]}

    first_5_destination_stations = {key: distances_to_destination_station[key] for key in
                                    list(distances_to_destination_station.keys())[:5]}

    for i in range(len(first_5_origin_stations)):
        origin_station_id = list(first_5_origin_stations.keys())[i]
        time_cost_to_origin_station = first_5_origin_stations[origin_station_id] / walking_speed
        for j in range(len(first_5_destination_stations)):
            path = Path([0])
            destination_station_id = list(first_5_destination_stations.keys())[j]
            time_cost_to_destination_station = first_5_destination_stations[destination_station_id] / walking_speed

            solution = Astar(origin_station_id, destination_station_id, map, 1)
            path.route.extend(solution.route)
            path.route.extend([-1])
            path.f = solution.f + time_cost_to_origin_station + time_cost_to_destination_station
            path.f = round(path.f, 6)

            list_of_solutions.append(path)

    sorted_solutions = sorted(list_of_solutions, key=lambda path: (path.f, len(path.route)))

    walking_time_cost_origin_destination = euclidean_dist(origin_coord, destination_coord) / walking_speed

    if walking_time_cost_origin_destination < sorted_solutions[0].f:
        path = Path([0, -1])
        path.f = walking_time_cost_origin_destination
        return path
    else:
        return sorted_solutions[0]

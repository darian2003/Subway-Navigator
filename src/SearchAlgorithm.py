# This file contains all the required routines to make an A* search algorithm.
#
import copy
__author__ = '1719581'
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

    # browse all the stations that are connected to this station
    for next_possible_station in map.connections[current_station]:
        # make a copy of the current route
        new_path = copy.deepcopy(path)

        # append the next possible station to the base route
        new_path.add_route(next_possible_station)
        path_list.append(new_path)

    return path_list


def remove_cycles(path_list):

    """Removes paths containing cycles from the given path list.

    Args:
        path_list (list): Expanded paths (list of Path objects).

    Returns:
        list: Expanded paths without cycles.
    """

    return [path for path in path_list if len(set(path.route)) == len(path.route)]



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
    for path in reversed(expand_paths):
        list_of_path.insert(0, path)

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
    # initializing the list (actually stack) of nodes that need to be visited with the origin station
    path_list = [Path(origin_id)]

    while path_list:

        # explore the first node from the list
        current_path = path_list.pop(0)

        # check if we reached the destination
        if current_path.last == destination_id:
            return current_path

        # expand node, remove cycles and add new paths to stack
        expand_paths = expand(current_path, map)
        expand_paths = remove_cycles(expand_paths)
        path_list = insert_depth_first_search(expand_paths, path_list)

    return []


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
    # initializing the list (actually stack) of nodes that need to be visited with the origin station
    path_list = [Path(origin_id)]

    while path_list:

        # explore the first node from the list
        current_path = path_list.pop(0)

        # check if we reached the destination
        if current_path.last == destination_id:
            return current_path

        # expand node, remove cycles and add new paths to stack
        expand_paths = expand(current_path, map)
        expand_paths = remove_cycles(expand_paths)
        path_list = insert_breadth_first_search(expand_paths, path_list)

    return []

# checks the coordinates of the two stations and decides whether they are the same
def same_station(station1, station2, map):
    if map.stations[station1]["x"] == map.stations[station2]["x"] and map.stations[station1]["y"] == map.stations[station2]["y"]:
        return True
    return False
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
    if type_preference == 0:
        for path in expand_paths:
            path.update_g(1)
    elif type_preference == 1:
        for path in expand_paths:
            path.update_g(map.connections[path.penultimate][path.last])
    elif type_preference == 2:
        for path in expand_paths:
            if not same_station(path.penultimate,path.last, map):
                path.update_g(map.connections[path.penultimate][path.last] * map.velocity[map.stations[path.penultimate]["line"]])
    else:
        for path in expand_paths:
            if map.stations[path.penultimate]["line"] != map.stations[path.last]["line"]:
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
    for path_to_add in expand_paths:
        added = 0
        for index, path in enumerate(list_of_path):
            if path_to_add.g < path.g:
                list_of_path.insert(index, path_to_add)
                added = 1
                break
        if added == 0:
            list_of_path.append(path_to_add)

    return list_of_path



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
    path_list = [Path(origin_id)]

    while path_list:
        # explore the first node from the list
        current_path = path_list.pop(0)

        # check if we reached the destination
        if current_path.last == destination_id:
            return current_path

        # expand node, remove cycles and add new paths to stack
        expand_paths = expand(current_path, map)
        expand_paths = remove_cycles(expand_paths)
        expand_paths = calculate_cost(expand_paths, map,type_preference)
        path_list = insert_cost(expand_paths, path_list)


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
    if type_preference == 0:
        for path in expand_paths:
            if path.last == destination_id:
                path.update_h(0)
            else :
                path.update_h(1)
    elif type_preference == 1:
        for path in expand_paths:
            if path.last == destination_id:
                path.update_h(0)
            else :
                id = path.last
                distance = euclidean_distance(map.stations[id]["x"], map.stations[id]["y"], map.stations[destination_id]["x"], map.stations[destination_id]["y"])
                # find the speed of the fastest line
                max_speed = max(map.velocity.values())
                path.update_h(distance/max_speed)
    elif type_preference == 2:
        for path in expand_paths:
            if path.last == destination_id:
                path.update_h(0)
            else:
                id = path.last
                distance = euclidean_distance(map.stations[id]["x"], map.stations[id]["y"],
                                                map.stations[destination_id]["x"], map.stations[destination_id]["y"])
                path.update_h(distance)
    else :
        for path in expand_paths:
            if map.stations[path.last]["line"] == map.stations[destination_id]["line"]:
                path.update_h(0)
            else :
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
    for path in expand_paths :
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

    # expand_paths = [path for path in expand_paths if path.g < visited_stations_cost[path.last]]

    new_expanded_paths = []

    for path in expand_paths:
        if path.last not in visited_stations_cost:
            new_expanded_paths.append(path)
            visited_stations_cost[path.last] = path.g
        elif path.g < visited_stations_cost[path.last]:
            new_expanded_paths.append(path)
            visited_stations_cost[path.last] = path.g
            list_of_path = [x for x in list_of_path if path.last not in x.route]

    return new_expanded_paths, list_of_path, visited_stations_cost

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
    for path_to_add in expand_paths:
        added = 0
        for index, path in enumerate(list_of_path):
            if path_to_add.f < path.f:
                list_of_path.insert(index, path_to_add)
                added = 1
                break
        if added == 0:
            list_of_path.append(path_to_add)

    return list_of_path

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
def distance_to_stations(coord, map):
    """
        From coordinates, it computes the distance to all stations in map.
        Format of the parameter is:
        Args:
            coord (list): Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            (dict): Dictionary containing as keys, all the Indexes of all the stations in the map, and as values, the
            distance between each station and the coord point
    """
    distances_list = []
    x = coord[0]
    y = coord[1]
    for station_id, info in map.stations.items():
        distance = euclidean_distance(x, y, info['x'], info['y'])
        # now add the tuple formed by (station_id, distance) into an ordered list
        if not distances_list:
            distances_list = [(station_id, distance)]
            continue
        for i in range(len(distances_list)):
            if distance < distances_list[i][1]:
                distances_list.insert(i, (station_id, distance))
                break
            elif distance == distances_list[i][1] and station_id < distances_list[i][0]:
                distances_list.insert(i, (station_id, distance))
                break
            elif i == len(distances_list) - 1:
                distances_list.append((station_id, distance))
            else:
                i += 1

    # now transform this list into a dict
    my_dict = dict()
    for station_id, distance in distances_list:
        my_dict[station_id] = distance

    return my_dict


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
    path_list = [Path(origin_id)]
    visited_station_cost = {origin_id: 0}

    while path_list:
        # explore the first node from the list
        current_path = path_list.pop(0)

        # check if we reached the destination
        if current_path.last == destination_id:
            return current_path

        # expand node, remove cycles and add new paths to stack
        expand_paths = expand(current_path, map)
        expand_paths = remove_cycles(expand_paths)
        expand_paths = calculate_cost(expand_paths, map, type_preference)
        expand_paths, path_list, visited_station_cost = remove_redundant_paths(expand_paths,path_list, visited_station_cost)
        expand_paths = calculate_heuristics(expand_paths, map, destination_id, type_preference)
        expand_paths = update_f(expand_paths)
        path_list = insert_cost_f(expand_paths, path_list)
       # print(f"{[p.route for p in path_list]} + {[p.f for p in path_list]}")


def Astar_improved(origin_coord, destination_coord, map):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_coord (list): Two REAL values, which refer to the coordinates of the starting position
            destination_coord (list): Two REAL values, which refer to the coordinates of the final position
            map (object of Map class): All the map information

        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_coord to destination_coord
    """

    walking_line_id = len(map.velocity) + 1
    map.velocity[walking_line_id] = 5
    origin_station = {"name": "origin", "line": walking_line_id, "x": origin_coord[0], "y": origin_coord[1], "velocity": 5}
    destination_station = {"name": "destination", "line": walking_line_id, "x": destination_coord[0], "y": destination_coord[1], "velocity": 5}
    distance = euclidean_distance(origin_coord[0], origin_coord[1], destination_coord[0], destination_coord[1])
    map.connections[0] = {-1: distance / 5}
    map.stations[0] = origin_station
    map.stations[-1] = destination_station

    # Enable walking from starting location to all the existing stations.
    # Enable walking from all the stations to the final destination
    for station_id, station_info in map.stations.items():
        if station_id == 0 or station_id == -1:
            continue
        distance = euclidean_distance(origin_coord[0], origin_coord[1], station_info["x"], station_info["y"])
        map.connections[0][station_id] = distance / 5
        distance = euclidean_distance(destination_coord[0], destination_coord[1], station_info["x"], station_info["y"])
        map.connections[station_id][-1] = distance / 5

    return Astar(0, -1, map, 1)












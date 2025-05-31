#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt

def transform_coordinates(coordinates, resolution, ros_origin=(-23.3, -13.5)):
    '''
    Converts map pgm image pixels into ROS occupancy grid coordinates.
    '''

    return np.array([
        [ros_origin[0] + resolution * coordinate[1], -ros_origin[1] - resolution * coordinate[0]]
        for coordinate in coordinates
    ])

def inverse_transform_coordinates(transformed_coords, resolution):
    '''
    Maps ROS occupancy grid coordinates into image pixels.
    '''

    return [
        [int((1/resolution) * (-transformed_coord[1] + 13.5)), int((1/resolution) * (transformed_coord[0] + 23.3))]
        for transformed_coord in transformed_coords
    ]

def generate_random_coords(obstacle_positions, x_range, y_range, n_points):
    '''
    Generates random points in the free space bounded by the obstacles.
    '''

    x_min, x_max = x_range
    y_min, y_max = y_range

    all_positions = set((x, y) for x in range(x_min, x_max + 1) for y in range(y_min, y_max + 1)) # maps space bounded by obstacles
    free_positions = list(all_positions - obstacle_positions) # returns all free space pixels from obstacle bounded space

    if n_points > len(free_positions): # in the case that we request more random points than free space pixels
        raise ValueError("Cannot place {} points. Maximum possible points are {}".format(n_points, len(free_positions)))

    return [free_positions[i] for i in np.random.choice(len(free_positions), n_points, replace=False)] # return random points (free space pixels) from obstacle bounded space, does not allow duplication

def line(point_1, point_2):
    '''
    Returns all points in the straight line between point 1 and 2.
    '''

    x1, y1 = map(int, point_1)
    x2, y2 = map(int, point_2)

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    steep = dy > dx

    if steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    else:
        swapped = False

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    error = dx // 2
    ystep = 1 if y1 < y2 else -1
    y = y1
    points = []

    for x in range(x1, x2 + 1):
        coord = (y, x) if steep else (x, y)
        points.append(coord)

        error -= dy
        if error < 0:
            y += ystep
            error += dx

    if swapped:
        points.reverse()

    return list(zip(*points))

def collision_free_line(point_1, point_2, obstacle_positions):
    '''
    Checks if two random points are in sight using line test.
    '''

    x, y = line(point_1, point_2)
    for i in range(len(x)):
        if (x[i], y[i]) in obstacle_positions:
            return False
    return True

def distance_matrix(random_coordinates, transformed_random_coordinates, obstacle_positions):
    '''
    Computes matrix of distances between random points.
    '''

    adjacency_matrix = np.zeros((len(transformed_random_coordinates), len(transformed_random_coordinates)))
    for i in range(len(transformed_random_coordinates)):
        for j in range(i + 1, len(transformed_random_coordinates)):
            if collision_free_line(random_coordinates[i], random_coordinates[j], obstacle_positions):
                distance = np.linalg.norm(transformed_random_coordinates[i] - transformed_random_coordinates[j])
                adjacency_matrix[i][j] = distance
                adjacency_matrix[j][i] = distance
    return adjacency_matrix

def dijkstras_algorithm(distance_matrix, coordinates):
    '''
    Find the shortest path in a graph of connected in-sight random points using Dijkstra's algorithm.
    '''

    num_nodes = len(coordinates)
    origin_index = num_nodes - 2
    goal_index = num_nodes - 1

    distances = np.full(num_nodes, np.inf)
    previous_nodes = np.full(num_nodes, -1)
    unvisited_nodes = list(range(num_nodes))
    distances[origin_index] = 0

    while unvisited_nodes:
        current_node = unvisited_nodes[np.argmin(distances[unvisited_nodes])]
        if distances[current_node] == np.inf or current_node == goal_index:
            break
        for neighbor, distance in enumerate(distance_matrix[current_node]):
            if distance and neighbor in unvisited_nodes:
                alt_distance = distances[current_node] + distance
                if alt_distance < distances[neighbor]:
                    distances[neighbor] = alt_distance
                    previous_nodes[neighbor] = current_node
        unvisited_nodes.remove(current_node)

    # Reconstruct path
    path_indices = []
    node = goal_index
    while node != -1:
        path_indices.append(node)
        node = previous_nodes[node]
    path_indices.reverse()

    if path_indices[0] == origin_index:
        return [coordinates[i] for i in path_indices], path_indices
    else:
        return []

def plot_path(random_coordinates, transformed_random_coordinates, transformed_obstacle_coordinates, obstacle_positions, path_indices):
    '''
    Plot the graph between in-sight coordinates and the shortest path.
    '''

    plt.figure(figsize=(10, 10))
    plt.scatter(transformed_obstacle_coordinates[:, 0], transformed_obstacle_coordinates[:, 1], c='b')
    plt.scatter(transformed_random_coordinates[:, 0], transformed_random_coordinates[:, 1], c='r')
    for i in range(len(transformed_random_coordinates)):
        for j in range(i + 1, len(transformed_random_coordinates)):
            if collision_free_line(random_coordinates[i], random_coordinates[j], obstacle_positions):
                plt.plot([transformed_random_coordinates[i, 0], transformed_random_coordinates[j, 0]], [transformed_random_coordinates[i, 1], transformed_random_coordinates[j, 1]], 'g-')
    for i in range(len(path_indices) - 1):
        plt.plot([transformed_random_coordinates[path_indices[i], 0], transformed_random_coordinates[path_indices[i+1], 0]],
                 [transformed_random_coordinates[path_indices[i], 1], transformed_random_coordinates[path_indices[i+1], 1]], 'm-', linewidth=2.0)
    plt.savefig('motion_planning.png')

def get_path(source, destination):
    '''
    Motion planning using Probabilistic Roadmap.
    '''

    img = cv2.imread('mapping.pgm', cv2.IMREAD_GRAYSCALE)
    occupied_thresh = 0.65
    thresh = int(occupied_thresh * 255)

    _, obstacles = cv2.threshold(img, thresh, 255, cv2.THRESH_BINARY_INV)

    obstacle_coordinates = np.column_stack(np.where(obstacles > 0))
    obstacle_positions = set((x, y) for x, y in obstacle_coordinates)

    min_x, max_x = obstacle_coordinates[:, 0].min(), obstacle_coordinates[:, 0].max()
    min_y, max_y = obstacle_coordinates[:, 1].min(), obstacle_coordinates[:, 1].max()

    n_points = 75
    random_coordinates = generate_random_coords(obstacle_positions, (min_x, max_x), (min_y, max_y), n_points)
    
    goal = inverse_transform_coordinates([destination], 0.05)[0]
    origin = inverse_transform_coordinates([source], 0.05)[0]

    random_coordinates.extend([origin, goal])
    random_coordinates = np.array(random_coordinates)

    s = 0.05
    transformed_random_coordinates = transform_coordinates(random_coordinates, s)
    transformed_obstacle_coordinates = transform_coordinates(obstacle_coordinates, s)

    adjacency_matrix = distance_matrix(random_coordinates, transformed_random_coordinates, obstacle_positions)

    path, path_indices = dijkstras_algorithm(adjacency_matrix, transformed_random_coordinates)    

    plot_path(random_coordinates, transformed_random_coordinates, transformed_obstacle_coordinates, obstacle_positions, path_indices)
    
    return path

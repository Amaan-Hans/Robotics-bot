#!/usr/bin/env python
# coding: utf-8

# In[828]:

# In[830]:


import cv2
import numpy as np
import matplotlib.pyplot as plt


def get_path(origin_point, goal_point):

    # Load the .pgm file
    img = cv2.imread('best_so_far_2.pgm', cv2.IMREAD_GRAYSCALE)

    # Define the area of interest
    x_center, y_center = 200, 190  # center of the map
    crop_size = 1200 # set this to 100 for a 200x200 output image

    # Crop the image
    img_cropped = img #img[y_center-crop_size:y_center+crop_size, x_center-crop_size:x_center+crop_size]

    # Define the threshold for obstacles based on the YAML file
    occupied_thresh = 0.65

    # Convert the threshold to a value between 0 and 255
    thresh = int(occupied_thresh * 255)

    # Threshold the image: below the threshold is considered obstacle
    _, obstacles = cv2.threshold(img_cropped, thresh, 255, cv2.THRESH_BINARY_INV)

    # Find the obstacle coordinates
    obstacle_coords = np.column_stack(np.where(obstacles > 0))

    # print(obstacle_coords)

    # Adjust obstacle coordinates according to the new cropped image
    # obstacle_coords[:, 0] = y_center - crop_size + obstacle_coords[:, 0]  # for y-coordinates
    # obstacle_coords[:, 1] = x_center - crop_size + obstacle_coords[:, 1]  # for x-coordinates

    # Write obstacle coordinates to a text file
    with open('obstacle_coords.txt', 'w') as f:
        for item in obstacle_coords:
            f.write("%s\n" % item)









    # I'm assuming that obstacle_coords are integers in the defined coordinate system
    obstacle_positions = set([(x,y) for x, y in obstacle_coords])

    # Define coordinate ranges based on obstacle_coords
    x_range = (min(obstacle_coords[:, 0]), max(obstacle_coords[:, 0]))
    y_range = (min(obstacle_coords[:, 1]), max(obstacle_coords[:, 1]))

    def inverse_transform_coords(transformed_coords, s):
        # Apply the inverse transformations
        inverse_transformed_coords = []
        for transformed_coord in transformed_coords:

            original_coord = [-transformed_coord[1] + 13.5, transformed_coord[0] + 23.3]  # Reverse of the third transformation
            original_coord = [original_coord[0], original_coord[1]]  # Reverse of the second transformation, which does nothing
            original_coord = [int(20*original_coord[0]), int(20*original_coord[1])]  # Reverse of the first transformation

            inverse_transformed_coords.append(original_coord)
        return inverse_transformed_coords

    # Function to generate random coordinates
    def generate_random_coords(obstacle_positions, x_range, y_range, n_points):
        all_positions = set([(x, y) for x in range(x_range[0], x_range[1]+1) for y in range(y_range[0], y_range[1]+1)])

        free_positions = list(all_positions - obstacle_positions)
        
        if n_points > len(free_positions):
            raise ValueError("Cannot place {} points. Maximum possible points are {}".format(n_points, len(free_positions)))


        random_points = np.random.choice(len(free_positions), n_points, replace=False)
        return [free_positions[i] for i in random_points]


    def line(p1, p2):
        """Generate coordinates of pixels that belong to the line."""
        x1, y1 = map(int, p1)
        x2, y2 = map(int, p2)

        dx = x2 - x1
        dy = y2 - y1

        is_steep = abs(dy) > abs(dx)

        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        dx = x2 - x1
        dy = y2 - y1

        error = int(dx / 2.0)
        y = y1
        ystep = None
        if y1 < y2:
            ystep = 1
        else:
            ystep = -1

        points = []  # Points discovered
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)

            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        if swapped:
            points.reverse()

        return tuple(map(list, zip(*points)))  # Unzipping and converting to list

    # Function to check if a line between two points intersects with any obstacles
    
    def line_of_sight(p1, p2, obstacle_positions):
        x, y = line(p1, p2)  # draw line from p1 to p2
        for i in range(len(x)):
            if (x[i], y[i]) in obstacle_positions:
                return False
        return True

    # Generate random points
    n_points = 50
    random_coords = generate_random_coords(obstacle_positions, x_range, y_range, n_points)

    goal = inverse_transform_coords([goal_point], 0.05)[0]
   
    origin = inverse_transform_coords([origin_point], 0.05)[0]
    
    random_coords.append(origin)
    random_coords.append(goal)
    print(random_coords)


    # Convert to numpy array for easier handling
    random_coords = np.array(random_coords)

    # Plotting
    plt.figure(figsize=(10,10))
    plt.scatter(obstacle_coords[:, 0], obstacle_coords[:, 1], c='b') 
    plt.scatter(random_coords[:, 0], random_coords[:, 1], c='r')
    plt.scatter(166.6, 511.8, color='green', label='New Point')

    # Iterate over all pairs of points, and connect them if there are no obstacles in between
    for i in range(len(random_coords)):
        for j in range(i + 1, len(random_coords)):
            if line_of_sight(random_coords[i], random_coords[j], obstacle_positions):
                plt.plot([random_coords[i, 0], random_coords[j, 0]], [random_coords[i, 1], random_coords[j, 1]], 'g-')


    # In[833]:


    # Assuming s is a scalar value, define it here
    s = 0.05  # replace with your value

    def transform_coords(coords, s):
        # Apply the transformations
        transformed_coords = []
        for coord in coords:
            transformed_coord = [s*coord[0], s*coord[1]]
            # transformed_coord = [transformed_coord[0], transformed_coord[1]]
            transformed_coord = [transformed_coord[1] - 23.3, -transformed_coord[0] + 13.5]
            transformed_coords.append(transformed_coord)

        return np.array(transformed_coords)

    # Apply transformation
    transformed_random_coords = transform_coords(random_coords, s)
    transformed_obstacle_coords = transform_coords(obstacle_coords, s)

    # Plotting
    plt.figure(figsize=(10,10))
    plt.scatter(transformed_obstacle_coords[:, 0], transformed_obstacle_coords[:, 1], c='b')
    plt.scatter(transformed_random_coords[:, 0], transformed_random_coords[:, 1], c='r')

    # Iterate over all pairs of points, and connect them if there are no obstacles in between
    for i in range(len(transformed_random_coords)):
        for j in range(i + 1, len(transformed_random_coords)):
            if line_of_sight(random_coords[i], random_coords[j], obstacle_positions):
                plt.plot([transformed_random_coords[i, 0], transformed_random_coords[j, 0]], [transformed_random_coords[i, 1], transformed_random_coords[j, 1]], 'g-')


    # In[834]:




    def get_distance(point1, point2):
        return np.linalg.norm(point1 - point2)

    # Build adjacency matrix
    adjacency_matrix = np.zeros((len(transformed_random_coords), len(transformed_random_coords)))
    for i in range(len(transformed_random_coords)):
        for j in range(i + 1, len(transformed_random_coords)):
            if line_of_sight(random_coords[i], random_coords[j], obstacle_positions):
                distance = get_distance(transformed_random_coords[i], transformed_random_coords[j])
                adjacency_matrix[i][j] = distance
                adjacency_matrix[j][i] = distance

    # Initialize dijkstra variables
    unvisited_nodes = list(range(len(transformed_random_coords)))
    distances = np.full(len(transformed_random_coords), np.inf)
    previous_nodes = np.full(len(transformed_random_coords), -1)

    # Set distance from origin to itself as 0
    origin = len(transformed_random_coords) - 2  # Second last point
    goal = len(transformed_random_coords) - 1  # Last point
    distances[origin] = 0

    while unvisited_nodes:
        # Select node with smallest tentative distance
        current_node = unvisited_nodes[np.argmin(distances[unvisited_nodes])]

        # Break if smallest distance is infinity (disconnected graph) or if we reached the goal
        if distances[current_node] == np.inf or current_node == goal:
            break

        # Calculate tentative distance to neighbors
        for neighbor, distance in enumerate(adjacency_matrix[current_node]):
            if distance and neighbor in unvisited_nodes:
                alt_distance = distances[current_node] + distance
                if alt_distance < distances[neighbor]:
                    distances[neighbor] = alt_distance
                    previous_nodes[neighbor] = current_node

        # Remove current node from unvisited
        unvisited_nodes.remove(current_node)

    # Retrieve shortest path
    shortest_path = []
    while goal != -1:
        shortest_path.append(goal)
        goal = previous_nodes[goal]
    shortest_path = shortest_path[::-1]  # Reverse the list

    # Print the shortest path
    path =  [transformed_random_coords[i] for i in shortest_path]

  

    # Plot all connections
    for i in range(len(transformed_random_coords)):
        for j in range(i + 1, len(transformed_random_coords)):
            if line_of_sight(random_coords[i], random_coords[j], obstacle_positions):
                plt.plot([transformed_random_coords[i, 0], transformed_random_coords[j, 0]],
                        [transformed_random_coords[i, 1], transformed_random_coords[j, 1]], 'g-')

    # Plot shortest path
    for i in range(len(shortest_path) - 1):
        plt.plot([transformed_random_coords[shortest_path[i], 0], transformed_random_coords[shortest_path[i+1], 0]],
                [transformed_random_coords[shortest_path[i], 1], transformed_random_coords[shortest_path[i+1], 1]], 'm-', linewidth=2.0)
        
    plt.savefig('shortest_path.png')
    print(path)
        
    return path



# In[835]:



# 

# In[835]:





# In[835]:





import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
### YOUR IMPORTS HERE ###
from queue import PriorityQueue
from math import sqrt, pi
#########################

        
def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, -1.3, -np.pi/2)))

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))

    # Example of draw 
    # draw_sphere_marker((0, 0, 1), 0.1, (1, 0, 0, 1))
    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    goal_config = (2.6, -1.3, -np.pi/2)
    path = []
    start_time = time.time()
    
    
    
    ### YOUR CODE HERE ###
    # print(obstacles)
    # path, explored_free, explored_colliding = astar_search(start_config, goal_config, collision_fn)
    if path is None:
        print("No solution found.")
    else:
        print("Path found.")

    WEIGHT_XY = 1 
    WEIGHT_THETA = 100
    BLUE = (0, 0, 1, 1) 
    RED = (1, 0, 0, 1)   
    BLACK = (0, 0, 0, 1) 
    Z_OFFSET = 0.05   


    def heuristic(node, goal):
        x1, y1, theta1 = node
        x2, y2, theta2 = goal
        return WEIGHT_XY * sqrt((x1 - x2)**2 + (y1 - y2)**2) + \
            WEIGHT_THETA * min(abs(theta1 - theta2), 2 * pi - abs(theta1 - theta2))

    def action_cost(n, m):
        x1, y1, theta1 = n
        x2, y2, theta2 = m
        return WEIGHT_XY * sqrt((x1 - x2)**2 + (y1 - y2)**2) + \
            WEIGHT_THETA * min(abs(theta1 - theta2), 2 * pi - abs(theta1 - theta2))

    def get_neighbors(node):
        x, y, theta = node
        resolution = 0.14
        resolution_theta = np.pi/2
        delta_positions = [
            (resolution, 0, 0), (-resolution, 0, 0), (0, resolution, 0), (0, -resolution, 0),  # 4-connected neighbors
            (resolution, resolution, 0), (-resolution, resolution, 0), (resolution, -resolution, 0), (-resolution, -resolution, 0)  # 8-connected neighbors
        ]
        delta_orientations = [0, resolution_theta, -resolution_theta]
        # delta_orientations = [0]#, resolution_theta, -resolution_theta]
        
        neighbors = []
        for dx, dy, dtheta in delta_positions:
            for d_orient in delta_orientations:
                new_node = (x + dx, y + dy, (theta + d_orient) % (2 * pi))
                neighbors.append(new_node)
        return neighbors

    def astar_search(start, goal, collision_fn):
        open_set = PriorityQueue()
        open_set.put((0 + heuristic(start, goal), 0, start))  # (priority, cost, node)
        came_from = {}
        cost_so_far = {start: 0}
        
        collision_free_explored = []  # For storing collision-free configurations
        colliding_explored = []       # For storing colliding configurations

        while not open_set.empty():
            _, current_cost, current_node = open_set.get()

            if collision_fn(current_node):
                colliding_explored.append(current_node)  # Add to colliding configurations
                continue  
            
            collision_free_explored.append(current_node)  # Add to collision-free configurations
            
            if sqrt((current_node[0] - goal[0])**2 + (current_node[1] - goal[1])**2) < 0.1:
                print("Goal reached.")
                return reconstruct_path(came_from, current_node), collision_free_explored, colliding_explored

            for neighbor in get_neighbors(current_node):
                new_cost = current_cost + action_cost(current_node, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    open_set.put((priority, new_cost, neighbor))
                    came_from[neighbor] = current_node

        return None, collision_free_explored, colliding_explored 

    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def visualize_path(path, explored_free, explored_colliding):
        for i in range(len(path) - 1):
            start = (path[i][0], path[i][1], Z_OFFSET)
            end = (path[i+1][0], path[i+1][1], Z_OFFSET)
            draw_line(start, end, width=3, color=BLACK)

        for node in explored_free:
            position = (node[0], node[1], Z_OFFSET)
            draw_sphere_marker(position, radius=0.05, color=BLUE)

        for node in explored_colliding:
            position = (node[0], node[1], Z_OFFSET)
            draw_sphere_marker(position, radius=0.05, color=RED)
            

    def compute_total_path_cost(path):
        total_cost = 0
        for i in range(len(path) - 1):
            total_cost += action_cost(path[i], path[i + 1])
        return total_cost

    start_time = time.time()
    path, explored_free, explored_colliding = astar_search(start_config, goal_config, collision_fn)
    computation_time = time.time() - start_time
    print("Computation time: ", computation_time)

    # if path is None:
    #     print("No solution found.")
    # else:
    #     print("Path found.")
    #     total_path_cost = compute_total_path_cost(path)
    #     print(f"Computation time ({connectivity}): {computation_time:.4f} seconds")
    #     print(f"Total path cost ({connectivity}): {total_path_cost:.4f}")
    
    # if path is not None:
    visualize_path(path, explored_free, explored_colliding)
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)

    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
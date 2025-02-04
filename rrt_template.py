import numpy as np
from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name
import random
### YOUR IMPORTS HERE ###
import time 
#########################


def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2table.json')

    # define active DoFs
    joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint')
    joint_idx = [joint_from_name(robots['pr2'], jn) for jn in joint_names]

    # parse active DoF joint limits
    joint_limits = {joint_names[i] : (get_joint_info(robots['pr2'], joint_idx[i]).jointLowerLimit, get_joint_info(robots['pr2'], joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}

    collision_fn = get_collision_fn_PR2(robots['pr2'], joint_idx, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, 1.19, -1.548, 1.557, -1.32, -0.1928)))

    start_config = tuple(get_joint_positions(robots['pr2'], joint_idx))
    goal_config = (0.5, 0.33, -1.548, 1.557, -1.32, -0.1928)
    path = []
    ### YOUR CODE HERE ###

    STEP_SIZE = 0.05  # Step size for tree expansion
    GOAL_BIAS = 0.1  # 10% probability to choose the goal directly
    MAX_ITERATIONS = 50000  # Maximum iterations for RRT-Connect
    GUIDANCE_FACTOR = 0.4  # Weight for guiding the tree towards the goal

    joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint')

    def sample_random_config(joint_limits, goal_config=None):
        """ Sample a random configuration within the joint limits with goal bias """
        if random.random() < GOAL_BIAS and goal_config:
            print("Goal bias enabled")
            return goal_config
        
        # Adjust random sample to be guided slightly toward the goal
        random_config = tuple(random.uniform(joint_limits[jn][0], joint_limits[jn][1]) for jn in joint_names)
        if goal_config:
            guided_config = tuple(random_config[i] * (1 - GUIDANCE_FACTOR) + goal_config[i] * GUIDANCE_FACTOR for i in range(len(random_config)))
            return guided_config
        
        return random_config

    def extend(tree, target_config, step_size, collision_fn, parent_map):
        """ Try to extend the tree towards the target configuration and store parent-child relationship """
        nearest_config = min(tree, key=lambda n: np.linalg.norm(np.array(n) - np.array(target_config)))
        direction = np.array(target_config) - np.array(nearest_config)
        distance = np.linalg.norm(direction)
        
        if distance < step_size:
            new_config = target_config
        else:
            new_config = tuple(np.array(nearest_config) + direction / distance * step_size)
        
        if not collision_fn(new_config):
            tree.append(new_config)
            parent_map[new_config] = nearest_config  # Store the parent node
            return new_config
        return None



    def reconstruct_path(goal_config, parent_map):
        """ Reconstruct the path by tracing back from goal to start using the parent_map """
        path = [goal_config]
        while path[-1] in parent_map:
            path.append(parent_map[path[-1]])
        return path[::-1]  # Reverse the path to get start -> goal

    def rrt_connect_unidirectional(start_config, goal_config, joint_limits, collision_fn):
        """ Implement unidirectional RRT-Connect algorithm with path reconstruction """
        tree = [start_config]
        parent_map = {}  # To store the parent of each node

        for _ in range(MAX_ITERATIONS):
            # time.sleep(0.5?)
            # Sample random configuration with goal bias
            rand_config = sample_random_config(joint_limits, goal_config)
            
            # Extend the tree towards the random configuration
            new_config = extend(tree, rand_config, STEP_SIZE, collision_fn, parent_map)
            if new_config is None:
                continue  # Skip to the next iteration if extension fails

            # Check if we reached the goal
            if np.linalg.norm(np.array(new_config) - np.array(goal_config)) < STEP_SIZE:
                tree.append(goal_config)  # Add the goal to the tree
                parent_map[goal_config] = new_config  # Connect goal to the tree
                return reconstruct_path(goal_config, parent_map)  # Return the reconstructed path
        
        print("Failed to find a path in the given iterations.")
        return None

    def shortcut_smoothing(path, collision_fn, iterations=150):
        """ Shortcut smoothing algorithm to shorten the path """
        smoothed_path = path[:]
        for _ in range(iterations):
            i, j = sorted(random.sample(range(len(smoothed_path)), 2))
            if i + 1 < j and not any(collision_fn(smoothed_path[k]) for k in range(i, j + 1)):
                smoothed_path = smoothed_path[:i + 1] + smoothed_path[j:]
        return smoothed_path

    def visualize_path(robots, path, color, joint_idx, link_name='l_gripper_tool_frame'):
        """ Draw the path of the end-effector """
        link_idx = link_from_name(robots['pr2'], link_name)
        for config in path:
            set_joint_positions(robots['pr2'], joint_idx, config)
            end_effector_pos = get_link_pose(robots['pr2'], link_idx)[0]  # Extract position
            draw_sphere_marker(end_effector_pos, radius=0.02, color=color)  # Draw end-effector position


    path = rrt_connect_unidirectional(start_config, goal_config, joint_limits, collision_fn)
    if path is None:
        print("Failed to find a path.")
        disconnect()
        return


    # Display the original path in red
    print("Drawing original path (red)...")
    visualize_path(robots, path, (1, 0, 0, 1), joint_idx)

    # Apply shortcut smoothing to optimize the path
    smoothed_path = shortcut_smoothing(path, collision_fn)
    time.sleep(10)
    # Display the smoothed path in blue
    print("Drawing smoothed path (blue)...")
    visualize_path(robots, smoothed_path, (0, 0, 1.2, 1.2), joint_idx)

    ######################
    # Execute planned path
    execute_trajectory(robots['pr2'], joint_idx, path, sleep=0.5)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
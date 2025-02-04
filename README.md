# PR2 Motion Planning: A* and RRT-Connect Implementation

This repository contains implementations of **A* and RRT-Connect** for planning collision-free paths for the PR2 robot in a given environment. The code follows structured templates:

---

## 1. A* Algorithm for PR2 Base Motion Planning (3 DOF)

This task involves implementing a **two-dimensional (X, Y) and rotational (θ) A* search** for the PR2’s base. The objective is to find the shortest **collision-free** path from a given **start configuration** to a **goal configuration** using the A* algorithm.

### A* Algorithm Overview
A* is a search algorithm that finds the shortest path by balancing **exploration** and **exploitation** using:
- **Action cost function**: Measures the cost of moving from one node to another.
- **Heuristic function:** Estimates the cost from a node to the goal.

The A* implementation **must use a priority queue** (see `priorityqueue_example.py` for reference).

---

### A* Cost Function
For nodes **n** and **m**, the action cost is computed as:

\[
c(n, m) = \sqrt{(n_x - m_x)^2 + (n_y - m_y)^2 + \min(|n_{\theta} - m_{\theta}|, 2\pi - |n_{\theta} - m_{\theta}|)^2}
\]

where:
- \( n_x, n_y, n_{\theta} \) are the coordinates and orientation of node **n**.
- \( m_x, m_y, m_{\theta} \) are the coordinates and orientation of node **m**.

### A* Heuristic Function
For node **n**, the heuristic function estimates the cost to the goal:

\[
h(n) = \sqrt{(n_x - g_x)^2 + (n_y - g_y)^2 + \min(|n_{\theta} - g_{\theta}|, 2\pi - |n_{\theta} - g_{\theta}|)^2}
\]

where **g** represents the goal configuration.

---

### Collision Checking
- **Nodes must be collision-checked**, but edges between nodes do not need to be checked.
- A proper discretization of X, Y, and θ must be selected to balance computation time and accuracy.

---

### A* Variants Implemented
1. **4-Connected Neighbors**: Each node is connected to its 4 adjacent neighbors.
2. **8-Connected Neighbors**: Each node is connected to 8 adjacent neighbors.

**Performance Metrics to Record in the PDF Report:**
- **Computation time** for finding a path.
- **Path cost** computed using the action cost function.

---

### **Visualization Requirements**
For each variant:
- **Computed Path (Black)**: The planned path from start to goal.
- **Collision-Free Explored Nodes (Blue)**: Nodes explored by A* that are in free space.
- **Colliding Nodes (Red)**: Nodes explored by A* that are in collision.

![image](https://github.com/user-attachments/assets/067c549a-7e1f-46b4-a222-f08bc507ed3a)

---

## **RRT-Connect for PR2 Arm Motion Planning (6 DOF)**

## **Overview**
This implementation of **RRT-Connect** finds a **collision-free path** for the **left arm** of the PR2 robot in a **6 DOF configuration space** from a **start configuration** to a **goal configuration**. The algorithm extends the Rapidly-exploring Random Tree (RRT) by using **bidirectional search** to improve efficiency.

## **Implementation Details**
- **Collision checking** is required **only for robot-environment interactions** (self-collision checks are skipped).
- A **step size of 0.05 rad** is used for smoother expansion.
- A **goal bias of 10%** is applied, meaning the algorithm will attempt to extend towards the goal with 10% probability.

## **Path Visualization**
After computing the RRT-Connect path:
- **Original Path (Red):** The computed **left end-effector** position at each step of the path.
- **Smoothed Path (Blue):** The **shortcut-smoothed** version of the path.

## **Path Execution**
- The robot should execute the **computed path** in the viewer.
- The PR2 arm **must successfully reach the goal without colliding**.

![image](https://github.com/user-attachments/assets/91f2128e-888e-4727-bd45-334f652e698d)


## **Shortcut Smoothing**
- The **shortcut smoothing algorithm** is applied for **150 iterations**.
- The **original RRT path is drawn in red**.
- The **smoothed path is drawn in blue**.

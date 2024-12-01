# AAE_1001_Group8
<!-- Background of Path Planning to Aviation Engineering -->
## Theory of Path Planning Algorithm
The Astar algorithm is an algorithm for finding a shortest traversal path from a start point to a goal among paths with multiple nodes in the graph plane. It belongs to heuristic search algorithms as it uses heuristics to compute the nodes in the graph, thus reducing the number of nodes actually computed and improving the search efficiency.

**Steps in the Astar algorithm:**
1. **Initialization:** Add the start node to the open list and set the target node as the goal.
2. **Loop:** When the open list is not empty, perform the following steps: First take the node with the smallest f(n) value from the open list. If this node is the target node, the algorithm completes and returns the path. If not move this node to the closed list and check its neighbor nodes.  
For each neighbor node, calculate its f(n) value: If the neighbor node is not in the closure list, calculate its cost from the origin to the neighbor node f(n) and the heuristically estimated cost from the neighbor node to the target node h(n), and then calculate the f(n) value. If this f(n) value is lower than the current recorded value of the neighboring node, or the neighboring node is not in the open list, update its parent node and f(n) and g(n) values and add the neighboring node to the open list.
3. **End:** If the open list is empty and the target node is not found, it means there is no reachable path.
## Task 1
### 1. Map setting
The A-star path planning is used to find the shortest path to the destination. But before the implementation, the obstacles, sensitive areas and the boundary need to be set.
####1.principle
In this project, there are nodes that during the searching procedure, the computer can’t put into open/close list. By adding these nodes in a sequence, we can add obstacles. The same methods can be used in boundary setting and sensitive area setting. 
### 2. Method
To interpret the nodes into code, the coordinate system is used. The numerous nodes of a line can be translated into a equation using y=ax+b form. For instance, the obstacle in fig.1 can be transformed into y=-2x+10, then we use a loop to generate x(i) range from 0 to 9, which represents the Ox of the nodes. After that, the Oy can be calculated by the fomular, and the nodes can be added (Ox, Oy).  
  `for i in range(0, 10):  
        ox.append(i)  
        oy.append(-2 * i + 10)`  
For sensetive area, the loop needs to be apply to both Ox and Oy.(As the Fig.2 shows)

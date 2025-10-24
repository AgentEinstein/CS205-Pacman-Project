Our group successfully implemented four fundamental search algorithms in search.py: Depth-First Search (DFS), Breadth-First Search (BFS), Uniform Cost Search (UCS), and A*. The DFS implementation uses recursion to explore the deepest nodes first and backtracks upon reaching dead ends. The BFS algorithm employs a queue to traverse the shallowest nodes level by level, ensuring the shortest path is found in terms of steps. For UCS, we used a priority queue to always expand the node with the lowest cumulative cost, keeping track of the best known path cost to each state. The A* implementation combined the actual path cost and heuristic estimates to prioritize exploration efficiently, reconstructing the final path using a helper function.

After completing all implementations, we verified our work using the provided autograder by running the commands:
```py
python autograder.py -q q1  
python autograder.py -q q2  
python autograder.py -q q3  
python autograder.py -q q4
```
Each test passed successfully, confirming that our algorithms correctly handle graph search, optimal cost computation, and heuristic-based search. Overall, this project strengthened our understanding of state-space exploration, cost functions, and optimal pathfinding in artificial intelligence.

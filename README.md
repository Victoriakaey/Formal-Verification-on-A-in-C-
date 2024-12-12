# Formal-Verification-on-A-in-C-

This project aims to **verify the correctness of the A\* algorithm’s implementation in C for solving the shortest-path problem**. By utilizing formal verification tools such as C Bounded Model Checker (CBMC) and Seahorn, we ensure the algorithm's logical correctness, computational soundness, memory safety, and overall robustness.

C was chosen for this implementation due to its precise control over system-level resources and strong compatibility with advanced verification tools, making it well-suited for performance-critical and reliability-focused applications.

## Tools

### [C Bounded Model Checker (CBMC)](https://github.com/diffblue/cbmc) [^1]

[^1]: Kroening, D., Schrammel, P., & Tautschnig, M. (2023, February 5). CBMC: The C bounded model Checker. arXiv.org. https://arxiv.org/abs/2302.02384

C Bounded Model Checker (CBMC) is a formal verification tool designed to verify the correctness of C and C++ programs by exhaustively exploring all possible states within defined bounds, making it highly effective for identifying subtle bugs and validating safety properties.

CBMC’s verification process employs bounded model checking techniques and integrates SAT/SMT solvers. The workflow typically involves the following steps:

1. **Parsing and Compilation**: CBMC parses the source code and converts it into an intermediate representation suitable for analysis. It supports ANSI-C and C++ language features, ensuring compatibility with a broad range of programs.

2. **Unwinding Loops and Recursions**: The tool unwinds loops and recursive calls up to a specified depth, transforming the program into a finite-state model for exhaustive checking.
   Property Specification: Users can specify properties to be verified, such as array bounds, pointer safety, division by zero, and user-defined assertions. CBMC also inserts checks automatically for standard safety properties.

3. **SAT/SMT Solving**: CBMC translates the program and its properties into logical formulas and uses SAT or SMT solvers to determine their satisfiability. If a property is violated, CBMC generates a counterexample trace illustrating the error.

CBMC’s exhaustive analysis is crucial for detecting subtle bugs in the A\* algorithm. Its automation and detailed counterexamples simplify debugging and validation, ensuring comprehensive verification within bounded constraints.

These features ensure the correctness of the A\* algorithm by simplifying debugging and validation, especially within the bounded constraints necessary for formal verification.

### [Seahorn](https://seahorn.github.io/) [^2]

[^2]: A.Gurfinkel, T.Kahsai, A. Komuravelli, J.A.Navas. The SeaHorn Verification Framework. At CAV 2015. LNCS 9206, pp. 343-361. 2015 ​https://seahorn.github.io/papers/cav15.pdf

Seahorn is a formal verification tool designed for software programs, particularly in C and C++. By using mathematical methods, it ensures that a program behaves correctly under all possible execution scenarios, making it a valuable asset for safety-critical applications.

Seahorn combines static analysis and model checking to verify software correctness. It leverages mathematical methods to ensure programs behave as expected under all scenarios, making it valuable for safety-critical applications.

Seahorn works by combining static analysis and model checking in the following steps:

1. **LLVM IR Compilation**: Programs are compiled into LLVM Intermediate Representation (IR), providing a platform-independent, low-level abstraction for analysis.

2. **Abstract Interpretation**: Seahorn over-approximates program behavior to simplify verification by focusing on relevant properties while ignoring extraneous details.

3. **Horn Clause Encoding:** Programs and specifications are transformed into Constrained Horn Clauses (CHCs) for representing control flow and data dependencies.

4. **SMT Solving**: Using solvers like Z3 or MathSAT, Seahorn verifies these clauses, identifying property violations if unsatisfiable and generating counterexamples if needed.

For this project, Seahorn is invaluable due to its scalability, efficiency, and suitability for rigorous program validation. Its modular architecture integrates seamlessly into verification workflows, automating complex steps and reducing manual intervention. While limitations like false positives and complexity exist, Seahorn's precision and robustness make it ideal for verifying the correctness of the A\* algorithm in critical use cases.

Seahorn’s scalability and efficiency make it ideal for validating the A\* algorithm. Its modular architecture integrates well with verification workflows, automating complex processes and reducing manual intervention.

## Related Work

### Prior Work and Algorithm Developed Solving Shortest Path Problem

The shortest-path problem has been studied in depth, and a variety of algorithms have been developed to solve the problem based on different graph characteristics and application requirements.

#### Classical algorithms

Dijkstra's[^3] algorithm guarantees the shortest path in graphs with non-negative edge weights by iteratively selecting the vertex with the smallest tentative distance and updating the distances of its neighbors. Similarly, the Bellman-Ford algorithm extends this capability to graphs with negative weight edges by iteratively relaxing all edges and further extends the added advantage of detecting negative weight cycles. In dense graphs, Floyd-Warshall solves the all-pairs shortest-path problem by considering each vertex as an intermediate point. Though powerful, its computational complexity makes it less suitable for large graphs. The Johnson algorithm, which combines the Bellman-Ford and Dijkstra algorithms, is more efficient for sparse graphs and can handle negative weights when computing all-pairs shortest paths.

[^3]: Dijkstra, E. W. (1959). A note on two problems in connexion with graphs. Numerische Mathematik, 1(1), 269–271. https://www.cs.yale.edu/homes/lans/readings/routing/dijkstra-routing-1959.pdf

#### Specialized Algorithms Developed to Solve Special Cases

The Viterbi algorithm is used in decoding hidden Markov models, finding the most probable sequence of hidden states, and effectively solving a stochastic variant of the shortest-path problem. Bidirectional search improves search efficiency by simultaneously exploring the graph from source to target, greatly reducing the search space when meeting in the middle. Similarly, fringe search, an optimized version of A\*, delays node expansion until absolutely necessary, making it particularly effective in reducing computational overhead for some use cases.

#### Why Choose A\*

While each of these algorithms has distinct strengths, A* is a balanced solution that combines the optimality of Dijkstra's algorithm with heuristic-driven exploration. Unlike Dijkstra's exhaustive approach, A* estimates the cost to the target node using a heuristic function and hence is capable of focusing on more promising paths to reduce computational effort. Particularly well adapted for big and complex graphs on single-source shortest-path problems, compared with Floyd-Warshall or even Johnson's algorithm, which works in most dense or all-pair pathfinding scenarios. This algorithm also allows a high degree of practical usage for areas such as robotics, game development, or other real-time navigation system functions, because it supports the ability to apply domain-specific knowledge to improve heuristics. These advantages, in addition to its balance of accuracy and computational efficiency, make A\* a compelling choice for further study and verification in this project.

## A\* Implementation Breakdown

The A\* algorithm is a heuristic-based search algorithm with a high efficiency in path exploration. It finds the shortest path from a start node to a goal node in a weighted graph.

It combines the advantages of Dijkstra's algorithm, being able to explore every path, and Greedy Best-First Search, using a heuristic-based search method, and uses a cost optimization `f(n)=g(n)+h(n)`, where `g(n)` is the actual cost from the start note to the current node, and `h(n)` is the estimated cost from the current node to the target node.

This algorithm guarantees the shortest path if the heuristic is admissible (i.e., never overestimates the actual cost).

### Heuristic Function

The heuristic function estimates the cost to reach the goal from a given node. The implementation uses the Manhattan distance:

```C
static double heuristic(int row, int col, int goalRow, int goalCol) {
    return fabs((double)row - (double)goalRow) + fabs((double)col - (double)goalCol);
}
```

This choice is ideal for grid-based problems where movement is restricted to horizontal and vertical directions.

### Data Structures

- Grid: A 2D array representing the environment with obstacles and open cells.
- Cost Matrix: Tracks the values for each cell.
- Parent Matrix: Stores the parent of each node to reconstruct the path.
- Closed List: A boolean matrix indicating explored nodes.
- Queue: A static array used to store nodes to be processed.

### Initialization

The algorithm initializes all nodes with infinite cost and null parent pointers:

```C
for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
        cost[i][j] = DBL_MAX;
        parent[i][j][0] = -1;
        parent[i][j][1] = -1;
    }
}

cost[startRow][startCol] = 0.0;
queue[rear][0] = startRow;
queue[rear][1] = startCol;
rear++;
```

### Main Loop

The algorithm processes nodes from the queue, exploring their neighbors:

```C
while (front < rear) {
    int currentRow = queue[front][0];
    int currentCol = queue[front][1];
    front++;

    if (currentRow == goalRow && currentCol == goalCol) {
        printf("Path Found!\n");
        return;
    }

    closed[currentRow][currentCol] = true;
    for (int i = 0; i < 4; i++) {
        int newRow = currentRow + dRow[i];
        int newCol = currentCol + dCol[i];
        if (isValid(newRow, newCol, grid, closed)) {
            double newCost = cost[currentRow][currentCol] + 1.0;
            double h = heuristic(newRow, newCol, goalRow, goalCol);
            double f = newCost + h;
            if (f < cost[newRow][newCol]) {
                cost[newRow][newCol] = f;
                parent[newRow][newCol][0] = currentRow;
                parent[newRow][newCol][1] = currentCol;
                queue[rear][0] = newRow;
                queue[rear][1] = newCol;
                rear++;
            }
        }
    }
}
```

### Path Reconstruction

Once the goal is reached, the path is reconstructed using the parent matrix:

```C
void reconstructPath(int goalRow, int goalCol) {
    int row = goalRow;
    int col = goalCol;
    while (parent[row][col][0] != -1 && parent[row][col][1] != -1) {
        printf("(%d, %d) <- ", row, col);
        int tempRow = parent[row][col][0];
        int tempCol = parent[row][col][1];
        row = tempRow;
        col = tempCol;
    }
    printf("Start\n");
}
```

### Validation

The function isValid ensures that nodes are within bounds and not already visited:

```C
static bool isValid(int row, int col, int grid[ROW][COL], bool closed[ROW][COL]) {
    if (row < 0 || row >= ROW || col < 0 || col >= COL) {
        return false;
    }
    return (grid[row][col] == 0 && !closed[row][col]);
}
```

### Analysis

- Time Complexity: `O(V + E)`, where `V` is the number of vertices, equivalent to the number of grid cells in the environment; `E` is the number of edges, representing the possible connections between adjacent cells.

- Space Complexity: `O(ROW × COL)`, accounting for the memory used by the cost matrix, parent matrix, and closed list, which all have dimensions matching the grid size.

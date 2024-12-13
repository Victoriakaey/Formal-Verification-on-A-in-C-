#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include <omp.h>
#include <time.h>

#define ROW 5
#define COL 5
#define MAX_QUEUE_SIZE (ROW * COL)

// Directions for neighbor nodes
static const int dRow[] = {-1, 0, 1, 0};
static const int dCol[] = {0, -1, 0, 1};

// Function to calculate Manhattan distance heuristic
static double heuristic(int row, int col, int goalRow, int goalCol) {
  return fabs((double)row - (double)goalRow) + fabs((double)col - (double)goalCol);
}

// Check if a cell is valid for exploration
static bool isValid(int row, int col, int grid[ROW][COL], bool closed[ROW][COL]) {
  return (row >= 0 && row < ROW && col >= 0 && col < COL && grid[row][col] == 0 && !closed[row][col]);
}

// BFS to find the shortest path length
static int bfs(int grid[ROW][COL], int startRow, int startCol, int goalRow, int goalCol) {
  bool visited[ROW][COL] = {false};
  int queue[MAX_QUEUE_SIZE][3]; // Stores row, col, and path length
  int front = 0, rear = 0;

  queue[rear][0] = startRow;
  queue[rear][1] = startCol;
  queue[rear][2] = 0;
  rear++;
  visited[startRow][startCol] = true;

  while (front < rear) {
    int currentRow = queue[front][0];
    int currentCol = queue[front][1];
    int currentLength = queue[front][2];
    front++;

    // If we reach the goal
    if (currentRow == goalRow && currentCol == goalCol) {
      return currentLength;
    }

    // Explore neighbors
    for (int i = 0; i < 4; i++) {
      int newRow = currentRow + dRow[i];
      int newCol = currentCol + dCol[i];

      if (newRow >= 0 && newRow < ROW && newCol >= 0 && newCol < COL && !visited[newRow][newCol] && grid[newRow][newCol] == 0) {
        visited[newRow][newCol] = true;
        queue[rear][0] = newRow;
        queue[rear][1] = newCol;
        queue[rear][2] = currentLength + 1;
        rear++;
      }
    }
  }

  // If no path is found
  return -1;
}

// verify path from goal to start to ensure valid path
static int verify_and_print_path(int parent[ROW][COL][2], int startRow, int startCol, int goalRow, int goalCol) {
  int path[MAX_QUEUE_SIZE][2];
  int length = 0;
  int cr = goalRow;
  int cc = goalCol;
  while (!(cr == startRow && cc == startCol)) {
    path[length][0] = cr;
    path[length][1] = cc;
    length++;
    int pr = parent[cr][cc][0];
    int pc = parent[cr][cc][1];
    cr = pr;
    cc = pc;
  }
  path[length][0] = startRow;
  path[length][1] = startCol;
  length++;
  return length;
}



// A* search algorithm using static arrays
static int aStar(int grid[ROW][COL], int startRow, int startCol, int goalRow, int goalCol) {
  int queue[MAX_QUEUE_SIZE][2];
  double cost[ROW][COL];
  int parent[ROW][COL][2];
  bool closed[ROW][COL] = {false};

  int front = 0, rear = 0;
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

  while (front < rear) {
    int currentRow = queue[front][0];
    int currentCol = queue[front][1];
    front++;

    // Check if goal is reached
    if (currentRow == goalRow && currentCol == goalCol) {
      return verify_and_print_path(parent, startRow, startCol, goalRow, goalCol) - 1; // Length is number of steps
    }

    closed[currentRow][currentCol] = true;

    // Explore neighbors
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

  return -1;
}

int main(void) {
  long long totalTested = 0;
  clock_t start, end;

  // Start the timer
  start = clock();

  // Iterate over all possible 5x5 matrices
  for (long long mask = 0; mask < (1LL << (ROW * COL)); mask++) {
    int grid[ROW][COL];

    // Create grid
    for (int i = 0; i < ROW; i++) {
      for (int j = 0; j < COL; j++) {
        grid[i][j] = (mask & (1LL << (i * COL + j))) ? 1 : 0;
      }
    }

    // Ensure start point is walkable
    grid[0][0] = 0;

    // Loop over all possible end points
    for (int endRow = 0; endRow < ROW; endRow++) {
      for (int endCol = 0; endCol < COL; endCol++) {

        // Ensure the end point is walkable
        grid[endRow][endCol] = 0;

        int aStarLength = aStar(grid, 0, 0, endRow, endCol);
        int bfsLength = bfs(grid, 0, 0, endRow, endCol);

        // Validate the lengths
        assert(aStarLength == bfsLength && "A* and BFS path lengths should match");

        // Update and print the current prograss
        totalTested++;
        if (totalTested % 1000000 == 0) {
          printf("Tested %lld matrices so far...\n", totalTested);
        }
      }
    }
  }

  // Stop the timer
  end = clock();

  // Calculate total run time
  double totalTime = ((double)(end - start)) / CLOCKS_PER_SEC;

  printf("Total Matrices Tested without any violation: %lld\n", totalTested);
  printf("Total Run Time: %.2f seconds\n", totalTime);

  return 0;
}

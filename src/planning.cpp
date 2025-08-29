#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <unordered_map>

using namespace std;

// Constructor: store the grid and its size
Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
    rows = grid.size();
    cols = grid[0].size();
}

// Check if a cell is inside the grid and not blocked
bool Planner::isvalid(int x, int y) const {
    return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

// Heuristic: estimate distance from (x1,y1) to (x2,y2)
// Here we use Euclidean distance
double Planner::heuristic(int x1, int y1, int x2, int y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Path planning using A* algorithm
vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
    // Structure to represent a node in the grid
    struct Node {
        int x, y;         // cell coordinates
        double g;         // cost from start to this cell
        double f;         // total cost = g + heuristic to goal
        Node* parent;     // pointer to previous node for path reconstruction
    };

    // Comparison for priority queue: node with lowest f has highest priority
    struct CompareF {
        bool operator()(Node* a, Node* b) {
            return a->f > b->f;
        }
    };

    vector<pair<int, int>> path; // final path to return

    // Open set: nodes to explore, ordered by lowest f
    priority_queue<Node*, vector<Node*>, CompareF> openSet;

    // Closed set: cells already visited
    unordered_map<int, unordered_map<int, bool>> closedSet;

    // Start node: g=0, f=g+heuristic, no parent
    Node* startNode = new Node{start.first, start.second, 0.0,
                               heuristic(start.first, start.second, goal.first, goal.second),
                               nullptr};
    openSet.push(startNode);

    // Directions to move: 4 straight + 4 diagonal
    int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    double cost[8] = {1, 1, 1, 1, 1.4142, 1.4142, 1.4142, 1.4142}; // diagonal ≈ sqrt(2)

    // Main A* loop
    while (!openSet.empty()) {
        Node* current = openSet.top(); // pick node with lowest f
        openSet.pop();

        // Check if we reached the goal
        if (current->x == goal.first && current->y == goal.second) {
            // Reconstruct path from goal to start
            while (current) {
                path.push_back({current->x, current->y});
                current = current->parent;
            }
            reverse(path.begin(), path.end()); // reverse to go from start -> goal
            break;
        }

        // Skip if already visited
        if (closedSet[current->x][current->y]) continue;
        closedSet[current->x][current->y] = true;

        // Explore all neighbors
        for (int i = 0; i < 8; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            // Skip invalid or already visited cells
            if (!isvalid(nx, ny) || closedSet[nx][ny]) continue;

            // Compute new g and f values for neighbor
            double gNew = current->g + cost[i];
            double fNew = gNew + heuristic(nx, ny, goal.first, goal.second);

            // Add neighbor to open set with current as parent
            openSet.push(new Node{nx, ny, gNew, fNew, current});
        }
    }

    return path; // return the computed path
}

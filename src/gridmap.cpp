#include "planning.h"
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>

using namespace std;

struct Node {
    int x, y;
    double g, h;
    Node* parent;
    double f() const { return g + h; }
};

struct Compare {
    bool operator()(Node* a, Node* b) {
        return a->f() > b->f();
    }
};

vector<pair<int,int>> Planner::pathplanning(pair<int,int> start, pair<int,int> goal) {
    vector<pair<int,int>> path;

    // Open list (priority queue for A*)
    priority_queue<Node*, vector<Node*>, Compare> open;
    map<pair<int,int>, double> gscore;
    map<pair<int,int>, Node*> visited;

    Node* startNode = new Node{start.first, start.second, 0.0,
                               heuristic(start.first, start.second, goal.first, goal.second),
                               nullptr};

    open.push(startNode);
    gscore[start] = 0.0;

    // Directions (4-connected grid)
    vector<pair<int,int>> dirs = {{1,0},{-1,0},{0,1},{0,-1}};

    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        pair<int,int> cpos = {current->x, current->y};
        if (cpos == goal) {
            // reconstruct path
            while (current) {
                path.push_back({current->x, current->y});
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            break;
        }

        for (auto d : dirs) {
            int nx = current->x + d.first;
            int ny = current->y + d.second;

            if (!isvalid(nx, ny)) continue;

            pair<int,int> npos = {nx, ny};
            double tentative_g = gscore[cpos] + 1.0;

            if (!gscore.count(npos) || tentative_g < gscore[npos]) {
                gscore[npos] = tentative_g;
                Node* neighbor = new Node{nx, ny, tentative_g,
                                          heuristic(nx, ny, goal.first, goal.second),
                                          current};
                open.push(neighbor);
                visited[npos] = neighbor;
            }
        }
    }

    return path;
}

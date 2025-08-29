#ifndef PLANNING_H
#define PLANNING_H

#include <vector>
#include <utility>

class Planner {
private:
    std::vector<std::vector<bool>> grid;
    int rows, cols;

public:
    Planner(const std::vector<std::vector<bool>> &grid);

    bool isvalid(int x, int y) const;

    double heuristic(int x1, int y1, int x2, int y2) const;

    std::vector<std::pair<int, int>> pathplanning(std::pair<int, int> start,
                                                  std::pair<int, int> goal);
};

#endif // PLANNING_H

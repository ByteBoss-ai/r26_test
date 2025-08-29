#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <utility>
#include <iostream>

struct GPS {
    double lat;
    double lon;
    double height;
};

class Gridmapper {
private:
    GPS origin;
    double cellsize;
    int rows, cols;
    std::vector<std::vector<bool>> grid;

public:
    Gridmapper(GPS origin, double cellsize, int rows, int cols);

    std::pair<int, int> gpstogrid(const GPS &point) const;
    const std::vector<std::vector<bool>> &getGrid() const;
    double deg2rad(double deg);
    bool isvalid(int row, int col) const;
    void makemap();
    void printgrid() const;
};

#endif // GRIDMAP_H

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <vector>
#include <utility>

struct MotionCommand {
    double time_sec;
    double angle_deg;
};

class Odometry {
private:
    double radius;
    double rpm;
    double linear_vel;

public:
    Odometry(double wheel_radius, double rpm);

    double distance(int x1, int y1, int x2, int y2);

    double angle(int x1, int y1, int x2, int y2);

    MotionCommand computeCommands(std::vector<std::pair<int, int>> &path);
};

#endif // ODOMETRY_H

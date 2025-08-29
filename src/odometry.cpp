#include "odometry.h"
#include <cmath>
#include <vector>

using namespace std;

// Constructor: calculates linear velocity based on wheel radius and RPM
Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
    // Revolutions per second
    double rps = rpm / 60.0;

    // Linear velocity = circumference * revolutions per second
    linear_vel = 2 * M_PI * radius * rps;
}

// Calculate Euclidean distance between two points
double Odometry::distance(int x1, int y1, int x2, int y2) {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Calculate angle (heading) from point1 to point2 in degrees
double Odometry::angle(int x1, int y1, int x2, int y2) {
    return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

// Convert a path into motion commands (total time and total angle turned)
MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
    MotionCommand res = {0.0, 0.0}; // total time and total angle

    if (path.size() < 2) return res; // if path is too short, nothing to do

    for (size_t i = 1; i < path.size(); ++i) {
        int x1 = path[i-1].first;
        int y1 = path[i-1].second;
        int x2 = path[i].first;
        int y2 = path[i].second;

        // Compute distance between consecutive points
        double dist = distance(x1, y1, x2, y2);

        // Compute angle to turn to face next point
        double ang = angle(x1, y1, x2, y2);

        // Add to total motion commands
        res.time += dist / linear_vel; // time = distance / velocity
        res.angle += ang;              // accumulate total rotation (simplified)
    }

    return res;
}

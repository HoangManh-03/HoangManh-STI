#include<ros/ros.h>
#include<vector>

namespace GoalControl{

    struct Point{
        double x, y;
        Point(double _x = 0, double _y = 0): x(_x), y(_y) {}
    };    
    Point cubicBezier(double t, const Point& P0, const Point& P1, const Point& P2, const Point& P3);
    std::vector<Point> generateBezierCurves(const Point& P0, const Point& P1, const Point& P2, const Point& P3, int numPoints);

}
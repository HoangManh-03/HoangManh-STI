#include"testMotionPlanning/bezier_curves.h"
#include<ros/ros.h>

namespace GoalControl{

    Point cubicBezier(double t, const Point& P0, const Point& P1, const Point& P2, const Point& P3){
        double x = pow(1 - t, 3) * P0.x +
                3 * pow(1 - t, 2) * t * P1.x +
                3 * (1 - t) * pow(t, 2) * P2.x +
                pow(t, 3) * P3.x;

        double y = pow(1 - t, 3) * P0.y +
                3 * pow(1 - t, 2) * t * P1.y +
                3 * (1 - t) * pow(t, 2) * P2.y +
                pow(t, 3) * P3.y;
        
        return Point(x, y);
    }

    std::vector<Point> generateBezierCurves(const Point& P0, const Point& P1, const Point& P2, const Point& P3, int numPoints){
        std::vector<Point> bezierPoints;
        for (int i = 0; i <= numPoints; ++i) {
            double t = static_cast<double>(i) / numPoints;
            bezierPoints.push_back(cubicBezier(t, P0, P1, P2, P3));
        }
        return bezierPoints;
    }

}

#include "Point.h"
#include <Vector.h>

static const Point directions[] = {
    Point(0, -1),
    Point(1, 0),
    Point(0, 1),
    Point(-1, 0)
};

Point getPoint(char point);

Vector<Point> getGateZones();

Vector<Point> getBestPath(Point curr, Point end, Vector<Point> seen, Vector<Point> path);

Vector<Point> getPath();
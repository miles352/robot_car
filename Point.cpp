#include "Point.h"

byte x, y;
bool Point::operator==(const Point& other) const
{
    return (x == other.x) && (y == other.y);
}
bool Point::operator<(const Point& other) const
{
    return (x < other.x) || ((x == other.x) && (y < other.y));
}
Point::Point() : x(0), y(0) {};
Point::Point(byte x, byte y) : x(x), y(y) {};
Point::Point(const Point& other) : x(other.x), y(other.y) {};

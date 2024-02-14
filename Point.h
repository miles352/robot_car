#include "Arduino.h"
#pragma once

struct Point 
{
    byte x, y;
    bool operator==(const Point& other) const;
    bool operator<(const Point& other) const;
    Point();
    Point(byte x, byte y);
    Point(const Point& other);
    // ~Point();
};

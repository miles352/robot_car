#include "OptimalPath.h"

static const int TRACK_SIZE = 9;
static const int GATE_ZONE_COUNT = 3;
static const int MAX_PATH_LENGTH = 50;

// this is equal to factorial of GATE_ZONE_COUNT
static const int GATE_ZONE_COMBINATIONS = 6;
Point permutations[GATE_ZONE_COMBINATIONS][GATE_ZONE_COUNT];
int permutationIndex = 0;

// track is filled with s for start point, t for target point, g for goal point, and x for a wall

const static char PROGMEM track[TRACK_SIZE][TRACK_SIZE] = {
        {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'}
};

char getTrack(byte i, byte j)
{
  return (char)pgm_read_byte(&track[i][j]);
}

Point getPoint(char point)
{
    for (int i = 0; i < TRACK_SIZE; i++)
    {
        for (int j = 0; j < TRACK_SIZE; j++)
        {
            if (getTrack(i, j) == point)
            {
                return Point(j, i);
            }
        }
    }
}

Vector<Point> getGateZones()
{
    Point gateZonesStorage[GATE_ZONE_COUNT];
    Vector<Point> gateZones(gateZonesStorage);
    for (int i = 0; i < TRACK_SIZE; i++)
    {
        for (int j = 0; j < TRACK_SIZE; j++)
        {
            if (getTrack(i, j) == 'g')
            {
                gateZones.push_back(Point(j, i));
            }
        }
    }
    return gateZones;
}

bool find(Vector<Point> vec, Point target)
{
  for (Point p : vec)
  {
    if (p == target)
    {
      return true;
    }
  }
  return false;
}

void swap(Point& a, Point& b) {
    Point temp = a;
    a = b;
    b = temp;
}

void getPermutations(Vector<Point>& points, int index)
{
  if (index == GATE_ZONE_COUNT - 1)
  {
    for (int i = 0; i < points.size(); i++)
    {
      permutations[permutationIndex][i] = points[i];
    }
    permutationIndex++;
  }
  for (int i = index; i < GATE_ZONE_COUNT; i++)
  {
    swap(points[index], points[i]);
    getPermutations(points, index + 1);
    swap(points[index], points[i]);
  }
}

Vector<Point> getBestPath(Point curr, Point end, Vector<Point> seen, Vector<Point> path)
{
    if (getTrack(curr.y, curr.x) == 'x') 
    {
        return Vector<Point>{};
    }
    // if current is seen
    if (find(seen, curr)) 
    {
        return Vector<Point>{};
    }
    if (curr == end) 
    {
        path.push_back(curr);
        return path;
    }
    
    seen.push_back(curr);
    path.push_back(curr);
    
    for (int i = 0; i < 4; i++)
    {  
      if (getTrack(curr.y + directions[i].y, curr.x + directions[i].x) != 'x')
      {
        Vector<Point> next = getBestPath(Point(curr.x + (directions[i].x * 2), curr.y + (directions[i].y * 2)), end, seen, path);
        // if we have completed a path
        if (!next.empty() && (next.back() == end))
        {
          return next;
        }
      }
    }

    path.pop_back();
    return Vector<Point>{};
}

Vector<Point> getPath()
{
    Vector<Point> gateZones = getGateZones();
    Point startPoint = getPoint('s');
    Point targetPoint = getPoint('t');

    getPermutations(gateZones, 0);

    Point bestPathStorage[MAX_PATH_LENGTH];
    Vector<Point> bestPath(bestPathStorage);

    int shortestPathLength = 999;
    
    for (int i = 0; i < GATE_ZONE_COMBINATIONS; i++)
    {     
      Point pointsStorage[1 + GATE_ZONE_COUNT + 1];
      Vector<Point> points(pointsStorage);
      points.push_back(startPoint);
      for (Point p : permutations[i])
      {
        points.push_back(p);   
      }
      points.push_back(targetPoint);
      
      Point pathStorage[MAX_PATH_LENGTH];
      Vector<Point> path(pathStorage);

      for (int j = 0; j < points.size() - 1; j++)
      {
        Point pathsStorage[25]; 
        Vector<Point> paths(pathsStorage);
        Point seenStorage[100];
        Vector<Point> seen(seenStorage);
        Vector<Point> bestPathPart = getBestPath(points[j], points[j+1], seen, paths);
        for (Point p : bestPathPart)
        {
          path.push_back(p);
        }
      }
      
      if (!path.empty() && (path.size() < shortestPathLength))
      {
        bestPath.clear();
        for (Point p : path)
        {
          bestPath.push_back(p);
        }
        shortestPathLength = path.size();
      }
    }
    
    return bestPath;
}


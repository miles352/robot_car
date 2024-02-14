#include "OptimalPath.h"

static const int TRACK_SIZE = 9;
static const int GATE_ZONE_COUNT = 3;
static const int MAX_PATH_LENGTH = 30;
// uint8_t * heapptr, * stackptr;

// this is equal to factorial of GATE_ZONE_COUNT
static const int GATE_ZONE_COMBINATIONS = 6;
Point permutations[GATE_ZONE_COMBINATIONS][GATE_ZONE_COUNT];
int permutationIndex = 0;


const static char PROGMEM track[TRACK_SIZE][TRACK_SIZE] = {
        {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'},
        {'x', ' ', ' ', 't', 'x', 'g', 'x', ' ', 'x'},
        {'x', ' ', ' ', 'x', ' ', ' ', ' ', ' ', 'x'},
        {'x', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'x'},
        {'x', 'x', ' ', ' ', ' ', 'x', ' ', ' ', 'x'},
        {'x', 'g', ' ', ' ', 'x', ' ', ' ', 'g', 'x'},
        {'x', ' ', ' ', 'x', ' ', ' ', ' ', 'x', 'x'},
        {'x', ' ', ' ', 's', ' ', ' ', ' ', ' ', 'x'},
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
  // Serial.print("curr ");
  // Serial.print(curr.y);
  // Serial.print(" ");
  // Serial.println(curr.x);
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
    // Vector<Point> paths[4];
    // Point paths2[4][MAX_PATH_LENGTH];
    // Serial.println(mem_left());

    
    for (int i = 0; i < 4; i++)
    {  
      if (getTrack(curr.y + directions[i].y, curr.x + directions[i].x) != 'x')
      {
        Vector<Point> next = getBestPath(Point(curr.x + (directions[i].x * 2), curr.y + (directions[i].y * 2)), end, seen, path);
        // if we have completed a path
        if (!next.empty() && (next.back() == end))
        {

          // paths.push_back(next);
          // for (int j = 0; j < next.size(); j++)
          // {
          //   paths[i][j] = next[j];
          // }
          // paths[i] = next;
        //   for (Point z : paths[i])
        // {
        //   Serial.print("dd: ");
        //   Serial.print(z.y);
        //   Serial.print(" ");
        //   Serial.println(z.x);
        // }
        // Serial.println("target");
          return next;
        }
      }
    }
    // int shortestPathLength = 999;
    // Vector<Point> shortestPath;
    // for (int i = 0; i < 4; i++)
    // {
      // if (!paths[i].empty())
      // {
        // for (int j = 0; j < paths[i].size(); j++)
        // {
        //   Serial.print(paths[i][j].y);
        //   Serial.print(" ");
        //   Serial.println(paths[i][j].x);
        // }
        // Serial.println();
      // }
      // if (!paths[i].empty() && (paths[i].size() < shortestPathLength))
      // {
        
      //   shortestPath = paths[i];
      //   shortestPathLength = paths[i].size();
      // }
    //   Point tempStorage[MAX_PATH_LENGTH];
    //   Vector<Point> temp(tempStorage);
    //   for (int j = 0; j < MAX_PATH_LENGTH; j++)
    //   {
    //     if ((paths[i][j].y == 0) && (paths[i][j].x == 0)) break;
    //     temp.push_back(paths[i][j]);
    //   }  
    //   if (!temp.empty() && (temp.size() < shortestPathLength))
    //   {
    //     shortestPath = temp;
    //     shortestPathLength = temp.size();
    //   }
    // }
    // if (!shortestPath.empty()) return shortestPath;

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

      for (int i = 0; i < points.size() - 1; i++)
      {
        Point pathsStorage[25]; 
        Vector<Point> paths(pathsStorage);
        Point seenStorage[100];
        Vector<Point> seen(seenStorage);
        Vector<Point> bestPathPart = getBestPath(points[i], points[i+1], seen, paths);
        // Serial.println("bestpathparts: ");
        for (Point p : bestPathPart)
        {
          // Serial.print(p.y);
          // Serial.print(" ");
          // Serial.println(p.x);
          path.push_back(p);
        }
      }
      // Serial.println("path complete");
      if (!path.empty() && (path.size() < shortestPathLength))
      {
        bestPath = path;
        shortestPathLength = path.size();
      }
    }

    return bestPath;
}


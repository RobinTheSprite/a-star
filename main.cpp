#include <iostream>
#include <memory>
#include <vector>
#include <deque>
#include <queue>
#include <cmath>
using std::shared_ptr;
using std::make_shared;
using std::vector;
using std::deque;
using std::priority_queue;
using std::sqrt;
using std::abs;
using std::cout;
using std::endl;
using std::min;

using Matrix = vector<vector<int>>;
const int wallWeight = 300000;

// printMatrix
// prints the given 2D matrix to the console
void printMatrix(const Matrix &matrix)
{
    for (const auto& row : matrix)
    {
        for (const auto& col : row)
        {
            cout << "[";
            if (col)
            {
                cout << "x";
            }
            else
            {
                cout << " ";
            }
            cout << "]";
        }
        cout << endl;
    }
}

// Point
// Stores information needed for the algorithm to operate
struct Point
{
    int x;
    int y;
    double movementCost;
    double heuristic;
    double totalCost;
    std::shared_ptr<Point> parent;
};

bool inBounds(const Matrix& matrix, const Point& p)
{
    return (p.x > -1 && p.y > -1) && p.y < matrix.size() && p.x < matrix[0].size();
}

void getSurroundingPoints(const Matrix& matrix, const Point& currentPoint, vector<Point> & surroundingPoints)
{
    auto stepSize = 1;

    auto eightPoints = [currentPoint](int x, int y)
    {
        return !(x == currentPoint.x && y == currentPoint.y);
    };

    auto fourPoints = [currentPoint](int x, int y)
    {
        return (x == currentPoint.x) != (y == currentPoint.y);
    };

    auto notAWall = [matrix](int x, int y)
    {
        return matrix[y][x] < wallWeight;
    };

    auto writeIndex = 0;
    for (auto x = currentPoint.x - stepSize; x <= currentPoint.x + stepSize; x += stepSize)
    {
        for (auto y = currentPoint.y - stepSize; y <= currentPoint.y + stepSize; y += stepSize)
        {
            if(eightPoints(x, y) && inBounds(matrix, {x, y}) && notAWall(x, y))
            {
                surroundingPoints[writeIndex] = {x, y, 0, 0, 0, std::make_shared<Point>(currentPoint)};
                ++writeIndex;
            }
        }
    }

    surroundingPoints.resize(writeIndex);
}


double euclideanDistance(const Point &a, const Point &b)
{
    auto dX = b.x - a.x;
    auto dY = b.y - a.y;

    return sqrt(dX * dX + dY * dY);
}


double chebyshevDistance(const Point &a, const Point &b)
{
    auto dX = abs(b.x - a.x);
    auto dY = abs(b.y - a.y);

    return (dX + dY) - min(dX, dY);
}


double octileDistance(const Point &a, const Point &b)
{
    auto dX = abs(b.x - a.x);
    auto dY = abs(b.y - a.y);

    return (dX + dY) - 0.6 * min(dX, dY);
}


double manhattanDistance(const Point &a, const Point &b)
{
    auto dX = abs(b.x - a.x);
    auto dY = abs(b.y - a.y);

    return dX + dY;
}


double crossProduct(const Point& start, const Point& target, const Point& current)
{
    auto dx1 = current.x - target.x;
    auto dy1 = current.y - target.y;

    auto dx2 = start.x - target.x;
    auto dy2 = start.y - target.y;

    return abs(dx1*dy2 - dx2*dy1);
}


Matrix getShortestPath(const Matrix &inputMatrix, deque<Point> controlPoints)
{
    auto outputMatrix = Matrix(inputMatrix.size(), vector<int>(inputMatrix[0].size(), 0));
    auto visited = 1;
    shared_ptr<Point> finishingPoint;
    while (controlPoints.size() >= 2)
    {
        Point startingPoint = controlPoints[0];
        Point target = controlPoints[1];

        auto minimumCost = [](auto a, auto b) { return a.totalCost > b.totalCost; };
//        auto minimumCost = [](auto a, auto b)
//        {
//            if (a.totalCost == b.totalCost)
//            {
//                return a.heuristic > b.heuristic;
//            }
//            else
//            {
//                return a.totalCost > b.totalCost;
//            }
//        };
        using PointQueue = priority_queue<Point, vector<Point>, decltype(minimumCost)>;
        PointQueue openSet(minimumCost);

        startingPoint = {startingPoint.x, startingPoint.y, 0, 0, 0, nullptr};
        openSet.push(startingPoint);

        auto pathMatrix = Matrix(inputMatrix.size(), vector<int>(inputMatrix[0].size(), 0));
        pathMatrix[startingPoint.y][startingPoint.x] = visited;

        vector<Point> surroundingPoints(8, {0, 0, 0, 0, 0, nullptr});

        bool stop = false;
        while (!stop && !openSet.empty())
        {
            auto currentPoint = openSet.top();
            openSet.pop();

            surroundingPoints.resize(8);
            getSurroundingPoints(inputMatrix, currentPoint, surroundingPoints);

            //For every valid point surrounding the current point
            for (auto &successor : surroundingPoints)
            {
                //If we reached the target, then stop
                finishingPoint = std::make_shared<Point>(currentPoint);
                if (successor.x == target.x && successor.y == target.y)
                {
                    cout << "Total Cost of Path: " << currentPoint.totalCost << endl;
                    outputMatrix[target.y][target.x] = visited;
                    stop = true;
                    break;
                }

                //Visit unvisited points
                if (pathMatrix[successor.y][successor.x] != visited)
                {
                    pathMatrix[successor.y][successor.x] = visited;

                    successor.movementCost = (currentPoint.movementCost
                                              + octileDistance(currentPoint, successor)
                                              + inputMatrix[successor.y][successor.x]);

                    successor.heuristic = octileDistance(successor, target);

                    //Tiebreaker
//                    if (successor.heuristic == openSet.top().heuristic)
//                    {
//                        successor.heuristic += crossProduct(startingPoint, target, currentPoint) * 0.001;
//                    }

                    successor.totalCost = successor.movementCost + successor.heuristic; //f = g + h
                    successor.parent = std::make_unique<Point>(currentPoint);

                    openSet.push(successor);
                }
            }
        }
        //Reset matrix for next run
        std::fill(pathMatrix.begin(), pathMatrix.end(), std::vector<int>(pathMatrix.size(), 0));
        controlPoints.pop_front();
        while(finishingPoint)
        {
            outputMatrix[finishingPoint->y][finishingPoint->x] = visited;
            finishingPoint = finishingPoint->parent;
        }
    }

    return outputMatrix;
}

int main()
{
    int w = wallWeight;

//    Matrix inputMatrix =
//    {
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, w, 0, 0, 0, 0},
//            {0, 0, 0, w, 0, 0, 0, 0},
//            {0, 0, 0, w, 0, 0, 0, 0},
//            {0, 0, 0, w, 0, 0, 0, 0},
//            {0, 0, 0, w, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0}
//    };

    Matrix inputMatrix =
    {
        {0, 0, 0, 0, 0, 0, 0, 0},
        {w, w, 0, w, w, w, w, 0},
        {0, 0, 0, w, 0, 0, 0, 0},
        {0, 0, 0, w, 0, 0, w, w},
        {0, 0, 0, w, 0, 0, w, w},
        {w, 0, w, w, 0, 0, w, w},
        {0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0}
    };

//    Matrix inputMatrix =
//    {
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0}
//    };

    Matrix outputMatrix = getShortestPath(inputMatrix, {{0,0}, {7,7}});

    printMatrix(outputMatrix);

    return 0;
}

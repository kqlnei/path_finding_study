#include "board.h"

std::map<Mass::status, MassInfo> Mass::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// 動的な要素
	{ START,	{-1.0f, 'S'}},
	{ GOAL,		{-1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>> &mass) const
{
	 struct Node {
        Point point;
        float costSoFar;
        float priority;

        bool operator>(const Node& other) const {
            return priority > other.priority;
        }
    };

    auto manhattanDistance = [](const Point& a, const Point& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };

    auto isValid = [&](const Point& p) {
        return p.x >= 0 && p.y >= 0 && p.y < mass.size() && p.x < mass[0].size() && mass[p.y][p.x].canMove();
    };

    auto updateMass = [&](const Point& p, float costSoFar) {
        if (isValid(p) && (costSoFar < mass[p.y][p.x].getCost() || mass[p.y][p.x].getCost() < 0)) {
            mass[p.y][p.x].set(Mass::WAYPOINT);
            return true;
        }
        return false;
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;
    std::vector<std::vector<bool>> visited(mass.size(), std::vector<bool>(mass[0].size(), false));

    frontier.push({start, 0, manhattanDistance(start, goal)});

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        if (current.point == goal) {
            return true; // Found the goal
        }

        if (visited[current.point.y][current.point.x]) {
            continue;
        }
        visited[current.point.y][current.point.x] = true;

        Point neighbors[] = {
            {current.point.x - 1, current.point.y},
            {current.point.x + 1, current.point.y},
            {current.point.x, current.point.y - 1},
            {current.point.x, current.point.y + 1}
        };

        for (const auto& neighbor : neighbors) {
            if (!isValid(neighbor) || visited[neighbor.y][neighbor.x]) {
                continue;
            }

            float newCost = current.costSoFar + mass[neighbor.y][neighbor.x].getCost();
            if (mass[neighbor.y][neighbor.x].getCost() > 0) {
                newCost += manhattanDistance(neighbor, goal);
            }

            if (updateMass(neighbor, newCost)) {
                frontier.push({neighbor, newCost, newCost + manhattanDistance(neighbor, goal)});
            }
        }
    }

    return false;
}

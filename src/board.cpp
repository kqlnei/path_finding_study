#include "board.h"
#include <queue>
#include <unordered_map>
#include <cmath>
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

float Board::getMoveCost(const Mass& mass, bool isDiagonal) const {
    float baseCost = mass.getCost();
    if (baseCost < 0) return std::numeric_limits<float>::max(); // 壁など
    float diagonalMultiplier = isDiagonal ? 1.414f : 1.0f;
    return baseCost * diagonalMultiplier;
}

bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const {


    mass[start.y][start.x].set(Mass::START);
    mass[goal.y][goal.x].set(Mass::GOAL);

    const int dx[] = { 1, 0, -1, 0, 1, 1, -1, -1 };
    const int dy[] = { 0, 1, 0, -1, 1, -1, 1, -1 };

    auto compare = [](const std::pair<Point, float>& a, const std::pair<Point, float>& b) {

        return a.second > b.second;
    };
    std::priority_queue<std::pair<Point, float>, std::vector<std::pair<Point, float>>, decltype(compare)> openSet(compare);

    std::unordered_map<int, Point> cameFrom;
    std::unordered_map<int, float> gScore;
    std::unordered_map<int, float> fScore;

    auto h = [&goal, this](const Point& p) {
        return Point::distance(p, goal) * Mass::getStatusCost(Mass::BLANK);
    };
    
    auto getKey = [&mass](const Point& p) {
        return p.y * mass[0].size() + p.x;
    };

    openSet.push({ start, h(start) });
    gScore[getKey(start)] = 0;
    fScore[getKey(start)] = h(start);

    while (!openSet.empty()) {
        Point current = openSet.top().first;
        openSet.pop();

        if (current == goal) {
            // 経路を再構築
            while (current != start) {
                if (current != goal) {
                    mass[current.y][current.x].set(Mass::WAYPOINT);
                }
                current = cameFrom[getKey(current)];
            }
            return true;
        }

        for (int i = 0; i < 8; ++i) {
            Point neighbor = { current.x + dx[i], current.y + dy[i] };
            if (neighbor.x < 0 || neighbor.x >= mass[0].size() || neighbor.y < 0 || neighbor.y >= mass.size()) {
                continue;
            }
            if (!mass[neighbor.y][neighbor.x].canMove()) {
                continue;
            }

            bool isDiagonal = (i >= 4);
            float moveCost = getMoveCost(mass[neighbor.y][neighbor.x], isDiagonal);
            float tentative_gScore = gScore[getKey(current)] + moveCost;

            if (!gScore.count(getKey(neighbor)) || tentative_gScore < gScore[getKey(neighbor)]) {
                cameFrom[getKey(neighbor)] = current;
                gScore[getKey(neighbor)] = tentative_gScore;
                fScore[getKey(neighbor)] = gScore[getKey(neighbor)] + h(neighbor);
                openSet.push({ neighbor, fScore[getKey(neighbor)] });
            }
        }
    }

    return false;  // パスが見つからなかった
}
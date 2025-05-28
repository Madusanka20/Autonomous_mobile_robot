#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <sstream>
#include <algorithm>

struct Node {
    int x, y;
    double g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

std::string getKey(int x, int y) {
    std::stringstream ss;
    ss << x << "," << y;
    return ss.str();
}

bool isValid(int x, int y, const std::vector<std::vector<double>>& costmap) {
    int rows = costmap.size();
    int cols = costmap[0].size();
    return x >= 0 && x < rows && y >= 0 && y < cols && costmap[x][y] < 1.0;
}

double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

std::vector<Node> reconstructPath(Node* node) {
    std::vector<Node> path;
    while (node != nullptr) {
        path.push_back(*node);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> aStar(const std::vector<std::vector<double>>& costmap, int startX, int startY, int goalX, int goalY) {
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    double move_cost[] = {1.4, 1.0, 1.4, 1.0, 1.0, 1.4, 1.0, 1.4};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<std::string, bool> closed;

    Node* start = new Node(startX, startY);
    start->g = 0;
    start->h = heuristic(startX, startY, goalX, goalY);
    start->f = start->g + start->h;

    open.push(*start);

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        std::string key = getKey(current.x, current.y);
        if (closed[key]) continue;
        closed[key] = true;

        if (current.x == goalX && current.y == goalY) {
            return reconstructPath(&current);
        }

        for (int i = 0; i < 8; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isValid(nx, ny, costmap)) continue;

            std::string nkey = getKey(nx, ny);
            if (closed[nkey]) continue;

            Node* neighbor = new Node(nx, ny, new Node(current));
            neighbor->g = current.g + move_cost[i] * costmap[nx][ny];
            neighbor->h = heuristic(nx, ny, goalX, goalY);
            neighbor->f = neighbor->g + neighbor->h;

            open.push(*neighbor);
        }
    }

    return {};
}

// Function to check if movement is straight (same row or column)
bool isStraightMovement(const Node& current, const Node& next) {
    return (current.x == next.x || current.y == next.y);
}

// Function to generate velocity commands
void generateVelocityCommands(const std::vector<Node>& path) {
    if (path.size() < 2) {
        std::cout << "No movement needed." << std::endl;
        return;
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        const Node& current = path[i];
        const Node& next = path[i + 1];

        if (isStraightMovement(current, next)) {
            std::cout << "Move from (" << current.x << ", " << current.y << ") to (" 
                      << next.x << ", " << next.y << "): v = 0.07 m/s, w = 0 rad/s" << std::endl;
        } else {
            std::cout << "Rotate from (" << current.x << ", " << current.y << ") to (" 
                      << next.x << ", " << next.y << "): v = 0 m/s, w = 0.5 rad/s" << std::endl;
            std::cout << "Move from (" << current.x << ", " << current.y << ") to (" 
                      << next.x << ", " << next.y << "): v = 0.07 m/s, w = 0 rad/s" << std::endl;
        }
    }
}

int main() {
    std::vector<std::vector<double>> costmap = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    int startX = 2, startY = 0;
    int goalX = 3, goalY = 4;

    std::vector<Node> path = aStar(costmap, startX, startY, goalX, goalY);

    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        std::cout << "Path found:" << std::endl;
        for (const auto& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ")" << std::endl;
        }

        std::cout << "\nVelocity Commands:" << std::endl;
        generateVelocityCommands(path);
    }

    return 0;
}
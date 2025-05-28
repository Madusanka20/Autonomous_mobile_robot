#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <sstream>
#include <algorithm>

// ======================= Node Structure ========================
struct Node {
    int x, y;
    double g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// ======================= Helper Functions ========================
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

// ======================= A* Algorithm ========================
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

// ======================= Main ========================
int main() {
    int startX = 0, startY = 0;
    int goalX = 3, goalY = 4;

    // Initial Costmap
    std::vector<std::vector<double>> costmap = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    for (int iteration = 0; iteration < 3; iteration++) {
        std::cout << "\n--- Iteration " << iteration + 1 << " ---" << std::endl;

        // Modify costmap for demonstration (e.g. add/remove obstacles)
        if (iteration == 1) {
            costmap[2][2] = 1.0;  // Add obstacle
        } else if (iteration == 2) {
            costmap[1][1] = 0.0;  // Remove obstacle
        }

        // Print current costmap
        std::cout << "Costmap:" << std::endl;
        for (const auto& row : costmap) {
            for (double cost : row) {
                std::cout << cost << " ";
            }
            std::cout << std::endl;
        }

        // Run A*
        std::vector<Node> path = aStar(costmap, startX, startY, goalX, goalY);

        // Show path
        if (path.empty()) {
            std::cout << "No path found!" << std::endl;
        } else {
            std::cout << "Path found:" << std::endl;
            for (const auto& node : path) {
                std::cout << "(" << node.x << ", " << node.y << ") ";
            }
            std::cout << std::endl;
        }
    }

    return 0;
}

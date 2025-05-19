#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <functional>

// Define a structure to represent a node in the grid
struct Node {
    int x, y;       // Coordinates
    double g, h, f; // Cost values (g = cost from start, h = heuristic to goal, f = g + h)
    Node* parent;   // Parent node for path reconstruction

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    // Overload the == operator for comparing nodes
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }

    // Overload the < operator for priority queue
    bool operator<(const Node& other) const {
        return f > other.f; // For min-heap
    }
};

// Hash function for Node to use in unordered_map
namespace std {
    template<> 
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            return hash<int>()(node.x) ^ hash<int>()(node.y);
        }
    };
}

class AStarPlanner {
private:
    std::vector<std::vector<double>> costmap; // Your 2D costmap from SLAM
    int width, height;

public:
    AStarPlanner(const std::vector<std::vector<double>>& costmap) 
        : costmap(costmap), width(costmap.size()), height(costmap[0].size()) {}

    // Heuristic function (Euclidean distance)
    double heuristic(int x1, int y1, int x2, int y2) {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

    // Check if a cell is valid (within bounds and not obstructed)
    bool isValid(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height && costmap[x][y] < 1.0;
    }

    // A* path planning
    std::vector<Node> findPath(int startX, int startY, int goalX, int goalY) {
        // Priority queue for open nodes
        std::priority_queue<Node> openSet;
        
        // Hash maps for open and closed sets
        std::unordered_map<Node, double> openSetLookup;
        std::unordered_map<Node, bool> closedSet;

        // Create start and goal nodes
        Node start(startX, startY);
        start.g = 0;
        start.h = heuristic(startX, startY, goalX, goalY);
        start.f = start.g + start.h;

        Node goal(goalX, goalY);

        openSet.push(start);
        openSetLookup[start] = start.f;

        // Possible movement directions (8-connected grid)
        const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const double move_cost[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414}; // Diagonal costs more

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();

            // Check if we've reached the goal
            if (current.x == goal.x && current.y == goal.y) {
                return reconstructPath(current);
            }

            closedSet[current] = true;

            // Explore neighbors
            for (int i = 0; i < 8; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (!isValid(nx, ny)) continue;

                Node neighbor(nx, ny, &current);
                
                // Skip if in closed set
                if (closedSet.find(neighbor) != closedSet.end()) {
                    continue;
                }

                // Calculate tentative g score
                double tentative_g = current.g + move_cost[i] * costmap[nx][ny];

                // Check if this path to neighbor is better
                if (openSetLookup.find(neighbor) == openSetLookup.end() || tentative_g < neighbor.g) {
                    neighbor.parent = &current;
                    neighbor.g = tentative_g;
                    neighbor.h = heuristic(nx, ny, goalX, goalY);
                    neighbor.f = neighbor.g + neighbor.h;

                    openSet.push(neighbor);
                    openSetLookup[neighbor] = neighbor.f;
                }
            }
        }

        // No path found
        return std::vector<Node>();
    }

    // Reconstruct the path from goal to start
    std::vector<Node> reconstructPath(Node current) {
        std::vector<Node> path;
        while (current.parent != nullptr) {
            path.push_back(current);
            current = *current.parent;
        }
        path.push_back(current); // Add the start node
        std::reverse(path.begin(), path.end());
        return path;
    }
};

int main() {
    // Example costmap (0 = free, 1 = obstacle, values between represent cost)
    std::vector<std::vector<double>> costmap = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    AStarPlanner planner(costmap);

    // Start at (0, 0), goal at (4, 4)
    auto path = planner.findPath(0, 0, 4, 4);

    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        std::cout << "Path found:" << std::endl;
        for (const auto& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ")" << std::endl;
        }
    }

    return 0;
}
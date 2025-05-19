#include "OccupancyGrid.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// Function to create a test environment with obstacles
void createTestEnvironment(OccupancyGrid& grid) {
    // Create some walls (obstacles) using world coordinates
    // Horizontal wall from (2.5m, 2.5m) to (10.0m, 3.75m)
    for (float x = 2.5f; x < 10.0f; x += 0.05f) {
        for (float y = 2.5f; y < 3.75f; y += 0.05f) {
            std::vector<std::pair<float, float>> point;
            point.emplace_back(x, y);
            grid.updateWithGlobalPoints(point);
        }
    }
    
    // Vertical wall from (2.5m, 2.5m) to (3.75m, 10.0m)
    for (float x = 2.5f; x < 3.75f; x += 0.05f) {
        for (float y = 2.5f; y < 10.0f; y += 0.05f) {
            std::vector<std::pair<float, float>> point;
            point.emplace_back(x, y);
            grid.updateWithGlobalPoints(point);
        }
    }
    
    // Create some random obstacles
    for (int i = 0; i < 10; ++i) {
        float x = 5.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/5.0f));
        float y = 5.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/5.0f));
        
        // Create circular obstacle with 0.25m radius
        for (float angle = 0; angle < 2*M_PI; angle += 0.1f) {
            for (float r = 0; r <= 0.25f; r += 0.05f) {
                std::vector<std::pair<float, float>> point;
                point.emplace_back(x + r*cos(angle), y + r*sin(angle));
                grid.updateWithGlobalPoints(point);
            }
        }
    }
}

int main() {
    // Create grid and test environment
    OccupancyGrid grid(500, 500, 0.05f);
    createTestEnvironment(grid);
    
    const std::string desktop_path = "/home/lahiru/Desktop/";
    
    // Save occupancy grid
    std::string occupancy_path = desktop_path + "test_occupancy.png";
    grid.saveAsImage(occupancy_path);
    
    // Create trajectory
    std::vector<Pose2D> trajectory;
    for (int i = 0; i < 100; ++i) {
        trajectory.push_back({i * 0.2f, i * 0.2f, i * 0.1f});
    }
    
    // Update cost map
    float robot_radius = 0.3f;
    grid.updateCostMap(robot_radius);
    
    // Create cost map visualization
    cv::Mat cost_map_img(grid.getHeight(), grid.getWidth(), CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Draw cost map
    for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            float cost = grid.getCost(x, y);
            if (std::isinf(cost)) {
                cost_map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red for obstacles
            }
        }
    }
    
    // Draw trajectory
    for (const auto& pose : trajectory) {
        int x = static_cast<int>(pose.x / grid.getResolution()) + grid.getWidth() / 2;
        int y = static_cast<int>(pose.y / grid.getResolution()) + grid.getHeight() / 2;
        if (grid.isCellInside(x, y)) {
            cost_map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Green for trajectory
        }
    }
    
    // Save and show results
    std::string cost_map_path = desktop_path + "test_cost_map.png";
    cv::imwrite(cost_map_path, cost_map_img);
    
    cv::imshow("Cost Map", cost_map_img);
    cv::waitKey(0);
    
    return 0;
}

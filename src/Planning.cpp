#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node ted.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_WARN(get_logger(), "Waiting for /map_server/map service...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        map_client_->async_send_request(
            request,
            std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        map_ready_ = true;

        RCLCPP_INFO(get_logger(), "Map received: %d x %d",
                    map_.info.width, map_.info.height);

        dilateMap();
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get map.");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                            std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    if (!map_ready_) {
        RCLCPP_ERROR(get_logger(), "Map is not ready yet.");
        return;
    }

    aStar(request->start, request->goal);

    if (!path_.poses.empty()) {
        smoothPath();
        path_pub_->publish(path_);
        response->plan = path_;
        RCLCPP_INFO(get_logger(), "Path published.");
    } else {
        RCLCPP_ERROR(get_logger(), "No path found.");
    }
}

void PlanningNode::dilateMap() {
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    int width = static_cast<int>(map_.info.width);
    int height = static_cast<int>(map_.info.height);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = index(x, y);

            if (map_.data[idx] >= obstacle_threshold_ || map_.data[idx] < 0) {
                for (int dy = -dilation_radius_; dy <= dilation_radius_; dy++) {
                    for (int dx = -dilation_radius_; dx <= dilation_radius_; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (!isInside(nx, ny)) continue;

                        if (dx * dx + dy * dy <= dilation_radius_ * dilation_radius_) {
                            dilatedMap.data[index(nx, ny)] = 100;
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Map dilated.");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start,
                         const geometry_msgs::msg::PoseStamped &goal) {
    path_.poses.clear();
    path_.header.frame_id = map_.header.frame_id;
    path_.header.stamp = this->now();

    int sx, sy, gx, gy;

    if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        RCLCPP_ERROR(get_logger(), "Start or goal outside map.");
        return;
    }

    if (!isFree(sx, sy) || !isFree(gx, gy)) {
        RCLCPP_ERROR(get_logger(), "Start or goal is occupied.");
        return;
    }

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);
    std::vector<float> gScore(map_.info.height * map_.info.width,
                              std::numeric_limits<float>::infinity());

    auto startCell = std::make_shared<Cell>(sx, sy);
    startCell->g = 0.0f;
    startCell->h = heuristic(sx, sy, gx, gy);
    startCell->f = startCell->g + startCell->h;

    openList.push_back(startCell);
    gScore[index(sx, sy)] = 0.0f;

    const int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while(!openList.empty() && rclcpp::ok()) {
        auto currentIt = std::min_element(
            openList.begin(), openList.end(),
            [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b) {
                return a->f < b->f;
            });

        auto current = *currentIt;
        openList.erase(currentIt);

        int currentIdx = index(current->x, current->y);
        if (closedList[currentIdx]) continue;
        closedList[currentIdx] = true;

        if (current->x == gx && current->y == gy) {
            std::vector<geometry_msgs::msg::PoseStamped> poses;

            auto node = current;
            while (node) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = map_.header.frame_id;
                pose.header.stamp = this->now();
                pose.pose.orientation.w = 1.0;

                mapToWorld(node->x, node->y, pose.pose.position.x, pose.pose.position.y);
                pose.pose.position.z = 0.0;

                poses.push_back(pose);
                node = node->parent;
            }

            std::reverse(poses.begin(), poses.end());
            path_.poses = poses;
            return;
        }

        for (const auto &d : dirs) {
            int nx = current->x + d[0];
            int ny = current->y + d[1];

            if (!isInside(nx, ny) || !isFree(nx, ny))
                continue;

            int nidx = index(nx, ny);
            if (closedList[nidx])
                continue;

            float stepCost = (d[0] != 0 && d[1] != 0) ? 1.41421356f : 1.0f;
            float tentativeG = current->g + stepCost;
            if (tentativeG < gScore[nidx]) {
                auto neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = tentativeG;
                neighbor->h = heuristic(nx, ny, gx, gy);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                gScore[nidx] = tentativeG;
                openList.push_back(neighbor);
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) return;

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    std::vector<geometry_msgs::msg::PoseStamped> originalPath = path_.poses;

    const double weight_data = 0.2;
    const double weight_smooth = 0.4;
    const double tolerance = 0.0001;
    const int max_iterations = 200;

    for (int iter = 0; iter < max_iterations; iter++) {
        double change = 0.0;

        for (size_t i = 1; i < newPath.size() - 1; i++) {
            double old_x = newPath[i].pose.position.x;
            double old_y = newPath[i].pose.position.y;

            newPath[i].pose.position.x +=
                weight_data * (originalPath[i].pose.position.x - newPath[i].pose.position.x) +
                weight_smooth * (newPath[i - 1].pose.position.x + newPath[i + 1].pose.position.x
                                 - 2.0 * newPath[i].pose.position.x);

            newPath[i].pose.position.y +=
                weight_data * (originalPath[i].pose.position.y - newPath[i].pose.position.y) +
                weight_smooth * (newPath[i - 1].pose.position.y + newPath[i + 1].pose.position.y
                                 - 2.0 * newPath[i].pose.position.y);

            change += std::abs(old_x - newPath[i].pose.position.x) +
                      std::abs(old_y - newPath[i].pose.position.y);
        }

        if (change < tolerance) break;
    }

    path_.poses = newPath;
}

Cell::Cell(int c, int r) {
    x = c;
    y = r;
    f = std::numeric_limits<float>::infinity();
    g = std::numeric_limits<float>::infinity();
    h = 0.0f;
    parent = nullptr;
}

bool PlanningNode::isInside(int x, int y) const {
    return x >= 0 && y >= 0 &&
           x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
}

bool PlanningNode::isFree(int x, int y) const {
    if (!isInside(x, y)) return false;

    int8_t value = map_.data[index(x, y)];
    if (value < 0) return false;

    return value < obstacle_threshold_;
}

int PlanningNode::index(int x, int y) const {
    return y * static_cast<int>(map_.info.width) + x;
}

bool PlanningNode::worldToMap(double wx, double wy, int &mx, int &my) const {
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double resolution = map_.info.resolution;

    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));

    return isInside(mx, my);
}

void PlanningNode::mapToWorld(int mx, int my, double &wx, double &wy) const {
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double resolution = map_.info.resolution;

    wx = origin_x + (mx + 0.5) * resolution;
    wy = origin_y + (my + 0.5) * resolution;
}

float PlanningNode::heuristic(int x1, int y1, int x2, int y2) const {
    float dx = static_cast<float>(x2 - x1);
    float dy = static_cast<float>(y2 - y1);
    return std::sqrt(dx * dx + dy * dy);
}

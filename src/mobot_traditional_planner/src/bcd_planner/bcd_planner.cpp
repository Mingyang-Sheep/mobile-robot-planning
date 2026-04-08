#include "my_path_planner/bcd_planner/bcd_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(my_path_planner::BCDPlanner, nav_core::BaseGlobalPlanner)

namespace my_path_planner {

BCDPlanner::BCDPlanner() : costmap_(nullptr), initialized_(false) {}

void BCDPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();

        ros::NodeHandle nh("~/" + name);
        nh.param("step_size", step_size_, 3); // 线条间距
        nh.param("y_step", y_step_, 1);       // 点间距

        initialized_ = true;
        ROS_INFO("Industrial-Grade BCD Planner Initialized.");
    }
}

void BCDPlanner::addPose(int mx, int my, std::vector<geometry_msgs::PoseStamped>& plan) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = origin_x_ + (mx + 0.5) * resolution_;
    pose.pose.position.y = origin_y_ + (my + 0.5) * resolution_;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
}

// 核心 1：像倒水一样，只找出与机器人连通的安全区域（解决灰色越界问题）
std::vector<std::vector<bool>> BCDPlanner::getReachableArea(int start_x, int start_y) {
    std::vector<std::vector<bool>> reachable(width_, std::vector<bool>(height_, false));
    std::queue<std::pair<int, int>> q;
    
    q.push({start_x, start_y});
    reachable[start_x][start_y] = true;

    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};

    while (!q.empty()) {
        auto curr = q.front(); q.pop();
        for (int i = 0; i < 4; ++i) {
            int nx = curr.first + dx[i];
            int ny = curr.second + dy[i];
            
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && !reachable[nx][ny]) {
                unsigned char cost = costmap_->getCost(nx, ny);
                // 只允许在空闲区域蔓延，遇到墙壁或未知区域立刻停止
                if (cost < 50 && cost != costmap_2d::NO_INFORMATION) {
                    reachable[nx][ny] = true;
                    q.push({nx, ny});
                }
            }
        }
    }
    return reachable;
}

// 核心 2：把连通区域切成一段段的线，避开柱子
std::vector<BCDSegment> BCDPlanner::extractSegments(const std::vector<std::vector<bool>>& reachable) {
    std::vector<BCDSegment> segments;
    for (int x = 0; x < width_; x += step_size_) {
        int y = 0;
        while (y < height_) {
            if (reachable[x][y]) {
                int y_start = y;
                while (y < height_ && reachable[x][y]) y++;
                int y_end = y - 1;
                // 只保留有一定长度的有效段
                if (y_end - y_start >= 1) {
                    segments.push_back({x, y_start, y_end, false});
                }
            } else {
                y++;
            }
        }
    }
    return segments;
}

// 核心 3：BFS 防穿墙寻路！两段线之间绝不跨墙连直线
std::vector<std::pair<int, int>> BCDPlanner::getTransitPath(int x1, int y1, int x2, int y2, const std::vector<std::vector<bool>>& reachable) {
    std::vector<std::pair<int, int>> path;
    if (x1 == x2 && y1 == y2) return path;

    std::vector<std::vector<std::pair<int,int>>> parent(width_, std::vector<std::pair<int,int>>(height_, {-1, -1}));
    std::queue<std::pair<int, int>> q;
    q.push({x1, y1});
    parent[x1][y1] = {x1, y1};

    int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};

    bool found = false;
    while (!q.empty()) {
        auto curr = q.front(); q.pop();
        if (curr.first == x2 && curr.second == y2) { found = true; break; }
        
        for (int i = 0; i < 8; ++i) {
            int nx = curr.first + dx[i];
            int ny = curr.second + dy[i];
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && reachable[nx][ny] && parent[nx][ny].first == -1) {
                parent[nx][ny] = curr;
                q.push({nx, ny});
            }
        }
    }

    if (found) {
        std::pair<int, int> curr = {x2, y2};
        while (curr.first != x1 || curr.second != y1) {
            path.push_back(curr);
            curr = parent[curr.first][curr.second];
        }
        std::reverse(path.begin(), path.end());
    }
    return path;
}

bool BCDPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) return false;
    plan.clear();

    unsigned int start_mx, start_my;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
        ROS_ERROR("Start point is out of bounds!");
        return false;
    }

    // 1. 获取真正的封闭房间区域
    auto reachable = getReachableArea(start_mx, start_my);
    
    // 2. 提取所有的安全垂直扫描段
    auto segments = extractSegments(reachable);
    if (segments.empty()) return false;

    // 3. 智能连接这些段
    int curr_x = start_mx, curr_y = start_my;
    bool sweep_up = true;
    int visited_count = 0;

    while (visited_count < segments.size()) {
        // 寻找距离当前点最近的未访问段
        BCDSegment* best_seg = nullptr;
        double min_dist = 1e9;
        bool next_sweep_up = true;

        for (auto& seg : segments) {
            if (seg.visited) continue;
            // 算起到段起点和终点的距离，选择最近的端点接入
            double dist_to_start = std::hypot(seg.x - curr_x, seg.y_start - curr_y);
            double dist_to_end = std::hypot(seg.x - curr_x, seg.y_end - curr_y);
            
            if (dist_to_start < min_dist) {
                min_dist = dist_to_start; best_seg = &seg; next_sweep_up = true;
            }
            if (dist_to_end < min_dist) {
                min_dist = dist_to_end; best_seg = &seg; next_sweep_up = false;
            }
        }

        if (!best_seg) break;

        // 生成从当前位置到下一个段的安全接驳路径 (防穿墙！)
        int target_y = next_sweep_up ? best_seg->y_start : best_seg->y_end;
        auto transit_path = getTransitPath(curr_x, curr_y, best_seg->x, target_y, reachable);
        for (auto& p : transit_path) addPose(p.first, p.second, plan);

        // 执行本段的垂直扫描
        if (next_sweep_up) {
            for (int y = best_seg->y_start; y <= best_seg->y_end; y += y_step_) addPose(best_seg->x, y, plan);
            curr_y = best_seg->y_end;
        } else {
            for (int y = best_seg->y_end; y >= best_seg->y_start; y -= y_step_) addPose(best_seg->x, y, plan);
            curr_y = best_seg->y_start;
        }

        curr_x = best_seg->x;
        best_seg->visited = true;
        visited_count++;
    }

    ROS_INFO("BCD Success! Generated %zu collision-free waypoints.", plan.size());
    return !plan.empty();
}

}
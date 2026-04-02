#ifndef BCD_PLANNER_H
#define BCD_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <algorithm>

namespace my_path_planner {

// 存储一条安全的垂直扫描段
struct BCDSegment {
    int x;
    int y_start;
    int y_end;
    bool visited = false;
};

class BCDPlanner : public nav_core::BaseGlobalPlanner {
public:
    BCDPlanner();
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
    double resolution_;
    double origin_x_, origin_y_;
    int width_, height_;
    
    int step_size_; 
    int y_step_;   

    // 核心函数 1：从起点蔓延，找出真正的“房间内部”
    std::vector<std::vector<bool>> getReachableArea(int start_x, int start_y);
    
    // 核心函数 2：在房间内部提取安全的垂直扫描段
    std::vector<BCDSegment> extractSegments(const std::vector<std::vector<bool>>& reachable);
    
    // 核心函数 3：使用 BFS 寻找绕过障碍物的安全接驳路径 (解决穿墙问题！)
    std::vector<std::pair<int, int>> getTransitPath(int x1, int y1, int x2, int y2, 
                                                    const std::vector<std::vector<bool>>& reachable);

    void addPose(int mx, int my, std::vector<geometry_msgs::PoseStamped>& plan);
};

} // namespace my_path_planner

#endif
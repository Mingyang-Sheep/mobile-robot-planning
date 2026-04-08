#ifndef STC_PLANNER_H
#define STC_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>
#include <set>

namespace my_path_planner {

struct Node {
    int r, c;
    bool operator==(const Node& other) const { return r == other.r && c == other.c; }
    bool operator<(const Node& other) const { return r < other.r || (r == other.r && c < other.c); }
};

class STCPlanner : public nav_core::BaseGlobalPlanner {
public:
    STCPlanner();
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    // 核心算法函数
    void performSTC(Node current, std::vector<std::vector<int>>& visit_times, std::vector<Node>& route);
    bool isValidNode(int r, int c);
    std::vector<Node> moveNode(Node p, Node q);
    std::vector<Node> getRoundTripPath(Node last, Node pivot);
    char getVectorDirection(Node p, Node q);
    Node getSubNode(Node node, std::string dir);
    Node getIntermediateNode(Node p, Node q);

    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
    int m_height_, m_width_; // 合并后的网格宽高
    std::vector<std::pair<Node, Node>> edges_; // 生成树的边
};

}
#endif
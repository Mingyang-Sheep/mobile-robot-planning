#include "my_path_planner/stc_planner/stc_planner.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(my_path_planner::STCPlanner, nav_core::BaseGlobalPlanner)

namespace my_path_planner {

STCPlanner::STCPlanner() : costmap_(nullptr), initialized_(false) {}

void STCPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        ROS_INFO("STC Planner (C++ Version) initialized.");
    }
}

bool STCPlanner::isValidNode(int r, int c) {
    if (r < 0 || r >= m_height_ || c < 0 || c >= m_width_) return false;
    // 检查 2x2 的子网格是否全部空闲
    for (int dr = 0; dr < 2; ++dr) {
        for (int dc = 0; dc < 2; ++dc) {
            if (costmap_->getCost(2 * c + dc, 2 * r + dr) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                return false;
        }
    }
    return true;
}

// 对应 Python 的 perform_spanning_tree_coverage
void STCPlanner::performSTC(Node current, std::vector<std::vector<int>>& visit_times, std::vector<Node>& route) {
    route.push_back(current);
    int dr[] = {1, 0, -1, 0}; // 南, 东, 北, 西 (逆时针顺序)
    int dc[] = {0, 1, 0, -1};

    bool found = false;
    for (int i = 0; i < 4; ++i) {
        int nr = current.r + dr[i];
        int nc = current.c + dc[i];
        if (isValidNode(nr, nc) && visit_times[nr][nc] == 0) {
            Node neighbor = {nr, nc};
            edges_.push_back({current, neighbor});
            visit_times[nr][nc] = 1;
            found = true;
            performSTC(neighbor, visit_times, route);
        }
    }

    if (!found) {
        bool has_unvisited = false;
        for (int i = route.size() - 1; i >= 0; --i) {
            Node node = route[i];
            if (visit_times[node.r][node.c] == 2) continue;

            visit_times[node.r][node.c]++;
            route.push_back(node);

            for (int j = 0; j < 4; ++j) {
                int nr = node.r + dr[j];
                int nc = node.c + dc[j];
                if (isValidNode(nr, nc) && visit_times[nr][nc] == 0) {
                    has_unvisited = true;
                    break;
                }
            }
            if (has_unvisited) break;
        }
    }
}

// 辅助函数：获取子节点坐标
Node STCPlanner::getSubNode(Node node, std::string dir) {
    if (dir == "SE") return {2 * node.r + 1, 2 * node.c + 1};
    if (dir == "SW") return {2 * node.r + 1, 2 * node.c};
    if (dir == "NE") return {2 * node.r, 2 * node.c + 1};
    if (dir == "NW") return {2 * node.r, 2 * node.c};
    return {0, 0};
}

// 获取方向
char STCPlanner::getVectorDirection(Node p, Node q) {
    if (p.r == q.r && p.c < q.c) return 'E';
    if (p.r == q.r && p.c > q.c) return 'W';
    if (p.r < q.r && p.c == q.c) return 'S';
    if (p.r > q.r && p.c == q.c) return 'N';
    return 'X';
}

// 对应 Python 的 move
std::vector<Node> STCPlanner::moveNode(Node p, Node q) {
    char dir = getVectorDirection(p, q);
    if (dir == 'E') return {getSubNode(p, "SE"), getSubNode(q, "SW")};
    if (dir == 'W') return {getSubNode(p, "NW"), getSubNode(q, "NE")};
    if (dir == 'S') return {getSubNode(p, "SW"), getSubNode(q, "NW")};
    if (dir == 'N') return {getSubNode(p, "NE"), getSubNode(q, "SE")};
    return {};
}

// 对应 Python 的 get_round_trip_path
std::vector<Node> STCPlanner::getRoundTripPath(Node last, Node pivot) {
    char dir = getVectorDirection(last, pivot);
    if (dir == 'E') return {getSubNode(pivot, "SE"), getSubNode(pivot, "NE")};
    if (dir == 'S') return {getSubNode(pivot, "SW"), getSubNode(pivot, "SE")};
    if (dir == 'W') return {getSubNode(pivot, "NW"), getSubNode(pivot, "SW")};
    if (dir == 'N') return {getSubNode(pivot, "NE"), getSubNode(pivot, "NW")};
    return {};
}

// 对应 Python 的 get_intermediate_node
Node STCPlanner::getIntermediateNode(Node p, Node q) {
    std::set<Node> p_ngb, q_ngb;
    for (auto& edge : edges_) {
        if (edge.first == p) p_ngb.insert(edge.second);
        if (edge.second == p) p_ngb.insert(edge.first);
        if (edge.first == q) q_ngb.insert(edge.second);
        if (edge.second == q) q_ngb.insert(edge.first);
    }
    for (auto& n : p_ngb) {
        if (q_ngb.count(n)) return n;
    }
    return p;
}

bool STCPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) return false;

    // 1. 初始化地图参数
    unsigned int width = costmap_->getSizeInCellsX();
    unsigned int height = costmap_->getSizeInCellsY();
    m_width_ = width / 2;
    m_height_ = height / 2;
    edges_.clear();

    // 2. 坐标转换：世界 -> 合并网格
    unsigned int mx, my;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
    Node start_node = {static_cast<int>(my / 2), static_cast<int>(mx / 2)};

    // 3. 执行 STC 生成树算法
    std::vector<std::vector<int>> visit_times(m_height_, std::vector<int>(m_width_, 0));
    visit_times[start_node.r][start_node.c] = 1;
    std::vector<Node> route;
    performSTC(start_node, visit_times, route);

    // 4. 生成子网格路径 (关键逻辑转换)
    std::vector<Node> full_path_nodes;
    for (size_t i = 0; i < route.size() - 1; ++i) {
        int dp = std::abs(route[i].r - route[i+1].r) + std::abs(route[i].c - route[i+1].c);
        std::vector<Node> segment;
        if (dp == 0) segment = getRoundTripPath(route[i-1], route[i]);
        else if (dp == 1) segment = moveNode(route[i], route[i+1]);
        else if (dp == 2) {
            Node mid = getIntermediateNode(route[i], route[i+1]);
            auto s1 = moveNode(route[i], mid);
            auto s2 = moveNode(mid, route[i+1]);
            segment.insert(segment.end(), s1.begin(), s1.end());
            segment.insert(segment.end(), s2.begin(), s2.end());
        }
        for (auto& n : segment) full_path_nodes.push_back(n);
    }

    // 5. 转换回 ROS 格式
    double res = costmap_->getResolution();
    double ox = costmap_->getOriginX();
    double oy = costmap_->getOriginY();

    for (auto& n : full_path_nodes) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = ox + (n.c + 0.5) * res;
        pose.pose.position.y = oy + (n.r + 0.5) * res;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }

    return !plan.empty();
}

}
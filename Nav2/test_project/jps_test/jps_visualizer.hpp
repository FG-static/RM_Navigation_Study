#include "opencv2/core/types.hpp"
#include "opencv2/opencv.hpp"
#include <queue>
#include <vector>
#include <random>
#include <functional>
#include <limits>
#include <string>
#include <cmath>
#include <algorithm>

// JPS 节点（使用栅格地图坐标）
struct JPSNode {

    cv::Point2d pos;
    double cost; // f = g + h, 用于优先级队列排序
    bool operator>(const JPSNode& other) const {
        return cost > other.cost;
    }
    JPSNode() : pos(0, 0), cost(0.0) {}
    JPSNode(const cv::Point2d& p, double c) : pos(p), cost(c) {}
};

class MapPlanner {

public:

    // 地图相关
    cv::Mat map_img, display_map, raw_map; // 栅格地图
    int img_w = 200, img_h = 162;
    double resolution = 0.05; // 分辨率
    double origin_x = -5.046, origin_y = -4.64; // 地图原点

    // JPS open list（优先级队列）
    std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>> open_list;

    cv::Point2d start_pt, goal_pt;

    // 可视化参数
    int iteration_delay_ms = 5;
    bool show_debug = true;

    // 结果与状态
    bool found_path = false;
    std::vector<cv::Point2d> final_path;

    MapPlanner(const std::string &pgm_path);
    void preprocessMap();
    cv::Point WorldToMap(double wx, double wy) const;
    void plan(cv::Point start, cv::Point goal);

private:

    // 基础工具函数
    bool isTraversable(int x, int y) const;
    double dist(const cv::Point2d &a, const cv::Point2d &b) const;
    bool hasLineOfSight(int x1, int y1, int x2, int y2) const;

    // JPS 核心: 判断当前格子沿方向 (dx,dy) 是否有强迫邻居（即是否为跳点）
    bool hasForcedNeighbor(int x, int y, int dx, int dy) const;

    // JPS 核心: 从 (x,y) 沿方向 (dx,dy) 递归跳跃，返回找到的跳点坐标
    // 返回 (-1,-1) 表示该方向无合法跳点
    cv::Point2d jump(int x, int y, int dx, int dy, const cv::Point2d &goal) const;

    // 从当前节点 cur 向 8 个方向搜索跳点，加入 open_list
    void jpSearch(
        const cv::Point2d &cur,
        const cv::Point2d &goal,
        std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>> &open_list,
        std::vector<std::vector<bool>> &closed_list,
        std::vector<std::vector<double>> &g_values,
        std::vector<std::vector<cv::Point>> &parents
    );

    // 绘制搜索树（绿线）和已探索节点
    void drawSearchStep(const std::vector<std::vector<cv::Point>> &parents,
                        const std::vector<std::vector<bool>> &closed_list) const;

};

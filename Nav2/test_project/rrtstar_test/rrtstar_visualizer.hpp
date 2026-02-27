#include "opencv2/opencv.hpp"
#include <vector>
#include <random>
#include <functional>
#include <limits>
#include <string>
#include <cmath>
#include <algorithm>

// RRT* 节点（使用世界坐标，单位 m）
struct RRTNode {

    cv::Point2d pos;
    int parent; // 在 tree 中的索引，-1 为根
    double cost; // 从起点到该节点的累计代价
    RRTNode() : pos(0,0), parent(-1), cost(0.0) {}
    RRTNode(const cv::Point2d &p, int par, double c) : pos(p), parent(par), cost(c) {}
};

class MapPlanner {

public:

    // 地图相关
    cv::Mat map_img, display_map, raw_map;
    int img_w = 200, img_h = 162;
    double resolution = 0.05;
    double origin_x = -5.046, origin_y = -4.64;

    // RRT* 参数
    double step_size = 1.0;
    int max_iterations = 50000;
    double 
        search_radius = 2.0, // 用于重连
        goal_sample_rate = 0.5, // 采样直接采中目标的概率
        goal_tolerance = 0.2,
        collision_check_resolution = 0.02;

    // 树结构
    std::vector<RRTNode> tree;

    // 起点/终点（像素，double用于插值计算）
    cv::Point2d start_pt, goal_pt;

    // 随机生成器
    std::mt19937 rng;
    std::uniform_real_distribution<double> uni_x;
    std::uniform_real_distribution<double> uni_y;

    // 结果与状态
    bool found_goal = false;
    std::vector<cv::Point2d> final_path;
    double best_cost = std::numeric_limits<double>::infinity();

    // 可视化
    cv::Mat draw_img;
    int iteration_delay_ms = 5;
    bool show_debug = true;

    // 构造与主要接口
    MapPlanner(const std::string &pgm_path);
    void preprocessMap(int robot_radius_px = 4);

    // 坐标转换辅助
    cv::Point worldToMap(double wx, double wy) const;

    // 规划接口：传入像素坐标的起点和终点
    void planRRTStar(const cv::Point &start_pix, const cv::Point &goal_pix);

    // 算法辅助函数
    cv::Point2d sampleRandomPoint();
    int nearestIndex(const cv::Point2d &p) const;
    cv::Point2d steer(const cv::Point2d &from, const cv::Point2d &to);
    std::vector<int> nearIndices(const cv::Point2d &p) const;
    bool collisionFree(const cv::Point2d &a, const cv::Point2d &b) const;
    double dist(const cv::Point2d &a, const cv::Point2d &b) const;
    void reconstructPath(int goal_idx);

    // 绘制
    void drawTree();
    void drawPath();

    // 地图查询
    bool isOccupiedPx(int mx, int my) const;
    bool isInsideMapPx(int mx, int my) const;

private:

    void initRandomGenerators();
};
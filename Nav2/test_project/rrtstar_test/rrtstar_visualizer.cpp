#include "rrtstar_visualizer.hpp"
#include <iostream>
#include <chrono>

MapPlanner::MapPlanner(const std::string &pgm_path) {

    map_img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
    raw_map = map_img;
    if (map_img.empty()) {
        std::cerr << "Failed to load .pgm" << std::endl;
        return;
    }
    img_w = map_img.cols;
    img_h = map_img.rows;
    display_map = map_img.clone();

    initRandomGenerators();
}

/**
 * @brief 对地图进行预处理，膨胀障碍物以考虑机器人尺寸
 * @param robot_radius_px 机器人半径（像素）
 * @return void
 */
void MapPlanner::preprocessMap(int robot_radius_px) {

    cv::cvtColor(map_img, display_map, cv::COLOR_GRAY2BGR);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                      cv::Size(2 * robot_radius_px + 1, 2 * robot_radius_px + 1));
    cv::erode(map_img, map_img, element);
}

cv::Point MapPlanner::worldToMap(double wx, double wy) const {
    int mx = static_cast<int>(std::round((wx - origin_x) / resolution));
    int my = static_cast<int>(std::round((wy - origin_y) / resolution));
    my = img_h - my;
    mx = std::max(0, std::min(mx, img_w - 1));
    my = std::max(0, std::min(my, img_h - 1));
    return cv::Point(mx, my);
}

void MapPlanner::initRandomGenerators() {

    std::random_device rd;
    rng = std::mt19937(rd());
    uni_x = std::uniform_real_distribution<double>(0.0, static_cast<double>(img_w));
    uni_y = std::uniform_real_distribution<double>(0.0, static_cast<double>(img_h));
}

/**
 * @brief 从地图范围内随机采样一个点，按照 goal_sample_rate 的概率直接返回目标点
 * @return cv::Point2d 采样点的像素坐标（double 用于插值计算）
 */
cv::Point2d MapPlanner::sampleRandomPoint() {

    double r = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
    if (r < goal_sample_rate) return goal_pt;
    return cv::Point2d(uni_x(rng), uni_y(rng));
}

int MapPlanner::nearestIndex(const cv::Point2d &p) const {

    int best = -1;
    double bestd = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(tree.size()); ++ i) {

        double d = dist(tree[i].pos, p);
        if (d < bestd) {

            bestd = d;
            best = i;
        }
    }
    return best;
}

/**
 * @brief 从 from 向 to 方向扩展一个新点，距离为 step_size
 * @param from 起点（像素坐标）
 * @param to 目标点（像素坐标）
 * @return cv::Point2d 新点的像素坐标（double 用于插值计算）
 */
cv::Point2d MapPlanner::steer(const cv::Point2d &from, const cv::Point2d &to) {

    double d = dist(from, to);
    if (d <= step_size) return to;
    cv::Point2d vec = to - from;
    return from + vec * (step_size / d);
}

/**
 * @brief 查找树中距离点 p 在 search_radius 范围内的所有节点索引
 * @param p 查询点（像素坐标）
 * @return 满足条件的节点索引列表
 */
std::vector<int> MapPlanner::nearIndices(const cv::Point2d &p) const {

    std::vector<int> inds;
    double r_pix = search_radius / resolution;
    for (int i = 0; i < static_cast<int>(tree.size()); ++ i) {

        if (dist(tree[i].pos, p) <= r_pix) {

            inds.push_back(i);
        }
    }
    return inds;
}

/**
 * @brief 检查线段 ab 上是否存在障碍物
 * @param a 起点（像素坐标）
 * @param b 终点（像素坐标）
 * @return true 如果线段上没有障碍物；false 如果存在障碍物
 */
bool MapPlanner::collisionFree(const cv::Point2d &a, const cv::Point2d &b) const {

    double d = dist(a, b);
    if (d < 1e-6) return !isOccupiedPx(static_cast<int>(a.x), static_cast<int>(a.y));
    double step = collision_check_resolution / resolution;
    int n = static_cast<int>(std::ceil(d / step));
    for (int i = 0; i <= n; ++ i) {

        double t = static_cast<double>(i) / n;
        double x = a.x + (b.x - a.x) * t;
        double y = a.y + (b.y - a.y) * t;
        int mx = static_cast<int>(std::round(x));
        int my = static_cast<int>(std::round(y));
        if (isOccupiedPx(mx, my)) return false;
    }
    return true;
}

inline double MapPlanner::dist(const cv::Point2d &a, const cv::Point2d &b) const {

    double 
        dx = a.x - b.x,
        dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 从 goal_idx 反向追踪父节点直到起点，重建路径
 * @param goal_idx 目标节点在树中的索引
 * @return void
 */
void MapPlanner::reconstructPath(int goal_idx) {

    final_path.clear();
    int idx = goal_idx;
    while (idx != -1) {

        final_path.push_back(tree[idx].pos);
        idx = tree[idx].parent;
    }
    std::reverse(final_path.begin(), final_path.end());
}

/**
 * @brief 绘制 RRT* 树结构
 * 每条边用绿色线段表示
 * @return void
 */
void MapPlanner::drawTree() {

    draw_img = display_map.clone();
    for (int i = 1; i < static_cast<int>(tree.size()); ++ i) {

        cv::Point p1(static_cast<int>(tree[i].pos.x), static_cast<int>(tree[i].pos.y));
        cv::Point p2(static_cast<int>(tree[tree[i].parent].pos.x),
                     static_cast<int>(tree[tree[i].parent].pos.y));
        cv::line(draw_img, p1, p2, cv::Scalar(0, 255, 0), 1);
    }
}

/**
 * @brief 绘制最终路径
 * 每条边用蓝色线段表示
 * @return void
 */
void MapPlanner::drawPath() {

    for (size_t i = 0; i + 1 < final_path.size(); ++ i) {

        cv::Point p1(static_cast<int>(final_path[i].x), static_cast<int>(final_path[i].y));
        cv::Point p2(static_cast<int>(final_path[i + 1].x), static_cast<int>(final_path[i + 1].y));
        cv::line(draw_img, p1, p2, cv::Scalar(255, 0, 0), 2);
    }
}

/**
 * @brief 检查像素坐标 (mx, my) 是否在地图范围内
 * @param mx 像素 x 坐标
 * @param my 像素 y 坐标
 * @return true 如果在地图范围内；false 如果超出地图边界
 */
bool MapPlanner::isInsideMapPx(int mx, int my) const {

    return mx >= 0 && mx < img_w && my >= 0 && my < img_h;
}

/**
 * @brief 检查像素坐标 (mx, my) 是否被占用（障碍物）
 * @param mx 像素 x 坐标
 * @param my 像素 y 坐标
 * @return true 如果被占用（障碍物）；false 如果空闲
 */
bool MapPlanner::isOccupiedPx(int mx, int my) const {

    if (!isInsideMapPx(mx, my)) return true;
    return map_img.at<uchar>(my, mx) == 0;
}

/**
 * @brief RRT* 规划主函数
 * @param start_pix 起点像素坐标
 * @param goal_pix 终点像素坐标
 * 流程：
 * 1. 初始化树，添加起点
 * 2. 循环 max_iterations 次：
 *   a. 随机采样一个点 p
 *   b. 找到树中距离 p 最近的节点 nearest
 *   c. 从 nearest 向 p 方向扩展一个新节点 new_pt，距离为 step_size
 *   d. 如果 new_pt 与 nearest 之间没有碰撞，则将 new_pt 添加到树中，父节点为 nearest，代价为 nearest 的代价加上两点距离
 *   e. 在 new_pt 周围 search_radius 范围内找到所有节点 near_ids，尝试通过 new_pt 重连这些节点以降低代价
 *   f. 如果 new_pt 距离目标点 goal_pt 在 goal_tolerance 范围内，则认为找到目标，添加 goal_pt 作为 new_pt 的子节点，并重建路径
 * 3. 可选：每隔一定迭代次数显示搜索过程
 * 4. 最终显示结果路径或失败信息
 * @return void
 */
void MapPlanner::planRRTStar(const cv::Point &start_pix, const cv::Point &goal_pix) {

    start_pt = cv::Point2d(start_pix.x, start_pix.y);
    goal_pt = cv::Point2d(goal_pix.x, goal_pix.y);

    tree.clear();
    tree.emplace_back(start_pt, -1, 0.0);
    found_goal = false;
    best_cost = std::numeric_limits<double>::infinity(); // 记录找到的路径的最优代价

    // 在地图上标记起点和终点
    cv::circle(display_map, cv::Point(static_cast<int>(start_pt.x), static_cast<int>(start_pt.y)), 3, cv::Scalar(0,0,255), -1);
    cv::circle(display_map, cv::Point(static_cast<int>(goal_pt.x), static_cast<int>(goal_pt.y)), 3, cv::Scalar(255,0,0), -1);

    static int count = 0;
    for (int it = 0; it < max_iterations; ++ it) {

        cv::Point2d p = sampleRandomPoint();
        int nearest = nearestIndex(p);
        if (nearest < 0) continue;
        cv::Point2d new_pt = steer(tree[nearest].pos, p);
        if (!collisionFree(tree[nearest].pos, new_pt)) continue;
        double new_cost = tree[nearest].cost + dist(tree[nearest].pos, new_pt);
        int new_idx = tree.size();
        tree.emplace_back(new_pt, nearest, new_cost);

        // 尝试重连附近节点以降低代价
        auto near_ids = nearIndices(new_pt);
        for (int nid : near_ids) {

            double c = tree[new_idx].cost + dist(new_pt, tree[nid].pos);
            if (c < tree[nid].cost && collisionFree(new_pt, tree[nid].pos)) {

                tree[nid].parent = new_idx;
                tree[nid].cost = c;
            }
        }

        // 检查是否接近目标点
        if (dist(new_pt, goal_pt) < goal_tolerance / resolution) {
            
            found_goal = true;
            // 直接将目标点作为 new_pt 的子节点添加到树中，方便路径重建
            cv::Point2d goal_node = goal_pt;
            double gcost = tree[new_idx].cost + dist(new_pt, goal_node);
            tree.emplace_back(goal_node, new_idx, gcost);
            reconstructPath(tree.size() - 1);
            best_cost = gcost;
            break;
        }

        // 每隔一定迭代次数显示搜索过程
        if (show_debug && (count++ % 50) == 0) {

            drawTree();
            cv::imshow("RRT* Searching", draw_img);
            cv::waitKey(iteration_delay_ms);
        }
    }

    // final visualization
    if (found_goal) {

        drawTree();
        drawPath();
        cv::imshow("RRT* Result", draw_img);
    } else {

        std::cout << "Failed to find path in " << max_iterations << " iterations\n";
    }
}

int main() {
    
    MapPlanner planner("map1.pgm");
    planner.preprocessMap();

    cv::Point start_px = planner.worldToMap(-4.5, 1.0);
    cv::Point goal_px = planner.worldToMap(1.0, -3.0);
    std::cout << "start pixel: " << start_px << "  goal pixel: " << goal_px << std::endl;

    // slow down visualization so process is visible
    planner.iteration_delay_ms = 30;
    planner.show_debug = true;

    planner.planRRTStar(start_px, goal_px);
    cv::waitKey(0);
    return 0;
}

#include "jps_visualizer.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <chrono>
#include <limits>
#include <vector>

MapPlanner::MapPlanner(const std::string &pgm_path) {

    map_img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
    raw_map = map_img.clone();
    if (map_img.empty()) {
        std::cerr << "Failed to load map image: " << pgm_path << std::endl;
        return;
    }
    img_w = map_img.cols;
    img_h = map_img.rows;
}

void MapPlanner::preprocessMap() {

    cv::cvtColor(raw_map, display_map, cv::COLOR_GRAY2BGR);

    int robot_radius_pixel = 4;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                      cv::Size(robot_radius_pixel * 2 + 1, robot_radius_pixel * 2 + 1));
    cv::erode(map_img, map_img, element);
}

cv::Point MapPlanner::WorldToMap(double wx, double wy) const {

    int mx = static_cast<int>(std::round((wx - origin_x) / resolution)),
        premy = static_cast<int>(std::round((wy - origin_y) / resolution));
    int my = img_h - premy;
    mx = std::max(0, std::min(mx, img_w - 1));
    my = std::max(0, std::min(my, img_h - 1));
    return cv::Point(mx, my);
}

// ─── 基础工具 ────────────────────────────────────────────

bool MapPlanner::isTraversable(int x, int y) const {
    if (x < 0 || x >= img_w || y < 0 || y >= img_h) return false;
    return map_img.at<uchar>(y, x) != 0;
}

double MapPlanner::dist(const cv::Point2d &a, const cv::Point2d &b) const {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/// 两点之间是否有直线视线（无遮挡）
bool MapPlanner::hasLineOfSight(int x1, int y1, int x2, int y2) const {
    int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1, sy = y1 < y2 ? 1 : -1;
    int err = dx - dy;
    while (x1 != x2 || y1 != y2) {
        if (!isTraversable(x1, y1)) return false;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x1 += sx; }
        if (e2 <  dx) { err += dx; y1 += sy; }
    }
    return isTraversable(x2, y2);
}

// ─── JPS 核心函数 ─────────────────────────────────────────

bool MapPlanner::hasForcedNeighbor(int x, int y, int dx, int dy) const {

    // 水平方向 (dx = ±1, dy = 0)
    if (dx != 0 && dy == 0) {
        if (!isTraversable(x, y - 1) && isTraversable(x + dx, y - 1)) return true;
        if (!isTraversable(x, y + 1) && isTraversable(x + dx, y + 1)) return true;
    }

    // 垂直方向 (dx = 0, dy = ±1)
    else if (dx == 0 && dy != 0) {
        if (!isTraversable(x - 1, y) && isTraversable(x - 1, y + dy)) return true;
        if (!isTraversable(x + 1, y) && isTraversable(x + 1, y + dy)) return true;
    }

    // 对角线方向 (dx = ±1, dy = ±1)
    else {
        if (!isTraversable(x - dx, y) && isTraversable(x - dx, y + dy)) return true;
        if (!isTraversable(x, y - dy) && isTraversable(x + dx, y - dy)) return true;
    }

    return false;
}

cv::Point2d MapPlanner::jump(int x, int y, int dx, int dy,
                              const cv::Point2d &goal) const {

    int nx = x + dx;
    int ny = y + dy;

    // 出界或障碍物 → 该方向无效
    if (!isTraversable(nx, ny)) {
        return cv::Point2d(-1, -1);
    }

    // 到达目标 → 跳点
    if (nx == static_cast<int>(goal.x) && ny == static_cast<int>(goal.y)) {
        return cv::Point2d(nx, ny);
    }

    // 有强迫邻居 → 跳点
    if (hasForcedNeighbor(nx, ny, dx, dy)) {
        return cv::Point2d(nx, ny);
    }

    // 对角线：递归检查两个子方向
    if (dx != 0 && dy != 0) {
        if (jump(nx, ny, dx, 0, goal).x >= 0) return cv::Point2d(nx, ny);
        if (jump(nx, ny, 0, dy, goal).x >= 0) return cv::Point2d(nx, ny);
    }

    // 检查是否会越过目标（目标在当前点和下一点之间或就在下一点）
    int gx = static_cast<int>(goal.x), gy = static_cast<int>(goal.y);
    if (dx > 0 && nx < gx && nx + 1 >= gx && gy == ny) return cv::Point2d(nx, ny);
    if (dx < 0 && nx > gx && nx - 1 <= gx && gy == ny) return cv::Point2d(nx, ny);
    if (dy > 0 && ny < gy && ny + 1 >= gy && gx == nx) return cv::Point2d(nx, ny);
    if (dy < 0 && ny > gy && ny - 1 <= gy && gx == nx) return cv::Point2d(nx, ny);
    if (dx != 0 && dy != 0) {
        // 对角线：目标在同一对角线上，且下一步会越过
        int dgx = gx - nx, dgy = gy - ny;
        if (dgx * dx > 0 && dgy * dy > 0 &&
            std::abs(dgx) <= std::abs(dx) * 2 && std::abs(dgx) == std::abs(dgy)) {
            return cv::Point2d(nx, ny);
        }
    }

    // 继续沿原方向递归
    return jump(nx, ny, dx, dy, goal);
}

void MapPlanner::jpSearch(
    const cv::Point2d &cur,
    const cv::Point2d &goal,
    std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>> &open_list,
    std::vector<std::vector<bool>> &closed_list,
    std::vector<std::vector<double>> &g_values,
    std::vector<std::vector<cv::Point>> &parents
) {
    int cx = static_cast<int>(cur.x), cy = static_cast<int>(cur.y);
    int gx = static_cast<int>(goal.x), gy = static_cast<int>(goal.y);

    // 先检查是否有直接视线到目标
    if (hasLineOfSight(cx, cy, gx, gy)) {
        double move_cost = dist(cur, goal);
        double new_g = g_values[cy][cx] + move_cost;
        if (new_g < g_values[gy][gx]) {
            g_values[gy][gx] = new_g;
            parents[gy][gx] = cv::Point(cx, cy);
            double h = 0.0; // goal 的 h = 0
            open_list.push(JPSNode(goal, new_g + h));
        }
    }

    // 8 方向
    const int dx[8] = { 1, -1,  0,  0,  1, -1,  1, -1 };
    const int dy[8] = { 0,  0, -1,  1, -1, -1,  1,  1 };

    for (int i = 0; i < 8; ++i) {
        cv::Point2d jp = jump(cx, cy, dx[i], dy[i], goal);
        if (jp.x < 0) continue;

        int jx = static_cast<int>(jp.x), jy = static_cast<int>(jp.y);
        if (closed_list[jy][jx]) continue;

        double move_cost = dist(cur, jp);
        double new_g = g_values[cy][cx] + move_cost;

        if (new_g < g_values[jy][jx]) {
            g_values[jy][jx] = new_g;
            parents[jy][jx] = cv::Point(cx, cy);
            double h = dist(jp, goal);
            open_list.push(JPSNode(jp, new_g + h));
        }
    }
}

// ─── 可视化 ──────────────────────────────────────────────

void MapPlanner::drawSearchStep(
    const std::vector<std::vector<cv::Point>> &parents,
    const std::vector<std::vector<bool>> &closed_list) const {

    // 从 display_map 复制一份作为画布
    cv::Mat draw_img = display_map.clone();

    // 绘制搜索树：绿线连接每个已探索节点及其父节点
    for (int y = 0; y < img_h; ++y) {
        for (int x = 0; x < img_w; ++x) {
            if (!closed_list[y][x]) continue;
            cv::Point p = parents[y][x];
            if (p.x < 0 || p.y < 0) continue;
            cv::line(draw_img, cv::Point(x, y), p, cv::Scalar(0, 255, 0), 1);
        }
    }

    // 已探索节点用浅绿色标记
    for (int y = 0; y < img_h; ++y) {
        for (int x = 0; x < img_w; ++x) {
            if (closed_list[y][x]) {
                draw_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 100);
            }
        }
    }

    // 重新绘制起点（红点）和终点（蓝点）
    cv::circle(draw_img,
        cv::Point(static_cast<int>(start_pt.x), static_cast<int>(start_pt.y)),
        3, cv::Scalar(0, 0, 255), -1);
    cv::circle(draw_img,
        cv::Point(static_cast<int>(goal_pt.x), static_cast<int>(goal_pt.y)),
        3, cv::Scalar(255, 0, 0), -1);

    cv::imshow("JPS Searching", draw_img);
    cv::waitKey(iteration_delay_ms);
}

// ─── 主规划函数 ──────────────────────────────────────────

void MapPlanner::plan(cv::Point start, cv::Point goal) {

    start_pt = cv::Point2d(start.x, start.y);
    goal_pt  = cv::Point2d(goal.x, goal.y);

    std::vector<std::vector<double>> g_values(
        img_h,
        std::vector<double>(img_w, std::numeric_limits<double>::infinity())
    );
    std::vector<std::vector<bool>> closed_list(
        img_h,
        std::vector<bool>(img_w, false)
    );
    std::vector<std::vector<cv::Point>> parents(
        img_h,
        std::vector<cv::Point>(img_w, cv::Point(-1, -1))
    );

    open_list = std::priority_queue<JPSNode,
                  std::vector<JPSNode>, std::greater<JPSNode>>();

    g_values[start.y][start.x] = 0.0;
    double h_start = dist(start_pt, goal_pt);
    open_list.push(JPSNode(start_pt, h_start));

    cv::circle(display_map, start, 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(display_map, goal,  3, cv::Scalar(255, 0, 0), -1);

    int iter = 0;
    while (!open_list.empty()) {

        JPSNode cur = open_list.top();
        open_list.pop();

        ++iter;

        if (closed_list[cur.pos.y][cur.pos.x])
            continue;
        closed_list[cur.pos.y][cur.pos.x] = true;

        // 到达目标 → 回溯路径
        int cx = static_cast<int>(cur.pos.x), cy = static_cast<int>(cur.pos.y);
        if (cx == goal.x && cy == goal.y) {

            std::cout << "Path found! Iterations: " << iter
                      << "  Cost: " << g_values[cy][cx] << std::endl;

            cv::Point curr = goal;
            final_path.clear();
            while (curr.x != -1 && curr.y != -1) {
                final_path.push_back(cv::Point2d(curr.x, curr.y));
                cv::Point prev = parents[curr.y][curr.x];
                if (prev == curr || (prev.x == -1 && prev.y == -1)) break;
                curr = prev;
            }
            std::reverse(final_path.begin(), final_path.end());

            // 先画搜索树
            drawSearchStep(parents, closed_list);

            // 在搜索树上叠加最终路径（蓝色粗线）
            cv::Mat result_img = display_map.clone();
            for (int y = 0; y < img_h; ++y) {
                for (int x = 0; x < img_w; ++x) {
                    if (closed_list[y][x]) {
                        cv::Point p = parents[y][x];
                        if (p.x >= 0 && p.y >= 0) {
                            cv::line(result_img, cv::Point(x, y), p,
                                     cv::Scalar(0, 255, 0), 1);
                        }
                    }
                }
            }
            for (size_t i = 0; i + 1 < final_path.size(); ++i) {
                cv::line(result_img,
                    cv::Point(static_cast<int>(final_path[i].x),
                              static_cast<int>(final_path[i].y)),
                    cv::Point(static_cast<int>(final_path[i+1].x),
                              static_cast<int>(final_path[i+1].y)),
                    cv::Scalar(255, 0, 0), 3);
            }
            cv::circle(result_img, start, 3, cv::Scalar(0, 0, 255), -1);
            cv::circle(result_img, goal,  3, cv::Scalar(255, 0, 0), -1);
            cv::imshow("JPS Result", result_img);
            cv::waitKey(0);
            found_path = true;
            return;
        }

        // 向 8 方向搜索跳点
        jpSearch(cur.pos, goal_pt, open_list, closed_list, g_values, parents);

        // 可视化：每步绘制搜索过程
        if (show_debug && iter % 5 == 0) {

            drawSearchStep(parents, closed_list);
        }
    }

    std::cout << "No path found after " << iter << " iterations." << std::endl;
    drawSearchStep(parents, closed_list);
    cv::waitKey(0);
    found_path = false;
}

int main() {

    MapPlanner planner("map1.pgm");

    cv::Point start_px = planner.WorldToMap(-4.5, 1.0);
    cv::Point goal_px  = planner.WorldToMap(1.0, -3.0);

    std::cout << "Start pixel: " << start_px
              << "  Goal pixel: " << goal_px << std::endl;

    // 可视化参数：类似 rrtstar_test
    planner.iteration_delay_ms = 30;
    planner.show_debug = true;

    planner.preprocessMap();
    planner.plan(start_px, goal_px);
    cv::waitKey(0);
    return 0;
}

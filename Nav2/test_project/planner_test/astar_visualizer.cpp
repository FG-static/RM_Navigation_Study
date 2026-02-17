#include "astar_visualizer.hpp"

MapPlanner::MapPlanner(std::string pgm_path) {

    // 加载灰度图
    map_img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
    raw_map = map_img;
    if (map_img.empty()) {

        std::cerr << "Failed to load .pgm";
        return;
    }
    //cv::threshold(map_img, map_img, 100, 255, cv::THRESH_BINARY);
}

void MapPlanner::preprocessMap() {

    cv::cvtColor(map_img, display_map, cv::COLOR_GRAY2BGR);

    // 腐蚀核Size(9, 9) -> r=4
    // 4像素*0.05 分辨率=0.2
    int robot_radius_pixel = 4; 
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, 
                      cv::Size(2 * robot_radius_pixel + 1, 2 * robot_radius_pixel + 1));
    cv::erode(map_img, map_img, element);
}

cv::Point MapPlanner::WorldToMap(double wx, double wy) {

    int mx = static_cast<int>(std::round(wx - origin_x) / resolution),
        premy = static_cast<int>(std::round(wy - origin_y) / resolution);
    int my = img_h - premy;
    mx = std::max(0, std::min(mx, img_w - 1));
    my = std::max(0, std::min(my, img_h - 1));
    return cv::Point(mx, my);
}

void MapPlanner::plan(cv::Point start, cv::Point goal) {

    std::vector<std::vector<double>> g_values(map_img.rows, std::vector<double>(map_img.cols, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<bool>> closed_list(map_img.rows, std::vector<bool>(map_img.cols, false));
    std::vector<std::vector<cv::Point>> parents(map_img.rows, std::vector<cv::Point>(map_img.cols, cv::Point(-1, -1)));

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

    // 放入起点
    g_values[start.y][start.x] = 0;
    double h_cost_ = std::sqrt(std::pow(start.x - goal.x, 2) + std::pow(start.y - goal.y, 2));
    open_list.push({start, 0, h_cost_, nullptr});

    // 寻路
    while (!open_list.empty()) {
        
        Node cur = open_list.top();
        open_list.pop();
        display_map.at<cv::Vec3b>(cur.pos.y, cur.pos.x) = cv::Vec3b(200, 200, 255);

        if (closed_list[cur.pos.y][cur.pos.x]) continue;
        closed_list[cur.pos.y][cur.pos.x] = true;

        if (cur.pos == goal) {

            std::cout << "Path Found" << std::endl;
            cv::Point curr = goal;

            while (curr != start && curr != cv::Point(-1, -1)) {

                path.push_back(curr);
                curr = parents[curr.y][curr.x];
            }
            path.push_back(start);

            // 绘制最终路径
            for (size_t i = 0; i < path.size() - 1; ++ i) {

                cv::line(display_map, path[i], path[i + 1], cv::Scalar(255, 0, 0), 2);
            }
            cv::imshow("A* Result", display_map);
            return;
        }

        for (int dx = -1; dx <= 1; ++ dx) {

            for (int dy = -1; dy <= 1; ++ dy) {

                if (dx == 0 && dy == 0) continue;

                cv::Point nei_pos(cur.pos.x + dx, cur.pos.y + dy);

                // 边界与碰撞检查
                if (nei_pos.x < 0 || nei_pos.x >= map_img.cols || 
                    nei_pos.y < 0 || nei_pos.y >= map_img.rows) continue;
                if (map_img.at<uchar>(nei_pos.y, nei_pos.x) == 0) continue;

                // 未知区域罚分
                uchar raw_pixel = raw_map.at<uchar>(nei_pos.y, nei_pos.x);
                double extra_cost = 0.0;
                if (raw_pixel < 250 && raw_pixel > 150) { 

                    extra_cost = 10.0;
                }

                double step_cost = (dx != 0 && dy != 0) ? 1.414 : 1.0,
                    tentative_g = g_values[cur.pos.y][cur.pos.x] + step_cost + extra_cost;

                if (tentative_g < g_values[nei_pos.y][nei_pos.x]) {

                    g_values[nei_pos.y][nei_pos.x] = tentative_g;
                    display_map.at<cv::Vec3b>(nei_pos.y, nei_pos.x) = cv::Vec3b(255, 200, 200);

                    static int count = 0;
                    if (count ++ % 50 == 0) { // 每50个点刷新一次界面

                        cv::imshow("A* Searching Process", display_map);
                        cv::waitKey(30);
                    }

                    parents[nei_pos.y][nei_pos.x] = cur.pos;
                    double h = std::sqrt(std::pow(nei_pos.x - goal.x, 2) + std::pow(nei_pos.y - goal.y, 2));
                    open_list.push({nei_pos, tentative_g, h, nullptr});
                }
            }
        }
    } 
}
int main() {
    
    MapPlanner planner("map1.pgm");
    
    cv::Point start_px = planner.WorldToMap(-4.0, 1.0);
    cv::Point goal_px = planner.WorldToMap(1.0, -3.0);

    planner.preprocessMap();

    planner.plan(start_px, goal_px);
    cv::waitKey(0);
    return 0;
}
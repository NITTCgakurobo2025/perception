#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_msgs/msg/obstacle.hpp"
#include "localization_msgs/msg/point_array.hpp"

class ObstacleDetector : public rclcpp::Node {
    using Point = geometry_msgs::msg::Point;

    struct Circle {
        Point center;
        double radius;
    };

public:
    ObstacleDetector() : Node("obstacle_detector") {
        this->declare_parameter<std::string>("intput_topic", "/R1/transformed_scan");
        this->declare_parameter<std::string>("output_topic", "/R1/obstacles");
        this->declare_parameter<double>("eps", 0.8);
        this->declare_parameter<int>("min_points", 5);

        input_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            this->get_parameter("intput_topic").as_string(), 10,
            std::bind(&ObstacleDetector::point_callback, this, std::placeholders::_1));

        output_pub_ = this->create_publisher<localization_msgs::msg::Obstacle>(
            this->get_parameter("output_topic").as_string(), 10);
        obstacle_hull_pub_ = this->create_publisher<localization_msgs::msg::PointArray>("hull", 10);
    }

private:
    void point_callback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        auto clustered_data = ClusterPoints(msg);

        std::vector<Point> centers;
        std::vector<double> radii;
        std::vector<Point> obstacle_hulls;
        for (const auto &d : clustered_data) {
            if (d.empty())
                continue;
            auto [circle, hull] = computeMinRadius(d);
            obstacle_hulls.insert(obstacle_hulls.end(), hull.begin(), hull.end());
            centers.emplace_back(circle.center);
            radii.emplace_back(circle.radius);
        }

        localization_msgs::msg::PointArray hull;
        hull.points = obstacle_hulls;

        localization_msgs::msg::Obstacle obstacles;
        obstacles.header = msg->header;
        obstacles.points = centers;
        obstacles.radius = radii;

        obstacle_hull_pub_->publish(hull);
        output_pub_->publish(obstacles);

        RCLCPP_INFO(this->get_logger(), "obstacles: %ld", centers.size());
    }

    std::vector<std::vector<Point>> ClusterPoints(const localization_msgs::msg::PointArray::SharedPtr msg) {
        const double eps = this->get_parameter("eps").as_double();
        const int min_points = this->get_parameter("min_points").as_int();

        std::vector<std::vector<Point>> clusters;

        for (const auto &point : msg->points) {
            bool added = false;

            for (auto &cluster : clusters) {
                for (const auto &p : cluster) {
                    double dist = std::hypot(point.x - p.x, point.y - p.y);
                    if (dist < eps) {
                        cluster.push_back(point);
                        added = true;
                        break;
                    }
                }
                if (added)
                    break;
            }

            if (!added) {
                clusters.push_back({point});
            }
        }

        clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                      [min_points](const std::vector<Point> &c) {
                                          return c.size() < static_cast<size_t>(min_points);
                                      }),
                       clusters.end());

        return clusters;
    }

    std::tuple<Circle, std::vector<Point>> computeMinRadius(const std::vector<Point> &points) {
        Circle circle;
        std::vector<Point> hull;
        if (points.empty() || points.size() == 1) {
            circle.center.x = circle.center.y = circle.center.z = 0.0;
            circle.radius = 0.0;
            return {circle, points};
        }

        if (points.size() == 2) {
            return {computeCircumcircle(points[0], points[1]), points};
        }

        hull = computeConvexHull(points);

        bool found = false;
        Circle tmp_circle;
        int max_points = 0;
        for (std::size_t i = 0; i < hull.size() && !found; i++) {
            for (std::size_t j = i + 1; j < hull.size() && !found; j++) {
                for (std::size_t k = i + 1; k < hull.size() && !found; k++) {
                    // 2点が接する円の方が半径が小さいことがあるためあえてj,kを重複
                    if (j == k) {
                        tmp_circle = computeCircumcircle(hull[i], hull[j]);
                    } else {
                        tmp_circle = computeCircumcircle(hull[i], hull[j], hull[k]);
                    }
                    int count = countContainPoints(hull, tmp_circle);
                    if (count > max_points) {
                        circle = tmp_circle;
                        max_points = count;
                        if (count >= static_cast<int>(hull.size()))
                            found = true;
                    }
                }
            }
        }

        return {circle, hull};
    }

    std::vector<Point> computeConvexHull(const std::vector<Point> &points) {
        if (points.size() <= 3) {
            return points;
        }

        std::vector<Point> sorted_points = points;
        std::sort(sorted_points.begin(), sorted_points.end(),
                  [](const Point &a, const Point &b) { return (a.x < b.x) || (a.x == b.x && a.y < b.y); });

        std::vector<Point> hull;
        hull.reserve(points.size());
        for (const auto &p : sorted_points) {
            while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0) {
                hull.pop_back();
            }
            hull.emplace_back(p);
        }

        size_t lower_hull_size = hull.size();
        for (auto it = sorted_points.rbegin(); it != sorted_points.rend(); ++it) {
            while (hull.size() > lower_hull_size &&
                   crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], *it) <= 0) {
                hull.pop_back();
            }
            hull.emplace_back(*it);
        }

        hull.pop_back();
        return hull;
    }

    double crossProduct(const Point &a, const Point &b, const Point &c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    Circle computeCircumcircle(const Point &a, const Point &b) {
        Circle circle;

        // 選択した点が重複した場合
        // 線分の中点を中心とする円を返す
        circle.center.x = (a.x + b.x) / 2.0;
        circle.center.y = (a.y + b.y) / 2.0;
        circle.center.z = 0.0;

        circle.radius = std::hypot(b.x - a.x, b.y - a.y) / 2.0;

        return circle;
    }

    Circle computeCircumcircle(const Point &a, const Point &b, const Point &c) {
        Circle circle;

        double ab2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
        double bc2 = (c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y);
        double ca2 = (a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y);

        // 鈍角三角形の場合
        // 最長の辺の中点を中心とする最小の円を返す
        if (ab2 + bc2 < ca2) {
            circle.center.x = (a.x + c.x) / 2.0;
            circle.center.y = (a.y + c.y) / 2.0;
            circle.center.z = 0.0;
            circle.radius = std::sqrt(ca2) / 2.0;
            return circle;
        } else if (bc2 + ca2 < ab2) {
            circle.center.x = (a.x + b.x) / 2.0;
            circle.center.y = (a.y + b.y) / 2.0;
            circle.center.z = 0.0;
            circle.radius = std::sqrt(ab2) / 2.0;
            return circle;
        } else if (ca2 + ab2 < bc2) {
            circle.center.x = (b.x + c.x) / 2.0;
            circle.center.y = (b.y + c.y) / 2.0;
            circle.center.z = 0.0;
            circle.radius = std::sqrt(bc2) / 2.0;
            return circle;
        }

        // 鋭角三角形の場合
        // 外接円を返す
        double D = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

        if (D == 0.0) {
            circle.center.x = circle.center.y = circle.center.z = 0.0;
            circle.radius = 0.0;
            RCLCPP_INFO(this->get_logger(), "points are collinear");
            return circle;
        }

        circle.center.x = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                           (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                          D;
        circle.center.y = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                           (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                          D;
        circle.center.z = 0.0;
        circle.radius = std::hypot(a.x - circle.center.x, a.y - circle.center.y);

        return circle;
    }

    int countContainPoints(const std::vector<Point> &points, const Circle &circle) {
        int count = 0;
        for (const auto &p : points) {
            double dist = std::hypot(p.x - circle.center.x, p.y - circle.center.y);
            if (dist <= circle.radius) {
                count++;
            }
        }
        return count;
    }

    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr input_sub_;
    rclcpp::Publisher<localization_msgs::msg::Obstacle>::SharedPtr output_pub_;
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr obstacle_hull_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}

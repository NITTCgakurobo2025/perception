#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_msgs/msg/obstacle.hpp"
#include "localization_msgs/msg/point_array.hpp"

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector() : Node("dbscan_cluster") {
        this->declare_parameter<std::string>("intput_topic", "/R1/transformed_scan");
        this->declare_parameter<std::string>("output_topic", "/R1/obstacles");
        this->declare_parameter<double>("eps", 0.8);
        this->declare_parameter<int>("min_points", 5);

        intput_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            this->get_parameter("intput_topic").as_string(), 10,
            std::bind(&ObstacleDetector::point_callback, this, std::placeholders::_1));

        output_pub_ = this->create_publisher<localization_msgs::msg::Obstacle>(
            this->get_parameter("output_topic").as_string(), 10);
    }

private:
    void point_callback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        auto clustered_data = ClusterPoints(msg);

        std::vector<geometry_msgs::msg::Point> centroids;
        std::vector<double> radius;
        for (const auto &d : clustered_data) {
            if (d.empty())
                continue;
            auto centroid = computeCentroid(d);
            centroids.emplace_back(centroid);
            radius.emplace_back(computeRadius(centroid, d));
        }

        localization_msgs::msg::Obstacle obstacles;
        obstacles.header = msg->header;
        obstacles.points = centroids;
        obstacles.radius = radius;

        output_pub_->publish(obstacles);

        RCLCPP_INFO(this->get_logger(), "obstacles: %ld", centroids.size());
    }

    std::vector<std::vector<geometry_msgs::msg::Point>>
    ClusterPoints(const localization_msgs::msg::PointArray::SharedPtr msg) {
        const double eps = this->get_parameter("eps").as_double();
        const int min_points = this->get_parameter("min_points").as_int();

        std::vector<std::vector<geometry_msgs::msg::Point>> clusters;

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
                                      [min_points](const std::vector<geometry_msgs::msg::Point> &c) {
                                          return c.size() < static_cast<size_t>(min_points);
                                      }),
                       clusters.end());

        return clusters;
    }

    geometry_msgs::msg::Point computeCentroid(const std::vector<geometry_msgs::msg::Point> &points) {
        geometry_msgs::msg::Point centroid;
        if (points.empty()) {
            RCLCPP_INFO(this->get_logger(), "cluster is empty");
            centroid.x = 0.0;
            centroid.y = 0.0;
            centroid.z = 0.0;
            return centroid;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        for (const auto &p : points) {
            sum_x += p.x;
            sum_y += p.y;
        }

        centroid.x = sum_x / points.size();
        centroid.y = sum_y / points.size();
        centroid.z = 0.0;

        return centroid;
    }

    double computeRadius(const geometry_msgs::msg::Point centroid,
                         const std::vector<geometry_msgs::msg::Point> &points) {
        double max = 0.0;
        for (const auto &p : points) {
            double dist = std::hypot(p.x - centroid.x, p.y - centroid.y);
            max = std::max(max, dist);
        }
        return max;
    }

    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr intput_sub_;
    rclcpp::Publisher<localization_msgs::msg::Obstacle>::SharedPtr output_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}

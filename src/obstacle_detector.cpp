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
        this->declare_parameter<double>("eps", 0.5);
        this->declare_parameter<int>("min_points", 3);

        intput_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            this->get_parameter("intput_topic").as_string(), 10,
            std::bind(&ObstacleDetector::point_callback, this, std::placeholders::_1));

        output_pub_ = this->create_publisher<localization_msgs::msg::Obstacle>(
            this->get_parameter("output_topic").as_string(), 10);
    }

private:
    void point_callback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        auto clustered_data = DBSCANClustering(msg);

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
    }

    std::vector<std::vector<geometry_msgs::msg::Point>>
    DBSCANClustering(const localization_msgs::msg::PointArray::SharedPtr msg) {
        const double eps = this->get_parameter("eps").as_double();
        const int min_points = this->get_parameter("min_points").as_int();
        int cluster_id = 0;

        std::vector<int> labels(msg->points.size(), -1); // -1: 未分類, 0: ノイズ, 1以上: クラスタID
        std::vector<std::vector<int>> neighbors(msg->points.size());

        // 近傍点を求める
        for (size_t i = 0; i < msg->points.size(); ++i) {
            for (size_t j = i + 1; j < msg->points.size(); ++j) {
                double dist = std::hypot(msg->points[i].x - msg->points[j].x, msg->points[i].y - msg->points[j].y);
                if (dist < eps) {
                    neighbors[i].push_back(j);
                    neighbors[j].push_back(i);
                }
            }
        }

        // DBSCANによるクラスタリング
        for (size_t i = 0; i < msg->points.size(); ++i) {
            if (labels[i] != -1)
                continue;

            if (neighbors[i].size() < min_points) {
                labels[i] = 0;
                continue;
            }

            cluster_id++;
            labels[i] = cluster_id;
            std::vector<int> expand_list = neighbors[i];

            while (!expand_list.empty()) {
                int index = expand_list.back();
                expand_list.pop_back();

                if (labels[index] == 0)
                    labels[index] = cluster_id;

                if (labels[index] != -1)
                    continue;

                labels[index] = cluster_id;

                if (neighbors[index].size() >= min_points) {
                    expand_list.insert(expand_list.end(), neighbors[index].begin(), neighbors[index].end());
                }
            }
        }

        std::vector<std::vector<geometry_msgs::msg::Point>> clustered_data(cluster_id);
        for (int i = 1; i < cluster_id; i++) { // 0はノイズ
            for (std::size_t j = 0; j < labels.size(); j++) {
                if (labels[j] == i) {
                    clustered_data[i].push_back(msg->points[j]);
                }
            }
        }

        return clustered_data;
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

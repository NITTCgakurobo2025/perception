#include <map>
#include <vector>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_msgs/msg/point_array.hpp"

class ObstacleFilter : public rclcpp::Node {
    struct Point2D {
        double x, y;
    };

public:
    ObstacleFilter() : Node("obstacle_filter") {
        this->declare_parameter<std::string>("scan_topic", "filtered_scan");
        this->declare_parameter<std::string>("transform_topic", "scan_transform");
        this->declare_parameter<std::string>("output_topic", "transformed_scan");
        this->declare_parameter<double>("field_width", 8.0);
        this->declare_parameter<double>("field_height", 15.0);
        this->declare_parameter<double>("fence_threshold", 0.2);
        this->declare_parameter<int>("max_buffer_size", 10);

        scan_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            this->get_parameter("scan_topic").as_string(), 10,
            std::bind(&ObstacleFilter::scanCallback, this, std::placeholders::_1));
        transform_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            this->get_parameter("transform_topic").as_string(), 10,
            std::bind(&ObstacleFilter::transformCallback, this, std::placeholders::_1));
        output_pub_ = this->create_publisher<localization_msgs::msg::PointArray>(
            this->get_parameter("output_topic").as_string(), 10);

        max_buffer_size_ = this->get_parameter("max_buffer_size").as_int();
    }

private:
    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr transform_sub_;
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr output_pub_;

    std::map<rclcpp::Time, localization_msgs::msg::PointArray> scan_buffer_;
    std::map<rclcpp::Time, geometry_msgs::msg::Vector3Stamped> transform_buffer_;
    std::size_t max_buffer_size_;

    void scanCallback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        rclcpp::Time timestamp = msg->header.stamp;
        scan_buffer_[timestamp] = *msg;
        if (transform_buffer_.count(timestamp) > 0) {
            process(timestamp);
        } else if (scan_buffer_.size() >= max_buffer_size_) {
            auto it = scan_buffer_.begin();
            scan_buffer_.erase(it);
        }
    }

    void transformCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        rclcpp::Time timestamp = msg->header.stamp;
        transform_buffer_[timestamp] = *msg;
        if (scan_buffer_.count(timestamp) > 0) {
            process(timestamp);
        } else if (transform_buffer_.size() >= max_buffer_size_) {
            auto it = scan_buffer_.begin();
            scan_buffer_.erase(it);
        }
    }

    void process(rclcpp::Time timestamp) {
        localization_msgs::msg::PointArray scan = scan_buffer_[timestamp];
        std::vector<Point2D> points = ([this, scan]() {
            std::vector<Point2D> points;
            points.reserve(scan.points.size());
            for (const auto &point : scan.points) {
                points.push_back({point.x, point.y});
            }
            return points;
        })();
        geometry_msgs::msg::Vector3 transform = transform_buffer_[timestamp].vector;
        scan_buffer_.erase(timestamp);
        transform_buffer_.erase(timestamp);

        // Transform points
        auto moved_to_origin = transformPoints(points, -scan.pose.x, -scan.pose.y, 0.0);
        auto transformed = transformPoints(moved_to_origin, transform.x, transform.y, transform.z);
        auto moved_back = transformPoints(transformed, scan.pose.x, scan.pose.y, 0.0);

        // Remove points outside the field
        double field_width = this->get_parameter("field_width").as_double();
        double field_height = this->get_parameter("field_height").as_double();
        double fence_threshold = this->get_parameter("fence_threshold").as_double();
        double x_min = -field_width / 2.0 + fence_threshold;
        double x_max = field_width / 2.0 - fence_threshold;
        double y_min = -field_height / 2.0 + fence_threshold;
        double y_max = field_height / 2.0 - fence_threshold;

        localization_msgs::msg::PointArray output;
        output.header = scan.header;
        geometry_msgs::msg::Point tmp;
        for (const auto &point : moved_back) {
            if (point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max) {
                tmp.x = point.x;
                tmp.y = point.y;
                output.points.push_back(tmp);
            }
        }

        output_pub_->publish(output);
        if (scan_buffer_.size() > 0 || transform_buffer_.size() > 0) {
            eraseOldData(timestamp);
        }
    }

    void eraseOldData(rclcpp::Time timestamp) {
        for (auto it = scan_buffer_.begin(); it != scan_buffer_.end();) {
            if (it->first < timestamp) {
                it = scan_buffer_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = transform_buffer_.begin(); it != transform_buffer_.end();) {
            if (it->first < timestamp) {
                it = transform_buffer_.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::vector<Point2D> transformPoints(const std::vector<Point2D> &points, double dx, double dy, double dtheta) {
        std::vector<Point2D> transformed_points;
        transformed_points.reserve(points.size());
        Point2D tmp;

        if (dtheta == 0.0) {
            for (const auto &point : points) {
                tmp.x = point.x + dx;
                tmp.y = point.y + dy;
                transformed_points.push_back(tmp);
            }
            return transformed_points;
        }

        for (const auto &point : points) {
            tmp.x = std::cos(dtheta) * point.x - std::sin(dtheta) * point.y + dx;
            tmp.y = std::sin(dtheta) * point.x + std::cos(dtheta) * point.y + dy;
            transformed_points.push_back(tmp);
        }
        return transformed_points;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleFilter>());
    rclcpp::shutdown();
    return 0;
}
#include <map>
#include <random>
#include <vector>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_msgs/msg/point_array.hpp"
#include "localization_msgs/msg/wall.hpp"

class ObstacleFilter : public rclcpp::Node {
    struct Point2D {
        double x, y;

        bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }
    };

public:
    ObstacleFilter() : Node("obstacle_filter") {
        this->declare_parameter<std::string>("scan_topic", "filtered_scan");
        this->declare_parameter<std::string>("transform_topic", "scan_transform");
        this->declare_parameter<std::string>("output_topic", "transformed_scan");
        this->declare_parameter<std::string>("wall_topic", "wall");
        this->declare_parameter<double>("field_width", 8.0);
        this->declare_parameter<double>("field_height", 15.0);
        this->declare_parameter<double>("fence_threshold", 0.6);
        this->declare_parameter<int>("max_buffer_size", 10);
        this->declare_parameter<int>("max_loop", 100);
        this->declare_parameter<int>("min_points", 50);
        this->declare_parameter<double>("wall_detect_area", 1.0);
        this->declare_parameter<double>("fence_error", 0.1);

        scan_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            this->get_parameter("scan_topic").as_string(), 10,
            std::bind(&ObstacleFilter::scanCallback, this, std::placeholders::_1));
        transform_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            this->get_parameter("transform_topic").as_string(), 10,
            std::bind(&ObstacleFilter::transformCallback, this, std::placeholders::_1));
        output_pub_ = this->create_publisher<localization_msgs::msg::PointArray>(
            this->get_parameter("output_topic").as_string(), 10);
        wall_pub_ =
            this->create_publisher<localization_msgs::msg::Wall>(this->get_parameter("wall_topic").as_string(), 10);

        max_buffer_size_ = this->get_parameter("max_buffer_size").as_int();
    }

private:
    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr transform_sub_;
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr output_pub_;
    rclcpp::Publisher<localization_msgs::msg::Wall>::SharedPtr wall_pub_;

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

    std::vector<Point2D> DetectWall(const std::vector<Point2D> &points, localization_msgs::msg::Wall &wall) {
        int max_loop = this->get_parameter("max_loop").as_int();
        int min_points = this->get_parameter("min_points").as_int();
        double fence_error = this->get_parameter("fence_error").as_double();

        if (points.size() < min_points)
            return std::vector<Point2D>();

        int id1 = -1, id2 = -1; // Initialize with invalid indices
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, points.size() - 1);
        double min_error = -1.0;

        for (int i = 0; i < max_loop; ++i) {
            double error_sum = 0.0;
            int inliers = 0;
            int idx1 = dis(gen);
            int idx2 = dis(gen);
            if (idx1 == idx2)
                continue;

            const auto &p1 = points[idx1];
            const auto &p2 = points[idx2];

            // ax + by + c = 0
            double a = p2.y - p1.y;
            double b = p1.x - p2.x;
            double c = p2.x * p1.y - p1.x * p2.y;

            double norm = std::sqrt(a * a + b * b);
            a /= norm;
            b /= norm;
            c /= norm;

            double error;
            for (const auto &point : points) {
                error = std::abs(a * point.x + b * point.y + c);
                if (error <= fence_error) {
                    error_sum += error;
                    inliers++;
                }
            }

            if (inliers <= 0)
                continue;
            error_sum /= (double)inliers;
            if (min_error == -1.0) {
                min_error = error_sum;
                id1 = idx1;
                id2 = idx2;
            } else if (error_sum < min_error && inliers >= min_points) {
                min_error = error_sum;
                id1 = idx1;
                id2 = idx2;
            }
        }

        // Ensure valid indices were found
        if (id1 == -1 || id2 == -1) {
            RCLCPP_WARN(this->get_logger(), "Failed to detect a wall. Returning empty inliers.");
            return std::vector<Point2D>();
        }

        const auto &p1 = points[id1];
        const auto &p2 = points[id2];

        // ax + by + c = 0
        double a = p2.y - p1.y;
        double b = p1.x - p2.x;
        double c = p2.x * p1.y - p1.x * p2.y;

        double norm = std::sqrt(a * a + b * b);
        a /= norm;
        b /= norm;
        c /= norm;

        std::vector<Point2D> inliers;
        for (const auto &point : points) {
            double distance = std::abs(a * point.x + b * point.y + c);
            if (distance <= fence_error) {
                inliers.emplace_back(point);
            }
        }

        geometry_msgs::msg::Point p;
        p.x = p1.x;
        p.y = p1.y;
        wall.point1.emplace_back(p);
        p.x = p2.x;
        p.y = p2.y;
        wall.point2.emplace_back(p);

        return inliers;
    }

    std::vector<Point2D> RemoveWall(const std::vector<Point2D> &points, double x_min, double x_max, double y_min,
                                    double y_max) {
        std::vector<Point2D> process_points;
        process_points.reserve(points.size());

        localization_msgs::msg::Wall wall;

        double wall_detect_area = this->get_parameter("wall_detect_area").as_double();

        // x_minus
        std::copy_if(points.begin(), points.end(), std::back_inserter(process_points),
                     [x_min, wall_detect_area](const Point2D &p) { return std::abs(p.x - x_min) < wall_detect_area; });
        auto x_minus_wall = DetectWall(process_points, wall);
        process_points.clear();

        // x_plus
        std::copy_if(points.begin(), points.end(), std::back_inserter(process_points),
                     [x_max, wall_detect_area](const Point2D &p) { return std::abs(p.x - x_max) < wall_detect_area; });
        auto x_plus_wall = DetectWall(process_points, wall);
        process_points.clear();

        // y_minus
        std::copy_if(points.begin(), points.end(), std::back_inserter(process_points),
                     [y_min, wall_detect_area](const Point2D &p) { return std::abs(p.y - y_min) < wall_detect_area; });
        auto y_minus_wall = DetectWall(process_points, wall);
        process_points.clear();

        // y_plus
        std::copy_if(points.begin(), points.end(), std::back_inserter(process_points),
                     [y_max, wall_detect_area](const Point2D &p) { return std::abs(p.y - y_max) < wall_detect_area; });
        auto y_plus_wall = DetectWall(process_points, wall);

        wall_pub_->publish(wall);

        // Combine all detected wall points
        std::vector<Point2D> wall_points;
        wall_points.insert(wall_points.end(), x_minus_wall.begin(), x_minus_wall.end());
        wall_points.insert(wall_points.end(), x_plus_wall.begin(), x_plus_wall.end());
        wall_points.insert(wall_points.end(), y_minus_wall.begin(), y_minus_wall.end());
        wall_points.insert(wall_points.end(), y_plus_wall.begin(), y_plus_wall.end());

        // Remove wall points from the original points
        std::vector<Point2D> remaining_points;
        std::copy_if(points.begin(), points.end(), std::back_inserter(remaining_points),
                     [&wall_points](const Point2D &p) {
                         return std::find(wall_points.begin(), wall_points.end(), p) == wall_points.end();
                     });

        return remaining_points;
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
        double x_min = -field_width / 2.0;
        double x_max = field_width / 2.0;
        double y_min = -field_height / 2.0;
        double y_max = field_height / 2.0;

        auto output_points = RemoveWall(moved_back, x_min, x_max, y_min, y_max);

        std::vector<ObstacleFilter::Point2D> ransac_filtered_points;

        for (const auto &point : output_points) {
            if (point.x >= (x_min + fence_threshold) && point.x <= (x_max - fence_threshold) &&
                point.y >= (y_min + fence_threshold) && point.y <= (y_max - fence_threshold)) {
                ransac_filtered_points.push_back(point);
            }
        }

        localization_msgs::msg::PointArray output;
        output.header = scan.header;
        output.points.reserve(ransac_filtered_points.size());
        geometry_msgs::msg::Point tmp;
        for (const auto &point : ransac_filtered_points) {
            tmp.x = point.x;
            tmp.y = point.y;
            output.points.emplace_back(tmp);
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
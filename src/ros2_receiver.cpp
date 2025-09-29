#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "bridge_protocol.h"

// Node for receiving data from ROS1/Robot and publishing to correct topics
class TcpRobotListenerNode : public rclcpp::Node {
  public:
    TcpRobotListenerNode() : Node("tcp_robot_listener") {
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/base_scan", 10);
        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        camera_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/image_raw/compressed", 10);

        socket_fd_ = connect_to_server("172.26.1.1", 2077);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Verbindung zum Server fehlgeschlagen");
            rclcpp::shutdown();
            return;
        }

        recv_thread_ = std::thread(&TcpRobotListenerNode::receive_loop, this);
        recv_thread_.detach();
    }

    ~TcpRobotListenerNode() {
        if (socket_fd_ >= 0)
            close(socket_fd_);
    }

  private:
    int socket_fd_;
    std::thread recv_thread_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_pub_;

    int connect_to_server(const std::string& ip, int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
            return -1;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            return -1;

        RCLCPP_INFO(this->get_logger(), "Verbindung hergestellt zu %s:%d", ip.c_str(), port);
        return sock;
    }

    // Runs all the time to get latest data from TCP connection
    void receive_loop() {
        while (rclcpp::ok()) {
            CombinedData data;
            if (!recv_all((char*)&data, sizeof(data))) {
                RCLCPP_INFO(this->get_logger(), "Fehler beim Empfang von CombinedData");
                break;
            }

            publish_laser(data.laser_data);
            // Alle TFs sammeln
            std::vector<TFData> tf_vec = {data.tf_data_odom, data.tf_data_base_foot,
                                          data.tf_data_base_link};
            publish_tfs(tf_vec);
            publish_odom(data.odom_data);
            publish_camera(data.camera_data);
        }
    }

    // Receive data
    bool recv_all(char* buffer, size_t size) {
        size_t received = 0;
        while (received < size) {
            ssize_t n = recv(socket_fd_, buffer + received, size - received, 0);
            if (n <= 0)
                return false;
            received += n;
        }
        return true;
    }

    void publish_laser(const LaserScanData& d) {
        auto msg = sensor_msgs::msg::LaserScan();
        // Zeit korrekt berechnen
        int32_t sec = static_cast<int32_t>(d.stamp);
        uint32_t nanosec = static_cast<uint32_t>((d.stamp - sec) * 1e9);

        // msg.header.stamp = rclcpp::Time(sec, nanosec);
        msg.header.stamp = this->now();
        msg.header.frame_id = std::string(d.frame_id);
        msg.angle_min = d.angle_min;
        msg.angle_max = d.angle_max;
        msg.angle_increment = d.angle_increment;
        msg.time_increment = d.time_increment;
        msg.scan_time = d.scan_time;
        msg.range_min = d.range_min;
        msg.range_max = d.range_max;
        msg.ranges.assign(d.ranges, d.ranges + 541);
        msg.intensities.assign(d.intensities, d.intensities + 541);

        laser_pub_->publish(msg);
    }

    void publish_tfs(const std::vector<TFData>& tf_datas) {
        tf2_msgs::msg::TFMessage tf_msg;

        for (const auto& d : tf_datas) {
            if (std::strcmp(d.frame_id, "err_no_frame") != 0) {
                geometry_msgs::msg::TransformStamped tf;

                int32_t sec = static_cast<int32_t>(d.stamp);
                uint32_t nanosec = static_cast<uint32_t>((d.stamp - sec) * 1e9);

                // tf.header.stamp = rclcpp::Time(sec, nanosec);
                tf.header.stamp = this->now();
                tf.header.frame_id = std::string(d.frame_id);
                tf.child_frame_id = std::string(d.child_frame_id);
                tf.transform.translation.x = d.trans[0];
                tf.transform.translation.y = d.trans[1];
                tf.transform.translation.z = d.trans[2];
                tf.transform.rotation.x = d.rot[0];
                tf.transform.rotation.y = d.rot[1];
                tf.transform.rotation.z = d.rot[2];
                tf.transform.rotation.w = d.rot[3];

                tf_msg.transforms.push_back(tf);
            }
        }

        tf_pub_->publish(tf_msg);
    }

    void publish_odom(const OdomData& d) {
        auto msg = nav_msgs::msg::Odometry();

        // Zeit korrekt setzen
        int32_t sec = static_cast<int32_t>(d.stamp);
        uint32_t nanosec = static_cast<uint32_t>((d.stamp - sec) * 1e9);
        msg.header.stamp = rclcpp::Time(sec, nanosec);
        // msg.header.stamp = this->now();

        msg.header.frame_id = std::string(d.frame_id);
        msg.child_frame_id = std::string(d.child_frame_id);

        // Pose
        msg.pose.pose.position.x = d.pose_position[0];
        msg.pose.pose.position.y = d.pose_position[1];
        msg.pose.pose.position.z = d.pose_position[2];

        msg.pose.pose.orientation.x = d.pose_orientation[0];
        msg.pose.pose.orientation.y = d.pose_orientation[1];
        msg.pose.pose.orientation.z = d.pose_orientation[2];
        msg.pose.pose.orientation.w = d.pose_orientation[3];

        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = d.pose_cov[i];
        }

        // Twist (Geschwindigkeit)
        msg.twist.twist.linear.x = d.twist_linear[0];
        msg.twist.twist.linear.y = d.twist_linear[1];
        msg.twist.twist.linear.z = d.twist_linear[2];

        msg.twist.twist.angular.x = d.twist_angular[0];
        msg.twist.twist.angular.y = d.twist_angular[1];
        msg.twist.twist.angular.z = d.twist_angular[2];

        for (int i = 0; i < 36; ++i) {
            msg.twist.covariance[i] = d.twist_cov[i];
        }

        odom_pub_->publish(msg);
    }

    void publish_camera(const CameraData& d) {
        sensor_msgs::msg::CompressedImage msg;

        int32_t sec = static_cast<int32_t>(d.stamp);
        uint32_t nanosec = static_cast<uint32_t>((d.stamp - sec) * 1e9);
        msg.header.stamp = rclcpp::Time(sec, nanosec);
        msg.header.frame_id = std::string(d.frame_id);

        msg.format = std::string(d.encoding);

        size_t max_size = sizeof(d.data);
        msg.data = std::vector<uint8_t>(d.data, d.data + max_size);

        camera_pub_->publish(msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpRobotListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

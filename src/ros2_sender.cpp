#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <cstring>
#include <mutex>
#include <sys/socket.h>
#include <unistd.h>

#include "bridge_protocol.h"

class TcpVelClientNode : public rclcpp::Node {
  public:
    TcpVelClientNode() : Node("tcp_vel_client"), socket_fd_(-1) {
        // Parameter: IP and Port of Robot/ROS1-PC
        this->declare_parameter<std::string>("robot_ip", "172.26.1.1");
        this->declare_parameter<int>("robot_port", 2205);
        this->get_parameter("robot_ip", robot_ip_);
        this->get_parameter("robot_port", robot_port_);

        connect_to_robot();

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TcpVelClientNode::cmdVelCallback, this, std::placeholders::_1));
    }

    ~TcpVelClientNode() {
        if (socket_fd_ != -1) {
            close(socket_fd_);
        }
    }

  private:
    void connect_to_robot() {
        if (socket_fd_ != -1) {
            close(socket_fd_);
        }

        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(robot_port_);
        if (inet_pton(AF_INET, robot_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid IP address");
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connecting to robot at %s:%d ...", robot_ip_.c_str(),
                    robot_port_);
        if (connect(socket_fd_, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed");
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to robot");
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(send_mutex_);

        if (socket_fd_ == -1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Not connected to robot, trying to reconnect...");
            connect_to_robot();
            if (socket_fd_ == -1) {
                return;
            }
        }

        VelocityData data;
        data.linear_x = msg->linear.x;
        data.linear_y = msg->linear.y;
        data.angular_z = msg->angular.z;

        ssize_t sent = send(socket_fd_, (char*)&data, sizeof(data), 0);
        if (sent == -1) {
            RCLCPP_ERROR(this->get_logger(), "Send failed, closing socket");
            close(socket_fd_);
            socket_fd_ = -1;
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Sent velocity: %.2f, %.2f, %.2f", data.linear_x,
                         data.linear_y, data.angular_z);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    int socket_fd_;
    std::string robot_ip_;
    int robot_port_;
    std::mutex send_mutex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpVelClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

#include "minirosbridge/bridge_protocol.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tcp_vel_server");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  int port;
  nh.param("listen_port", port, 2205);

  // Socket erstellen
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    ROS_ERROR("Failed to create socket");
    return 1;
  }

  // Adresse vorbereiten
  sockaddr_in serv_addr{};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);

  if (bind(server_fd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    ROS_ERROR("Bind failed");
    close(server_fd);
    return 1;
  }

  if (listen(server_fd, 1) < 0) {
    ROS_ERROR("Listen failed");
    close(server_fd);
    return 1;
  }

  ROS_INFO("TCP server listening on port %d", port);

  sockaddr_in client_addr{};
  socklen_t client_len = sizeof(client_addr);
  int client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
  if (client_fd < 0) {
    ROS_ERROR("Accept failed");
    close(server_fd);
    return 1;
  }

  ROS_INFO("Client connected");

  // Hauptloop
  while (ros::ok()) {
    VelocityData data{};
    ssize_t received = recv(client_fd, &data, sizeof(data), 0);

    if (received <= 0) {
      ROS_WARN("Connection lost or recv failed");
      // stop robot when connection is lost
      geometry_msgs::Twist msg;
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.angular.z = 0.0;

      pub.publish(msg);
      break;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = data.linear_x;
    msg.linear.y = data.linear_y;
    msg.angular.z = data.angular_z;

    pub.publish(msg);

    ROS_INFO("Published: x=%.2f y=%.2f z=%.2f", data.linear_x, data.linear_y,
              data.angular_z);
    ros::spinOnce();
  }

  close(client_fd);
  close(server_fd);
  return 0;
}

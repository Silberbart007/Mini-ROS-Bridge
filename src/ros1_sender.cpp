#include <arpa/inet.h>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sys/socket.h>
#include <tf2_msgs/TFMessage.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <mutex>

#include "minirosbridge/bridge_protocol.h"

// Global buffers for latest messages
sensor_msgs::LaserScan::ConstPtr latest_laser_msg;
tf2_msgs::TFMessage::ConstPtr latest_tf_msg_odom;
tf2_msgs::TFMessage::ConstPtr latest_tf_msg_base_foot;
tf2_msgs::TFMessage::ConstPtr latest_tf_msg_base_link;
nav_msgs::Odometry::ConstPtr latest_odom_msg;
sensor_msgs::CompressedImage::ConstPtr latest_camera_msg;

CombinedData combined_data;

// Mutex to protect shared data
std::mutex data_mutex;

// TCP client socket
int g_client_socket = -1;

// set default for tf messages, frame_id err_no_frame shall be ignored
void init_tf_msgs() {
    // create temporary tf messages for initialization
    tf2_msgs::TFMessage::Ptr tf_msg_odom(new tf2_msgs::TFMessage);
    tf2_msgs::TFMessage::Ptr tf_msg_base_foot(new tf2_msgs::TFMessage);
    tf2_msgs::TFMessage::Ptr tf_msg_base_link(new tf2_msgs::TFMessage);

    // Create default transform
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "err_no_frame";

    tf_msg_odom->transforms.push_back(transform);
    tf_msg_base_foot->transforms.push_back(transform);
    tf_msg_base_link->transforms.push_back(transform);

    // init global buffers with temporary tf messages
    latest_tf_msg_odom = boost::const_pointer_cast<const tf2_msgs::TFMessage>(tf_msg_odom);
    latest_tf_msg_base_foot =
        boost::const_pointer_cast<const tf2_msgs::TFMessage>(tf_msg_base_foot);
    latest_tf_msg_base_link =
        boost::const_pointer_cast<const tf2_msgs::TFMessage>(tf_msg_base_link);
}

//------------------ Callbacks to store latest message in buffer

// Callback for LaserScan messages
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_laser_msg = msg;
}

// Callback for TF messages
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!msg->transforms[0].header.frame_id.compare("base_link")) {
        latest_tf_msg_base_link = msg;
        return;
    } else if (!msg->transforms[0].header.frame_id.compare("base_footprint")) {
        latest_tf_msg_base_foot = msg;
        return;
    } else {
        latest_tf_msg_odom = msg;
        return;
    }
}

// Callback for Odom messages
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_odom_msg = msg;
}

void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_camera_msg = msg;
}

// TCP send at 20Hz
void processDataCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(data_mutex);

    if (!latest_laser_msg || !latest_odom_msg || !latest_camera_msg || g_client_socket == -1) {
        std::cout << "\nEine Message ist nicht da, deswegen wird nichts geschickt\n";
        // waiting for topic data or TCP connection
        return;
    }

    // write data from global buffers into TCP transport struct

    // Laser
    combined_data.laser_data.stamp = (double)latest_laser_msg->header.stamp.sec +
                                     (double)latest_laser_msg->header.stamp.nsec / 1e9;
    strcpy(combined_data.laser_data.frame_id, latest_laser_msg->header.frame_id.c_str());
    combined_data.laser_data.angle_min = latest_laser_msg->angle_min;
    combined_data.laser_data.angle_max = latest_laser_msg->angle_max;
    combined_data.laser_data.angle_increment = latest_laser_msg->angle_increment;
    combined_data.laser_data.time_increment = latest_laser_msg->time_increment;
    combined_data.laser_data.scan_time = latest_laser_msg->scan_time;
    combined_data.laser_data.range_min = latest_laser_msg->range_min;
    combined_data.laser_data.range_max = latest_laser_msg->range_max;
    std::copy(latest_laser_msg->ranges.begin(), latest_laser_msg->ranges.begin() + 541,
              combined_data.laser_data.ranges);
    std::copy(latest_laser_msg->intensities.begin(), latest_laser_msg->intensities.begin() + 541,
              combined_data.laser_data.intensities);

    // Tf_odom
    combined_data.tf_data_odom.stamp =
        (double)latest_tf_msg_odom->transforms[0].header.stamp.sec +
        (double)latest_tf_msg_odom->transforms[0].header.stamp.nsec / 1e9;
    strcpy(combined_data.tf_data_odom.frame_id,
           latest_tf_msg_odom->transforms[0].header.frame_id.c_str());
    strcpy(combined_data.tf_data_odom.child_frame_id,
           latest_tf_msg_odom->transforms[0].child_frame_id.c_str());
    combined_data.tf_data_odom.trans[0] = latest_tf_msg_odom->transforms[0].transform.translation.x;
    combined_data.tf_data_odom.trans[1] = latest_tf_msg_odom->transforms[0].transform.translation.y;
    combined_data.tf_data_odom.trans[2] = latest_tf_msg_odom->transforms[0].transform.translation.z;
    combined_data.tf_data_odom.rot[0] = latest_tf_msg_odom->transforms[0].transform.rotation.x;
    combined_data.tf_data_odom.rot[1] = latest_tf_msg_odom->transforms[0].transform.rotation.y;
    combined_data.tf_data_odom.rot[2] = latest_tf_msg_odom->transforms[0].transform.rotation.z;
    combined_data.tf_data_odom.rot[3] = latest_tf_msg_odom->transforms[0].transform.rotation.w;

    // Tf_base_foot
    combined_data.tf_data_base_foot.stamp =
        (double)latest_tf_msg_base_foot->transforms[0].header.stamp.sec +
        (double)latest_tf_msg_base_foot->transforms[0].header.stamp.nsec / 1e9;
    strcpy(combined_data.tf_data_base_foot.frame_id,
           latest_tf_msg_base_foot->transforms[0].header.frame_id.c_str());

    strcpy(combined_data.tf_data_base_foot.child_frame_id,
           latest_tf_msg_base_foot->transforms[0].child_frame_id.c_str());
    combined_data.tf_data_base_foot.trans[0] =
        latest_tf_msg_base_foot->transforms[0].transform.translation.x;
    combined_data.tf_data_base_foot.trans[1] =
        latest_tf_msg_base_foot->transforms[0].transform.translation.y;
    combined_data.tf_data_base_foot.trans[2] =
        latest_tf_msg_base_foot->transforms[0].transform.translation.z;
    combined_data.tf_data_base_foot.rot[0] =
        latest_tf_msg_base_foot->transforms[0].transform.rotation.x;
    combined_data.tf_data_base_foot.rot[1] =
        latest_tf_msg_base_foot->transforms[0].transform.rotation.y;
    combined_data.tf_data_base_foot.rot[2] =
        latest_tf_msg_base_foot->transforms[0].transform.rotation.z;
    combined_data.tf_data_base_foot.rot[3] =
        latest_tf_msg_base_foot->transforms[0].transform.rotation.w;

    // Tf_base_link
    combined_data.tf_data_base_link.stamp =
        (double)latest_tf_msg_base_link->transforms[0].header.stamp.sec +
        (double)latest_tf_msg_base_link->transforms[0].header.stamp.nsec / 1e9;
    strcpy(combined_data.tf_data_base_link.frame_id,
           latest_tf_msg_base_link->transforms[0].header.frame_id.c_str());
    strcpy(combined_data.tf_data_base_link.child_frame_id,
           latest_tf_msg_base_link->transforms[0].child_frame_id.c_str());
    combined_data.tf_data_base_link.trans[0] =
        latest_tf_msg_base_link->transforms[0].transform.translation.x;
    combined_data.tf_data_base_link.trans[1] =
        latest_tf_msg_base_link->transforms[0].transform.translation.y;
    combined_data.tf_data_base_link.trans[2] =
        latest_tf_msg_base_link->transforms[0].transform.translation.z;
    combined_data.tf_data_base_link.rot[0] =
        latest_tf_msg_base_link->transforms[0].transform.rotation.x;
    combined_data.tf_data_base_link.rot[1] =
        latest_tf_msg_base_link->transforms[0].transform.rotation.y;
    combined_data.tf_data_base_link.rot[2] =
        latest_tf_msg_base_link->transforms[0].transform.rotation.z;
    combined_data.tf_data_base_link.rot[3] =
        latest_tf_msg_base_link->transforms[0].transform.rotation.w;

    // Odom
    combined_data.odom_data.stamp = (double)latest_odom_msg->header.stamp.sec +
                                    (double)latest_odom_msg->header.stamp.nsec / 1e9;
    strcpy(combined_data.odom_data.frame_id, latest_odom_msg->header.frame_id.c_str());
    strcpy(combined_data.odom_data.child_frame_id, latest_odom_msg->child_frame_id.c_str());
    combined_data.odom_data.pose_position[0] = latest_odom_msg->pose.pose.position.x;
    combined_data.odom_data.pose_position[1] = latest_odom_msg->pose.pose.position.y;
    combined_data.odom_data.pose_position[2] = latest_odom_msg->pose.pose.position.z;
    combined_data.odom_data.pose_orientation[0] = latest_odom_msg->pose.pose.orientation.x;
    combined_data.odom_data.pose_orientation[1] = latest_odom_msg->pose.pose.orientation.y;
    combined_data.odom_data.pose_orientation[2] = latest_odom_msg->pose.pose.orientation.z;
    combined_data.odom_data.pose_orientation[3] = latest_odom_msg->pose.pose.orientation.w;
    std::memcpy(combined_data.odom_data.pose_cov, latest_odom_msg->pose.covariance.data(),
                sizeof(double) * 36);
    combined_data.odom_data.twist_linear[0] = latest_odom_msg->twist.twist.linear.x;
    combined_data.odom_data.twist_linear[1] = latest_odom_msg->twist.twist.linear.y;
    combined_data.odom_data.twist_linear[2] = latest_odom_msg->twist.twist.linear.z;
    combined_data.odom_data.twist_angular[0] = latest_odom_msg->twist.twist.angular.x;
    combined_data.odom_data.twist_angular[1] = latest_odom_msg->twist.twist.angular.y;
    combined_data.odom_data.twist_angular[2] = latest_odom_msg->twist.twist.angular.z;
    std::memcpy(combined_data.odom_data.twist_cov, latest_odom_msg->twist.covariance.data(),
                sizeof(double) * 36);

    // Cam Image
    if (latest_camera_msg) {
        // Zeitstempel übernehmen
        combined_data.camera_data.stamp = (double)latest_camera_msg->header.stamp.sec +
                                          (double)latest_camera_msg->header.stamp.nsec / 1e9;

        // Frame-ID übernehmen
        strncpy(combined_data.camera_data.frame_id, latest_camera_msg->header.frame_id.c_str(),
                sizeof(combined_data.camera_data.frame_id));

        // Encoding übernehmen (z. B. "jpeg" oder was auch immer original verwendet wurde)
        strncpy(combined_data.camera_data.encoding, latest_camera_msg->format.c_str(),
                sizeof(combined_data.camera_data.encoding));

        // Komprimierte Bilddaten kopieren
        size_t copy_size =
            std::min(latest_camera_msg->data.size(), sizeof(combined_data.camera_data.data));
        memcpy(combined_data.camera_data.data, latest_camera_msg->data.data(), copy_size);
    }

    // Send data
    ssize_t bytes_sent = send(g_client_socket, &combined_data, sizeof(CombinedData), 0);

    if (bytes_sent < 0) {
        perror("Send failed");
        g_client_socket = -1; // Drop the socket
    }
}

int setupTCPServer(int port) { // this function listens for incomming TCP
                               // connection requests on port and returns a fd
                               // of the socket used fo TCP communication
    int server_fd = socket(AF_INET, SOCK_STREAM,
                           0); // socket filedescriptor for IPv4 TCP socket
    if (server_fd == 0) {
        perror("Socket failed");
        return -1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port); // host to network byteorder

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        return -1;
    }

    if (listen(server_fd, 2) < 0) { // queue for pending connections has length 2
        perror("Listen failed");
        return -1;
    }

    std::cout << "Waiting for TCP connection on port " << port << "..." << std::endl;

    int addrlen = sizeof(address);
    int client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
    if (client_socket < 0) {
        perror("Accept failed");
        return -1;
    }

    std::cout << "Client connected!" << std::endl;
    return client_socket;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "minirosbridge_listener");
    ros::NodeHandle nh;

    init_tf_msgs();

    // Wait for TCP connection
    g_client_socket = setupTCPServer(2077); // You can change the port if needed
    if (g_client_socket < 0) {
        ROS_ERROR("TCP server failed to start.");
        return 1;
    }

    ros::Subscriber laser_sub = nh.subscribe("/laserscan", 10, laserCallback);
    ros::Subscriber tf_sub = nh.subscribe("/tf", 10, tfCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);
    ros::Subscriber cam_sub = nh.subscribe("/image_raw/compressed", 10, cameraCallback);

    // 20Hz timer for processing
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), processDataCallback);

    ros::spin();

    close(g_client_socket);
    return 0;
}

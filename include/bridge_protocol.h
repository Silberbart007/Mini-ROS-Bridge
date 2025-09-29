#pragma once
#include <cstdint>

#pragma pack(push, 1)

struct LaserScanData {
    double stamp;
    char frame_id[64];
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[541];
    float intensities[541];
};

struct TFData {
    double stamp;
    char frame_id[64];
    char child_frame_id[64];
    float trans[3];
    float rot[4];
};

struct OdomData {
    double stamp;
    char frame_id[64];
    char child_frame_id[64];
    double pose_position[3];
    double pose_orientation[4];
    double pose_cov[36];
    double twist_linear[3];
    double twist_angular[3];
    double twist_cov[36];
};

struct CameraData {
  double stamp;
  char frame_id[64];
  char encoding[32];
  uint8_t data[1024*30];
};

struct CombinedData {
    LaserScanData laser_data;
    TFData tf_data_odom;
    TFData tf_data_base_foot;
    TFData tf_data_base_link;
    OdomData odom_data;
    CameraData camera_data;
};

struct VelocityData {
    double linear_x;
    double linear_y;
    double angular_z;
};

#pragma pack(pop)

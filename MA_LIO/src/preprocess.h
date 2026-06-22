#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64, ROBOSENSE}; //{1, 2, 3, 4}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint8_t  ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint8_t, ring, ring)
)

// RoboSense clouds published by 10xCode realtime_perception (PipelineOrchestrator::buildCloud).
// Layout: x,y,z,intensity,tag,time — all FLOAT32. `time` is the per-point acquisition
// time in SECONDS, relative to the message header.stamp (no `ring` field).
namespace robosense_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float tag;
      float time;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, tag, tag)
    (float, time, time)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out, int lidar);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  float time_unit_scale;
  int time_unit;
  std::vector<int> point_filter_num;
  std::vector<int> N_SCANS;
  std::vector<int> lid_type;
  double blind;
  bool given_offset_time;
  double maximum_time;


  private:
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar);
  void robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar);

};

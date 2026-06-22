#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : blind(0.01)
{
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out, int lidar)
{

  switch (time_unit)
  {
  case SEC:
    time_unit_scale = 1.e3f;
    break;
  case MS:
    time_unit_scale = 1.f;
    break;
  case US:
    time_unit_scale = 1.e-3f;
    break;
  case NS:
    time_unit_scale = 1.e-6f;
    break;
  default:
    time_unit_scale = 1.f;
    break;
  }

  switch (lid_type[lidar])
  {
  case OUST64:
    oust64_handler(msg, lidar);
    break;

  case VELO16:
    velodyne_handler(msg, lidar);
    break;

  case ROBOSENSE:
    robosense_handler(msg, lidar);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  maximum_time = -9999;
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num[lidar] != 0)
      continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < (blind * blind))
      continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale * 1.e-9f; // curvature unit: ms
    if (maximum_time < added_pt.curvature)
    {
      maximum_time = added_pt.curvature;
    }
    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf.reserve(plsize);

  /*** These variables only works when no point timestamps given ***/
  std::vector<bool> is_first(N_SCANS[lidar], true);
  std::vector<double> yaw_fp(N_SCANS[lidar], 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS[lidar], 0.0);  // yaw of last scan point
  std::vector<float> time_last(N_SCANS[lidar], 0.0); // last offset time
  /*****************************************************************/

  if (pl_orig.points[plsize - 1].time > 0)
  {
    given_offset_time = true;
  }
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  maximum_time = -9999;
  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;
 
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = pl_orig.points[i].time * time_unit_scale; 

    if (i % point_filter_num[lidar] == 0)
    {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
      {
        if (maximum_time < added_pt.curvature)
          maximum_time = added_pt.curvature;
        pl_surf.points.push_back(added_pt);
      }
    }
  }
}

void Preprocess::robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);

  maximum_time = -9999;
  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num[lidar] != 0)
      continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < (blind * blind))
      continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    // realtime_perception publishes per-point `time` in SECONDS relative to
    // header.stamp; MA-LIO stores curvature in milliseconds.
    added_pt.curvature = pl_orig.points[i].time * 1000.0f; // curvature unit: ms
    // The window reference stamp can slightly post-date the first point(s);
    // clamp to keep curvature non-negative for downstream undistortion.
    if (added_pt.curvature < 0.f)
      added_pt.curvature = 0.f;
    if (maximum_time < added_pt.curvature)
      maximum_time = added_pt.curvature;
    pl_surf.points.push_back(added_pt);
  }
}

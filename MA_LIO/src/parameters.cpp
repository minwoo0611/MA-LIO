#include "parameters.h"

bool path_en = true, scan_pub_en = false, dense_pub_en = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true;
int NUM_MAX_ITERATIONS = 0, lid_num, pcd_save_interval = -1, add_point_size = 0, kdtree_delete_counter = 0, feats_down_size = 0, pcd_index = 0;
int livox_num = 0, spin_num = 0;
std::string map_file_path, imu_topic, root_dir = ROOT_DIR;
std::vector<std::string> lid_topic;
std::vector<int> lid_type, N_SCANS, point_filter_num;
std::vector<double> extrinT, extrinR;
double time_diff_lidar_to_imu = 0.0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0, cube_len = 0, range_min, range_max;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
float plane_th, DET_RANGE = 300.0f;

double cov_threshold, point_cov_max, point_cov_min, plane_cov_max, plane_cov_min, localize_cov_max, localize_cov_min, localize_thresh_max, localize_thresh_min;
shared_ptr<Preprocess> p_pre;

namespace
{
// ROS2 parameter types are limited (bool, int64, double, string + their arrays).
// These helpers declare-with-default then read, casting int64->int and double->float
// where the MA-LIO globals use narrower types.
template <typename T>
T get_param(rclcpp::Node *node, const std::string &name, const T &def)
{
  node->declare_parameter<T>(name, def);
  T val = def;
  node->get_parameter(name, val);
  return val;
}

int get_int(rclcpp::Node *node, const std::string &name, int def)
{
  return static_cast<int>(get_param<int64_t>(node, name, static_cast<int64_t>(def)));
}

float get_float(rclcpp::Node *node, const std::string &name, float def)
{
  return static_cast<float>(get_param<double>(node, name, static_cast<double>(def)));
}

std::vector<int> get_int_vec(rclcpp::Node *node, const std::string &name)
{
  auto tmp = get_param<std::vector<int64_t>>(node, name, std::vector<int64_t>());
  return std::vector<int>(tmp.begin(), tmp.end());
}
}  // namespace

void readParameters(rclcpp::Node *node)
{
  p_pre.reset(new Preprocess());

  path_en = get_param<bool>(node, "publish.path_en", true);
  scan_pub_en = get_param<bool>(node, "publish.scan_publish_en", true);
  dense_pub_en = get_param<bool>(node, "publish.dense_publish_en", true);
  NUM_MAX_ITERATIONS = get_int(node, "max_iteration", 4);
  map_file_path = get_param<std::string>(node, "map_file_path", std::string(""));

  lid_topic = get_param<std::vector<std::string>>(node, "common.lid_topic", std::vector<std::string>());
  imu_topic = get_param<std::string>(node, "common.imu_topic", std::string("/livox/imu"));
  lid_num = get_int(node, "common.lid_num", 2);
  lid_type = get_int_vec(node, "common.lid_type");
  N_SCANS = get_int_vec(node, "common.N_SCANS");
  point_filter_num = get_int_vec(node, "common.point_filter_num");
  time_sync_en = get_param<bool>(node, "common.time_sync_en", false);
  time_diff_lidar_to_imu = get_param<double>(node, "common.time_offset_lidar_to_imu", 0.0);

  filter_size_surf_min = get_param<double>(node, "filter_size_surf", 0.5);
  filter_size_map_min = get_param<double>(node, "filter_size_map", 0.5);

  cube_len = get_param<double>(node, "cube_side_length", 200.0);
  plane_th = get_float(node, "plane_th", 0.1f);
  range_min = get_param<double>(node, "range_min", 0.0);
  range_max = get_param<double>(node, "range_max", 1.0);

  extrinT = get_param<std::vector<double>>(node, "mapping.extrinsic_T", std::vector<double>());
  extrinR = get_param<std::vector<double>>(node, "mapping.extrinsic_R", std::vector<double>());
  DET_RANGE = get_float(node, "mapping.det_range", 300.0f);
  fov_deg = get_param<double>(node, "mapping.fov_degree", 180.0);
  gyr_cov = get_param<double>(node, "mapping.gyr_cov", 0.1);
  acc_cov = get_param<double>(node, "mapping.acc_cov", 0.1);
  b_gyr_cov = get_param<double>(node, "mapping.b_gyr_cov", 0.0001);
  b_acc_cov = get_param<double>(node, "mapping.b_acc_cov", 0.0001);
  p_pre->blind = get_param<double>(node, "preprocess.blind", 0.01);
  p_pre->time_unit = get_int(node, "preprocess.timestamp_unit", US);
  extrinsic_est_en = get_param<bool>(node, "mapping.extrinsic_est_en", true);
  pcd_save_en = get_param<bool>(node, "pcd_save.pcd_save_en", false);
  pcd_save_interval = get_int(node, "pcd_save.interval", -1);

  cov_threshold = get_param<double>(node, "cov_threshold", 0.3);
  point_cov_max = get_param<double>(node, "uncertainty.point_cov_max", 0.002);
  point_cov_min = get_param<double>(node, "uncertainty.point_cov_min", 0.0005);
  plane_cov_max = get_param<double>(node, "uncertainty.plane_cov_max", 1.0);
  plane_cov_min = get_param<double>(node, "uncertainty.plane_cov_min", 0.7);
  localize_cov_max = get_param<double>(node, "uncertainty.localize_cov_max", 2.0);
  localize_cov_min = get_param<double>(node, "uncertainty.localize_cov_min", 0.4);
  localize_thresh_max = get_param<double>(node, "uncertainty.localize_thresh_max", 0.8);
  localize_thresh_min = get_param<double>(node, "uncertainty.localize_thresh_min", 0.3);
}

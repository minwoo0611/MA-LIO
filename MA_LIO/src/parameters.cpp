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

void readParameters(ros::NodeHandle &nh)
{
  p_pre.reset(new Preprocess());
  nh.param<bool>("publish/path_en", path_en, true);
  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
  nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  nh.param<string>("map_file_path", map_file_path, "");

  nh.param<vector<string>>("common/lid_topic", lid_topic, vector<string>());
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<int>("common/lid_num", lid_num, 2);
  nh.param<vector<int>>("common/lid_type", lid_type, vector<int>());
  nh.param<vector<int>>("common/N_SCANS", N_SCANS, vector<int>());
  nh.param<vector<int>>("common/point_filter_num", point_filter_num, vector<int>());
  nh.param<bool>("common/time_sync_en", time_sync_en, false);
  nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);

  nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", filter_size_map_min, 0.5);

  nh.param<double>("cube_side_length", cube_len, 200);
  nh.param<float>("plane_th", plane_th, 0.1);
  nh.param<double>("range_min", range_min, 0);
  nh.param<double>("range_max", range_max, 1);

  nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
  nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
  nh.param<double>("mapping/fov_degree", fov_deg, 180);
  nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
  nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);

  nh.param<double>("cov_threshold", cov_threshold, 0.3);
  nh.param<double>("uncertainty/point_cov_max", point_cov_max, 0.002);
  nh.param<double>("uncertainty/point_cov_min", point_cov_min, 0.0005);
  nh.param<double>("uncertainty/plane_cov_max", plane_cov_max, 1);
  nh.param<double>("uncertainty/plane_cov_min", plane_cov_min, 0.7);
  nh.param<double>("uncertainty/localize_cov_max", localize_cov_max, 2);
  nh.param<double>("uncertainty/localize_cov_min", localize_cov_min, 0.4);
  nh.param<double>("uncertainty/localize_thresh_max", localize_thresh_max, 0.8);
  nh.param<double>("uncertainty/localize_thresh_min", localize_thresh_min, 0.3);
}
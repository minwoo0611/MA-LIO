#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"

extern bool path_en, scan_pub_en, dense_pub_en, pcd_save_en, time_sync_en, extrinsic_est_en;
extern int NUM_MAX_ITERATIONS, lid_num, pcd_save_interval, add_point_size, kdtree_delete_counter, feats_down_size, pcd_index;
extern int livox_num, spin_num;
extern std::string map_file_path, imu_topic, root_dir;
extern std::vector<std::string> lid_topic;
extern std::vector<int> lid_type, N_SCANS, point_filter_num;
extern std::vector<double> extrinT, extrinR;
extern double time_diff_lidar_to_imu, filter_size_surf_min, filter_size_map_min, fov_deg, cube_len, range_min, range_max;
extern double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
extern float plane_th, DET_RANGE;

extern double cov_threshold, point_cov_max, point_cov_min, plane_cov_max, plane_cov_min, localize_cov_max, localize_cov_min, localize_thresh_max, localize_thresh_min;
extern shared_ptr<Preprocess> p_pre;
void readParameters(ros::NodeHandle &n);
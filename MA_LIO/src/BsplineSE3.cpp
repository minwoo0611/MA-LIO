/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "BsplineSE3.h"

using namespace ov_core;

void BsplineSE3::feed_trajectory(std::vector<Eigen::VectorXd> traj_points) {
  time_feed = omp_get_wtime();
  // Find the average frequency to use as our uniform timesteps
  double sumdt = 0;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    sumdt += traj_points.at(i + 1)(0) - traj_points.at(i)(0);
  }
  dt = sumdt / (traj_points.size() - 1);
  dt = (dt < 0.01) ? 0.01 : 0.01;

  // convert all our trajectory points into SE(3) matrices
  // we are given [timestamp, p_IinG, q_GtoI]
  AlignedEigenMat4d trajectory_points;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    Eigen::Matrix4d T_IinG = Eigen::Matrix4d::Identity();
    T_IinG.block(0, 0, 3, 3) = quat_2_Rot(traj_points.at(i).block(4, 0, 4, 1)).transpose();
    T_IinG.block(0, 3, 3, 1) = traj_points.at(i).block(1, 0, 3, 1);
    trajectory_points.insert({traj_points.at(i)(0), T_IinG});
  }

  // Get the oldest timestamp
  double timestamp_min = INFINITY;
  double timestamp_max = -INFINITY;
  for (const auto &pose : trajectory_points) {
    if (pose.first <= timestamp_min) {
      timestamp_min = pose.first;
    }
    if (pose.first >= timestamp_min) {
      timestamp_max = pose.first;
    }
  }

  // then create spline control points
  double timestamp_curr = timestamp_min;
  while (true) {

    // Get bounding posed for the current time
    double t0, t1;
    Eigen::Matrix4d pose0, pose1;
    bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0, pose0, t1, pose1);

    // If we didn't find a bounding pose, then that means we are at the end of the dataset
    // Thus break out of this loop since we have created our max number of control points
    if (!success)
      break;

    // Linear interpolation and append to our control points
    double lambda = (timestamp_curr - t0) / (t1 - t0);
    Eigen::Matrix4d pose_interp = exp_se3(lambda * log_se3(pose1 * Inv_se3(pose0))) * pose0;
    control_points.insert({timestamp_curr, pose_interp});
    timestamp_curr += dt;
  }

  // The start time of the system is two dt in since we need at least two older control points
  timestamp_start = timestamp_min + 2 * dt;

}

bool BsplineSE3::get_pose(double timestamp, Eigen::Quaterniond &q_GtoI, Eigen::Vector3d &p_IinG) {
  time_feed = omp_get_wtime();
  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  Eigen::Matrix3d R_GtoI;
  bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // Return failure if we can't get bounding poses
  if (!success) {
    R_GtoI.setIdentity();
    p_IinG.setZero();
    return false;
  }
  // Our De Boor-Cox matrix scalars
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);

  // Calculate interpolated poses
  Eigen::Matrix4d A0 = exp_se3(b0 * log_se3(Inv_se3(pose0) * pose1));
  Eigen::Matrix4d A1 = exp_se3(b1 * log_se3(Inv_se3(pose1) * pose2));
  Eigen::Matrix4d A2 = exp_se3(b2 * log_se3(Inv_se3(pose2) * pose3));

  // Finally get the interpolated pose
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3);
  q_GtoI = R_GtoI;
  p_IinG = pose_interp.block(0, 3, 3, 1);
  total_time += (omp_get_wtime() - time_feed);

  return true;
}


bool BsplineSE3::find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                     Eigen::Matrix4d &pose1) {

  // Set the default values
  t0 = -1;
  t1 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();

  // Find the bounding poses
  bool found_older = false;
  bool found_newer = false;

  // Find the bounding poses for interpolation.
  auto lower_bound = poses.lower_bound(timestamp); // Finds timestamp or next(timestamp) if not available
  auto upper_bound = poses.upper_bound(timestamp); // Finds next(timestamp)

  if (lower_bound != poses.end()) {
    // Check that the lower bound is the timestamp.
    // If not then we move iterator to previous timestamp so that the timestamp is bounded
    if (lower_bound->first == timestamp) {
      found_older = true;
    } else if (lower_bound != poses.begin()) {
      --lower_bound;
      found_older = true;
    }
  }

  if (upper_bound != poses.end()) {
    found_newer = true;
  }

  // If we found the older one, set it
  if (found_older) {
    t0 = lower_bound->first;
    pose0 = lower_bound->second;
  }

  // If we found the newest one, set it
  if (found_newer) {
    t1 = upper_bound->first;
    pose1 = upper_bound->second;
  }

  // Assert the timestamps
  if (found_older && found_newer)
    assert(t0 < t1);

  // Return true if we found both bounds
  return (found_older && found_newer);
}

bool BsplineSE3::find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                              double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                              Eigen::Matrix4d &pose3) {

  // Set the default values
  t0 = -1;
  t1 = -1;
  t2 = -1;
  t3 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();
  pose2 = Eigen::Matrix4d::Identity();
  pose3 = Eigen::Matrix4d::Identity();

  // Get the two bounding poses
  bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

  // Return false if this was a failure
  if (!success)
    return false;

  // Now find the poses that are below and above
  auto iter_t1 = poses.find(t1);
  auto iter_t2 = poses.find(t2);

  // Check that t1 is not the first timestamp
  if (iter_t1 == poses.begin()) {
    return false;
  }
 
  // Move the older pose backwards in time
  // Move the newer one forwards in time
  auto iter_t0 = --iter_t1;
  auto iter_t3 = ++iter_t2;

  // Check that it is valid
  if (iter_t3 == poses.end()) {
    return false;
  }

  // Set the oldest one
  t0 = iter_t0->first;
  pose0 = iter_t0->second;

  // Set the newest one
  t3 = iter_t3->first;
  pose3 = iter_t3->second;

  // Assert the timestamps
  if (success) {
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
  }

  // Return true if we found all four bounding poses
  return success;
}


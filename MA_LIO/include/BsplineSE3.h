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

#ifndef OV_CORE_BSPLINESE3_H
#define OV_CORE_BSPLINESE3_H

#include <Eigen/Eigen>
#include <map>
#include <vector>
#include <omp.h>
#include "quat_ops.h"

namespace ov_core {

class BsplineSE3 {

public:
  /**
   * @brief Default constructor
   */
    double total_time = 0;
  double time_feed;
  BsplineSE3() {}

  /**
   * @brief Will feed in a series of poses that we will then convert into control points.
   *
   * Our control points need to be uniformly spaced over the trajectory, thus given a trajectory we will
   * uniformly sample based on the average spacing between the pose points specified.
   *
   * @param traj_points Trajectory poses that we will convert into control points (timestamp(s), q_GtoI, p_IinG)
   */
  void feed_trajectory(std::vector<Eigen::VectorXd> traj_points);

  /**
   * @brief Gets the orientation and position at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @return False if we can't find it
   */
  bool get_pose(double timestamp, Eigen::Quaterniond &R_GtoI, Eigen::Vector3d &p_IinG);

  /**
   * @brief Gets the angular and linear velocity at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @param w_IinI Angular velocity in the inertial frame
   * @param v_IinG Linear velocity in the global frame
   * @return False if we can't find it
   */

  /// Returns the simulation start time that we should start simulating from
  double get_start_time() { return total_time; }

protected:
  /// Uniform sampling time for our control points
  double dt;

  /// Start time of the system
  double timestamp_start;

  /// Type defintion of our aligned eigen4d matrix: https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
  typedef std::map<double, Eigen::Matrix4d, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4d>>>
      AlignedEigenMat4d;

  /// Our control SE3 control poses (R_ItoG, p_IinG)
  AlignedEigenMat4d control_points;

  /**
   * @brief Will find the two bounding poses for a given timestamp.
   *
   * This will loop through the passed map of poses and find two bounding poses.
   * If there are no bounding poses then this will return false.
   *
   * @param timestamp Desired timestamp we want to get two bounding poses of
   * @param poses Map of poses and timestamps
   * @param t0 Timestamp of the first pose
   * @param pose0 SE(3) pose of the first pose
   * @param t1 Timestamp of the second pose
   * @param pose1 SE(3) pose of the second pose
   * @return False if we are unable to find bounding poses
   */
  static bool find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                  Eigen::Matrix4d &pose1);

  /**
   * @brief Will find two older poses and two newer poses for the current timestamp
   *
   * @param timestamp Desired timestamp we want to get four bounding poses of
   * @param poses Map of poses and timestamps
   * @param t0 Timestamp of the first pose
   * @param pose0 SE(3) pose of the first pose
   * @param t1 Timestamp of the second pose
   * @param pose1 SE(3) pose of the second pose
   * @param t2 Timestamp of the third pose
   * @param pose2 SE(3) pose of the third pose
   * @param t3 Timestamp of the fourth pose
   * @param pose3 SE(3) pose of the fourth pose
   * @return False if we are unable to find bounding poses
   */
  static bool find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                           double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                           Eigen::Matrix4d &pose3);
};

} // namespace ov_core

#endif // OV_CORE_BSPLINESE3_H


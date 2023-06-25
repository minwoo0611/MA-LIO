#include <cmath>
#include <omp.h>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"
#include "BsplineSE3.h"
#include "associate_uct.hpp"
#include <typeinfo>
/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_sort(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, std::vector<PointCloudXYZI::Ptr> &cur_pcl_un_);

  double first_lidar_time;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;

  std::vector<Eigen::VectorXd> traj_points;
  Eigen::VectorXd traj_point;
  std::vector<pair<pair<double, M6D>, input_ikfom>> imu_cov;
  M6D state_cov;
  input_ikfom last_in;

private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, std::vector<PointCloudXYZI::Ptr> &cur_pcl_un_);

  PointCloudXYZI::Ptr cur_pcl_un_;
  PointCloudXYZI::Ptr cur_pcl_un_2_;
  PointCloudXYZI::Ptr cur_pcl_un_3_;

  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;

  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
  int num = 0;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;
  traj_point.resize(8);
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);

  // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg = mean_gyr;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  
  for(int i = 6; i < init_P.rows(); i++){
    if(i < init_P.rows() - 8)
      init_P(i,i) = 0.000001;
    else if(i < init_P.rows()-5)
      init_P(i,i) = 0.0001;
    else if(i < init_P.rows()-2)
      init_P(i,i) = 0.001;
    else
      init_P(i,i) = 0.00001;
  }

  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

  Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
  Q.block<3, 3>(3, 3).diagonal() = cov_acc;
  Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
  Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, std::vector<PointCloudXYZI::Ptr> &feats_undistort_vec)
{

  shared_ptr<ov_core::BsplineSE3> spline_traj(new ov_core::BsplineSE3());

  input_ikfom in;
  kf_state.propagate_cov();
  kf_state.prepareBack();
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  auto c_imu = meas.imu_cont;
  v_imu.push_front(last_imu_);
    int lid_num = meas.lidar_multi.size();
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time[0];
  const double &pcl_end_time = meas.lidar_end_time[lid_num - 1];

  /*** sort point clouds by offset time ***/
  for (int num = 0; num < lid_num; num++)
  {
    *(feats_undistort_vec[num]) = *(meas.lidar_multi[lid_num - num - 1]);
    sort(feats_undistort_vec[num]->begin(), feats_undistort_vec[num]->end(), time_sort);
  }
   /*** Delete the previous pose ***/
  if (!traj_points.empty())
  {
    while (traj_points[0][0] + 0.2 < pcl_beg_time)
    {
      if (traj_points.empty())
        break;
      traj_points.erase(traj_points.begin());
      imu_cov.erase(imu_cov.begin());
      IMUpose.erase(IMUpose.begin());
      if (traj_points.empty())
        break;
    }
  }

  if (!traj_points.empty())
  {
    while (traj_points[traj_points.size() - 1][0] > imu_beg_time)
    {
      if (traj_points.empty())
        break;
      traj_points.pop_back();
      imu_cov.pop_back();
      IMUpose.pop_back();
      if (traj_points.empty())
        break;
    }
  }

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  if (last_lidar_end_time_ != 0)
  {
    traj_point << last_lidar_end_time_, imu_state.pos(0), imu_state.pos(1), imu_state.pos(2), imu_state.rot.x(), imu_state.rot.y(), imu_state.rot.z(), imu_state.rot.w();
    traj_points.push_back(traj_point);
    IMUpose.push_back(set_pose6d(last_lidar_end_time_, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    state_cov = kf_state.get_U();
    imu_cov.push_back(make_pair(make_pair(last_lidar_end_time_, state_cov), last_in));
  }

  /*** back propagation from last optimize ***/
  state_ikfom back_state;
  for (int i = imu_cov.size() - 1; i > 1; i--)
  {
    double dt = imu_cov[i - 1].first.first - imu_cov[i].first.first;
    back_state = kf_state.back_predict(dt, Q, imu_cov[i].second);
    state_cov = kf_state.get_U();

    traj_points[i - 1][1] = back_state.pos(0);
    traj_points[i - 1][2] = back_state.pos(1);
    traj_points[i - 1][3] = back_state.pos(2);

    traj_points[i - 1][4] = back_state.rot.x();
    traj_points[i - 1][5] = back_state.rot.y();
    traj_points[i - 1][6] = back_state.rot.z();
    traj_points[i - 1][7] = back_state.rot.w();

    imu_cov[i - 1].first.second = state_cov;
  }
  

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  Eigen::VectorXd imu_meas(7);
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    Eigen::VectorXd traj_point(8);
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if (head->header.stamp.toSec() < last_lidar_end_time_)
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
    else
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);

    for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i]; // gravity exclude
    }

    traj_point << tail->header.stamp.toSec(), imu_state.pos(0), imu_state.pos(1), imu_state.pos(2), imu_state.rot.x(), imu_state.rot.y(), imu_state.rot.z(), imu_state.rot.w();
    traj_points.push_back(traj_point);

    state_cov = kf_state.getUncertainty();

    imu_cov.push_back(make_pair(make_pair(tail->header.stamp.toSec(), state_cov), in));
    IMUpose.push_back(set_pose6d(tail->header.stamp.toSec(), acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    imu_meas << tail->header.stamp.toSec(), tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z, tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z;
  }

  kf_state.propagate_cov();
  kf_state.prepareCont();
  
  /* Continous SLAM */
  for (auto it_imu = c_imu.begin(); it_imu < (c_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();

    in.acc = acc_avr;
    in.gyro = angvel_avr;

    state_ikfom x_cont;
    x_cont = kf_state.predict_cont(dt, Q, in);

    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);

    for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i]; 
    }

    traj_point << tail->header.stamp.toSec(), x_cont.pos(0), x_cont.pos(1), x_cont.pos(2), x_cont.rot.x(), x_cont.rot.y(), x_cont.rot.z(), x_cont.rot.w();
    traj_points.push_back(traj_point);
    IMUpose.push_back(set_pose6d(tail->header.stamp.toSec(), acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    state_cov = kf_state.get_U();
    imu_cov.push_back(make_pair(make_pair(tail->header.stamp.toSec(), state_cov), in));
  }

  spline_traj->feed_trajectory(traj_points);

  /*** calculated the pos and attitude prediction at the frame-end ***/
  c_imu.pop_front();
  double ratio = (pcl_end_time - imu_meas(0)) / (c_imu.front()->header.stamp.toSec() - imu_meas(0));

  angvel_avr << ratio * imu_meas(1) + (1 - ratio) * c_imu.front()->angular_velocity.x,
      ratio * imu_meas(2) + (1 - ratio) * c_imu.front()->angular_velocity.y,
      ratio * imu_meas(3) + (1 - ratio) * c_imu.front()->angular_velocity.z;
  acc_avr << ratio * imu_meas(4) + (1 - ratio) * c_imu.front()->linear_acceleration.x,
      ratio * imu_meas(5) + (1 - ratio) * c_imu.front()->linear_acceleration.y,
      ratio * imu_meas(6) + (1 - ratio) * c_imu.front()->linear_acceleration.z;

  acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); 

  in.acc = acc_avr;
  in.gyro = angvel_avr;
  dt = (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();
  last_in = in;
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  std::vector<V3D> lt_imu_frame_trans;
  std::vector<Eigen::Quaterniond> lt_imu_frame_quat;
  std::vector<Pose> extrinsic;
  std::vector<Pose> lt_lidar_frame;
  std::vector<std::vector<Pose>> uncertainty;
  std::vector<V3D> extrinsic_trans;
  std::vector<Eigen::Quaterniond> extrinsic_quat;

  lt_imu_frame_trans.resize(lid_num);
  lt_imu_frame_quat.resize(lid_num);
  extrinsic.resize(lid_num);
  uncertainty.resize(lid_num);
  lt_lidar_frame.resize(lid_num);

  bool spline_flag = spline_traj->get_pose(pcl_end_time, lt_imu_frame_quat[0], lt_imu_frame_trans[0]);
  if (spline_flag == 0)
  {
    lt_imu_frame_quat[0].x() = imu_state.rot.x();
    lt_imu_frame_quat[0].y() = imu_state.rot.y();
    lt_imu_frame_quat[0].z() = imu_state.rot.z();
    lt_imu_frame_quat[0].w() = imu_state.rot.w();

    lt_imu_frame_trans[0] = imu_state.pos;
  }
  else
    kf_state.change_pos(lt_imu_frame_quat[0], lt_imu_frame_trans[0]);

  for(int num = 0; num < lid_num; num++){
    V3D extrinsic_t = *(static_cast<MTK::vect<3, double>*>(imu_state.vect_state_ptr[1 + num]));
    Eigen::Quaterniond extrinsic_q = *(static_cast<MTK::SO3<double>*>(imu_state.SO3_state_ptr[1 + num]));
    extrinsic_trans.push_back(extrinsic_t);
    extrinsic_quat.push_back(extrinsic_q);      
    PoseInitial(extrinsic[num], extrinsic_trans[num], extrinsic_quat[num], kf_state.getExtrinsicUncertainty(num));
  }
  PoseInitial(lt_lidar_frame[0], lt_imu_frame_trans[0], lt_imu_frame_quat[0], kf_state.getUncertainty());

  for(int num = 0; num < lid_num; num++){
    int cov_pointer = imu_cov.size() - 1;
    int idx = -1;
    
    while (true)
    {
      if (imu_cov[cov_pointer].first.first > meas.lidar_end_time[lid_num - num - 1])
      {
        cov_pointer = cov_pointer - 1;
      }
      else
      {
        cov_pointer = cov_pointer + 1;
        break;
      }
    }

    if(num != 0){
      spline_flag = spline_traj -> get_pose(meas.lidar_end_time[lid_num - num - 1], lt_imu_frame_quat[num], lt_imu_frame_trans[num]);
      PoseInitial(lt_lidar_frame[num], lt_imu_frame_trans[num], lt_imu_frame_quat[num], imu_cov[cov_pointer].first.second);
    }
      
    
    auto it_pcl = feats_undistort_vec[num]->points.end() - 1;
    for (; it_pcl != feats_undistort_vec[num]->points.begin(); it_pcl--)
      {
        V3D pt_imu_frame_trans;
        Eigen::Quaterniond pt_imu_frame_quat;
        Pose pt_imu_frame;

        double point_t = it_pcl->curvature / double(1000) + meas.lidar_beg_time[lid_num - num - 1];
        spline_flag = spline_traj->get_pose(point_t, pt_imu_frame_quat, pt_imu_frame_trans); // point pose
        if (imu_cov[cov_pointer].first.first > point_t)
        {
          cov_pointer = cov_pointer - 1;
          Pose pos_calculated;
          PoseInitial(pt_imu_frame, pt_imu_frame_trans, pt_imu_frame_quat, imu_cov[cov_pointer + 1].first.second);
          compoundPoseWithCov(pt_imu_frame, pt_imu_frame.cov_, extrinsic[num], extrinsic[num].cov_, pos_calculated, pos_calculated.cov_, 2);
          compoundInvPoseWithCov(lt_lidar_frame[num], lt_lidar_frame[num].cov_, pos_calculated, pos_calculated.cov_, pos_calculated, pos_calculated.cov_, 2);
          compoundInvPoseWithCov(extrinsic[num], extrinsic[num].cov_, pos_calculated, pos_calculated.cov_, pos_calculated, pos_calculated.cov_, 2);
          uncertainty[num].push_back(pos_calculated);
          idx += 1;
        }

        if (spline_flag != 0)
        {
          V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
          V3D T_ei = pt_imu_frame_trans - lt_imu_frame_trans[num];
          V3D P_compensate = extrinsic_quat[num].conjugate() * (lt_imu_frame_quat[num].conjugate() * (pt_imu_frame_quat * (extrinsic_quat[num] * P_i + extrinsic_trans[num]) + T_ei) - extrinsic_trans[num]);
          it_pcl->x = P_compensate(0);
          it_pcl->y = P_compensate(1);
          it_pcl->z = P_compensate(2);
          it_pcl->intensity = idx;
        }

      }
  }

  for(int num = 0; num < lid_num; num++){
    if(num == 0){
      kf_state.temporal_comp.clear();
      kf_state.lidar_uncertainty.clear();
      kf_state.lidar_uncertainty.push_back(uncertainty[0]);
    }
    else{
      Pose temporal_comp;
      compoundInvPoseWithCov(lt_lidar_frame[0], lt_lidar_frame[0].cov_, lt_lidar_frame[num], lt_lidar_frame[num].cov_, temporal_comp, temporal_comp.cov_, 2);
      kf_state.temporal_comp.push_back(temporal_comp);
      kf_state.lidar_uncertainty.push_back(uncertainty[num]);
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, std::vector<PointCloudXYZI::Ptr> &cur_pcl_un_)
{
  if (meas.imu.empty())
  {
    return;
  };

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
    }

    return;
  }
  UndistortPcl(meas, kf_state, cur_pcl_un_);
}

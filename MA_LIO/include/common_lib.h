#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ma_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;

#define USE_IKFOM

#define PI_M (3.14159265358)
#define G_m_s2 (9.81) // Gravaty const in GuangDong/China
#define CUBE_LEN (6.0)
#define NUM_MATCH_POINTS (5)
#define LIDAR_SP_LEN    (2)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))
#define VF(a) Matrix<float, (a), 1>

typedef ma_lio::Pose6D Pose6D;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;
typedef Eigen::Matrix<double, 6, 6> M6D;

M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);

struct MeasureGroup // Lidar data and imu dates for the curent process
{
    MeasureGroup(){};
    std::vector<double> lidar_beg_time;
    std::vector<double> lidar_end_time;

    std::vector<PointCloudXYZI::Ptr> lidar_multi;
    deque<sensor_msgs::Imu::ConstPtr> imu;
    deque<sensor_msgs::Imu::ConstPtr> imu_cont;
    pair<Eigen::Quaterniond, Eigen::Vector3d> ext_set;
};

struct Pose
{
    Eigen::Quaterniond q_; // q = [cos(theta/2), u*sin(theta/2)]
    Eigen::Vector3d t_;
    Eigen::Matrix4d T_;
    Eigen::Matrix<double, 6, 6> cov_;
};

template <typename T>
T rad2deg(T radians)
{
    return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees)
{
    return degrees * PI_M / 180.0;
}

float calc_dist(PointType p1, PointType p2)
{
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g,
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    return move(rot_kp);
}

template <typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

void PoseInitial(Pose &pose, V3D &trans, Eigen::Quaterniond &quat, M6D cov)
{
    Eigen::Matrix4d transform;
    Eigen::Matrix<double, 1, 4> bottom_row;
    bottom_row << 0, 0, 0, 1;
    transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
    transform.block<3, 1>(0, 3) = trans;
    transform.block<1, 4>(3, 0) = bottom_row;

    pose.t_ = trans;
    pose.q_ = quat;
    pose.T_ = transform;
    pose.cov_ = cov;
}

template <typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold, double &plane_cov, double cov_threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    Matrix<T, NUM_MATCH_POINTS, NUM_MATCH_POINTS> W = Matrix<T, NUM_MATCH_POINTS, NUM_MATCH_POINTS>::Identity();
    Matrix<T, 3, 1> normvec;
    Matrix<T, 3, NUM_MATCH_POINTS> ATW;

    double cov_sum = 0;
    plane_cov = 0;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
        W(j, j) = point[j].normal_y;
        cov_sum += abs(cov_threshold - W(j, j));
    }
    if ((W(0, 0) > 0.00001))
    {
        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            plane_cov += ((cov_threshold - W(j, j)) / cov_sum) * ((cov_threshold - W(j, j)) / cov_sum) * W(j, j);
            W(j, j) = 1 / W(j, j);
        }
    }
    normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

template <typename T>
bool esti_plane_sub(Matrix<T, 4, 1> &pca_result, const PointVector &point)
{
    Matrix<T, NUM_MATCH_POINTS + 1, 3> A;
    Matrix<T, NUM_MATCH_POINTS + 1, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS + 1; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;

    return true;
}

template <typename T>
static Eigen::Matrix<typename T::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<T> &q)
{
    Eigen::Matrix<typename T::Scalar, 3, 3> ans;
    ans << typename T::Scalar(0), -q(2), q(1),
        q(2), typename T::Scalar(0), -q(0),
        -q(1), q(0), typename T::Scalar(0);
    return ans;
}
#endif

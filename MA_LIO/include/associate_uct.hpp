#pragma once

#include <eigen3/Eigen/Dense>

#include "common_lib.h"

// ****************** Barfoot's method on associating uncertainty on SE3
inline Eigen::Matrix<double, 6, 6> adjointMatrix(const Eigen::Matrix4d &T)
{
    Eigen::Matrix<double, 6, 6> AdT = Eigen::Matrix<double, 6, 6>::Zero();
    AdT.topLeftCorner<3, 3>() = T.topLeftCorner<3, 3>();
    AdT.topRightCorner<3, 3>() = skewSymmetric(T.topRightCorner<3, 1>()) * T.topLeftCorner<3, 3>();
    AdT.bottomRightCorner<3, 3>() = T.topLeftCorner<3, 3>();
    return AdT;
}

inline Eigen::Matrix3d covop1(const Eigen::Matrix3d &B)
{
    Eigen::Matrix3d A = -B.trace() * Eigen::Matrix3d::Identity() + B;
    return A;
}

inline Eigen::Matrix3d covop2(const Eigen::Matrix3d &B, const Eigen::Matrix3d &C)
{
    Eigen::Matrix3d A = covop1(B) * covop1(C) + covop1(C * B);
    return A;
}

inline void compoundInvPoseWithCov(const Pose &pose_1, const Eigen::Matrix<double, 6, 6> &cov_1,
                                   const Pose &pose_2, const Eigen::Matrix<double, 6, 6> &cov_2,
                                   Pose &pose_cp, Eigen::Matrix<double, 6, 6> &cov_cp,
                                   const int &method = 2)
{
    // pose1 = pose_point, pose2 = pose_cont
    pose_cp.q_ = pose_1.q_.conjugate() * pose_2.q_;
    pose_cp.t_ = pose_1.q_.conjugate() * (pose_2.t_ - pose_1.t_);

    Eigen::Matrix<double, 1, 4> bottom_row;
    bottom_row << 0, 0, 0, 1;
    pose_cp.T_.topLeftCorner<3, 3>() = pose_cp.q_.toRotationMatrix();
    pose_cp.T_.topRightCorner<3, 1>() = pose_cp.t_.transpose();
    pose_cp.T_.bottomLeftCorner<1, 4>() = bottom_row;

    Eigen::Matrix<double, 6, 6> AdT = adjointMatrix(pose_cp.T_.inverse()); // the adjoint matrix of T1
    Eigen::Matrix<double, 6, 6> cov_1_prime = AdT * cov_1 * AdT.transpose();

    if (method == 1)
    {
        cov_cp = cov_1_prime + cov_2;
    }
    else
    {
        Eigen::Matrix3d cov_1_rr = cov_1_prime.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_1_rp = cov_1_prime.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_1_pp = cov_1_prime.bottomRightCorner<3, 3>();

        Eigen::Matrix3d cov_2_rr = cov_2.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_2_rp = cov_2.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_2_pp = cov_2.bottomRightCorner<3, 3>();

        Eigen::Matrix<double, 6, 6> A1 = Eigen::Matrix<double, 6, 6>::Zero();
        A1.topLeftCorner<3, 3>() = covop1(cov_1_pp);
        A1.topRightCorner<3, 3>() = covop1(cov_1_rp + cov_1_rp.transpose());
        A1.bottomRightCorner<3, 3>() = covop1(cov_1_pp);

        Eigen::Matrix<double, 6, 6> A2 = Eigen::Matrix<double, 6, 6>::Zero();
        A2.topLeftCorner<3, 3>() = covop1(cov_2_pp);
        A2.topRightCorner<3, 3>() = covop1(cov_2_rp + cov_2_rp.transpose());
        A2.bottomRightCorner<3, 3>() = covop1(cov_2_pp);

        Eigen::Matrix3d Brr = covop2(cov_1_pp, cov_2_rr) + covop2(cov_1_rp.transpose(), cov_2_rp) +
                              covop2(cov_1_rp, cov_2_rp.transpose()) + covop2(cov_1_rr, cov_2_pp);
        Eigen::Matrix3d Brp = covop2(cov_1_pp, cov_2_rp.transpose()) + covop2(cov_1_rp.transpose(), cov_2_pp);
        Eigen::Matrix3d Bpp = covop2(cov_1_pp, cov_2_pp);
        Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
        B.topLeftCorner<3, 3>() = Brr;
        B.topRightCorner<3, 3>() = Brp;
        B.bottomLeftCorner<3, 3>() = Brp.transpose();
        B.bottomRightCorner<3, 3>() = Bpp;

        cov_cp = cov_1_prime + cov_2 + (A1 * cov_2 + cov_2 * A1.transpose() + A2 * cov_1_prime + cov_1_prime * A2.transpose()) / 12 + B / 4;
    }
}

inline void compoundPoseWithCov(const Pose &pose_1, const Eigen::Matrix<double, 6, 6> &cov_1,
                                const Pose &pose_2, const Eigen::Matrix<double, 6, 6> &cov_2,
                                Pose &pose_cp, Eigen::Matrix<double, 6, 6> &cov_cp,
                                const int &method = 2)
{
    pose_cp.q_ = pose_1.q_ * pose_2.q_;
    pose_cp.t_ = pose_1.q_ * pose_2.t_ + pose_1.t_;

    Eigen::Matrix<double, 1, 4> bottom_row;
    bottom_row << 0, 0, 0, 1;
    pose_cp.T_.topLeftCorner<3, 3>() = pose_cp.q_.toRotationMatrix();
    pose_cp.T_.topRightCorner<3, 1>() = pose_cp.t_.transpose();
    pose_cp.T_.bottomLeftCorner<1, 4>() = bottom_row;

    Eigen::Matrix<double, 6, 6> AdT2 = adjointMatrix(pose_2.T_.inverse()); // the adjoint matrix of T1
    Eigen::Matrix<double, 6, 6> cov_1_prime = AdT2 * cov_1 * AdT2.transpose();

    if (method == 1)
        cov_cp = cov_1_prime + cov_2;
    else if (method == 2)
    {
        Eigen::Matrix3d cov_1_rr = cov_1_prime.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_1_rp = cov_1_prime.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_1_pp = cov_1_prime.bottomRightCorner<3, 3>();

        Eigen::Matrix3d cov_2_rr = cov_2.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_2_rp = cov_2.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_2_pp = cov_2.bottomRightCorner<3, 3>();

        Eigen::Matrix<double, 6, 6> A1 = Eigen::Matrix<double, 6, 6>::Zero();
        A1.topLeftCorner<3, 3>() = covop1(cov_1_pp);
        A1.topRightCorner<3, 3>() = covop1(cov_1_rp + cov_1_rp.transpose());
        A1.bottomRightCorner<3, 3>() = covop1(cov_1_pp);

        Eigen::Matrix<double, 6, 6> A2 = Eigen::Matrix<double, 6, 6>::Zero();
        A2.topLeftCorner<3, 3>() = covop1(cov_2_pp);
        A2.topRightCorner<3, 3>() = covop1(cov_2_rp + cov_2_rp.transpose());
        A2.bottomRightCorner<3, 3>() = covop1(cov_2_pp);

        Eigen::Matrix3d Brr = covop2(cov_1_pp, cov_2_rr) + covop2(cov_1_rp.transpose(), cov_2_rp) +
                              covop2(cov_1_rp, cov_2_rp.transpose()) + covop2(cov_1_rr, cov_2_pp);
        Eigen::Matrix3d Brp = covop2(cov_1_pp, cov_2_rp.transpose()) + covop2(cov_1_rp.transpose(), cov_2_pp);
        Eigen::Matrix3d Bpp = covop2(cov_1_pp, cov_2_pp);
        Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
        B.topLeftCorner<3, 3>() = Brr;
        B.topRightCorner<3, 3>() = Brp;
        B.bottomLeftCorner<3, 3>() = Brp.transpose();
        B.bottomRightCorner<3, 3>() = Bpp;

        cov_cp = cov_1_prime + cov_2 + (A1 * cov_2 + cov_2 * A1.transpose() + A2 * cov_1_prime + cov_1_prime * A2.transpose()) / 12 + B / 4;
        pose_cp.cov_ = cov_cp;
    }
    else
    {
        printf("[compoundPoseWithCov] No %dth method !\n", method);
        cov_cp.setZero();
    }
}

// pointToFS turns a 4x1 homogeneous point into a special 4x6 matrix
inline Eigen::Matrix<double, 4, 6> pointToFS(const Eigen::Vector4d &point)
{
    Eigen::Matrix<double, 4, 6> G = Eigen::Matrix<double, 4, 6>::Zero();
    G.block<3, 3>(0, 0) = point(3) * Eigen::Matrix3d::Identity();
    G.block<3, 3>(0, 3) = -skewSymmetric(point.block<3, 1>(0, 0));
    return G;
}

template <typename PointType>
inline void evalPointUncertainty(const PointType &pi, Eigen::Matrix3d &cov_point, const Pose &pose)
{
    // THETA: diag(P, Phi, Z) includes the translation, rotation, measurement uncertainty
    Eigen::Matrix<double, 9, 9> cov_input = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 3, 3> COV_MEASUREMENT = Eigen::Matrix3d::Identity() * 0.1;

    cov_input.topLeftCorner<6, 6>() = pose.cov_ * 10000;
    cov_input.bottomRightCorner<3, 3>() = COV_MEASUREMENT;
    double distance_weight = 0.05;
    Eigen::Vector4d point_curr(pi.x * distance_weight, pi.y * distance_weight, pi.z * distance_weight, 1);
    Eigen::Matrix4d T = pose.T_;

    Eigen::Matrix<double, 4, 3> D;
    D << 1, 0, 0,
        0, 1, 0,
        0, 0, 1,
        0, 0, 0;
    Eigen::Matrix<double, 4, 9> G = Eigen::Matrix<double, 4, 9>::Zero();
    G.block<4, 6>(0, 0) = pointToFS(T * point_curr);
    G.block<4, 3>(0, 6) = T * D;
    cov_point = Eigen::Matrix4d(G * cov_input * G.transpose()).topLeftCorner<3, 3>(); // 3x3
}

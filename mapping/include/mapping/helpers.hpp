#pragma once

#ifndef MAPPING_HELPERS_HPP_
#define MAPPING_HELPERS_HPP_

#include <mapping/types.hpp>

#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>

namespace mapping
{
    /// @ingroup helpers
    /// @brief Convert LidarData to Open3D PointCloud (no undistortion).
    /// @param lidar_data The raw LiDAR scan, potentially undistorted.
    /// @param minPointDist Minimum point distance to include
    /// @param maxPointDist Maximum point distance to include
    /// @return an Open3D PointCloud, with scan timestamp info removed.
    inline open3d::geometry::PointCloud Scan2PCD(
        const std::shared_ptr<LidarData> &lidar_data,
        double minPointDist,
        double maxPointDist)
    {
        open3d::geometry::PointCloud pcd;
        size_t point_num = lidar_data->points.size();
        pcd.points_.reserve(point_num);
        for (size_t i = 0; i < point_num; ++i)
        {
            const Eigen::Vector3d &pt = lidar_data->points[i];
            const double dist = pt.norm();
            if (dist < minPointDist || dist > maxPointDist)
                continue;
            pcd.points_.push_back(lidar_data->points[i]);
        }
        return pcd;
    }

    /// @ingroup helpers
    /// @brief Fit a plane to a set of 3D points using SVD
    /// @param points Input points to fit the plane to
    /// @param planarityThreshold Maximum ratio σ₃/σ₂ for valid plane (default 0.3)
    /// @param linearityThreshold Minimum ratio σ₂/σ₁ to reject collinear points (default 0.1)
    /// @return Tuple of (isValid, planeNormal, planeCenter, planePoints) where:
    ///         - isValid: true if plane passes planarity and non-linearity checks
    ///         - planeNormal: fitted plane normal vector
    ///         - planeCenter: centroid of input points
    ///         - planePoints: Nx3 matrix of centered points (w_p - w_planeCenter)
    ///         - planeThickness: mean squared point-to-plane distance (lower is better)
    inline std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, double> planeFitSVD(
        const std::vector<Eigen::Vector3d> &points,
        double planarityThreshold = 0.1,
        double linearityThreshold = 0.5)
    {
        const size_t numPoints = points.size();

        // Compute centroid
        Eigen::Vector3d planeCenter = Eigen::Vector3d::Zero();
        for (const auto &pt : points)
            planeCenter += pt;
        planeCenter /= static_cast<double>(numPoints);

        // Build centered points matrix (Nx3)
        Eigen::MatrixXd planePoints(numPoints, 3);
        for (size_t i = 0; i < numPoints; ++i)
            planePoints.row(i) = (points[i] - planeCenter).transpose();

        // SVD to find plane normal (smallest singular vector)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(planePoints, Eigen::ComputeThinV);
        Eigen::Vector3d planeNormal = svd.matrixV().col(2).normalized();

        // Get singular values: s1 >= s2 >= s3
        const Eigen::VectorXd singularValues = svd.singularValues();
        const double sigma1 = singularValues(0);
        const double sigma2 = singularValues(1);
        const double sigma3 = singularValues(2);

        // Check validity:
        // 1. Planarity: s3 / s2 <= threshold (points are flat, not a blob)
        // 2. Non-linearity: s2 / s1 >= threshold (points have 2D spread, not a line)
        const bool isPlanar = (sigma2 > 1e-10) && (sigma3 / sigma2) <= planarityThreshold;
        const bool notLinear = (sigma1 > 1e-10) && (sigma2 / sigma1) >= linearityThreshold;
        const bool isValid = isPlanar && notLinear;

        double planeThickness = 0.0;
        if (isValid)
        {
            // Compute plane thickness as mean squared point-to-plane distance
            for (size_t i = 0; i < numPoints; ++i)
            {
                const double pointToPlaneDist = std::abs(planeNormal.dot(planePoints.row(i)));
                planeThickness += std::pow(pointToPlaneDist, 2.0);
            }
            planeThickness /= static_cast<double>(numPoints);
        }

        return {isValid, planeNormal, planeCenter, planePoints, planeThickness};
    }

} // namespace mapping

#endif // MAPPING_HELPERS_HPP_
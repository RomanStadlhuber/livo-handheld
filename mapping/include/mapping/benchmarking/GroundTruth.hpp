/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_GROUNDTRUTH_HPP_
#define MAPPING_BENCHMARKING_GROUNDTRUTH_HPP_

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>

#include <optional>
#include <string>
#include <vector>

namespace mapping
{
    namespace benchmarking
    {
        /// @ingroup benchmarking
        /// @brief Ground-truth trajectory (benchmark build only), loaded from a TUM file.
        /// @details Poses are in the frame of the file (e.g. the IMU frame for Hilti `*_imu.txt`), sorted by time.
        /// Interpolation: slerp on the rotation, lerp on the translation.
        class GroundTruth
        {
        public:
            /// @brief Load a TUM file (`t x y z qx qy qz qw` per line, `#` comments).
            /// @throws std::runtime_error if the file cannot be read or holds fewer than two poses.
            static GroundTruth loadTum(const std::string &path);

            /// @brief Subtract `t0` from all stamps, e.g. to use the time base of the system (first bag message).
            void shiftTime(double t0);

            /// @brief Interpolated pose at `t`, empty outside the time range.
            std::optional<gtsam::Pose3> poseAt(double t) const;

            /// @brief World-frame velocity at `t` from the finite difference of the two enclosing poses.
            std::optional<Eigen::Vector3d> velocityAt(double t) const;

            std::size_t size() const { return stamps_.size(); }
            double front() const { return stamps_.front(); }
            double back() const { return stamps_.back(); }

        private:
            /// @brief Index `i` with `stamps_[i] <= t < stamps_[i + 1]`, empty outside the range.
            std::optional<std::size_t> bracket(double t) const;

            std::vector<double> stamps_;
            std::vector<gtsam::Pose3> poses_;
        };
    } // namespace benchmarking
} // namespace mapping

#endif // MAPPING_BENCHMARKING_GROUNDTRUTH_HPP_

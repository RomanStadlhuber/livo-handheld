#pragma once

#ifndef MAPPING_FRONTEND_IMUFRONTEND_HPP_
#define MAPPING_FRONTEND_IMUFRONTEND_HPP_

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <mapping/types.hpp>
#include <mapping/Buffers.hpp>
#include <mapping/States.hpp>
#include <mapping/Config.hpp>

namespace mapping
{    

    class ImuFrontend
    {
    public:
        ImuFrontend() = default;
        ~ImuFrontend() = default;

        /// @brief Lock the IMU buffer and preintegrate all measurements within it.
        /// @param states The current system states, used for preintegrator reset and bias updates.
        /// @param buffers The system buffers, used to access the IMU buffer and clear it
        /// @return A NavState containing the preintegrated pose and velocity.
        gtsam::NavState preintegrateIMU(const States &states, Buffers &buffers);

        /// @brief Undistort all scans based on previous motion and store them to the pointcloud buffer.
        /// @param states The current system states, used to get the last keyframe pose for undistortion.
        /// @param buffers The system buffers, used to access the lidar buffer and
        /// @param config The system config, used to get the undistortion parameters.
        void undistortScans(const States &states, Buffers &buffers, const MappingConfig &config);

        /// @brief Reset the preintegrator with updated state (bias) estimate.
        /// @details At this point, the States should have been updated from the smoother estimate already.
        /// @param states The current system states, used for preintegrator reset and bias updates.
        void resetPreintegrator(const States &states);

        /// @brief Create a CombinedImuFactor between the previous and current keyframe using the preintegrated measurements.
        gtsam::CombinedImuFactor createPreintegrationFactor(
            const uint32_t &idxPrevKeyframe,
            const uint32_t &idxCurrKeyframe) const
        {
            return gtsam::CombinedImuFactor(
                X(idxPrevKeyframe), V(idxPrevKeyframe),
                X(idxCurrKeyframe), V(idxCurrKeyframe),
                B(idxPrevKeyframe), B(idxCurrKeyframe),
                preintegrator_);
        };

    private:
        // IMU preintegrator
        gtsam::PreintegratedCombinedMeasurements preintegrator_;
    };

} // namespace mapping

#endif // MAPPING_FRONTEND_IMUFRONTEND_HPP_
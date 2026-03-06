/// @file
/// @ingroup frontend_recovery
#pragma once

#ifndef MAPPING_FRONTEND_RECOVERYFRONTEND_HPP_
#define MAPPING_FRONTEND_RECOVERYFRONTEND_HPP_

#include <mapping/types.hpp>
#include <mapping/States.hpp>
#include <mapping/Config.hpp>

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>

namespace mapping
{
    /// @brief Frontend specifically used for recovreing from tracking failure.
    /// Aims to produce a pose/state estimate that can be used to reset the smoother and reinitialize the frontend.
    class RecoveryFrontend
    {
    private:
        /* data */
    public:
        RecoveryFrontend();
        ~RecoveryFrontend();

        /// @brief Estimate the full recovery state including velocity with
        /// Open3D's point-to-plane ICP.
        /// @param states The current state, used to get LiDAR keyframe a pose history
        /// as well as the predicted `gtsam::NavState` at the time where tracking was lost.
        /// @param config The config used to tune ICP behavior
        /// @return Updated `gtsam::NavState` with the refined ICP pose and transformed velocity.
        static gtsam::NavState estimateRecoveryState(
            const States &states,
            const MappingConfig &config);
    };

} // namespace mapping

#endif // MAPPING_FRONTEND_RECOVERYFRONTEND_HPP_
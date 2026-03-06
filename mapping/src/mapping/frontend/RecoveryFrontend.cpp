#include <mapping/frontend/RecoveryFrontend.hpp>

namespace mapping
{

    RecoveryFrontend::RecoveryFrontend()
    {
    }

    RecoveryFrontend::~RecoveryFrontend()
    {
    }

    gtsam::NavState RecoveryFrontend::estimateRecoveryState(
        const States &states,
        const MappingConfig &config)
    {
        // index of the keyframe that is to be recovered
        const uint32_t
            idxKfRecovery{states.getLatestKeyframeIdx()},
            idxRecoveryWindowEnd{idxKfRecovery - 1 - config.recovery.reference_lag},
            idxRecoveryWindowStart{idxRecoveryWindowEnd - config.recovery.reference_window_size + 1};
        // estimate initial recovery pose by extrapolating from the reference keyframe
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> &keyframePoses = states.getKeyframePoses();
        std::map<uint32_t, double> &keyframeTimestamps = states.getKeyframeTimestamps();
        // poses used for extrapolation
        const gtsam::Pose3 &
            w_T_l1 = *keyframePoses.at(idxRecoveryWindowEnd),
           w_T_l2 = *keyframePoses.at(idxRecoveryWindowEnd - 1);
        // timestamps used for extrapolation
        const double &
            t1 = keyframeTimestamps.at(idxRecoveryWindowEnd),
           t2 = keyframeTimestamps.at(idxRecoveryWindowEnd - 1),
           t3 = keyframeTimestamps.at(idxKfRecovery);
        // extrapolate pose using constant velocity motion model
        const double
            dtInterpolate = t1 - t2,
            dtExtrapolate = t3 - t1;
        // twist (veloicty) vector between
        // Log{w_T_l1^(-) * w_T_l2} / dt_inter
        const gtsam::Vector6 twist = gtsam::Pose3::Logmap(w_T_l1.between(w_T_l2)) / dtInterpolate;
        // estimated initial recovery pose, obtained from extrapolation
        // w_T_l1 * Exp{twist * dt_extrapolate}
        const gtsam::Pose3 w_T_lRecovery = w_T_l1.compose(gtsam::Pose3::Expmap(twist * dtExtrapolate));
        // merge reference PCDs
        open3d::geometry::PointCloud pcdRefAccumulator;
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &keyframeSubmaps = states.getKeyframeSubmaps();
        for (uint32_t i = idxRecoveryWindowStart; i <= idxRecoveryWindowEnd; ++i)
        {
            pcdRefAccumulator += *keyframeSubmaps.at(i);
        }
        // the voxel-downsampled reference point cloud used for recovery
        std::shared_ptr<open3d::geometry::PointCloud>
            ptrPcdRef = pcdRefAccumulator.VoxelDownSample(config.recovery.voxel_size),
            ptrPcdQuery = states.getKeyframeSubmaps().at(idxKfRecovery);
        // NOTE: estimating normals is required for open3d's built-in point-to-plane ICP
        ptrPcdRef->EstimateNormals();
        // ICP exit/convergence criteria
        open3d::pipelines::registration::ICPConvergenceCriteria icpSettings{
            1e-5,                           // relative fitness
            1e-5,                           // relative rmse
            config.recovery.icp_iterations, // max iterations
        };
        // estimate recovery alignment with ICP, see example here:
        // https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
        open3d::pipelines::registration::RegistrationResult icpResult = open3d::pipelines::registration::RegistrationICP(
            *ptrPcdQuery,                                // source (to align)
            *ptrPcdRef,                                  // target (reference)
            config.recovery.max_correspondence_distance, // for picking point to plane correspondences
            w_T_lRecovery.matrix(),                      // alignment initial guess
            /*method=*/open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            icpSettings);
        gtsam::Pose3 w_T_l_icpOut{icpResult.transformation_};
        const gtsam::Pose3 &w_T_pred{*keyframePoses.at(idxKfRecovery)};
        const gtsam::Pose3 pred_T_recovery{w_T_pred.between(w_T_l_icpOut)};
        const gtsam::Rot3 &
            w_R_pred{w_T_pred.rotation()},
            w_R_icp{w_T_l_icpOut.rotation()};
        const gtsam::NavState
            w_X_pred{states.getCurrentState()},
            w_X_recovery{
                w_T_l_icpOut, // pose
                // transform velocity form predicted frame to ICP-refined frame
                // NOTE that the velocity of gtsam::NavState is w.r.t. the world frame, so we have
                // w_v_final = w_R_b_icp * b_pred_R_w * w_v_pred
                w_R_icp * w_R_pred.inverse() * w_X_pred.v()};
        return w_X_recovery;
    }

} // namespace mapping
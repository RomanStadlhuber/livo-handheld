/// @file
/// @ingroup frontend_scan_to_map
#include <mapping/frontend/ScanToMapFrontend.hpp>
#include <mapping/helpers.hpp>
#include <mapping/logging.hpp>

#include <open3d/t/pipelines/registration/TransformationEstimation.h>

#include <gtsam/slam/PriorFactor.h>

#include <set>

SETUP_LOGS(DEBUG, "ScanToMapFrontend");

namespace mapping
{
    ScanToMapFrontend::ScanToMapFrontend(const BundleAdjustment &ba, const MappingConfig &config)
        : ba_(&ba), config_(config)
    {
    }

    std::optional<ScanToMapFrontend::IcpOutcome>
    ScanToMapFrontend::runScanToMapIcp(const open3d::geometry::PointCloud &pcdScan, const gtsam::Pose3 &world_T_imu,
                                       const States &states)
    {
        const auto &cfg = config_.bundle_adjustment.scan_to_map_registration;

        if (!ba_ || ba_->getNumFrozenSubmaps() == 0)
        {
            LOG(ERROR, "no frozen submaps available for scan-to-map registration");
            return std::nullopt;
        }

        const gtsam::Pose3 world_T_lidar = world_T_imu.compose(states.getImuToLidarExtrinsic());
        const open3d::geometry::AxisAlignedBoundingBox scanAabb = pcdScan.GetAxisAlignedBoundingBox();
        const auto candidates = ba_->getGlobalMap(world_T_lidar, scanAabb, cfg.max_candidates);

        std::set<uint32_t> newIds;
        for (const auto &sm : candidates)
            newIds.insert(sm->keyframeIdx);

        std::vector<std::shared_ptr<const FrozenSubmap>> toAdd;
        std::vector<uint32_t> toRemove;
        for (const auto &sm : candidates)
        {
            if (registrationCache_.submaps.find(sm->keyframeIdx) == registrationCache_.submaps.end())
                toAdd.push_back(sm);
        }
        for (const auto &[id, sm] : registrationCache_.submaps)
        {
            if (newIds.find(id) == newIds.end())
                toRemove.push_back(id);
        }

        const bool hasRemovals = !toRemove.empty();
        const bool hasAdditions = !toAdd.empty();

        if (hasRemovals || hasAdditions)
        {
            for (uint32_t id : toRemove)
                registrationCache_.submaps.erase(id);
            for (const auto &sm : toAdd)
                registrationCache_.submaps[sm->keyframeIdx] = sm;
            registrationCache_.dirty = true;
        }

        if (registrationCache_.submaps.empty())
        {
            LOG(ERROR, "registration cache empty after candidate search, no overlapping submaps found");
            return std::nullopt;
        }

        if (registrationCache_.dirty)
        {
            if (hasRemovals || !registrationCache_.pcd)
            {
                // full rebuild: merge all cached world-frame clouds from scratch
                open3d::geometry::PointCloud pcdMerged;
                for (const auto &[id, sm] : registrationCache_.submaps)
                    pcdMerged += *sm->pcdWorld;
                pcdMerged = *pcdMerged.VoxelDownSample(cfg.cache_voxel_size);
                registrationCache_.pcd = std::make_shared<open3d::t::geometry::PointCloud>(
                    open3d::t::geometry::PointCloud::FromLegacy(pcdMerged, open3d::core::Float64));
            }
            else
            {
                // incremental: convert only the new clouds to tensor and merge into the existing cache
                open3d::geometry::PointCloud pcdNewLegacy;
                for (const auto &sm : toAdd)
                    pcdNewLegacy += *sm->pcdWorld;
                const open3d::t::geometry::PointCloud pcdNewTensor =
                    open3d::t::geometry::PointCloud::FromLegacy(pcdNewLegacy, open3d::core::Float64);
                registrationCache_.pcd = std::make_shared<open3d::t::geometry::PointCloud>(
                    (*registrationCache_.pcd + pcdNewTensor).VoxelDownSample(cfg.cache_voxel_size));
            }
            registrationCache_.dirty = false;
            LOG(DEBUG, "registration cache rebuilt: " << registrationCache_.submaps.size() << " submaps, "
                                                      << registrationCache_.pcd->GetPointPositions().GetLength()
                                                      << " pts");
        }

        const size_t numScales = cfg.voxel_sizes.size();
        std::vector<open3d::t::pipelines::registration::ICPConvergenceCriteria> criteriaList;
        criteriaList.reserve(numScales);
        for (size_t i = 0; i < numScales; ++i)
            criteriaList.emplace_back(1e-6, 1e-6, cfg.max_iterations_per_scale[i]);

        open3d::t::geometry::PointCloud pcdScanTensor =
            open3d::t::geometry::PointCloud::FromLegacy(pcdScan, open3d::core::Float64);

        // row-major layout required by the Open3D tensor API
        const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> initGuessMat{world_T_lidar.matrix()};
        const open3d::core::Tensor initGuess(initGuessMat.data(), {4, 4}, open3d::core::Float64,
                                             open3d::core::Device("CPU:0"));

        const auto result = open3d::t::pipelines::registration::MultiScaleICP(
            pcdScanTensor, *registrationCache_.pcd, cfg.voxel_sizes, criteriaList, cfg.max_correspondence_distances,
            initGuess, open3d::t::pipelines::registration::TransformationEstimationPointToPoint());

        LOG(DEBUG, "scan-to-map ICP: fitness=" << result.fitness_ << " rmse=" << result.inlier_rmse_);

        const auto transformData = result.transformation_.ToFlatVector<double>();
        const Eigen::Matrix4d icpMat =
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(transformData.data());
        const gtsam::Pose3 world_T_lidar_icp{gtsam::Rot3(icpMat.block<3, 3>(0, 0)), icpMat.block<3, 1>(0, 3)};
        const gtsam::Pose3 world_T_imu_icp = world_T_lidar_icp.compose(states.getImuToLidarExtrinsic().inverse());

        return IcpOutcome{world_T_imu_icp, std::move(pcdScanTensor), result};
    }

    std::optional<gtsam::NonlinearFactor::shared_ptr>
    ScanToMapFrontend::registerScanToMap(const open3d::geometry::PointCloud &pcdScan, const gtsam::Pose3 &world_T_imu,
                                         const States &states, uint32_t idxKeyframe)
    {
        const auto &cfg = config_.bundle_adjustment.scan_to_map_registration;

        auto icpOut = runScanToMapIcp(pcdScan, world_T_imu, states);
        if (!icpOut)
            return std::nullopt;

        LOG(DEBUG,
            "scan-to-map ICP fitness=" << icpOut->result.fitness_ << " (threshold=" << cfg.fitness_threshold << ")");

        if (icpOut->result.fitness_ < cfg.fitness_threshold)
        {
            LOG(DEBUG, "scan-to-map registration rejected, fitness below threshold");
            return std::nullopt;
        }

        open3d::core::Tensor icpInformationTensor = open3d::t::pipelines::registration::GetInformationMatrix(
            icpOut->pcdScan, *registrationCache_.pcd, cfg.max_correspondence_distances.back(),
            icpOut->result.transformation_);
        const auto informationData = icpInformationTensor.ToFlatVector<double>();
        const Eigen::Matrix<double, 6, 6> icpInformation =
            Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(informationData.data());

        LOG_MULTI(DEBUG, "scan-to-map ICP result",
                  STREAM("T_pred  t=[" << world_T_imu.translation().transpose() << "]"
                                       << "  rpy=[" << world_T_imu.rotation().rpy().transpose() << "]"),
                  STREAM("T_icp   t=[" << icpOut->world_T_imu.translation().transpose() << "]"
                                       << "  rpy=[" << icpOut->world_T_imu.rotation().rpy().transpose() << "]"));
        LOG_MULTI(DEBUG, "information matrix", STREAM(icpInformation.format(MTX_FMT)));

        const auto noiseModel = gtsam::noiseModel::Gaussian::Information(icpInformation);
        return boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(idxKeyframe), icpOut->world_T_imu, noiseModel);
    }

    std::optional<gtsam::NavState> ScanToMapFrontend::estimateRecoveryState(const open3d::geometry::PointCloud &pcdScan,
                                                                            const gtsam::NavState &world_X_imu,
                                                                            const States &states)
    {
        auto icpOut = runScanToMapIcp(pcdScan, world_X_imu.pose(), states);
        if (!icpOut)
            return std::nullopt;

        // rotate predicted world-frame velocity into the ICP-refined IMU orientation
        const gtsam::Rot3 w_R_icp = icpOut->world_T_imu.rotation();
        const gtsam::Rot3 w_R_pred = world_X_imu.pose().rotation();
        const gtsam::Vector3 w_v_recovered = (w_R_icp * w_R_pred.inverse()) * world_X_imu.v();

        return gtsam::NavState{icpOut->world_T_imu, w_v_recovered};
    }

} // namespace mapping

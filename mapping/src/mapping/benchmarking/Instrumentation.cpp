/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/Instrumentation.hpp>
#include <mapping/factors/PointToPlaneFactor.hpp>
#include <mapping/logging.hpp>

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <exception>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

SETUP_LOGS(INFO, "Instrumentation")

namespace mapping
{
    namespace benchmarking
    {
        namespace instrumentation
        {
            namespace
            {
                /// @brief Run one column group, log and continue if it throws.
                template <typename Fn> void collectGroup(const char *group, uint64_t idxKeyframe, Fn &&fn) noexcept
                {
                    try
                    {
                        fn();
                    }
                    catch (const std::exception &e)
                    {
                        LOG(WARN, "keyframe " << idxKeyframe << ": " << group << " telemetry failed: " << e.what());
                    }
                    catch (...)
                    {
                        LOG(WARN, "keyframe " << idxKeyframe << ": " << group << " telemetry failed");
                    }
                }

                using Matrix6 = Eigen::Matrix<double, 6, 6>;

                /// @brief Smallest eigenvalue of the rotation and translation blocks of a pose information matrix.
                void setMinEigenvalues(const Matrix6 &information, double &rot, double &trans)
                {
                    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigRot(information.topLeftCorner<3, 3>(),
                                                                                Eigen::EigenvaluesOnly),
                        eigTrans(information.bottomRightCorner<3, 3>(), Eigen::EigenvaluesOnly);
                    rot = eigRot.eigenvalues().minCoeff();
                    trans = eigTrans.eigenvalues().minCoeff();
                }

                std::size_t countLidarFactors(const gtsam::NonlinearFactorGraph &factors)
                {
                    std::size_t n = 0;
                    for (const gtsam::NonlinearFactor::shared_ptr &factor : factors)
                        if (boost::dynamic_pointer_cast<PointToPlaneFactor>(factor))
                            n++;
                    return n;
                }

                /// @brief `X`, `V` and `B` keys of the keyframes.
                gtsam::KeySet navStateKeys(const std::vector<uint32_t> &keyframes)
                {
                    gtsam::KeySet keys;
                    for (const uint32_t k : keyframes)
                        keys.insert({X(k), V(k), B(k)});
                    return keys;
                }

                bool touchesAny(const gtsam::NonlinearFactor &factor, const gtsam::KeySet &keys)
                {
                    for (const gtsam::Key key : factor.keys())
                        if (keys.count(key))
                            return true;
                    return false;
                }

                /// @brief Linearize the factors at `estimate`, eliminate `margKeys` and sum the diagonal pose blocks of
                /// the remaining information on the Markov blanket.
                Matrix6 blanketPoseInformation(const std::vector<gtsam::NonlinearFactor::shared_ptr> &factors,
                                               const gtsam::Values &estimate, const gtsam::KeySet &margKeys)
                {
                    gtsam::GaussianFactorGraph linear;
                    for (const gtsam::NonlinearFactor::shared_ptr &factor : factors)
                        linear.push_back(factor->linearize(estimate));
                    gtsam::Ordering ordering;
                    const gtsam::KeySet linearKeys = linear.keys();
                    for (const gtsam::Key key : margKeys)
                        if (linearKeys.count(key))
                            ordering.push_back(key);
                    const auto [bayesNet, remaining] = linear.eliminatePartialSequential(ordering);
                    Matrix6 information = Matrix6::Zero();
                    for (const auto &[key, block] : remaining->hessianBlockDiagonal())
                        if (gtsam::Symbol(key).chr() == 'x' && block.rows() == 6)
                            information += block;
                    return information;
                }

                void collectImu(KeyframeTelemetry &row, const gtsam::Pose3 &imuPropagated, const States &states)
                {
                    const gtsam::Values &estimate = states.getSmootherEstimate();
                    const uint32_t kf = static_cast<uint32_t>(*row.kf);
                    const gtsam::Pose3 optimized = estimate.at<gtsam::Pose3>(X(kf));
                    row.imu_pred_dpos = (imuPropagated.translation() - optimized.translation()).norm();
                    row.imu_pred_drot =
                        gtsam::Rot3::Logmap(imuPropagated.rotation().between(optimized.rotation())).norm();
                    row.vel_norm = estimate.at<gtsam::Vector3>(V(kf)).norm();
                    const auto bias = estimate.at<gtsam::imuBias::ConstantBias>(B(kf));
                    row.ba_norm = bias.accelerometer().norm();
                    row.bg_norm = bias.gyroscope().norm();
                }

                /// @brief Frontend and geometry columns, both from the `Tracked` clusters with a point in `kf`.
                void collectFrontendAndGeometry(KeyframeTelemetry &row, const States &states,
                                                const FeatureManager &featureManager)
                {
                    const uint32_t kf = static_cast<uint32_t>(*row.kf);
                    const open3d::geometry::PointCloud &submap = *states.getKeyframeSubmaps().at(kf);
                    const gtsam::Pose3 &w_T_i = *states.getKeyframeImuPoses().at(kf);
                    const Eigen::Matrix3d i_R_w = w_T_i.rotation().matrix().transpose();
                    std::vector<double> thickness;
                    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
                    for (const auto &[clusterId, points] : featureManager.clusters_)
                    {
                        if (featureManager.clusterStates_.at(clusterId) != ClusterState::Tracked)
                            continue;
                        const auto itPoint = points.find(kf);
                        if (itPoint == points.end())
                            continue;
                        thickness.push_back(featureManager.clusterPlaneThicknessHistory_.at(clusterId).back());
                        // point and normal in the keyframe IMU frame
                        const Eigen::Vector3d p = w_T_i.transformTo(gtsam::Point3(submap.points_.at(itPoint->second)));
                        const Eigen::Vector3d n = i_R_w * *featureManager.clusterNormals_.at(clusterId);
                        Eigen::Matrix<double, 1, 6> J;
                        J << p.cross(n).transpose(), n.transpose();
                        information += J.transpose() * J / featureManager.factorVariance(clusterId);
                    }
                    row.thickness = makeStats(std::move(thickness));
                    setMinEigenvalues(information, row.eig_rot_min, row.eig_trans_min);
                }

                void collectBackend(KeyframeTelemetry &row, const States &states, const FeatureManager &featureManager,
                                    const Smoother &smoother)
                {
                    const gtsam::Values &estimate = states.getSmootherEstimate();
                    std::vector<double> errImu, errLidar, wLidar, varLidar;
                    std::size_t numFailed = 0;
                    for (const gtsam::NonlinearFactor::shared_ptr &factor : smoother.getFactors())
                    {
                        if (!factor)
                            continue;
                        try
                        {
                            if (boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor))
                                errImu.push_back(factor->error(estimate));
                            else if (const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor))
                            {
                                const PointToPlaneFactor::ErrorDiagnostic diagnostic =
                                    ptp->evaluateErrorDiagnostic(estimate);
                                errLidar.push_back(diagnostic.error);
                                wLidar.push_back(diagnostic.weight);
                                gtsam::SharedNoiseModel noise = ptp->noiseModel();
                                if (const auto robust = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise))
                                    noise = robust->noise();
                                if (const auto gaussian =
                                        boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise))
                                    varLidar.push_back(std::pow(gaussian->sigmas()(0), 2));
                            }
                        }
                        catch (const std::exception &)
                        {
                            numFailed++;
                        }
                    }
                    if (numFailed > 0)
                        LOG(WARN, "keyframe " << *row.kf << ": error of " << numFailed << " factors not evaluated");
                    row.err_imu = makeStats(std::move(errImu));
                    row.err_lidar = makeStats(std::move(errLidar));
                    row.w_lidar = makeStats(std::move(wLidar));
                    row.lidar_var = makeStats(std::move(varLidar));
                    row.lidar_fac_after = countLidarFactors(smoother.getFactors());
                    const gtsam::Matrix cov = smoother.marginalCovariance(X(static_cast<uint32_t>(*row.kf)));
                    row.cov_rot_trace = cov.topLeftCorner<3, 3>().trace();
                    row.cov_trans_trace = cov.bottomRightCorner<3, 3>().trace();

                    // stats of removeKeyframeFromClusters / createAndUpdateFactors for this keyframe
                    const FeatureManager::KeyframeFactorStats &stats = featureManager.lastKeyframeFactorStats_;
                    row.lidar_fac_removed = stats.numFactorsRemoved;
                    row.lidar_fac_readded = stats.numFactorsUpdated;
                    row.lidar_fac_new = stats.numFactorsAdded;
                    row.clusters_pruned = stats.numPruned;
                    row.clusters_shifted_idle = stats.numShiftedIdle;
                    row.marg_n = stats.numMargFactors;
                    row.marg_w = makeStats(stats.margWeights);
                    if (stats.numMargFactors > 0)
                        setMinEigenvalues(stats.margInformation, row.marg_info_rot, row.marg_info_trans);
                }
            } // namespace

            void snapshot(KeyframeTelemetry &row, gtsam::Pose3 &imuPropagated, const States &states,
                          const Smoother &smoother, uint32_t idxKeyframe)
            {
                row.lidar_fac_before = countLidarFactors(smoother.getFactors());
                row.kf = idxKeyframe;
                const auto &timestamps = states.getKeyframeTimestamps();
                const auto &imuPoses = states.getKeyframeImuPoses();
                row.t = timestamps.at(idxKeyframe);
                const auto itKf = imuPoses.find(idxKeyframe);
                if (itKf == imuPoses.end())
                    throw std::out_of_range("no IMU pose for keyframe " + std::to_string(idxKeyframe));
                imuPropagated = *itKf->second;
                if (itKf != imuPoses.begin())
                {
                    const auto itPrev = std::prev(itKf);
                    const gtsam::Pose3 &prev = *itPrev->second;
                    row.trigger_dist = (imuPropagated.translation() - prev.translation()).norm();
                    row.trigger_angle = gtsam::Rot3::Logmap(prev.rotation().between(imuPropagated.rotation())).norm();
                    row.trigger_dt = row.t - timestamps.at(itPrev->first);
                }
                row.pts_submap = states.getKeyframeSubmaps().at(idxKeyframe)->points_.size();
            }

            std::vector<uint32_t> marginalizedKeyframes(const States &states, uint32_t idxKeyframe,
                                                        uint32_t slidingWindowSize)
            {
                std::vector<uint32_t> keyframes;
                const gtsam::Values &estimate = states.getSmootherEstimate();
                for (const auto &[k, _] : states.getKeyframeImuPoses())
                    if (k + slidingWindowSize < idxKeyframe && estimate.exists(X(k)))
                        keyframes.push_back(k);
                return keyframes;
            }

            void snapshotMarginalization(KeyframeTelemetry &row, const std::vector<uint32_t> &keyframes,
                                         const States &states, const Smoother &smoother)
            {
                if (keyframes.empty())
                    return;
                row.marg_kf = keyframes.front();
                const gtsam::KeySet margKeys = navStateKeys(keyframes);
                std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
                for (const gtsam::NonlinearFactor::shared_ptr &factor : smoother.getFactors())
                    if (factor && touchesAny(*factor, margKeys))
                        factors.push_back(factor);
                const Matrix6 information = blanketPoseInformation(factors, states.getSmootherEstimate(), margKeys);
                setMinEigenvalues(information, row.marg_ref_info_rot, row.marg_ref_info_trans);
            }

            void collectMarginalization(KeyframeTelemetry &row, const std::vector<uint32_t> &keyframes,
                                        const States &states, const FeatureManager &featureManager,
                                        const Smoother &smoother, const gtsam::NonlinearFactorGraph &newFactors,
                                        const gtsam::FactorIndices &factorsToRemove)
            {
                row.lidar_fac_marg_pruned = 0;
                row.lidar_fac_marg_lost = 0;
                if (keyframes.empty())
                    return;
                const gtsam::KeySet margKeys = navStateKeys(keyframes);
                const std::set<gtsam::FactorIndex> removed(factorsToRemove.begin(), factorsToRemove.end());
                const gtsam::NonlinearFactorGraph &smootherFactors = smoother.getFactors();
                // the factors on the marginalized keys after the update, as iSAM2 sees them when it marginalizes
                std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
                for (gtsam::FactorIndex i = 0; i < smootherFactors.size(); i++)
                {
                    const gtsam::NonlinearFactor::shared_ptr &factor = smootherFactors[i];
                    if (!factor || removed.count(i) || !touchesAny(*factor, margKeys))
                        continue;
                    factors.push_back(factor);
                    if (const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor))
                    {
                        const auto itState = featureManager.clusterStates_.find(ptp->clusterId_);
                        if (itState == featureManager.clusterStates_.end() || itState->second == ClusterState::Pruned)
                            (*row.lidar_fac_marg_pruned)++;
                        else
                            (*row.lidar_fac_marg_lost)++;
                    }
                }
                for (const gtsam::NonlinearFactor::shared_ptr &factor : newFactors)
                    if (factor && touchesAny(*factor, margKeys))
                        factors.push_back(factor);
                const Matrix6 information = blanketPoseInformation(factors, states.getSmootherEstimate(), margKeys);
                setMinEigenvalues(information, row.marg_cur_info_rot, row.marg_cur_info_trans);
            }

            void collect(KeyframeTelemetry &row, const gtsam::Pose3 &imuPropagated, const States &states,
                         const FeatureManager &featureManager, const Smoother &smoother) noexcept
            {
                if (!row.kf)
                    return;
                collectGroup("frontend/geometry", *row.kf,
                             [&] { collectFrontendAndGeometry(row, states, featureManager); });
                if (row.lost_tracking)
                    return; // no smoother update for this keyframe, IMU and backend columns stay nan
                collectGroup("IMU", *row.kf, [&] { collectImu(row, imuPropagated, states); });
                collectGroup("backend", *row.kf, [&] { collectBackend(row, states, featureManager, smoother); });
            }
        } // namespace instrumentation
    } // namespace benchmarking
} // namespace mapping

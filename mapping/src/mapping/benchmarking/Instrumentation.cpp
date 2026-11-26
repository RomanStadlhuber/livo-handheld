/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/Instrumentation.hpp>
#include <mapping/factors/PointToPlaneFactor.hpp>
#include <mapping/helpers.hpp>
#include <mapping/logging.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
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

                /// @brief A rebuilt cluster with `||J'|| / ||J||` below this value counts as collapsed.
                constexpr double kCollapsedJacRatio = 1e-6;

                std::size_t countLidarFactors(const gtsam::NonlinearFactorGraph &factors)
                {
                    std::size_t n = 0;
                    for (const gtsam::NonlinearFactor::shared_ptr &factor : factors)
                        if (boost::dynamic_pointer_cast<PointToPlaneFactor>(factor))
                            n++;
                    return n;
                }

                /// @brief For a given set of keyframe indices, returns the keys `X(k)`, `V(k)` and `B(k)` that make up
                /// the `gtsam::NavState` of keyframes `k`.
                gtsam::KeySet navStateKeys(const std::vector<uint32_t> &keyframes)
                {
                    gtsam::KeySet keys;
                    for (const uint32_t k : keyframes)
                        keys.insert({X(k), V(k), B(k)});
                    return keys;
                }

                /// @brief Indicates whether a given `gtsam::NonlinearFactor` connects to any parameter key in `keys`.
                bool touchesAny(const gtsam::NonlinearFactor &factor, const gtsam::KeySet &keys)
                {
                    for (const gtsam::Key key : factor.keys())
                        if (keys.count(key))
                            return true;
                    return false;
                }

                /// @brief Rows of a `PointToPlaneFactor` at `values`. `linearize` whitens the Jacobian as
                /// `A = sqrt(w) / sigma * J`, so `J` is recovered from `A` with the noise sigma and the kernel weight.
                /// Empty if the noise model is not Gaussian or the kernel weight is 0.
                std::optional<ClusterRows> clusterRows(const PointToPlaneFactor &factor, const gtsam::Values &values)
                {
                    gtsam::SharedNoiseModel noise = factor.noiseModel();
                    if (const auto robust = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise))
                        noise = robust->noise();
                    const auto gaussian = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise);
                    if (!gaussian)
                        return std::nullopt;
                    ClusterRows rows;
                    rows.sigma = gaussian->sigmas()(0);
                    rows.weight = factor.evaluateErrorDiagnostic(values).weight;
                    if (!(rows.sigma > 0.0) || !(rows.weight > 0.0))
                        return std::nullopt;
                    const auto jacobian = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(factor.linearize(values));
                    if (!jacobian)
                        return std::nullopt;
                    const double unwhiten = rows.sigma / std::sqrt(rows.weight);
                    for (auto it = jacobian->begin(); it != jacobian->end(); ++it)
                        if (gtsam::Symbol(*it).chr() == 'x')
                            rows.jacobians[*it] = jacobian->getA(it) * unwhiten;
                    return rows;
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
                    row.t_update_ms = 1e3 * smoother.getLastUpdateSeconds();
                    std::size_t numFixed = 0, numWindow = 0, numFixedVb = 0;
                    double fixedFirst = std::numeric_limits<double>::quiet_NaN();
                    double fixedLast = fixedFirst;
                    const gtsam::KeySet &fixedKeys = smoother.getFixedVariables();
                    for (const auto &[k, _] : states.getKeyframeImuPoses())
                    {
                        numWindow++;
                        if (fixedKeys.exists(X(k)))
                        {
                            numFixed++;
                            if (std::isnan(fixedFirst))
                                fixedFirst = static_cast<double>(k); // map is ordered by k
                            fixedLast = static_cast<double>(k);
                        }
                        if (fixedKeys.exists(V(k)) || fixedKeys.exists(B(k)))
                            numFixedVb++;
                    }
                    row.n_fixed_kf = numFixed;
                    row.n_window_kf = numWindow;
                    row.fixed_kf_first = fixedFirst;
                    row.fixed_kf_last = fixedLast;
                    row.n_fixed_vb = numFixedVb;
                    const gtsam::Matrix cov = smoother.marginalCovariance(X(static_cast<uint32_t>(*row.kf)));
                    row.cov_rot_trace = cov.topLeftCorner<3, 3>().trace();
                    row.cov_trans_trace = cov.bottomRightCorner<3, 3>().trace();

                    // stats of removeKeyframeFromClusters / createAndUpdateFactors for this keyframe
                    const FeatureManager::KeyframeFactorStats &stats = featureManager.lastKeyframeFactorStats();
                    row.lidar_fac_removed = stats.numFactorsRemoved;
                    row.lidar_fac_readded = stats.numFactorsUpdated;
                    row.lidar_fac_new = stats.numFactorsAdded;
                    row.clusters_pruned = stats.numPruned;
                    row.clusters_shifted_idle = stats.numShiftedIdle;
                    if (!row.marg_kf)
                        return; // no marginalization in this update, the marginalization columns stay nan
                    row.marg_n = stats.numMargFactors;
                    row.marg_w = makeStats(stats.margWeights);
                    if (stats.numMargFactors > 0)
                        setMinEigenvalues(stats.margInformation, row.marg_info_rot, row.marg_info_trans);
                }
            } // namespace

            void collectGroundTruth(KeyframeTelemetry &row, const States &states, const Smoother &smoother,
                                    const GroundTruth &groundTruth, double planarityThreshold,
                                    double linearityThreshold)
            {
                const gtsam::Values &estimate = states.getSmootherEstimate();
                const std::map<uint32_t, double> &stamps = states.getKeyframeTimestamps();
                // 1: window keyframes and their ground-truth poses
                std::vector<uint32_t> window;
                for (const auto &[k, t] : stamps)
                    if (estimate.exists(X(k)))
                        window.push_back(k);
                if (window.size() < 2)
                    return;
                const uint32_t oldest = window.front(), newest = window.back();
                const std::optional<gtsam::Pose3> gtOldest = groundTruth.poseAt(stamps.at(oldest));
                if (!gtOldest)
                    return;
                // 2: anchor at the oldest window keyframe, only the relative motion in the window changes
                const gtsam::Pose3 anchor = estimate.at<gtsam::Pose3>(X(oldest)) * gtOldest->inverse();
                gtsam::Values estValues, gtValues;
                for (const uint32_t k : window)
                {
                    const std::optional<gtsam::Pose3> gtPose = groundTruth.poseAt(stamps.at(k));
                    if (!gtPose)
                        return;
                    estValues.insert(X(k), estimate.at<gtsam::Pose3>(X(k)));
                    gtValues.insert(X(k), anchor * *gtPose);
                }
                const gtsam::Pose3 &estNewest = estValues.at<gtsam::Pose3>(X(newest)),
                                   &gtNewest = gtValues.at<gtsam::Pose3>(X(newest));
                row.gt_win_dpos = (gtNewest.translation() - estNewest.translation()).norm();
                row.gt_win_drot = gtsam::Rot3::Logmap(estNewest.rotation().between(gtNewest.rotation())).norm();
                row.gt_win_span = stamps.at(newest) - stamps.at(oldest);

                // 3: per factor, refit the plane at the given poses (frozen associations) and evaluate
                const gtsam::Pose3 imu_T_lidar = states.getImuToLidarExtrinsic();
                struct Cost
                {
                    double plain, robust;
                };
                const auto robustLoss = [](const PointToPlaneFactor &factor, double error)
                {
                    if (const auto noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(factor.noiseModel()))
                        return noise->robust()->loss(std::sqrt(2.0 * error));
                    return error;
                };
                auto costAt = [&](const PointToPlaneFactor &factor, const gtsam::Values &values) -> std::optional<Cost>
                {
                    std::vector<Eigen::Vector3d> points;
                    for (const auto &[key, l_p] : factor.lidarPoints())
                        if (values.exists(key))
                            points.push_back((values.at<gtsam::Pose3>(key) * imu_T_lidar).transformFrom(l_p));
                    if (points.size() < 3)
                        return std::nullopt;
                    const auto [valid, normal, center, centered, thickness] =
                        planeFitCovariance(points, planarityThreshold, linearityThreshold);
                    const auto clone = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor.clone());
                    clone->updatePlaneParameters(std::make_shared<Eigen::Vector3d>(normal),
                                                 std::make_shared<Eigen::Vector3d>(center), factor.noiseModel());
                    const double error = clone->evaluateErrorDiagnostic(values).error;
                    return Cost{error, robustLoss(factor, error)};
                };
                // line search from the estimate (a = 0) to the ground truth (a = 1)
                const auto interpolate = [&](double a)
                {
                    gtsam::Values values;
                    for (const uint32_t k : window)
                    {
                        const gtsam::Pose3 &est = estValues.at<gtsam::Pose3>(X(k)),
                                           &gt = gtValues.at<gtsam::Pose3>(X(k));
                        values.insert(X(k), est * gtsam::Pose3::Expmap(a * gtsam::Pose3::Logmap(est.between(gt))));
                    }
                    return values;
                };
                const std::vector<double> alphasRefit{0.05, 0.25, 0.5, 0.75}, alphasLive{0.0, 0.05, 0.25, 1.0};
                std::vector<gtsam::Values> valuesRefit, valuesLive;
                for (const double a : alphasRefit)
                    valuesRefit.push_back(interpolate(a));
                for (const double a : alphasLive)
                    valuesLive.push_back(interpolate(a));
                std::vector<double> sumRefit(alphasRefit.size(), 0.0), sumLive(alphasLive.size(), 0.0);
                std::vector<double> costEst, costGt, rcostEst, rcostGt;
                std::size_t numGtLower = 0;
                for (const gtsam::NonlinearFactor::shared_ptr &factor : smoother.getFactors())
                {
                    const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
                    if (!ptp)
                        continue;
                    const std::optional<Cost> est = costAt(*ptp, estValues), gt = costAt(*ptp, gtValues);
                    if (!est || !gt)
                        continue;
                    costEst.push_back(est->plain);
                    costGt.push_back(gt->plain);
                    rcostEst.push_back(est->robust);
                    rcostGt.push_back(gt->robust);
                    if (gt->plain < est->plain)
                        numGtLower++;
                    for (std::size_t i = 0; i < valuesRefit.size(); ++i)
                        if (const std::optional<Cost> c = costAt(*ptp, valuesRefit[i]))
                            sumRefit[i] += c->robust;
                    for (std::size_t i = 0; i < valuesLive.size(); ++i)
                        sumLive[i] += robustLoss(*ptp, ptp->evaluateErrorDiagnostic(valuesLive[i]).error);
                }
                row.lidar_rcost_a05_sum = sumRefit[0];
                row.lidar_rcost_a25_sum = sumRefit[1];
                row.lidar_rcost_a50_sum = sumRefit[2];
                row.lidar_rcost_a75_sum = sumRefit[3];
                row.lidar_rcost_live_a00_sum = sumLive[0];
                row.lidar_rcost_live_a05_sum = sumLive[1];
                row.lidar_rcost_live_a25_sum = sumLive[2];
                row.lidar_rcost_live_a100_sum = sumLive[3];
                const auto sum = [](const std::vector<double> &v) { return std::accumulate(v.begin(), v.end(), 0.0); };
                row.lidar_cost_est_sum = sum(costEst);
                row.lidar_cost_gt_sum = sum(costGt);
                row.lidar_rcost_est_sum = sum(rcostEst);
                row.lidar_rcost_gt_sum = sum(rcostGt);
                if (!costEst.empty())
                    row.lidar_cost_gt_lower = static_cast<double>(numGtLower) / static_cast<double>(costEst.size());
                row.lidar_cost_est = makeStats(std::move(costEst));
                row.lidar_cost_gt = makeStats(std::move(costGt));
                row.lidar_rcost_est = makeStats(std::move(rcostEst));
                row.lidar_rcost_gt = makeStats(std::move(rcostGt));
            }

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

            /// @brief Records the LiDAR factors on the marginalized keyframes and the estimate before the update.
            /// @details Runs in the `KeyframeProbe` constructor, before the update changes the smoother.
            /// It finds the `PointToPlaneFactor` LiDAR clusters that connect to the keyframes that the update
            /// marginalizes, and records what `collectMarginalization` needs after the update:
            /// 1. the marginalized keyframes: `X(j)` in the estimate and `j + slidingWindowSize < idxKeyframe`
            /// 2. `marg_kf`: the first (oldest) marginalized keyframe, `front()` of the list
            /// 3. the estimate before the update, plus the IMU-propagated pose of the new keyframe
            /// 4. per `PointToPlaneFactor` that touches a marginalized `NavState`: its index and cluster, and its rows
            ///    in the `ClusterRowsMap`, the baseline of the `rebuild_*` columns
            /// 5. per `PointToPlaneFactor` in the smoother (all of them): a `ClusterSpanRow` for `clusters.csv`,
            ///    the pose keys and the trace of the whitened `H^T H` on them
            ///
            /// The factor search is more general than needed. A `PointToPlaneFactor` only connects to the poses
            /// `X(k)`, but the search checks `X`, `V` and `B`. It stays general, so it also works for visual factors
            /// later. The function does nothing else when no keyframe is marginalized.
            void snapshotMarginalization(KeyframeTelemetry &row, MarginalizationSnapshot &snapshot,
                                         const States &states, const Smoother &smoother, uint32_t idxKeyframe,
                                         uint32_t slidingWindowSize)
            {
                snapshot = {};
                // 1. find the marginalized keyframes, ascending
                const gtsam::Values &estimate = states.getSmootherEstimate();
                for (const auto &[k, _] : states.getKeyframeImuPoses())
                    if (k + slidingWindowSize < idxKeyframe && estimate.exists(X(k)))
                        snapshot.keyframes.push_back(k);
                if (snapshot.keyframes.empty())
                    return;
                // 2. the first keyframe is the oldest one, normally the only one
                row.marg_kf = snapshot.keyframes.front();
                // all keys that a factor can connect to, `X`, `V` and `B` of each marginalized keyframe
                const gtsam::KeySet margKeys = navStateKeys(snapshot.keyframes);
                // 3. keep the estimate before the update, the new keyframe is not in it yet
                snapshot.values = estimate;
                if (!snapshot.values.exists(X(idxKeyframe)))
                    snapshot.values.insert(X(idxKeyframe), *states.getKeyframeImuPoses().at(idxKeyframe));
                // 4. go over all smoother factors that connect to a marginalized `NavState`, at least in part
                const gtsam::NonlinearFactorGraph &smootherFactors = smoother.getFactors();
                for (gtsam::FactorIndex i = 0; i < smootherFactors.size(); i++)
                {
                    const gtsam::NonlinearFactor::shared_ptr &factor = smootherFactors[i];
                    if (!factor)
                        continue;
                    const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
                    const bool touchesMarg = touchesAny(*factor, margKeys);
                    std::optional<ClusterRows> rows;
                    if (ptp)
                        rows = clusterRows(*ptp, snapshot.values);
                    // 5. every LiDAR cluster factor: its key set and information, the rows of `clusters.csv`
                    if (ptp)
                    {
                        ClusterSpanRow span;
                        span.kf = idxKeyframe;
                        span.clusterId = ptp->clusterId_;
                        for (const gtsam::Key key : ptp->keys())
                            if (gtsam::Symbol(key).chr() == 'x')
                                span.keyframes.push_back(static_cast<uint32_t>(gtsam::Symbol(key).index()));
                        std::sort(span.keyframes.begin(), span.keyframes.end());
                        span.touchesMarg = touchesMarg;
                        if (rows)
                        {
                            double jSq = 0.0;
                            for (const auto &[key, J] : rows->jacobians)
                                jSq += J.squaredNorm();
                            span.info = rows->weight / (rows->sigma * rows->sigma) * jSq;
                            span.weight = rows->weight;
                        }
                        row.clusterSpans.push_back(std::move(span));
                    }
                    if (!touchesMarg)
                        continue;
                    // a LiDAR cluster: record it and its rows before the rebuild
                    if (ptp)
                    {
                        snapshot.factors.push_back({i, ptp->clusterId_});
                        if (rows)
                            snapshot.rebuildBaseline[ptp->clusterId_] = std::move(*rows);
                    }
                }
            }

            /// @brief Computes `lidar_fac_marg_pruned`, `lidar_fac_marg_lost` and the `rebuild_*` columns.
            /// @details Runs in the `KeyframeProbe` destructor, after the smoother update. It reads the state from
            /// before the update from `snapshot`, and the factors that the update added and removed from `smoother`.
            /// 1. factor accounting: a factor on a marginalized keyframe that the update removes marks its cluster as
            ///    rebuilt. Any other such factor goes into the iSAM2 marginalization, and counts as `pruned` (the
            ///    cluster is gone or `Pruned`) or `lost` (the cluster is alive, but has no factor afterwards)
            /// 2. evaluation values: the estimate before the update from `snapshot`, for the old and the rebuilt rows
            /// 3. rebuild comparison per cluster: the rows of the rebuilt factor against the rows of the removed one,
            ///    on their common pose keys, sorted into total, `Tracked` and shifted (`Idle`) groups
            /// 4. reduction: the per-cluster values become the `rebuild_*` columns
            ///
            /// Both columns are 0 when no keyframe is marginalized, and the `rebuild_*` columns stay unset.
            void collectMarginalization(KeyframeTelemetry &row, const MarginalizationSnapshot &snapshot,
                                        const FeatureManager &featureManager, const Smoother &smoother)
            {
                row.lidar_fac_marg_pruned = 0;
                row.lidar_fac_marg_lost = 0;
                if (snapshot.keyframes.empty())
                    return;
                const gtsam::KeySet margKeys = navStateKeys(snapshot.keyframes);
                const gtsam::FactorIndices &factorsToRemove = smoother.getLastFactorsToRemove();
                const std::set<gtsam::FactorIndex> removed(factorsToRemove.begin(), factorsToRemove.end());
                const ClusterRowsMap &rebuildBaseline = snapshot.rebuildBaseline;
                // 2. evaluation values: the estimate before the update, for the old and the rebuilt rows
                const gtsam::Values &values = snapshot.values;
                // 1. factor accounting, clusters whose removed factor touched a marginalized key: their rebuilt factor
                // (without the marginalized keyframes) carries the information of the other points of the cluster
                std::set<uint32_t> rebuiltClusters;
                for (const MargFactor &factor : snapshot.factors)
                {
                    if (removed.count(factor.index))
                    {
                        rebuiltClusters.insert(factor.clusterId);
                        continue;
                    }
                    const auto itState = featureManager.clusterStates_.find(factor.clusterId);
                    if (itState == featureManager.clusterStates_.end() || itState->second == ClusterState::Pruned)
                        (*row.lidar_fac_marg_pruned)++;
                    else
                        (*row.lidar_fac_marg_lost)++;
                }
                // 3. rebuild comparison per cluster: rows of each rebuilt factor vs. its removed factor, on their
                // common pose keys: total, `Tracked` (plane refit at this keyframe) and shifted (`Idle`
                // after `createAndUpdateFactors`, shifted without a refit); any other state counts in the total only
                struct Group
                {
                    double massBefore = 0.0, massAfter = 0.0;
                    std::vector<double> jacRatio;
                };
                Group total, tracked, shifted;
                std::vector<double> sigmaRatio, weightRatio, collapsedPts;
                double margMass = 0.0, margBlanketMass = 0.0;
                for (const gtsam::NonlinearFactor::shared_ptr &factor : smoother.getLastNewFactors())
                {
                    if (!factor)
                        continue;
                    const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
                    if (!ptp || !rebuiltClusters.count(ptp->clusterId_))
                        continue;
                    const auto itBefore = rebuildBaseline.find(ptp->clusterId_);
                    if (itBefore == rebuildBaseline.end())
                        continue;
                    const std::optional<ClusterRows> after = clusterRows(*ptp, values);
                    if (!after)
                        continue;
                    const ClusterRows &before = itBefore->second;
                    double jBefore = 0.0, jAfter = 0.0; // squared norms on the common pose keys
                    double jMarg = 0.0;                 // squared norm on the marginalized pose keys
                    for (const auto &[key, J] : before.jacobians)
                    {
                        const auto itAfter = after->jacobians.find(key);
                        if (margKeys.count(key))
                        {
                            jMarg += J.squaredNorm();
                            continue;
                        }
                        if (itAfter == after->jacobians.end())
                            continue;
                        jBefore += J.squaredNorm();
                        jAfter += itAfter->second.squaredNorm();
                    }
                    if (!(jBefore > 0.0))
                        continue;
                    const double infoBefore = before.weight / (before.sigma * before.sigma);
                    const double massBefore = infoBefore * jBefore;
                    const double massAfter = after->weight / (after->sigma * after->sigma) * jAfter;
                    const double jacRatio = std::sqrt(jAfter / jBefore);
                    margMass += infoBefore * jMarg;
                    margBlanketMass += infoBefore * (jMarg + jBefore);
                    const auto addTo = [&](Group &group)
                    {
                        group.massBefore += massBefore;
                        group.massAfter += massAfter;
                        group.jacRatio.push_back(jacRatio);
                    };
                    addTo(total);
                    if (const auto itState = featureManager.clusterStates_.find(ptp->clusterId_);
                        itState != featureManager.clusterStates_.end())
                    {
                        if (itState->second == ClusterState::Tracked)
                            addTo(tracked);
                        else if (itState->second == ClusterState::Idle)
                            addTo(shifted);
                    }
                    if (jacRatio < kCollapsedJacRatio)
                        collapsedPts.push_back(static_cast<double>(after->jacobians.size()));
                    sigmaRatio.push_back(before.sigma / after->sigma);
                    weightRatio.push_back(std::sqrt(after->weight / before.weight));
                }
                // 4. reduction into the columns
                const auto infoRatio = [](const Group &group)
                { return group.massBefore > 0.0 ? group.massAfter / group.massBefore : field::NaN; };
                row.rebuild_n = total.jacRatio.size();
                row.rebuild_n_tracked = tracked.jacRatio.size();
                row.rebuild_n_shifted = shifted.jacRatio.size();
                row.rebuild_info_ratio = infoRatio(total);
                row.rebuild_info_ratio_tracked = infoRatio(tracked);
                row.rebuild_info_ratio_shifted = infoRatio(shifted);
                row.rebuild_jac_ratio = makeStats(std::move(total.jacRatio));
                row.rebuild_jac_ratio_tracked = makeStats(std::move(tracked.jacRatio));
                row.rebuild_jac_ratio_shifted = makeStats(std::move(shifted.jacRatio));
                row.rebuild_sigma_ratio = makeStats(std::move(sigmaRatio));
                row.rebuild_w_ratio = makeStats(std::move(weightRatio));
                row.rebuild_n_collapsed = collapsedPts.size();
                row.rebuild_collapsed_pts = makeStats(std::move(collapsedPts));
                row.rebuild_marg_share = margBlanketMass > 0.0 ? margMass / margBlanketMass : field::NaN;
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

/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/Instrumentation.hpp>
#include <mapping/factors/PointToPlaneFactor.hpp>
#include <mapping/logging.hpp>

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <limits>
#include <map>
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

                /// @brief Relative threshold for an eigenvalue of a symmetric matrix to count as zero (rank) or as
                /// negative (indefinite), relative to the largest absolute eigenvalue.
                constexpr double kRankTol = 1e-12;
                constexpr double kNegTol = 1e-9;

                /// @brief Information on the Markov blanket after the Schur complement of the marginalized keys.
                struct BlanketInformation
                {
                    /// @brief Symmetrized Schur complement `L_bb - L_bm L_mm^+ L_mb` on the blanket keys.
                    gtsam::Matrix information;
                    /// @brief Offset and dimension of each blanket key in `information`.
                    std::map<gtsam::Key, std::pair<Eigen::Index, size_t>> blanket;
                    /// @brief Condition number of `L_mm`, `inf` if it is not positive definite, `nan` if empty.
                    double mmCond = field::NaN;
                    /// @brief Smallest eigenvalue of the full symmetrized Hessian, divided by its largest one.
                    double hessMinEig = field::NaN;
                    /// @brief False if `L_mm` is rank deficient or the full Hessian is indefinite.
                    bool valid = true;
                };

                /// @brief Linearize the factors at `estimate`, build the dense Hessian with the marginalized keys
                /// first, and form the Schur complement onto the other keys explicitly. `L_mm` is inverted through its
                /// eigendecomposition (pseudo-inverse with relative threshold `kRankTol`), so a near-singular `L_mm`
                /// shows in `mmCond` and `valid` and does not blow up the result.
                BlanketInformation blanketInformation(const std::vector<gtsam::NonlinearFactor::shared_ptr> &factors,
                                                      const gtsam::Values &estimate, const gtsam::KeySet &margKeys)
                {
                    gtsam::GaussianFactorGraph linear;
                    for (const gtsam::NonlinearFactor::shared_ptr &factor : factors)
                        linear.push_back(factor->linearize(estimate));
                    const std::map<gtsam::Key, size_t> dims = linear.getKeyDimMap();
                    gtsam::Ordering ordering;
                    Eigen::Index nm = 0;
                    for (const auto &[key, dim] : dims)
                        if (margKeys.count(key))
                        {
                            ordering.push_back(key);
                            nm += static_cast<Eigen::Index>(dim);
                        }
                    BlanketInformation result;
                    Eigen::Index offset = 0;
                    for (const auto &[key, dim] : dims)
                        if (!margKeys.count(key))
                        {
                            ordering.push_back(key);
                            result.blanket[key] = {offset, dim};
                            offset += static_cast<Eigen::Index>(dim);
                        }
                    const Eigen::Index nb = offset;
                    gtsam::Matrix hessian = linear.hessian(ordering).first;
                    hessian = 0.5 * (hessian + hessian.transpose()).eval();

                    const Eigen::VectorXd eigFull =
                        Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(hessian, Eigen::EigenvaluesOnly).eigenvalues();
                    const double maxAbsFull = eigFull.size() > 0 ? eigFull.cwiseAbs().maxCoeff() : 0.0;
                    if (maxAbsFull > 0.0)
                        result.hessMinEig = eigFull.minCoeff() / maxAbsFull;
                    if (eigFull.size() > 0 && eigFull.minCoeff() < -kNegTol * maxAbsFull)
                        result.valid = false;

                    if (nm == 0)
                    {
                        result.information = hessian;
                        return result;
                    }
                    const Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigMm(hessian.topLeftCorner(nm, nm));
                    const Eigen::VectorXd &lambda = eigMm.eigenvalues(); // ascending
                    const double maxMm = lambda.maxCoeff(), minMm = lambda.minCoeff();
                    result.mmCond = minMm > 0.0 ? maxMm / minMm : std::numeric_limits<double>::infinity();
                    const double threshold = kRankTol * std::max(maxMm, 0.0);
                    if (maxMm <= 0.0 || minMm <= threshold)
                        result.valid = false;
                    Eigen::VectorXd lambdaInv = Eigen::VectorXd::Zero(nm);
                    for (Eigen::Index i = 0; i < nm; i++)
                        if (lambda(i) > threshold && lambda(i) > 0.0)
                            lambdaInv(i) = 1.0 / lambda(i);
                    // L_bm L_mm^+ L_mb = (L_bm V) diag(1/lambda) (L_bm V)^T
                    const gtsam::Matrix bmV = hessian.bottomLeftCorner(nb, nm) * eigMm.eigenvectors();
                    gtsam::Matrix schur =
                        hessian.bottomRightCorner(nb, nb) - bmV * lambdaInv.asDiagonal() * bmV.transpose();
                    result.information = 0.5 * (schur + schur.transpose());
                    return result;
                }

                /// @brief Pose keys `X(j)` on the blanket, ascending.
                gtsam::KeyVector poseKeys(const BlanketInformation &info)
                {
                    gtsam::KeyVector keys;
                    for (const auto &[key, _] : info.blanket)
                        if (gtsam::Symbol(key).chr() == 'x')
                            keys.push_back(key);
                    return keys;
                }

                /// @brief Smallest eigenvalue of the rotation and translation blocks of the joint information of
                /// `poses` (all `3 n` rotation or translation rows, incl. the cross-pose blocks). All other blanket
                /// keys are held fixed. A pose that is not on the blanket has no information, so both values are 0.
                /// Both values are `nan` if `info` is not valid, or if the block is indefinite beyond rounding
                /// (eigenvalue below `-kNegTol` times the largest absolute one). A smaller negative value is rounding
                /// noise and is clamped to 0.
                void setJointMinEigenvalues(const BlanketInformation &info, const gtsam::KeyVector &poses, double &rot,
                                            double &trans)
                {
                    if (!info.valid)
                    {
                        rot = trans = field::NaN;
                        return;
                    }
                    std::vector<Eigen::Index> rotIdx, transIdx;
                    for (const gtsam::Key key : poses)
                    {
                        const auto it = info.blanket.find(key);
                        if (it == info.blanket.end() || it->second.second != 6)
                        {
                            rot = trans = 0.0;
                            return;
                        }
                        for (Eigen::Index i = 0; i < 3; i++)
                        {
                            rotIdx.push_back(it->second.first + i);
                            transIdx.push_back(it->second.first + 3 + i);
                        }
                    }
                    const gtsam::Matrix &information = info.information;
                    const auto minEigenvalue = [&information](const std::vector<Eigen::Index> &idx)
                    {
                        const Eigen::Index n = static_cast<Eigen::Index>(idx.size());
                        if (n == 0)
                            return field::NaN;
                        gtsam::Matrix block(n, n);
                        for (Eigen::Index r = 0; r < n; r++)
                            for (Eigen::Index c = 0; c < n; c++)
                                block(r, c) = information(idx[r], idx[c]);
                        const Eigen::VectorXd eig =
                            Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(block, Eigen::EigenvaluesOnly).eigenvalues();
                        const double minEig = eig.minCoeff();
                        if (minEig >= 0.0)
                            return minEig;
                        return minEig < -kNegTol * eig.cwiseAbs().maxCoeff() ? field::NaN : 0.0;
                    };
                    rot = minEigenvalue(rotIdx);
                    trans = minEigenvalue(transIdx);
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

            void snapshotMarginalization(KeyframeTelemetry &row, gtsam::KeyVector &blanketPoses,
                                         const std::vector<uint32_t> &keyframes, const States &states,
                                         const Smoother &smoother)
            {
                blanketPoses.clear();
                if (keyframes.empty())
                    return;
                row.marg_kf = keyframes.front();
                const gtsam::KeySet margKeys = navStateKeys(keyframes);
                std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
                for (const gtsam::NonlinearFactor::shared_ptr &factor : smoother.getFactors())
                    if (factor && touchesAny(*factor, margKeys))
                        factors.push_back(factor);
                const BlanketInformation info = blanketInformation(factors, states.getSmootherEstimate(), margKeys);
                blanketPoses = poseKeys(info);
                row.marg_ref_mm_cond = info.mmCond;
                row.marg_ref_hess_min_eig = info.hessMinEig;
                setJointMinEigenvalues(info, blanketPoses, row.marg_ref_info_rot, row.marg_ref_info_trans);
            }

            void collectMarginalization(KeyframeTelemetry &row, const std::vector<uint32_t> &keyframes,
                                        const gtsam::KeyVector &blanketPoses, const States &states,
                                        const FeatureManager &featureManager, const Smoother &smoother,
                                        const gtsam::NonlinearFactorGraph &newFactors,
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
                // clusters whose removed factor touched a marginalized key: their rebuilt factor (without the
                // marginalized keyframes) carries the information of the other points of the cluster
                std::set<uint32_t> rebuiltClusters;
                for (gtsam::FactorIndex i = 0; i < smootherFactors.size(); i++)
                {
                    const gtsam::NonlinearFactor::shared_ptr &factor = smootherFactors[i];
                    if (!factor || !touchesAny(*factor, margKeys))
                        continue;
                    const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
                    if (removed.count(i))
                    {
                        if (ptp)
                            rebuiltClusters.insert(ptp->clusterId_);
                        continue;
                    }
                    factors.push_back(factor);
                    if (ptp)
                    {
                        const auto itState = featureManager.clusterStates_.find(ptp->clusterId_);
                        if (itState == featureManager.clusterStates_.end() || itState->second == ClusterState::Pruned)
                            (*row.lidar_fac_marg_pruned)++;
                        else
                            (*row.lidar_fac_marg_lost)++;
                    }
                }
                for (const gtsam::NonlinearFactor::shared_ptr &factor : newFactors)
                {
                    if (!factor)
                        continue;
                    const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
                    if (touchesAny(*factor, margKeys) || (ptp && rebuiltClusters.count(ptp->clusterId_)))
                        factors.push_back(factor);
                }
                // a rebuilt factor can hold the point of the new keyframe, which is not in the estimate yet
                gtsam::Values values = states.getSmootherEstimate();
                const uint32_t idxKeyframe = static_cast<uint32_t>(*row.kf);
                if (!values.exists(X(idxKeyframe)))
                    values.insert(X(idxKeyframe), *states.getKeyframeImuPoses().at(idxKeyframe));
                const BlanketInformation info = blanketInformation(factors, values, margKeys);
                row.marg_cur_mm_cond = info.mmCond;
                row.marg_cur_hess_min_eig = info.hessMinEig;
                setJointMinEigenvalues(info, blanketPoses, row.marg_cur_info_rot, row.marg_cur_info_trans);
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

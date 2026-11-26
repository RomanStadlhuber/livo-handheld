/// @file
/// @ingroup benchmarking
#include <mapping/MappingSystem.hpp>
#include <mapping/benchmarking/Instrumentation.hpp>
#include <mapping/benchmarking/KeyframeProbe.hpp>
#include <mapping/logging.hpp>

#include <exception>

SETUP_LOGS(INFO, "KeyframeProbe")

namespace mapping
{
    namespace benchmarking
    {
        namespace
        {
            /// @brief Run a probe step, log and continue if it throws.
            /// @return Whether the step finished without a throw.
            template <typename Fn> bool runStep(const char *step, uint64_t idxKeyframe, Fn &&fn) noexcept
            {
                try
                {
                    fn();
                    return true;
                }
                catch (const std::exception &e)
                {
                    LOG(WARN, "keyframe " << idxKeyframe << ": telemetry " << step << " failed: " << e.what());
                    return false;
                }
                catch (...)
                {
                    LOG(WARN, "keyframe " << idxKeyframe << ": telemetry " << step << " failed");
                    return false;
                }
            }
        } // namespace

        KeyframeProbe::KeyframeProbe(MappingSystem &system, uint32_t idxKeyframe) noexcept : system_(system)
        {
            row_.kf = idxKeyframe;
            runStep(
                "snapshot", idxKeyframe, [&]
                { instrumentation::snapshot(row_, imuPropagated_, system_.states_, system_.smoother_, idxKeyframe); });
            // nothing between the keyframe creation and the marginalization changes the smoother or the keyframes
            const bool snapshotOk =
                runStep("marginalization snapshot", idxKeyframe,
                        [&]
                        {
                            instrumentation::snapshotMarginalization(
                                row_, marginalization_, system_.states_, system_.smoother_, idxKeyframe,
                                static_cast<uint32_t>(system_.config_.backend.sliding_window_size));
                        });
            if (!snapshotOk)
            {
                // a half-filled snapshot must not reach the collect step: all marginalization columns stay nan
                marginalization_ = {};
                row_.marg_kf.reset();
                marginalizationValid_ = false;
            }
        }

        KeyframeProbe::~KeyframeProbe()
        {
            const States &states = system_.states_;
            row_.lost_tracking = states.getLifecycleState() == SystemState::Recovery ? 1 : 0;
            const auto &numValidClusters = system_.lidarFrontend_.keyframeNumValidClusters;
            if (const auto it = numValidClusters.find(*row_.kf); it != numValidClusters.end())
                row_.clusters_valid = it->second;
            // The smoother copies of the added and removed factors belong to this keyframe only when the update of
            // this keyframe ran. Lost tracking and an exception in `track()` before the update leave the copies of
            // the previous keyframe, so the collect step is skipped and the marginalization columns stay nan.
            // The collect step is also skipped when the snapshot step failed.
            const auto &lastUpdate = system_.smoother_.getLastUpdateKeyframe();
            if (!row_.lost_tracking && lastUpdate && *lastUpdate == *row_.kf && marginalizationValid_)
                runStep("marginalization", *row_.kf,
                        [&] {
                            instrumentation::collectMarginalization(row_, marginalization_, system_.featureManager_,
                                                                    system_.smoother_);
                        });
            instrumentation::collect(row_, imuPropagated_, states, system_.featureManager_, system_.smoother_);
            if (system_.groundTruth_ && !row_.lost_tracking)
                runStep("ground truth", *row_.kf,
                        [&]
                        {
                            const auto &planarity = system_.config_.lidar_frontend.planarity_check;
                            instrumentation::collectGroundTruth(row_, states, system_.smoother_,
                                                                system_.featureManager_, *system_.groundTruth_,
                                                                planarity.planarity, planarity.linearity);
                        });
            try
            {
                system_.telemetryQueue_.push_back(std::move(row_));
            }
            catch (...)
            {
                LOG(ERROR, "keyframe " << *row_.kf << ": failed to queue the telemetry row");
            }
        }
    } // namespace benchmarking
} // namespace mapping

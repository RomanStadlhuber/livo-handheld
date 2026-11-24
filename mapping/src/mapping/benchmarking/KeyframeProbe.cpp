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
            template <typename Fn> void runStep(const char *step, uint64_t idxKeyframe, Fn &&fn) noexcept
            {
                try
                {
                    fn();
                }
                catch (const std::exception &e)
                {
                    LOG(WARN, "keyframe " << idxKeyframe << ": telemetry " << step << " failed: " << e.what());
                }
                catch (...)
                {
                    LOG(WARN, "keyframe " << idxKeyframe << ": telemetry " << step << " failed");
                }
            }
        } // namespace

        KeyframeProbe::KeyframeProbe(MappingSystem &system, uint32_t idxKeyframe) noexcept : system_(system)
        {
            row_.kf = idxKeyframe;
            runStep(
                "snapshot", idxKeyframe, [&]
                { instrumentation::snapshot(row_, imuPropagated_, system_.states_, system_.smoother_, idxKeyframe); });
        }

        void KeyframeProbe::beforeMarginalization() noexcept
        {
            runStep("marginalization snapshot", *row_.kf,
                    [&]
                    {
                        marginalizedKeyframes_ = instrumentation::marginalizedKeyframes(
                            system_.states_, static_cast<uint32_t>(*row_.kf),
                            static_cast<uint32_t>(system_.config_.backend.sliding_window_size));
                        instrumentation::snapshotMarginalization(row_, marginalizedKeyframes_, system_.states_,
                                                                 system_.smoother_);
                    });
        }

        void KeyframeProbe::beforeSmootherUpdate(const gtsam::NonlinearFactorGraph &newFactors,
                                                 const gtsam::FactorIndices &factorsToRemove) noexcept
        {
            runStep("marginalization", *row_.kf,
                    [&]
                    {
                        instrumentation::collectMarginalization(row_, marginalizedKeyframes_, system_.states_,
                                                                system_.featureManager_, system_.smoother_, newFactors,
                                                                factorsToRemove);
                    });
        }

        KeyframeProbe::~KeyframeProbe()
        {
            const States &states = system_.states_;
            row_.lost_tracking = states.getLifecycleState() == SystemState::Recovery ? 1 : 0;
            const auto &numValidClusters = system_.lidarFrontend_.keyframeNumValidClusters;
            if (const auto it = numValidClusters.find(*row_.kf); it != numValidClusters.end())
                row_.clusters_valid = it->second;
            instrumentation::collect(row_, imuPropagated_, states, system_.featureManager_, system_.smoother_);
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

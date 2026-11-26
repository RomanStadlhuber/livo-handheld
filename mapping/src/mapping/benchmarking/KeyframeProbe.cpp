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
            // nothing between the keyframe creation and the marginalization changes the smoother or the keyframes
            runStep("marginalization snapshot", idxKeyframe,
                    [&]
                    {
                        instrumentation::snapshotMarginalization(
                            row_, marginalization_, system_.states_, system_.smoother_, idxKeyframe,
                            static_cast<uint32_t>(system_.config_.backend.sliding_window_size));
                    });
        }

        KeyframeProbe::~KeyframeProbe()
        {
            const States &states = system_.states_;
            row_.lost_tracking = states.getLifecycleState() == SystemState::Recovery ? 1 : 0;
            const auto &numValidClusters = system_.lidarFrontend_.keyframeNumValidClusters;
            if (const auto it = numValidClusters.find(*row_.kf); it != numValidClusters.end())
                row_.clusters_valid = it->second;
            if (!row_.lost_tracking) // no smoother update at lost tracking, the smoother holds the last keyframe
                runStep("marginalization", *row_.kf,
                        [&] {
                            instrumentation::collectMarginalization(row_, marginalization_, system_.featureManager_,
                                                                    system_.smoother_);
                        });
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

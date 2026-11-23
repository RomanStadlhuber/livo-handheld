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
        KeyframeProbe::KeyframeProbe(MappingSystem &system, uint32_t idxKeyframe) noexcept : system_(system)
        {
            row_.kf = idxKeyframe;
            try
            {
                instrumentation::snapshot(row_, imuPropagated_, system_.states_, idxKeyframe);
            }
            catch (const std::exception &e)
            {
                LOG(WARN, "keyframe " << idxKeyframe << ": telemetry snapshot failed: " << e.what());
            }
            catch (...)
            {
                LOG(WARN, "keyframe " << idxKeyframe << ": telemetry snapshot failed");
            }
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

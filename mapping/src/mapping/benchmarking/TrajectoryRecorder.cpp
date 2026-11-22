/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/TrajectoryRecorder.hpp>

#include <fstream>
#include <iomanip>

namespace mapping
{
    void TrajectoryRecorder::update(const SlidingWindowStates &states, uint32_t keyframeCount)
    {
        for (const auto &[idxKf, state] : states)
            if (idxKf < keyframeCount)
                keyframes_[idxKf] = state;
    }

    bool TrajectoryRecorder::writeTum(const std::string &path, double timeOffset) const
    {
        std::ofstream file(path);
        if (!file)
            return false;
        file << "# timestamp x y z qx qy qz qw\n" << std::fixed << std::setprecision(9);
        for (const auto &[_, stamped] : keyframes_)
        {
            const gtsam::Pose3 &pose = stamped.state.pose();
            const gtsam::Point3 &t = pose.translation();
            const gtsam::Quaternion q = pose.rotation().toQuaternion();
            file << (stamped.timestamp + timeOffset) << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' ' << q.x()
                 << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        }
        file.flush();
        return file.good();
    }
} // namespace mapping

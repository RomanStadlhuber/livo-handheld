/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/GroundTruth.hpp>

#include <algorithm>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace mapping
{
    namespace benchmarking
    {
        GroundTruth GroundTruth::loadTum(const std::string &path)
        {
            std::ifstream file(path);
            if (!file)
                throw std::runtime_error("cannot open ground truth file '" + path + "'");
            std::vector<std::pair<double, gtsam::Pose3>> entries;
            std::string line;
            while (std::getline(file, line))
            {
                if (line.empty() || line[0] == '#')
                    continue;
                std::istringstream ss(line);
                double t, x, y, z, qx, qy, qz, qw;
                if (!(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw))
                    continue;
                entries.emplace_back(t, gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(x, y, z)));
            }
            if (entries.size() < 2)
                throw std::runtime_error("ground truth file '" + path + "' holds fewer than two poses");
            std::sort(entries.begin(), entries.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
            GroundTruth gt;
            for (auto &[t, pose] : entries)
            {
                gt.stamps_.push_back(t);
                gt.poses_.push_back(pose);
            }
            return gt;
        }

        void GroundTruth::shiftTime(double t0)
        {
            for (double &t : stamps_)
                t -= t0;
        }

        std::optional<std::size_t> GroundTruth::bracket(double t) const
        {
            if (stamps_.size() < 2 || t < stamps_.front() || t > stamps_.back())
                return std::nullopt;
            auto it = std::upper_bound(stamps_.begin(), stamps_.end(), t);
            if (it == stamps_.end())
                --it; // t == back()
            return static_cast<std::size_t>(std::distance(stamps_.begin(), it)) - 1;
        }

        std::optional<gtsam::Pose3> GroundTruth::poseAt(double t) const
        {
            const auto i = bracket(t);
            if (!i)
                return std::nullopt;
            const double t0 = stamps_[*i], t1 = stamps_[*i + 1];
            const double a = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0;
            const gtsam::Pose3 &p0 = poses_[*i], &p1 = poses_[*i + 1];
            const gtsam::Rot3 R = p0.rotation().slerp(a, p1.rotation());
            const gtsam::Point3 p = (1.0 - a) * p0.translation() + a * p1.translation();
            return gtsam::Pose3(R, p);
        }

        std::optional<Eigen::Vector3d> GroundTruth::velocityAt(double t) const
        {
            const auto i = bracket(t);
            if (!i)
                return std::nullopt;
            const double dt = stamps_[*i + 1] - stamps_[*i];
            if (!(dt > 0.0))
                return std::nullopt;
            return Eigen::Vector3d((poses_[*i + 1].translation() - poses_[*i].translation()) / dt);
        }
    } // namespace benchmarking
} // namespace mapping

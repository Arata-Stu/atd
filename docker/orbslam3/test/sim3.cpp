#include <CameraModels/Pinhole.h>
#include <Sim3Solver.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace {
using Cloud = std::array<Eigen::Vector3f, 4>;

// Inject matched camera-space points at the boundary immediately after keyframe matching.
// Sampling, the Horn solve, forward/backward projections and inlier tests use the core.
class Solver : public ORB_SLAM3::Sim3Solver {
public:
    Solver(ORB_SLAM3::KeyFrame& frame, const Cloud& first, const Cloud& second, bool fixed_scale)
        : Sim3Solver(&frame, &frame, {}, fixed_scale) {
        mvX3Dc1.assign(first.begin(), first.end());
        mvX3Dc2.assign(second.begin(), second.end());
        mN1 = first.size();
        mvpMapPoints1.resize(mN1);  // SetRansacParameters reads the number of matches.
        mvnIndices1.resize(mN1);
        std::iota(mvnIndices1.begin(), mvnIndices1.end(), 0);
        mvAllIndices = mvnIndices1;
        mvnMaxError1.assign(mN1, 9);
        mvnMaxError2.assign(mN1, 9);
        FromCameraToImage(mvX3Dc1, mvP1im1, pCamera1);
        FromCameraToImage(mvX3Dc2, mvP2im2, pCamera2);
        SetRansacParameters(0.99, 3, 50);
    }
};

void check(ORB_SLAM3::KeyFrame& frame, const std::string& name, const Cloud& first,
           const Cloud& second, bool fixed_scale, const Eigen::Matrix4f* expected) {
    for (bool with_convergence : {false, true}) {
        Solver solver(frame, first, second, fixed_scale);
        bool exhausted = false, converged = false;
        int count = 0;
        std::vector<bool> inliers;
        const Eigen::Matrix4f pose = with_convergence
            ? solver.iterate(50, exhausted, inliers, count, converged)
            : solver.iterate(50, exhausted, inliers, count);
        const auto marked = std::count(inliers.begin(), inliers.end(), true);
        if (expected) {
            if (count != static_cast<int>(first.size()) || marked != count ||
                (with_convergence && !converged) || !pose.isApprox(*expected, 2e-5f))
                throw std::runtime_error(name + " failed to recover the known transform");
        } else if (!exhausted || converged || count != 0 || marked != 0 || !pose.isIdentity()) {
            throw std::runtime_error(name + " accepted an invalid sample or produced an invalid result");
        }
    }
    std::cout << "Passed " << name << '\n';
}
}  // namespace

int main() {
    try {
        ORB_SLAM3::Pinhole camera({420.f, 420.f, 320.f, 240.f});
        ORB_SLAM3::KeyFrame frame;
        frame.mpCamera = &camera;
        frame.SetPose(Sophus::SE3f());
        const Cloud points{{{1,1,6}, {1,-1,6}, {-2,0,6}, {0,1,8}}};
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        const auto check_transform = [&](const char* name, bool fixed_scale) {
            Cloud transformed;
            for (size_t i = 0; i < points.size(); ++i)
                transformed[i] = transform.topLeftCorner<3,3>() * points[i] + transform.topRightCorner<3,1>();
            check(frame, name, transformed, points, fixed_scale, &transform);
        };
        check_transform("identity", true);
        transform.topRightCorner<3,1>() << 3.f, 0.f, 1.f;
        check_transform("translation", true);
        transform.topLeftCorner<3,3>() =
            Eigen::AngleAxisf(0.41f, Eigen::Vector3f(1, 2, 3).normalized()).toRotationMatrix();
        check_transform("rotation and translation", true);
        transform.topLeftCorner<3,3>() *= 1.6f;
        check_transform("estimated scale", false);

        const Cloud line{{{-2,0,6}, {-1,0,6}, {0,0,6}, {1,0,6}}};
        check(frame, "collinear matches", line, line, true, nullptr);
        check(frame, "one-sided collinearity", points, line, false, nullptr);
        Cloud duplicate;
        duplicate.fill(points.front());
        check(frame, "coincident matches", points, duplicate, false, nullptr);
        Cloud invalid = points;
        for (auto& point : invalid) point.x() = std::numeric_limits<float>::quiet_NaN();
        check(frame, "non-finite matches", invalid, points, true, nullptr);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

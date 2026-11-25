#include <System.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace fs = std::filesystem;

namespace {
constexpr int width = 640, height = 480;
constexpr float focal = 420.0F, baseline = 0.095F;
constexpr std::array<float, 4> depths{2.0F, 3.0F, 4.0F, 2.5F};

// Four textured fronto-parallel strips at different depths provide known geometry.
// A camera translated to the right sees every strip shift left by fx * x / depth.
cv::Mat render(const cv::Mat& scene, float x) {
    cv::Mat image(height, width, CV_8U);
    for (int i = 0; i < 4; ++i) {
        const cv::Rect strip(0, i * height / 4, width, height / 4);
        const cv::Point2f center(0.5F * (width - 1) + 300 + focal * x / depths[i],
                                 strip.y + 0.5F * (strip.height - 1));
        cv::Mat crop;
        cv::getRectSubPix(scene, strip.size(), center, crop);
        crop.copyTo(image(strip));
    }
    return image;
}

void write_settings(const fs::path& path, const fs::path& stem, bool fixed) {
    std::ofstream out(path);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << "%YAML:1.0\nFile.version: \"1.0\"\nCamera.type: \"Rectified\"\n"
        << "Camera1.fx: 420.0\nCamera1.fy: 420.0\nCamera1.cx: 319.5\nCamera1.cy: 239.5\n"
        << "Camera.width: 640\nCamera.height: 480\nCamera.fps: 25\nCamera.RGB: 0\n"
        << "Stereo.b: 0.095\nStereo.ThDepth: 50.0\nORBextractor.nFeatures: 1800\n"
        << "ORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\n"
        << "ORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n"
        << (fixed ? "System.LoadAtlasFromFile" : "System.SaveAtlasToFile") << ": " << stem << '\n';
}

void check_landmarks(const fs::path& directory, bool fixed, const std::vector<Eigen::Vector3f>& points) {
    std::vector<std::array<float, 3>> xyz;
    for (const auto& point : points) {
        if (!point.allFinite()) throw std::runtime_error("Non-finite saved landmark");
        xyz.push_back({point.x(), point.y(), point.z()});
    }
    if (xyz.size() < 50) throw std::runtime_error("Too few saved landmarks");
    for (float z : depths) {
        const auto on_surface = std::count_if(points.begin(), points.end(), [z](const Eigen::Vector3f& p) {
            return std::abs(p.z() - z) < 0.15F;
        });
        if (on_surface < 10) throw std::runtime_error("Saved landmarks do not cover the synthetic depth planes");
    }
    // Map point allocation addresses are different in a fresh process. Compare geometry.
    std::sort(xyz.begin(), xyz.end());
    const fs::path expected = directory / "landmarks.bin";
    if (!fixed) {
        std::ofstream output(expected, std::ios::binary);
        output.exceptions(std::ios::badbit | std::ios::failbit);
        for (const auto& point : xyz)
            output.write(reinterpret_cast<const char*>(point.data()), 3 * sizeof(float));
    } else {
        std::ifstream input(expected, std::ios::binary);
        input.exceptions(std::ios::badbit | std::ios::failbit);
        for (const auto& point : xyz) {
            std::array<float, 3> original;
            input.read(reinterpret_cast<char*>(original.data()), 3 * sizeof(float));
            if (original != point) throw std::runtime_error("Reloaded Atlas landmarks differ from mapping");
        }
        if (input.peek() != std::ifstream::traits_type::eof())
            throw std::runtime_error("Reloaded Atlas has fewer landmarks than the saved map");
        std::cout << "Reloaded " << xyz.size() << " identical finite landmarks\n";
    }
}
}  // namespace

// Run `map` and `localize` in separate processes against a new temporary directory.
int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::runtime_error("Usage: atd_orbslam3_smoke map|localize VOCAB DIRECTORY");
        const std::string mode = argv[1];
        const bool fixed = mode == "localize";
        if (!fixed && mode != "map") throw std::runtime_error("Unknown mode");
        const fs::path directory = fs::absolute(argv[3]);
        const auto stem = directory / "course";
        if (!fixed && fs::exists(stem.string() + ".osa")) throw std::runtime_error("Use a new map directory");
        fs::create_directories(directory);
        const auto settings = directory / (mode + ".yaml");
        write_settings(settings, stem, fixed);
        ORB_SLAM3::System system(argv[2], settings, ORB_SLAM3::System::STEREO, false, 0, "", fixed);
        cv::Mat texture(height, 2000, CV_8U);
        cv::RNG rng(627);
        rng.fill(texture, cv::RNG::UNIFORM, 0, 255);
        cv::GaussianBlur(texture, texture, cv::Size(3, 3), 0.6);
        int stamp = 0;
        cv::Mat blank = cv::Mat::zeros(height, width, CV_8U);
        if (!fixed) {
            system.TrackStereo(render(texture, 0.0F), blank, ++stamp / 25.0);
            if (system.GetTrackingState() == ORB_SLAM3::Tracking::OK || system.GetTrackingMapOriginId() >= 0)
                throw std::runtime_error("Initialized a map without stereo matches");
            for (int frame = 0; frame < 85; ++frame) {
                const float x = frame * 0.018F;
                system.TrackStereo(render(texture, x), render(texture, x + baseline), ++stamp / 25.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (system.GetTrackingState() != ORB_SLAM3::Tracking::OK || system.GetTrackingMapOriginId() < 0)
                throw std::runtime_error("Synthetic mapping did not finish tracking");
        } else {
            cv::Mat unrelated(height, 2000, CV_8U);
            rng.fill(unrelated, cv::RNG::UNIFORM, 0, 255);
            for (const auto [start, step] : {std::pair{0.91F, -0.03F}, std::pair{0.37F, 0.03F}}) {
                // At startup and again after tracking loss, a blank stereo pair cannot localize.
                system.TrackStereo(blank, blank, ++stamp / 25.0);
                if (system.GetFixedMapInliers() != 0 || system.GetTrackingState() == ORB_SLAM3::Tracking::OK)
                    throw std::runtime_error("Reported a fixed-map pose on a blank image");
                system.TrackStereo(render(unrelated, start), render(unrelated, start + baseline), ++stamp / 25.0);
                if (system.GetFixedMapInliers() != 0 || system.GetTrackingState() == ORB_SLAM3::Tracking::OK)
                    throw std::runtime_error("Localized unrelated texture in the saved map");
                int verified = 0;
                float worst_error = 0;
                // Move towards the other end of the mapped area after each acquisition.
                // A tracker must sustain the known motion, including across reference keyframes.
                float x = start - step;
                Eigen::Vector3f previous_position = Eigen::Vector3f::Zero();
                constexpr int acquisition_frames = 8, continuous_frames = 20;
                for (int i = 0; i < acquisition_frames + continuous_frames; ++i) {
                    // Skip one stereo pair while preserving the true timestamp and displacement.
                    const int ticks = i == acquisition_frames + 6 ? 2 : 1;
                    stamp += ticks;
                    x += ticks * step;
                    const auto pose = system.TrackStereo(render(texture, x), render(texture, x + baseline), stamp / 25.0);
                    if (system.GetFixedMapInliers() <= 0 || system.GetTrackingState() != ORB_SLAM3::Tracking::OK) {
                        if (i >= acquisition_frames)
                            throw std::runtime_error("Lost the fixed map during continuous motion");
                        continue;
                    }
                    const Eigen::Vector3f position = pose.inverse().translation();
                    const float error = (position - Eigen::Vector3f(x, 0, 0)).norm();
                    worst_error = std::max(worst_error, error);
                    if (!std::isfinite(error) || error > 0.10F) throw std::runtime_error("Returned a pose in the wrong coordinate frame");
                    if (i > acquisition_frames) {
                        const float step_error = (position - previous_position - Eigen::Vector3f(ticks * step, 0, 0)).norm();
                        if (step_error > 0.02F)
                            throw std::runtime_error("Fixed-map pose jumped between consecutive frames");
                    }
                    previous_position = position;
                    ++verified;
                }
                if (verified < continuous_frames + 3) throw std::runtime_error("Failed to recognize the requested course segment");
                std::cout << "Verified " << verified << " fixed-map poses starting at x=" << start
                          << " including " << continuous_frames << " consecutive poses, maximum error=" << worst_error << " m\n";
            }
        }
        bool premature_export = false;
        try {
            (void)system.GetSavedMapPoints();
            premature_export = true;
        } catch (const std::logic_error&) {
            // The mapper may still be optimizing these coordinates.
        }
        if (premature_export) throw std::runtime_error("Exported landmarks from an active session");
        system.Shutdown();
        check_landmarks(directory, fixed, system.GetSavedMapPoints());
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

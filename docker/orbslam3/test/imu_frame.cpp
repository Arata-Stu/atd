#include <CameraModels/Pinhole.h>
#include <Frame.h>
#include <LocalMapping.h>
#include <ORBextractor.h>

#include <iostream>

int main() {
    // The tracking thread can query the mapper before its first keyframe is dequeued.
    ORB_SLAM3::LocalMapping mapper(nullptr, nullptr, false, true);
    if (mapper.GetCurrKFTime() != 0.0 || mapper.mFirstTs.load() != 0.0) {
        std::cerr << "Unexpected IMU initialization time before the first keyframe\n";
        return 1;
    }

    const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat intrinsics = (cv::Mat_<float>(3, 3) << 337, 0, 320, 0, 337, 240, 0, 0, 1);
    cv::Mat distortion = cv::Mat::zeros(4, 1, CV_32F);
    ORB_SLAM3::Pinhole camera({337, 337, 320, 240});
    ORB_SLAM3::ORBextractor left(1250, 1.2f, 8, 20, 7), right(1250, 1.2f, 8, 20, 7);
    ORB_SLAM3::Frame frame(blank, blank, 0.0, &left, &right, nullptr, intrinsics,
                           distortion, 337.0f * 0.095f, 40.0f, &camera);

    // Tracking::PreintegrateIMU marks the current stereo frame even when it has no keypoints.
    frame.setIntegrated();
    if (!frame.imuIsPreintegrated() || frame.N != 0 || frame.Nleft != -1 || frame.Nright != -1) {
        std::cerr << "Featureless stereo frame lost its IMU or stereo state\n";
        return 1;
    }
    return 0;
}

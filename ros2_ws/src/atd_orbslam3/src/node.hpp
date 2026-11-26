#pragma once

#include <memory>
#include <rclcpp/node.hpp>

namespace atd_orbslam3 {
std::shared_ptr<rclcpp::Node> make_orbslam_node(const rclcpp::NodeOptions & options);
}

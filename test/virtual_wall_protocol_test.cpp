#include "behavior_tree/plugins/action/dataReceiver/virtual_wall_ros.h"
#include <gtest/gtest.h>

namespace {

virtual_walls::Snapshot walls() {
    virtual_walls::Snapshot result;
    result.frame = "map";
    virtual_walls::Wall wall;
    wall.id = 17;
    wall.start = Eigen::Vector3d(0, 0, 0.4);
    wall.end = Eigen::Vector3d(2, 0, 1.0);
    wall.height = 1.5;
    wall.thickness = 0.2;
    result.walls.push_back(wall);
    return result;
}

TEST(VirtualWallProtocol, RoundTripRetainsSlopeDimensionsAndTilt) {
    auto input = walls();
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()).toRotationMatrix();
    auto &wall = input.walls.front();
    wall.start = rotation * wall.start;
    wall.end = rotation * wall.end;
    wall.up = rotation * wall.up;
    const auto decoded = pp_behavior_tree::virtual_wall_ros::decode(
        pp_behavior_tree::virtual_wall_ros::encode(input, builtin_interfaces::msg::Time()));
    ASSERT_EQ(decoded->walls.size(), 1u);
    EXPECT_EQ(decoded->frame, "map");
    const auto &actual = decoded->walls.front();
    EXPECT_EQ(actual.id, 17);
    EXPECT_NEAR((actual.start - wall.start).norm(), 0, 1e-12);
    EXPECT_NEAR((actual.end - wall.end).norm(), 0, 1e-12);
    EXPECT_NEAR((actual.up - wall.up).norm(), 0, 1e-12);
    EXPECT_NEAR(actual.height, 1.5, 1e-12);
    EXPECT_DOUBLE_EQ(actual.thickness, 0.2);
}

TEST(VirtualWallProtocol, EmptySnapshotCarriesExplicitClear) {
    virtual_walls::Snapshot input;
    input.frame = "map";
    const auto message =
        pp_behavior_tree::virtual_wall_ros::encode(input, builtin_interfaces::msg::Time());
    ASSERT_EQ(message.markers.size(), 1u);
    EXPECT_EQ(message.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
    EXPECT_TRUE(pp_behavior_tree::virtual_wall_ros::decode(message)->walls.empty());
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(visualization_msgs::msg::MarkerArray()),
                 std::invalid_argument);
}

TEST(VirtualWallProtocol, RejectsPartialMixedFrameAndDuplicateSnapshots) {
    auto message =
        pp_behavior_tree::virtual_wall_ros::encode(walls(), builtin_interfaces::msg::Time());
    auto invalid = message;
    invalid.markers.erase(invalid.markers.begin());
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(invalid), std::invalid_argument);
    invalid = message;
    invalid.markers.back().header.frame_id = "odom";
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(invalid), std::invalid_argument);
    invalid = message;
    invalid.markers.push_back(invalid.markers.back());
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(invalid), std::invalid_argument);
}

TEST(VirtualWallProtocol, RejectsNonfiniteAndMalformedParallelograms) {
    auto message =
        pp_behavior_tree::virtual_wall_ros::encode(walls(), builtin_interfaces::msg::Time());
    auto invalid = message;
    invalid.markers.back().points[2].z = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(invalid), std::invalid_argument);
    invalid = message;
    invalid.markers.back().points[4].x += 0.1;
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(invalid), std::invalid_argument);
    invalid = message;
    invalid.markers.back().scale.x = -1;
    EXPECT_THROW(pp_behavior_tree::virtual_wall_ros::decode(invalid), std::invalid_argument);
}

TEST(VirtualWallGeometry, BodyIntersectionClipsASlopedWall) {
    auto wall = walls().walls.front();
    wall.start = Eigen::Vector3d(0, 0, 0);
    wall.end = Eigen::Vector3d(2, 0, 2);
    const auto polygon = virtual_walls::bodyPolygon(wall, 0, 0.5);
    EXPECT_DOUBLE_EQ(virtual_walls::distanceXY(polygon, 0.25, 0), 0);
    EXPECT_NEAR(virtual_walls::distanceXY(polygon, 1.5, 0), 1.0, 1e-12);
    EXPECT_NEAR(virtual_walls::distanceXY(polygon, 0.25, 0.5), 0.4, 1e-12);
    EXPECT_TRUE(virtual_walls::bodyPolygon(wall, 4, 5).empty());
}

TEST(VirtualWallGeometry, HorizontalWallUsesItsThicknessAsTheVerticalExtent) {
    virtual_walls::Wall wall;
    wall.up = Eigen::Vector3d::UnitY();
    wall.thickness = 0.2;
    EXPECT_FALSE(virtual_walls::bodyPolygon(wall, -0.05, 0.05).empty());
    EXPECT_TRUE(virtual_walls::bodyPolygon(wall, 0.2, 0.5).empty());
    wall.end = wall.start;
    EXPECT_FALSE(wall.valid());
}

} // namespace

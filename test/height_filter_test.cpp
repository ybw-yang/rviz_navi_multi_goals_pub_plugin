#include "navi_multi_goals_pub_rviz_plugin/height_filter.hpp"
#include <gtest/gtest.h>

namespace {

sensor_msgs::msg::PointCloud2 cloud(bool big = false) {
    sensor_msgs::msg::PointCloud2 result;
    result.header.frame_id = "map";
    result.width = 2;
    result.height = 2;
    result.point_step = 16;
    result.row_step = 40;
    result.is_bigendian = big;
    for (int i = 0; i < 4; ++i) {
        sensor_msgs::msg::PointField field;
        field.name = i == 3 ? "intensity" : std::string(1, "xyz"[i]);
        field.offset = i * 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        result.fields.push_back(field);
    }
    result.data.assign(80, 0xff);
    for (int i = 0; i < 4; ++i) {
        const std::array<float, 4> point = {float(i), 0, float(i - 1), float(i * 10)};
        for (int axis = 0; axis < 4; ++axis) {
            std::array<uint8_t, 4> bytes;
            std::memcpy(bytes.data(), &point[axis], 4);
            const uint16_t endian_test = 1;
            const bool host_big = *reinterpret_cast<const uint8_t *>(&endian_test) == 0;
            if (big != host_big)
                std::reverse(bytes.begin(), bytes.end());
            std::memcpy(result.data.data() + (i / 2) * 40 + (i % 2) * 16 + axis * 4, bytes.data(),
                        4);
        }
    }
    return result;
}

TEST(CloudHeightFilter, InclusiveIntervalRetainsFieldsAndStripsRowPadding) {
    const auto input = cloud();
    const auto result = navi_multi_goals_pub_rviz_plugin::filterHeight(input, true, 0, 1);
    EXPECT_EQ(result.cloud->width, 2u);
    EXPECT_EQ(result.cloud->height, 1u);
    EXPECT_EQ(result.cloud->row_step, 32u);
    EXPECT_EQ(result.cloud->header.frame_id, "map");
    EXPECT_EQ(result.cloud->fields, input.fields);
    EXPECT_EQ(result.finite_count, 4u);
    EXPECT_DOUBLE_EQ(result.minimum, -1);
    EXPECT_DOUBLE_EQ(result.maximum, 2);
    EXPECT_TRUE(
        std::equal(input.data.begin() + 16, input.data.begin() + 32, result.cloud->data.begin()));
    EXPECT_TRUE(std::equal(input.data.begin() + 40, input.data.begin() + 56,
                           result.cloud->data.begin() + 16));
    EXPECT_EQ(input.data.size(), 80u);
}

TEST(CloudHeightFilter, SupportsBigEndianInput) {
    const auto input = cloud(true);
    const auto result = navi_multi_goals_pub_rviz_plugin::filterHeight(input, true, -1, -1);
    EXPECT_EQ(result.cloud->width, 1u);
    EXPECT_TRUE(result.cloud->is_bigendian);
    EXPECT_TRUE(
        std::equal(input.data.begin(), input.data.begin() + 16, result.cloud->data.begin()));
}

TEST(CloudHeightFilter, EmptySelectionAndShowAllUseTheSameOriginalSnapshot) {
    const auto input = cloud();
    EXPECT_TRUE(
        navi_multi_goals_pub_rviz_plugin::filterHeight(input, true, 3, 4).cloud->data.empty());
    EXPECT_EQ(navi_multi_goals_pub_rviz_plugin::filterHeight(input, false, 3, 4).cloud->width, 4u);
    EXPECT_EQ(navi_multi_goals_pub_rviz_plugin::filterHeight(input, true, -1, 2).cloud->width, 4u);
}

TEST(CloudHeightFilter, RejectsInvalidLayoutAndDropsNonfiniteXYZ) {
    auto input = cloud();
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(input.data.data(), &invalid, 4);
    EXPECT_EQ(navi_multi_goals_pub_rviz_plugin::filterHeight(input, false, 0, 1).cloud->width, 3u);
    EXPECT_THROW(navi_multi_goals_pub_rviz_plugin::filterHeight(input, true, 2, 1),
                 std::invalid_argument);
    input.data.resize(4);
    EXPECT_THROW(navi_multi_goals_pub_rviz_plugin::filterHeight(input, false, 0, 1),
                 std::invalid_argument);
    input = cloud();
    input.fields[2].offset = 16;
    EXPECT_THROW(navi_multi_goals_pub_rviz_plugin::filterHeight(input, false, 0, 1),
                 std::invalid_argument);
}

TEST(CloudHeightFilter, SupportsDoublePrecisionCoordinates) {
    sensor_msgs::msg::PointCloud2 input;
    input.width = input.height = 1;
    input.point_step = input.row_step = 24;
    input.data.resize(24);
    const std::array<double, 3> point = {1.0, 2.0, 0.125};
    std::memcpy(input.data.data(), point.data(), 24);
    const uint16_t endian_test = 1;
    input.is_bigendian = *reinterpret_cast<const uint8_t *>(&endian_test) == 0;
    for (int axis = 0; axis < 3; ++axis) {
        sensor_msgs::msg::PointField field;
        field.name = std::string(1, "xyz"[axis]);
        field.offset = axis * 8;
        field.count = 1;
        field.datatype = sensor_msgs::msg::PointField::FLOAT64;
        input.fields.push_back(field);
    }
    const auto result = navi_multi_goals_pub_rviz_plugin::filterHeight(input, true, 0.125, 0.125);
    EXPECT_EQ(result.cloud->data, input.data);
    EXPECT_DOUBLE_EQ(result.minimum, 0.125);
}

} // namespace

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>

namespace navi_multi_goals_pub_rviz_plugin {

struct CloudHeightResult {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    size_t finite_count = 0;
};

inline CloudHeightResult filterHeight(const sensor_msgs::msg::PointCloud2 &input, bool enabled,
                                      double lower, double upper) {
    if (enabled && (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper))
        throw std::invalid_argument("Invalid height interval");
    const uint64_t row_bytes = uint64_t(input.width) * input.point_step;
    if (input.point_step == 0 || input.row_step < row_bytes ||
        uint64_t(input.row_step) * input.height > input.data.size())
        throw std::invalid_argument("Invalid point cloud layout");
    std::array<sensor_msgs::msg::PointField, 3> fields;
    for (size_t axis = 0; axis < fields.size(); ++axis) {
        const std::string name(1, "xyz"[axis]);
        const auto it = std::find_if(input.fields.begin(), input.fields.end(),
                                     [&](const auto &field) { return field.name == name; });
        if (it == input.fields.end() || it->count != 1 ||
            (it->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
             it->datatype != sensor_msgs::msg::PointField::FLOAT64))
            throw std::invalid_argument("Expected scalar float XYZ fields");
        const size_t bytes = it->datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
        if (uint64_t(it->offset) + bytes > input.point_step)
            throw std::invalid_argument("XYZ field exceeds point stride");
        fields[axis] = *it;
    }
    const uint16_t endian_test = 1;
    const bool host_big = *reinterpret_cast<const uint8_t *>(&endian_test) == 0;
    auto read = [&](const uint8_t *point, const auto &field) {
        std::array<uint8_t, 8> bytes{};
        const size_t size = field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
        std::memcpy(bytes.data(), point + field.offset, size);
        if (input.is_bigendian != host_big)
            std::reverse(bytes.begin(), bytes.begin() + size);
        if (size == 4) {
            float value;
            std::memcpy(&value, bytes.data(), size);
            return double(value);
        }
        double value;
        std::memcpy(&value, bytes.data(), size);
        return value;
    };
    CloudHeightResult result;
    result.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto &output = *result.cloud;
    output.header = input.header;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
    output.height = 1;
    output.is_dense = true;
    const uint64_t maximum_bytes = row_bytes * input.height;
    if (maximum_bytes > std::numeric_limits<uint32_t>::max())
        throw std::invalid_argument("Point cloud exceeds output row capacity");
    output.data.reserve(size_t(maximum_bytes));
    for (uint32_t row = 0; row < input.height; ++row) {
        for (uint32_t column = 0; column < input.width; ++column) {
            const auto *point = input.data.data() + size_t(row) * input.row_step +
                                size_t(column) * input.point_step;
            const double x = read(point, fields[0]), y = read(point, fields[1]),
                         z = read(point, fields[2]);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            ++result.finite_count;
            result.minimum = std::min(result.minimum, z);
            result.maximum = std::max(result.maximum, z);
            if (!enabled || (z >= lower && z <= upper))
                output.data.insert(output.data.end(), point, point + input.point_step);
        }
    }
    output.width = uint32_t(output.data.size() / output.point_step);
    output.row_step = uint32_t(output.data.size());
    return result;
}

} // namespace navi_multi_goals_pub_rviz_plugin

/**
 * @file solid_time.h
 * @brief 校验并转换 SOLID PointCloud2 的绝对时间戳。
 */
#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace solid_time
{
namespace detail
{
/** @brief 返回 PointField 数据类型的字节数，未知类型返回零。 */
inline std::size_t FieldTypeSize(std::uint8_t datatype)
{
    switch (datatype)
    {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
        return 1;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
        return 2;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
        return 4;
    case sensor_msgs::PointField::FLOAT64:
        return 8;
    default:
        return 0;
    }
}

/** @brief 判断当前主机字节序。 */
inline bool IsHostBigEndian()
{
    const std::uint16_t marker = 0x0102;
    return reinterpret_cast<const std::uint8_t *>(&marker)[0] == 0x01;
}

/** @brief 从消息缓冲区按其字节序安全读取一个 FLOAT64。 */
inline double ReadFloat64(const std::uint8_t *data, bool isBigEndian)
{
    std::array<std::uint8_t, sizeof(double)> bytes{};
    std::copy(data, data + bytes.size(), bytes.begin());
    if (isBigEndian != IsHostBigEndian())
    {
        std::reverse(bytes.begin(), bytes.end());
    }

    double value = 0.0;
    std::memcpy(&value, bytes.data(), sizeof(value));
    return value;
}

/** @brief 检查 PointCloud2 的字段表、步长和数据长度是否一致。 */
inline bool ValidateLayout(const sensor_msgs::PointCloud2 &cloud, std::uint32_t &timestampOffset,
                           std::string &reason)
{
    if (cloud.point_step == 0)
    {
        reason = "point_step is zero";
        return false;
    }

    const std::uint64_t minRowBytes = static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
    if (cloud.row_step < minRowBytes)
    {
        reason = "row_step is smaller than width * point_step";
        return false;
    }

    const std::uint64_t expectedDataBytes = static_cast<std::uint64_t>(cloud.row_step) * cloud.height;
    if (expectedDataBytes > std::numeric_limits<std::size_t>::max() ||
        cloud.data.size() != static_cast<std::size_t>(expectedDataBytes))
    {
        reason = "data size does not match row_step * height";
        return false;
    }

    const std::uint64_t pointCount = static_cast<std::uint64_t>(cloud.width) * cloud.height;
    if (pointCount > std::numeric_limits<std::size_t>::max())
    {
        reason = "point count exceeds addressable size";
        return false;
    }

    std::set<std::string> fieldNames;
    std::vector<std::pair<std::uint64_t, std::uint64_t>> fieldRanges;
    std::size_t timestampCount = 0;
    std::array<bool, 3> hasCoordinates{};
    for (const auto &field : cloud.fields)
    {
        const std::size_t typeSize = FieldTypeSize(field.datatype);
        if (field.name.empty() || !fieldNames.insert(field.name).second || field.count == 0 || typeSize == 0)
        {
            reason = "field name, count, datatype, or uniqueness is invalid";
            return false;
        }

        const std::uint64_t fieldEnd = static_cast<std::uint64_t>(field.offset) +
                                       static_cast<std::uint64_t>(field.count) * typeSize;
        if (fieldEnd > cloud.point_step)
        {
            reason = "field extends past point_step";
            return false;
        }
        fieldRanges.emplace_back(field.offset, fieldEnd);

        if (field.name == "x" || field.name == "y" || field.name == "z")
        {
            if (field.count != 1 || field.datatype != sensor_msgs::PointField::FLOAT32)
            {
                reason = "x, y, and z must each be one FLOAT32 value";
                return false;
            }
            const std::size_t coordinateIndex = field.name == "x" ? 0 : (field.name == "y" ? 1 : 2);
            hasCoordinates[coordinateIndex] = true;
        }

        if (field.name == "timestamp")
        {
            ++timestampCount;
            timestampOffset = field.offset;
            if (field.count != 1 || field.datatype != sensor_msgs::PointField::FLOAT64)
            {
                reason = "timestamp must be one FLOAT64 value";
                return false;
            }
        }
    }

    if (!std::all_of(hasCoordinates.begin(), hasCoordinates.end(), [](bool present) { return present; }))
    {
        reason = "x, y, and z fields are required";
        return false;
    }

    std::sort(fieldRanges.begin(), fieldRanges.end());
    for (std::size_t i = 1; i < fieldRanges.size(); ++i)
    {
        if (fieldRanges[i].first < fieldRanges[i - 1].second)
        {
            reason = "field byte ranges overlap";
            return false;
        }
    }

    if (timestampCount != 1)
    {
        reason = "exactly one timestamp field is required";
        return false;
    }
    return true;
}

/** @brief 按行列索引读取 timestamp，支持行尾填充和大端数据。 */
inline double ReadTimestamp(const sensor_msgs::PointCloud2 &cloud, std::uint32_t timestampOffset,
                            std::size_t pointIndex)
{
    const std::size_t row = pointIndex / cloud.width;
    const std::size_t column = pointIndex % cloud.width;
    const std::size_t byteOffset = row * cloud.row_step + column * cloud.point_step + timestampOffset;
    return ReadFloat64(cloud.data.data() + byteOffset, cloud.is_bigendian);
}
}  // namespace detail

/**
 * @brief 将 SOLID 的 timestamp 字段校验并转换为按行列展开的帧内秒偏移。
 * @param cloud 输入 PointCloud2；timestamp 必须为 FLOAT64 且 count 为 1。
 * @param mode frame_absolute 要求整帧时间相差不超过 1 微秒；point_absolute 相对 header.stamp 转换。
 * @param headerTimeSec PointCloud2 头时间，单位秒。
 * @param scanRate 正扫描频率；point_absolute 的最大偏移为 2 / scanRate 秒。
 * @param offsetsSec 成功时输出每点帧内偏移秒，失败时保持原内容不变。
 * @param reason 失败原因，可选。
 * @return 布局及时间均有效时返回 true。
 */
inline bool PrepareTimestampOffsets(const sensor_msgs::PointCloud2 &cloud, const std::string &mode,
                                    double headerTimeSec, int scanRate, std::vector<float> &offsetsSec,
                                    std::string *reason = nullptr)
{
    std::string error;
    if (mode != "frame_absolute" && mode != "point_absolute")
    {
        error = "solid_timestamp_mode must be frame_absolute or point_absolute";
    }

    std::uint32_t timestampOffset = 0;
    if (error.empty())
    {
        detail::ValidateLayout(cloud, timestampOffset, error);
    }
    if (!error.empty())
    {
        if (reason != nullptr)
        {
            *reason = error;
        }
        return false;
    }

    const std::size_t pointCount = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (error.empty() && mode == "point_absolute" && scanRate <= 0)
    {
        error = "scan_rate must be positive for point_absolute";
    }
    if (error.empty() && mode == "point_absolute" && !std::isfinite(headerTimeSec))
    {
        error = "header timestamp must be finite for point_absolute";
    }

    std::vector<float> prepared;
    if (error.empty() && mode == "frame_absolute")
    {
        double minTimestamp = std::numeric_limits<double>::infinity();
        double maxTimestamp = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < pointCount; ++i)
        {
            const double timestamp = detail::ReadTimestamp(cloud, timestampOffset, i);
            if (!std::isfinite(timestamp))
            {
                error = "timestamp contains a non-finite value";
                break;
            }
            minTimestamp = std::min(minTimestamp, timestamp);
            maxTimestamp = std::max(maxTimestamp, timestamp);
        }
        if (error.empty() && pointCount > 0 && maxTimestamp - minTimestamp > 1e-6)
        {
            error = "timestamp varies within frame; use point_absolute";
        }
        if (error.empty())
        {
            prepared.assign(pointCount, 0.0F);
        }
    }
    else if (error.empty())
    {
        const double maxOffsetSec = 2.0 / static_cast<double>(scanRate);
        prepared.reserve(pointCount);
        for (std::size_t i = 0; i < pointCount; ++i)
        {
            const double timestamp = detail::ReadTimestamp(cloud, timestampOffset, i);
            if (!std::isfinite(timestamp))
            {
                error = "timestamp contains a non-finite value";
                break;
            }

            double offsetSec = timestamp - headerTimeSec;
            if (!std::isfinite(offsetSec) || offsetSec < -1e-6 || offsetSec > maxOffsetSec)
            {
                error = "point timestamp offset is outside the accepted scan interval";
                break;
            }
            if (offsetSec < 0.0)
            {
                offsetSec = 0.0;
            }
            prepared.push_back(static_cast<float>(offsetSec));
        }
    }

    if (!error.empty())
    {
        if (reason != nullptr)
        {
            *reason = error;
        }
        return false;
    }

    offsetsSec.swap(prepared);
    if (reason != nullptr)
    {
        reason->clear();
    }
    return true;
}

/**
 * @brief 构造 PCL 可读的 timestamp 输入，补上 F32 time 字段并规范字节序与行填充。
 * @param cloud 已通过时间布局校验的原始点云。
 * @param offsetsSec 与 cloud 按行列顺序对应的帧内秒偏移。
 * @param pclCloud 输出副本；失败时保持原内容不变。
 * @param reason 失败原因，可选。
 * @return 点数及字段有效时返回 true。
 */
inline bool PreparePclInput(const sensor_msgs::PointCloud2 &cloud, const std::vector<float> &offsetsSec,
                            sensor_msgs::PointCloud2 &pclCloud, std::string *reason = nullptr)
{
    std::string error;
    std::uint32_t timestampOffset = 0;
    if (!detail::ValidateLayout(cloud, timestampOffset, error))
    {
        if (reason != nullptr)
        {
            *reason = error;
        }
        return false;
    }

    const std::size_t pointCount = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (offsetsSec.size() != pointCount)
    {
        if (reason != nullptr)
        {
            *reason = "converted point count does not match layout";
        }
        return false;
    }

    sensor_msgs::PointCloud2 prepared = cloud;
    const bool hostIsBigEndian = detail::IsHostBigEndian();
    if (prepared.is_bigendian != hostIsBigEndian)
    {
        for (std::uint32_t row = 0; row < prepared.height; ++row)
        {
            for (std::uint32_t column = 0; column < prepared.width; ++column)
            {
                const std::size_t pointOffset = static_cast<std::size_t>(row) * prepared.row_step +
                                                static_cast<std::size_t>(column) * prepared.point_step;
                for (const auto &field : prepared.fields)
                {
                    const std::size_t typeSize = detail::FieldTypeSize(field.datatype);
                    for (std::uint32_t element = 0; element < field.count; ++element)
                    {
                        const std::size_t fieldOffset = pointOffset + field.offset + element * typeSize;
                        std::reverse(prepared.data.begin() + fieldOffset,
                                     prepared.data.begin() + fieldOffset + typeSize);
                    }
                }
            }
        }
    }
    prepared.is_bigendian = hostIsBigEndian;

    auto timestampField = std::find_if(prepared.fields.begin(), prepared.fields.end(),
                                       [](const sensor_msgs::PointField &field) { return field.name == "timestamp"; });
    if (timestampField == prepared.fields.end())
    {
        if (reason != nullptr)
        {
            *reason = "timestamp field is missing";
        }
        return false;
    }

    timestampField->name = "time";
    timestampField->datatype = sensor_msgs::PointField::FLOAT32;
    timestampField->count = 1;
    for (std::size_t i = 0; i < pointCount; ++i)
    {
        const std::size_t row = i / prepared.width;
        const std::size_t column = i % prepared.width;
        const std::size_t timeOffset = row * prepared.row_step + column * prepared.point_step + timestampOffset;
        std::memcpy(prepared.data.data() + timeOffset, &offsetsSec[i], sizeof(float));
    }

    const std::uint64_t packedRowBytes = static_cast<std::uint64_t>(prepared.width) * prepared.point_step;
    if (packedRowBytes != prepared.row_step)
    {
        const std::size_t packedDataBytes = static_cast<std::size_t>(packedRowBytes) * prepared.height;
        std::vector<std::uint8_t> packedData(packedDataBytes);
        for (std::uint32_t row = 0; row < prepared.height; ++row)
        {
            const std::size_t sourceOffset = static_cast<std::size_t>(row) * prepared.row_step;
            const std::size_t destinationOffset = static_cast<std::size_t>(row) * packedRowBytes;
            std::memcpy(packedData.data() + destinationOffset, prepared.data.data() + sourceOffset,
                        static_cast<std::size_t>(packedRowBytes));
        }
        prepared.data.swap(packedData);
        prepared.row_step = static_cast<std::uint32_t>(packedRowBytes);
    }

    pclCloud = std::move(prepared);
    if (reason != nullptr)
    {
        reason->clear();
    }
    return true;
}
}  // namespace solid_time

/**
 * @file solid_time_tests.cpp
 * @brief SOLID 绝对时间戳转换及生产预处理器回归测试。
 */
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../src/preprocess.h"
#include "preprocess/solid_time.h"

namespace
{
constexpr std::uint32_t kTimestampPointStep = 28;
constexpr std::uint32_t kFloatTimePointStep = 24;

/** @brief 将标量按 PointCloud2 字节序写入点数据。 */
template <typename TValue>
void WriteValue(std::vector<std::uint8_t> &data, std::size_t offset, TValue value, bool isBigEndian)
{
    std::uint8_t bytes[sizeof(TValue)];
    std::memcpy(bytes, &value, sizeof(value));
    const std::uint16_t marker = 0x0102;
    const bool hostIsBigEndian = reinterpret_cast<const std::uint8_t *>(&marker)[0] == 0x01;
    if (hostIsBigEndian != isBigEndian)
    {
        std::reverse(bytes, bytes + sizeof(value));
    }
    std::copy(bytes, bytes + sizeof(value), data.begin() + offset);
}

/** @brief 设置 PointCloud2 的整秒头时间，便于测试绝对时间相减。 */
void SetHeaderTime(sensor_msgs::PointCloud2 &cloud, double headerTimeSec)
{
    cloud.header.stamp.sec = static_cast<std::uint32_t>(headerTimeSec);
    cloud.header.stamp.nsec = static_cast<std::uint32_t>((headerTimeSec - cloud.header.stamp.sec) * 1e9);
}

/** @brief 添加 SOLID 坐标、强度及时间字段。 */
void SetSolidFields(sensor_msgs::PointCloud2 &cloud, const std::string &timeName, std::uint8_t timeDatatype,
                    std::uint32_t timeOffset, std::uint32_t ringOffset)
{
    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
    field.name = "y";
    field.offset = 4;
    cloud.fields.push_back(field);
    field.name = "z";
    field.offset = 8;
    cloud.fields.push_back(field);
    field.name = "intensity";
    field.offset = 12;
    cloud.fields.push_back(field);
    field.name = timeName;
    field.offset = timeOffset;
    field.datatype = timeDatatype;
    cloud.fields.push_back(field);
    field.name = "ring";
    field.offset = ringOffset;
    field.datatype = sensor_msgs::PointField::UINT16;
    cloud.fields.push_back(field);
}

/** @brief 生成包含 F64 timestamp 字段的 SOLID PointCloud2。 */
sensor_msgs::PointCloud2 MakeTimestampCloud(const std::vector<double> &timestamps, std::uint32_t width = 0,
                                            std::uint32_t height = 1, std::uint32_t rowPadding = 0,
                                            bool isBigEndian = false, double headerTimeSec = 1700000000.0)
{
    sensor_msgs::PointCloud2 cloud;
    if (width == 0)
    {
        width = static_cast<std::uint32_t>(timestamps.size());
    }
    cloud.width = width;
    cloud.height = height;
    cloud.point_step = kTimestampPointStep;
    cloud.row_step = width * cloud.point_step + rowPadding;
    cloud.is_bigendian = isBigEndian;
    cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * height, 0);
    SetHeaderTime(cloud, headerTimeSec);
    SetSolidFields(cloud, "timestamp", sensor_msgs::PointField::FLOAT64, 16, 24);

    for (std::size_t i = 0; i < timestamps.size(); ++i)
    {
        const std::size_t row = i / width;
        const std::size_t column = i % width;
        const std::size_t pointOffset = row * cloud.row_step + column * cloud.point_step;
        WriteValue(cloud.data, pointOffset, 5.0F + static_cast<float>(i), isBigEndian);
        WriteValue(cloud.data, pointOffset + 4, 0.0F, isBigEndian);
        WriteValue(cloud.data, pointOffset + 8, 0.0F, isBigEndian);
        WriteValue(cloud.data, pointOffset + 12, static_cast<float>(i + 1), isBigEndian);
        WriteValue(cloud.data, pointOffset + 16, timestamps[i], isBigEndian);
        WriteValue(cloud.data, pointOffset + 24, static_cast<std::uint16_t>(0), isBigEndian);
    }
    return cloud;
}

/** @brief 生成原生 F32 time 字段，供兼容路径测试使用。 */
sensor_msgs::PointCloud2 MakeFloatTimeCloud(const std::vector<float> &times)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.width = static_cast<std::uint32_t>(times.size());
    cloud.height = 1;
    cloud.point_step = kFloatTimePointStep;
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.assign(cloud.row_step, 0);
    SetSolidFields(cloud, "time", sensor_msgs::PointField::FLOAT32, 16, 20);

    for (std::size_t i = 0; i < times.size(); ++i)
    {
        const std::size_t pointOffset = i * cloud.point_step;
        WriteValue(cloud.data, pointOffset, 5.0F + static_cast<float>(i), false);
        WriteValue(cloud.data, pointOffset + 4, 0.0F, false);
        WriteValue(cloud.data, pointOffset + 8, 0.0F, false);
        WriteValue(cloud.data, pointOffset + 12, static_cast<float>(i + 1), false);
        WriteValue(cloud.data, pointOffset + 16, times[i], false);
        WriteValue(cloud.data, pointOffset + 20, static_cast<std::uint16_t>(0), false);
    }
    return cloud;
}

/** @brief 使用独立时间 helper 转换测试消息。 */
bool Prepare(const sensor_msgs::PointCloud2 &cloud, const std::string &mode, double headerTimeSec,
             int scanRate, std::vector<float> &offsetsSec, std::string *reason = nullptr)
{
    return solid_time::PrepareTimestampOffsets(cloud, mode, headerTimeSec, scanRate, offsetsSec, reason);
}

/** @brief 调用真实 Preprocess SOLID 分支并返回筛选后的点云。 */
PointCloudXYZI ProcessSolid(const sensor_msgs::PointCloud2 &cloud, const std::string &mode, int scanRate = 10)
{
    Preprocess preprocess;
    preprocess.lidar_type = SOLID;
    preprocess.SCAN_RATE = scanRate;
    preprocess.solidTimestampMode_ = mode;
    sensor_msgs::PointCloud2::ConstPtr message(new sensor_msgs::PointCloud2(cloud));
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    preprocess.process(message, output);
    return *output;
}

TEST(SolidTimestampAdapter, FrameAbsoluteProductionKeepsCompatibleConstantFrame)
{
    const double frameTime = 1700000000.25;
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud(
        {frameTime, frameTime + 0.4e-6, frameTime - 0.4e-6});
    const PointCloudXYZI output = ProcessSolid(cloud, "frame_absolute");

    ASSERT_EQ(output.size(), 3u);
    for (const auto &point : output.points)
    {
        EXPECT_FLOAT_EQ(point.curvature, 0.0F);
    }
}

TEST(SolidTimestampAdapter, FrameAbsoluteRejectsPerPointTimestamps)
{
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud(
        {1700000000.0, 1700000000.02, 1700000000.04});
    const PointCloudXYZI output = ProcessSolid(cloud, "frame_absolute");
    EXPECT_TRUE(output.empty());
}

TEST(SolidTimestampAdapter, PointAbsoluteUsesHeaderAndSortsProductionOutput)
{
    constexpr double headerTime = 1700000000.0;
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud(
        {headerTime + 0.020, headerTime + 0.005, headerTime + 0.040}, 0, 1, 0, false, headerTime);
    const PointCloudXYZI output = ProcessSolid(cloud, "point_absolute");

    ASSERT_EQ(output.size(), 3u);
    EXPECT_NEAR(output.points[0].curvature, 5.0F, 0.04F);
    EXPECT_NEAR(output.points[1].curvature, 20.0F, 0.04F);
    EXPECT_NEAR(output.points[2].curvature, 40.0F, 0.04F);
    EXPECT_FLOAT_EQ(output.points[0].intensity, 2.0F);
    EXPECT_FLOAT_EQ(output.points[1].intensity, 1.0F);
    EXPECT_FLOAT_EQ(output.points[2].intensity, 3.0F);
}

TEST(SolidTimestampAdapter, NativeFloatTimeKeepsRelativeSecondsBehavior)
{
    const PointCloudXYZI output = ProcessSolid(MakeFloatTimeCloud({0.02F, 0.10F, 0.05F}), "point_absolute");

    ASSERT_EQ(output.size(), 3u);
    EXPECT_FLOAT_EQ(output.points[0].curvature, 0.0F);
    EXPECT_NEAR(output.points[1].curvature, 80.0F, 1e-5F);
    EXPECT_NEAR(output.points[2].curvature, 30.0F, 1e-5F);
}

TEST(SolidTimestampAdapter, EmptyCloudAndMissingTimeAreRejectedBeforePclConversion)
{
    sensor_msgs::PointCloud2 empty;
    empty.height = 1;
    EXPECT_TRUE(ProcessSolid(empty, "point_absolute").empty());
    auto missingTime = MakeTimestampCloud({1700000000.0});
    missingTime.fields.erase(
        std::remove_if(missingTime.fields.begin(), missingTime.fields.end(),
                       [](const sensor_msgs::PointField &field) { return field.name == "timestamp"; }),
        missingTime.fields.end());
    EXPECT_TRUE(ProcessSolid(missingTime, "point_absolute").empty());
}

TEST(SolidTimestampAdapter, RejectsTimestampWithWrongDatatype)
{
    sensor_msgs::PointCloud2 cloud = MakeTimestampCloud({1700000000.0});
    auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                              [](const sensor_msgs::PointField &value) { return value.name == "timestamp"; });
    ASSERT_NE(field, cloud.fields.end());
    field->datatype = sensor_msgs::PointField::FLOAT32;
    std::vector<float> offsets;
    EXPECT_FALSE(Prepare(cloud, "frame_absolute", 1700000000.0, 10, offsets));
}

TEST(SolidTimestampAdapter, RejectsTimestampCountOtherThanOne)
{
    sensor_msgs::PointCloud2 cloud = MakeTimestampCloud({1700000000.0});
    auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                              [](const sensor_msgs::PointField &value) { return value.name == "timestamp"; });
    ASSERT_NE(field, cloud.fields.end());
    field->count = 2;
    std::vector<float> offsets;
    EXPECT_FALSE(Prepare(cloud, "frame_absolute", 1700000000.0, 10, offsets));
}

TEST(SolidTimestampAdapter, RejectsTimestampOffsetPastPointStep)
{
    sensor_msgs::PointCloud2 cloud = MakeTimestampCloud({1700000000.0});
    auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                              [](const sensor_msgs::PointField &value) { return value.name == "timestamp"; });
    ASSERT_NE(field, cloud.fields.end());
    field->offset = cloud.point_step - 4;
    std::vector<float> offsets;
    EXPECT_FALSE(Prepare(cloud, "frame_absolute", 1700000000.0, 10, offsets));
}

TEST(SolidTimestampAdapter, RejectsMalformedPointCloudLayouts)
{
    const sensor_msgs::PointCloud2 valid = MakeTimestampCloud({1700000000.0, 1700000000.01});
    std::vector<float> offsets;

    sensor_msgs::PointCloud2 badPointStep = valid;
    badPointStep.point_step = 0;
    EXPECT_FALSE(Prepare(badPointStep, "frame_absolute", 1700000000.0, 10, offsets));

    sensor_msgs::PointCloud2 badRowStep = valid;
    badRowStep.row_step = badRowStep.point_step;
    EXPECT_FALSE(Prepare(badRowStep, "frame_absolute", 1700000000.0, 10, offsets));

    sensor_msgs::PointCloud2 shortData = valid;
    shortData.data.pop_back();
    EXPECT_FALSE(Prepare(shortData, "frame_absolute", 1700000000.0, 10, offsets));

    sensor_msgs::PointCloud2 badOtherField = valid;
    badOtherField.fields.front().offset = badOtherField.point_step;
    EXPECT_FALSE(Prepare(badOtherField, "frame_absolute", 1700000000.0, 10, offsets));

    sensor_msgs::PointCloud2 overlappingFields = valid;
    overlappingFields.fields[1].offset = 2;
    EXPECT_FALSE(Prepare(overlappingFields, "frame_absolute", 1700000000.0, 10, offsets));
}

TEST(SolidTimestampAdapter, RejectsNonFiniteTimestamp)
{
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud({std::numeric_limits<double>::quiet_NaN()});
    std::vector<float> offsets;
    EXPECT_FALSE(Prepare(cloud, "point_absolute", 1700000000.0, 10, offsets));
}

TEST(SolidTimestampAdapter, RequiresFloatXYZAndSkipsNonFiniteTimestampGeometry)
{
    constexpr double headerTime = 1700000000.0;
    sensor_msgs::PointCloud2 invalidSchema = MakeTimestampCloud({headerTime + 0.01}, 0, 1, 0, false, headerTime);
    invalidSchema.fields.front().datatype = sensor_msgs::PointField::UINT32;
    std::vector<float> offsets;
    EXPECT_FALSE(Prepare(invalidSchema, "point_absolute", headerTime, 10, offsets));

    sensor_msgs::PointCloud2 cloud = MakeTimestampCloud(
        {headerTime + 0.01, headerTime + 0.02}, 0, 1, 0, false, headerTime);
    WriteValue(cloud.data, 0, std::numeric_limits<float>::quiet_NaN(), false);
    const PointCloudXYZI output = ProcessSolid(cloud, "point_absolute");
    ASSERT_EQ(output.size(), 1u);
    EXPECT_FLOAT_EQ(output.points.front().x, 6.0F);
}

TEST(SolidTimestampAdapter, ClampsOneMicrosecondNegativeAndRejectsEarlierPoint)
{
    constexpr double headerTime = 1700000000.0;
    std::vector<float> offsets;
    const sensor_msgs::PointCloud2 tolerated = MakeTimestampCloud({headerTime - 0.8e-6}, 0, 1, 0, false, headerTime);
    ASSERT_TRUE(Prepare(tolerated, "point_absolute", headerTime, 10, offsets));
    ASSERT_EQ(offsets.size(), 1u);
    EXPECT_FLOAT_EQ(offsets.front(), 0.0F);

    const sensor_msgs::PointCloud2 rejected = MakeTimestampCloud({headerTime - 2e-6}, 0, 1, 0, false, headerTime);
    EXPECT_FALSE(Prepare(rejected, "point_absolute", headerTime, 10, offsets));
}

TEST(SolidTimestampAdapter, RejectsExcessiveOffsetAndNonPositiveScanRate)
{
    constexpr double headerTime = 1700000000.0;
    std::vector<float> offsets;
    const sensor_msgs::PointCloud2 excessive = MakeTimestampCloud(
        {headerTime + 0.2001}, 0, 1, 0, false, headerTime);
    EXPECT_FALSE(Prepare(excessive, "point_absolute", headerTime, 10, offsets));

    const sensor_msgs::PointCloud2 valid = MakeTimestampCloud(
        {headerTime + 0.01}, 0, 1, 0, false, headerTime);
    EXPECT_FALSE(Prepare(valid, "point_absolute", headerTime, 0, offsets));
}

TEST(SolidTimestampAdapter, DecodesBigEndianTimestamp)
{
    constexpr double headerTime = 1700000000.0;
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud(
        {headerTime + 0.0125}, 0, 1, 0, true, headerTime);
    std::vector<float> offsets;
    ASSERT_TRUE(Prepare(cloud, "point_absolute", headerTime, 10, offsets));
    ASSERT_EQ(offsets.size(), 1u);
    EXPECT_NEAR(offsets.front(), 0.0125F, 1e-7F);

    const PointCloudXYZI output = ProcessSolid(cloud, "point_absolute");
    ASSERT_EQ(output.size(), 1u);
    EXPECT_NEAR(output.points.front().curvature, 12.5F, 1e-4F);
    EXPECT_FLOAT_EQ(output.points.front().x, 5.0F);
    EXPECT_FLOAT_EQ(output.points.front().intensity, 1.0F);
}

TEST(SolidTimestampAdapter, ReadsOrganizedCloudWithRowPaddingAndUnorderedTimes)
{
    constexpr double headerTime = 1700000000.0;
    // Unix 秒本身已按 double 量化，偏移误差界使用该绝对时刻的一 ULP。
    const float timeTolerance = static_cast<float>(
        std::nextafter(headerTime, std::numeric_limits<double>::infinity()) - headerTime);
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud(
        {headerTime + 0.04, headerTime + 0.005, headerTime + 0.02, headerTime + 0.01}, 2, 2, 12, false,
        headerTime);
    std::vector<float> offsets;
    ASSERT_TRUE(Prepare(cloud, "point_absolute", headerTime, 10, offsets));
    ASSERT_EQ(offsets.size(), 4u);
    EXPECT_NEAR(offsets[0], 0.04F, timeTolerance);
    EXPECT_NEAR(offsets[1], 0.005F, timeTolerance);
    EXPECT_NEAR(offsets[2], 0.02F, timeTolerance);
    EXPECT_NEAR(offsets[3], 0.01F, timeTolerance);

    const PointCloudXYZI output = ProcessSolid(cloud, "point_absolute");
    ASSERT_EQ(output.size(), 4u);
    EXPECT_NEAR(output.points[0].curvature, 5.0F, 0.04F);
    EXPECT_NEAR(output.points[1].curvature, 10.0F, 0.04F);
    EXPECT_NEAR(output.points[2].curvature, 20.0F, 0.04F);
    EXPECT_NEAR(output.points[3].curvature, 40.0F, 0.04F);
}

TEST(SolidTimestampAdapter, RejectsMissingTimestampAndUnknownMode)
{
    const sensor_msgs::PointCloud2 cloud = MakeTimestampCloud({1700000000.0});
    sensor_msgs::PointCloud2 missingTimestamp = cloud;
    missingTimestamp.fields.erase(
        std::remove_if(missingTimestamp.fields.begin(), missingTimestamp.fields.end(),
                       [](const sensor_msgs::PointField &field) { return field.name == "timestamp"; }),
        missingTimestamp.fields.end());
    std::vector<float> offsets;
    EXPECT_FALSE(Prepare(missingTimestamp, "frame_absolute", 1700000000.0, 10, offsets));
    EXPECT_FALSE(Prepare(cloud, "unknown", 1700000000.0, 10, offsets));
}
}  // namespace

/** @brief 初始化本地 ROS 时间并运行 SOLID 时间适配测试。 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "pv_lio_plus_solid_time_tests", ros::init_options::NoSigintHandler);
    ros::Time::init();
    return RUN_ALL_TESTS();
}

#include "hakoniwa/pdu/converter/sensor_msgs/point_cloud2.hpp"
#include "tests/sensors/support/sensor_test_utils.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace
{
using hako::robots::sensor::test::NearlyEqual;
using hako::robots::sensor::lidar::LiDAR3DConfig;
using hako::robots::sensor::lidar::PointCloudFrame;
namespace conv = hako::robots::pdu::converter::sensor_msgs;

PointCloudFrame MakeFrame(size_t points)
{
    PointCloudFrame frame;
    frame.frame_id = "lidar";
    frame.rays_cast = static_cast<int>(points) * 4;
    for (size_t i = 0; i < points; ++i) {
        const float value = static_cast<float>(i);
        frame.xyz.push_back(value);
        frame.xyz.push_back(value + 0.5F);
        frame.xyz.push_back(value + 0.25F);
        frame.distances.push_back(value);
        frame.geom_ids.push_back(static_cast<int>(i));
    }
    return frame;
}

LiDAR3DConfig MakeConfig()
{
    LiDAR3DConfig config {};
    config.frame_id = "mid360s";
    return config;
}

void TestLayoutMatchesTheProfilePointStep()
{
    const auto pdu = conv::ToHakoPdu(MakeConfig(), MakeFrame(3), 1.5);

    HAKO_TEST_EXPECT(pdu.height == 1, "an unordered cloud has height 1");
    HAKO_TEST_EXPECT(pdu.width == 3, "width should be the point count");
    // point_step is what a publisher budgets the fixed channel against, so it
    // has to agree with pdu_config.point_step in the lidar_3d profile.
    HAKO_TEST_EXPECT(pdu.point_step == 16, "x, y, z, intensity as float32 is 16 bytes");
    HAKO_TEST_EXPECT(pdu.row_step == pdu.point_step * pdu.width, "row_step should span the row");
    HAKO_TEST_EXPECT(pdu.data.size() == pdu.row_step, "data should hold exactly the points");
    HAKO_TEST_EXPECT(!pdu.is_bigendian, "the payload is little endian");
    HAKO_TEST_EXPECT(pdu.is_dense, "returns carry no invalid points");

    HAKO_TEST_EXPECT(pdu.fields.size() == 4, "four fields");
    const char* const names[] = {"x", "y", "z", "intensity"};
    for (size_t i = 0; i < pdu.fields.size(); ++i) {
        HAKO_TEST_EXPECT(pdu.fields[i].name == names[i], "unexpected field name");
        HAKO_TEST_EXPECT(pdu.fields[i].offset == i * 4, "fields should be packed in order");
        HAKO_TEST_EXPECT(pdu.fields[i].datatype == 7, "FLOAT32 is datatype 7");
        HAKO_TEST_EXPECT(pdu.fields[i].count == 1, "one value per field per point");
    }
}

void TestPointsSurviveTheRoundTripIntoBytes()
{
    const auto frame = MakeFrame(4);
    const auto pdu = conv::ToHakoPdu(MakeConfig(), frame, 0.0);

    for (size_t i = 0; i < 4; ++i) {
        float point[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        std::memcpy(point, pdu.data.data() + i * pdu.point_step, sizeof(point));
        HAKO_TEST_EXPECT(NearlyEqual(point[0], frame.xyz[3 * i + 0]), "x was not preserved");
        HAKO_TEST_EXPECT(NearlyEqual(point[1], frame.xyz[3 * i + 1]), "y was not preserved");
        HAKO_TEST_EXPECT(NearlyEqual(point[2], frame.xyz[3 * i + 2]), "z was not preserved");
        HAKO_TEST_EXPECT(NearlyEqual(point[3], 1.0F), "reflectivity is not simulated");
    }
}

void TestAFrameOverBudgetIsTruncatedRatherThanOverrunning()
{
    const auto pdu = conv::ToHakoPdu(MakeConfig(), MakeFrame(10), 0.0, 4);
    HAKO_TEST_EXPECT(pdu.width == 4, "the budget should cap the published point count");
    HAKO_TEST_EXPECT(pdu.data.size() == 4 * pdu.point_step, "data should shrink with the count");
}

void TestAnEmptyFrameIsStillAValidCloud()
{
    const auto pdu = conv::ToHakoPdu(MakeConfig(), PointCloudFrame {}, 0.0);
    HAKO_TEST_EXPECT(pdu.width == 0, "an empty frame has no points");
    HAKO_TEST_EXPECT(pdu.data.empty(), "an empty frame carries no bytes");
    // The fields still describe the layout, so a reader can tell an empty cloud
    // from a malformed one.
    HAKO_TEST_EXPECT(pdu.fields.size() == 4, "an empty cloud still declares its fields");
}

void TestFrameIdAndStampComeFromTheConfigAndTheCaller()
{
    const auto pdu = conv::ToHakoPdu(MakeConfig(), MakeFrame(1), 2.25);
    HAKO_TEST_EXPECT(pdu.header.frame_id == "mid360s", "frame_id should come from the config");
    HAKO_TEST_EXPECT_TIME(pdu.header.stamp, 2, 250000000U);
}
}

int main()
{
    try {
        TestLayoutMatchesTheProfilePointStep();
        TestPointsSurviveTheRoundTripIntoBytes();
        TestAFrameOverBudgetIsTruncatedRatherThanOverrunning();
        TestAnEmptyFrameIsStillAValidCloud();
        TestFrameIdAndStampComeFromTheConfigAndTheCaller();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "lidar_3d_point_cloud2_converter_test passed" << std::endl;
    return EXIT_SUCCESS;
}

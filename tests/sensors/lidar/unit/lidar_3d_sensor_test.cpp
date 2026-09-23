#include "physics.hpp"
#include "sensors/lidar/lidar_3d_sensor.hpp"
#include "tests/sensors/support/sensor_test_utils.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

namespace
{
using hako::robots::sensor::test::NearlyEqual;
using hako::robots::sensor::test::RepoRoot;
using hako::robots::sensor::lidar::LiDAR3DSensor;
using hako::robots::sensor::lidar::PointCloudFrame;
using hako::robots::sensor::lidar::ScanPatternType;

class TestWorld final : public hako::robots::physics::IWorld {
public:
    void loadModel(const std::string& model_file) override
    {
        char error[1024] = {0};
        model = mj_loadXML(model_file.c_str(), nullptr, error, sizeof(error));
        if (model == nullptr) {
            throw std::runtime_error(
                std::string("failed to load MuJoCo model: ") + model_file + "\n" + error);
        }

        data = mj_makeData(model);
        if (data == nullptr) {
            throw std::runtime_error("failed to allocate MuJoCo data");
        }

        mj_forward(model, data);
    }

    void advanceTimeStep() override
    {
        if (model != nullptr && data != nullptr) {
            mj_step(model, data);
        }
    }

    std::shared_ptr<hako::robots::physics::IRigidBody>
    getRigidBody(const std::string& /*model_name*/) override
    {
        return nullptr;
    }

    std::shared_ptr<hako::robots::actuator::ITorqueActuator>
    getTorqueActuator(const std::string& /*name*/) override
    {
        return nullptr;
    }
};

std::shared_ptr<TestWorld> MakeWorld()
{
    auto world = std::make_shared<TestWorld>();
    world->loadModel((RepoRoot() / "models/sensors/lidar_3d/livox-mid360s-sample.xml").string());
    return world;
}

LiDAR3DSensor MakeSensor(const std::shared_ptr<TestWorld>& world, bool apply_noise)
{
    // No names here: they come from the profile's mjcf_binding.
    LiDAR3DSensor sensor(world);
    const auto path = (RepoRoot() / "config/sensors/lidar/livox-mid360s.json").string();
    if (!sensor.LoadConfig(path)) {
        throw std::runtime_error("livox-mid360s.json should load");
    }
    sensor.SetApplyNoise(apply_noise);
    sensor.SetSeed(7U);
    return sensor;
}

void TestDatasheetValuesArePreserved()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    const auto& config = sensor.GetConfig();

    HAKO_TEST_EXPECT(config.field_of_view.vertical.min_deg == -7.0, "unexpected vertical min");
    HAKO_TEST_EXPECT(config.field_of_view.vertical.max_deg == 52.0, "unexpected vertical max");
    HAKO_TEST_EXPECT(
        NearlyEqual(config.field_of_view.horizontal.max_deg - config.field_of_view.horizontal.min_deg, 360.0),
        "the horizontal field of view should span 360 degrees");
    // Millimetres in the profile, metres in the sensor, as lidar_2d does.
    HAKO_TEST_EXPECT(NearlyEqual(config.detection_distance.min, 0.1), "unexpected min range");
    HAKO_TEST_EXPECT(NearlyEqual(config.detection_distance.max, 100.0), "unexpected max range");
    HAKO_TEST_EXPECT(config.scan_pattern.type == ScanPatternType::Uniform, "unexpected pattern");
    // The profile names the mount, so the asset does not have to restate it.
    HAKO_TEST_EXPECT(config.mjcf_binding.source_site == "lidar_site", "unexpected source site");
    HAKO_TEST_EXPECT(config.mjcf_binding.source_body == "lidar_mount", "unexpected source body");
    HAKO_TEST_EXPECT(config.mjcf_binding.exclude_body == "lidar_mount", "unexpected exclude body");
    // The publisher budgets the fixed channel against these.
    HAKO_TEST_EXPECT(config.pdu_config.message_type == "sensor_msgs/PointCloud2", "unexpected message type");
    HAKO_TEST_EXPECT(config.pdu_config.max_points == 24000, "unexpected point budget");
    HAKO_TEST_EXPECT(config.pdu_config.point_step == 16, "unexpected point step");
}

void TestRaysPerFrameFollowPointRateOverFrameRate()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    HAKO_TEST_EXPECT(sensor.GetSamplesPerFrame() == 20000, "200000 points at 10 Hz is 20000 rays");
    HAKO_TEST_EXPECT(NearlyEqual(sensor.GetUpdatePeriodSec(), 0.1), "unexpected update period");
}

void TestReturnsStayInsideTheRangeGate()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    PointCloudFrame frame;
    sensor.Scan(frame);

    HAKO_TEST_EXPECT(frame.size() > 0, "the sample scene should return points");
    HAKO_TEST_EXPECT(frame.rays_cast == 20000, "rays_cast should report the rays, not the returns");
    HAKO_TEST_EXPECT(frame.size() < static_cast<size_t>(frame.rays_cast),
                     "some rays should miss, so returns are fewer than rays");
    HAKO_TEST_EXPECT(frame.xyz.size() == frame.size() * 3U, "three coordinates per point");
    HAKO_TEST_EXPECT(frame.geom_ids.size() == frame.size(), "one geom id per point");

    const auto& config = sensor.GetConfig();
    for (size_t i = 0; i < frame.size(); ++i) {
        HAKO_TEST_EXPECT(frame.distances[i] >= config.detection_distance.min,
                         "a return closer than the blind zone escaped the gate");
        HAKO_TEST_EXPECT(frame.distances[i] < config.detection_distance.max,
                         "a return beyond the cutoff escaped the gate");
    }
}

void TestPointsAreConsistentWithDistances()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    PointCloudFrame frame;
    sensor.Scan(frame);

    for (size_t i = 0; i < frame.size(); ++i) {
        const double x = frame.xyz[3 * i + 0];
        const double y = frame.xyz[3 * i + 1];
        const double z = frame.xyz[3 * i + 2];
        const double radius = std::sqrt(x * x + y * y + z * z);
        HAKO_TEST_EXPECT(NearlyEqual(radius, static_cast<double>(frame.distances[i]), 1.0e-4),
                         "a point should lie at its reported distance");
    }
}

void TestSensorDoesNotDetectItsOwnMount()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    PointCloudFrame frame;
    sensor.Scan(frame);

    const int mount = mj_name2id(world->getModel(), mjOBJ_BODY, "lidar_mount");
    HAKO_TEST_EXPECT(mount >= 0, "the sample scene should have a lidar_mount body");
    for (const int geom : frame.geom_ids) {
        HAKO_TEST_EXPECT(world->getModel()->geom_bodyid[geom] != mount,
                         "exclude_body should keep the mount out of the returns");
    }
}

void TestLowObjectInsideTheBlindConeReturnsNothing()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    PointCloudFrame frame;
    sensor.Scan(frame);

    const int hidden = mj_name2id(world->getModel(), mjOBJ_GEOM, "box_low_near_geom");
    const int visible = mj_name2id(world->getModel(), mjOBJ_GEOM, "box_tall_near_geom");
    HAKO_TEST_EXPECT(hidden >= 0 && visible >= 0, "the sample scene should have both near boxes");

    const std::set<int> hits(frame.geom_ids.begin(), frame.geom_ids.end());
    // At 0.5 m the -7 degree lower limit puts the nearest floor return at
    // 0.5 / tan(7 deg) = 4.07 m, so a short box 1.2 m away is under the cone.
    HAKO_TEST_EXPECT(hits.count(hidden) == 0, "a low near object must fall under the blind cone");
    HAKO_TEST_EXPECT(hits.count(visible) > 0, "a tall object at the same distance must be seen");
}

void TestNoisePerturbsRangesWithoutMovingTheGeometry()
{
    auto world = MakeWorld();
    PointCloudFrame clean;
    PointCloudFrame noisy;
    MakeSensor(world, false).Scan(clean);
    MakeSensor(world, true).Scan(noisy);

    HAKO_TEST_EXPECT(clean.size() == noisy.size(), "noise should not change which rays return");
    double largest = 0.0;
    for (size_t i = 0; i < clean.size(); ++i) {
        HAKO_TEST_EXPECT(clean.geom_ids[i] == noisy.geom_ids[i], "noise moved a return to another geom");
        const double delta = std::abs(static_cast<double>(noisy.distances[i]) -
                              static_cast<double>(clean.distances[i]));
        largest = std::max(largest, delta);
    }
    HAKO_TEST_EXPECT(largest > 0.0, "the accuracy model should do something");
    // Both bands declare a 4 cm or 2 cm sigma; 6 sigma is a generous ceiling.
    HAKO_TEST_EXPECT(largest < 0.24, "noise far exceeded the declared accuracy bands");
}

void TestTablePatternIsRejectedRatherThanSilentlyIgnored()
{
    auto world = MakeWorld();
    hako::robots::sensor::lidar::LiDAR3DSensor sensor(world);
    const auto path = (RepoRoot() / "config/sensors/lidar/livox-mid360s-table.json").string();
    // Replaying a recorded table is implemented in the Python sensor only.
    // Scanning a uniform pattern while the profile asks for a table would be a
    // silently different sensor, so loading has to fail instead.
    HAKO_TEST_EXPECT(!sensor.LoadConfig(path), "a table profile should be refused by the C++ sensor");
}

void TestAChildOfTheExcludedBodyIsAlsoExcluded()
{
    // mj_multiRay's bodyexclude drops only the named body's own geoms. A mount
    // on a robot carries child bodies, so without a second pass the sensor
    // sees its own bracket and every ray through it stops there.
    auto world = std::make_shared<TestWorld>();
    world->loadModel(
        (RepoRoot() / "models/sensors/lidar_3d/livox-mid360s-nested-mount-test.xml").string());
    LiDAR3DSensor sensor(world);
    const auto path = (RepoRoot() / "config/sensors/lidar/livox-mid360s.json").string();
    HAKO_TEST_EXPECT(sensor.LoadConfig(path), "the profile should load");
    sensor.SetApplyNoise(false);
    sensor.SetSeed(7U);

    PointCloudFrame frame;
    sensor.Scan(frame);

    const int bracket = mj_name2id(world->getModel(), mjOBJ_GEOM, "lidar_bracket_geom");
    const int target = mj_name2id(world->getModel(), mjOBJ_GEOM, "target_behind_bracket_geom");
    HAKO_TEST_EXPECT(bracket >= 0 && target >= 0, "the test scene should have both geoms");

    const std::set<int> hits(frame.geom_ids.begin(), frame.geom_ids.end());
    HAKO_TEST_EXPECT(hits.count(bracket) == 0,
                     "a child of the excluded body must not appear in the returns");
    // The ray has to pass through the mount, not be dropped at it.
    HAKO_TEST_EXPECT(hits.count(target) > 0,
                     "what stands behind the mount must still be seen");
}

void TestTheProfileOverridesTheConstructorNames()
{
    // lidar_2d lets mjcf_binding win over the constructor arguments, which are
    // the default. Naming a body that is not in the scene must not survive.
    auto world = MakeWorld();
    LiDAR3DSensor sensor(world, "not_a_body", "not_a_site", "not_a_body");
    const auto path = (RepoRoot() / "config/sensors/lidar/livox-mid360s.json").string();
    HAKO_TEST_EXPECT(sensor.LoadConfig(path), "the profile should load");
    sensor.SetApplyNoise(false);

    PointCloudFrame frame;
    sensor.Scan(frame);
    HAKO_TEST_EXPECT(frame.size() > 0, "the profile's mjcf_binding should have been used");
}

void TestTheFirstUpdateScansImmediately()
{
    // Every other sensor here calls StartReady, so the first ShouldUpdate
    // fires rather than waiting out a period.
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    HAKO_TEST_EXPECT(sensor.ShouldUpdate(0.0), "the first update should be ready");
}

void TestAUniformProfileCannotClaimNonRepetitiveScanning()
{
    auto world = MakeWorld();
    LiDAR3DSensor sensor(world);
    // A uniform pattern draws fresh angles every frame and does not reproduce
    // the structured coverage of a non-repetitive scanner, so the profile must
    // not be able to claim it.
    const auto path = (RepoRoot() / "models/sensors/lidar_3d").string();
    (void)path;
    HAKO_TEST_EXPECT(!sensor.LoadConfig(
        (RepoRoot() / "tests/sensors/lidar/unit/uniform-non-repetitive.json").string()),
        "uniform plus NonRepetitive should be refused");
}

void TestTheSameSeedGivesTheSameScan()
{
    auto world = MakeWorld();
    PointCloudFrame first;
    PointCloudFrame second;
    MakeSensor(world, false).Scan(first);
    MakeSensor(world, false).Scan(second);

    HAKO_TEST_EXPECT(first.size() == second.size(), "a seeded scan should be reproducible");
    for (size_t i = 0; i < first.size(); ++i) {
        HAKO_TEST_EXPECT(NearlyEqual(first.distances[i], second.distances[i]),
                         "a seeded scan should be reproducible");
    }
}

void TestConsecutiveFramesDifferer()
{
    auto world = MakeWorld();
    auto sensor = MakeSensor(world, false);
    PointCloudFrame first;
    PointCloudFrame second;
    sensor.Scan(first);
    sensor.Scan(second);

    // The pattern advances, so a second frame samples different directions.
    // Without this the accumulated coverage of a static scene never improves.
    bool differs = first.size() != second.size();
    for (size_t i = 0; !differs && i < first.size(); ++i) {
        differs = !NearlyEqual(first.distances[i], second.distances[i]);
    }
    HAKO_TEST_EXPECT(differs, "consecutive frames should not repeat the same directions");
}
}

int main()
{
    try {
        TestDatasheetValuesArePreserved();
        TestRaysPerFrameFollowPointRateOverFrameRate();
        TestReturnsStayInsideTheRangeGate();
        TestPointsAreConsistentWithDistances();
        TestSensorDoesNotDetectItsOwnMount();
        TestLowObjectInsideTheBlindConeReturnsNothing();
        TestNoisePerturbsRangesWithoutMovingTheGeometry();
        TestTablePatternIsRejectedRatherThanSilentlyIgnored();
        TestAChildOfTheExcludedBodyIsAlsoExcluded();
        TestTheProfileOverridesTheConstructorNames();
        TestTheFirstUpdateScansImmediately();
        TestAUniformProfileCannotClaimNonRepetitiveScanning();
        TestTheSameSeedGivesTheSameScan();
        TestConsecutiveFramesDifferer();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "lidar_3d_sensor_test passed" << std::endl;
    return EXIT_SUCCESS;
}

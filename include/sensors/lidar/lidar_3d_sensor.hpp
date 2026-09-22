#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/common/update_scheduler.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor::lidar
{
    // DetectionDistance and DistanceAccuracy are shared with the 2D sensor and
    // are declared in lidar_2d_sensor.hpp. A lidar_common.hpp would be the
    // tidier home for them; that is a refactor of existing code and is left to
    // the owner of this repository.

    struct AngleSpan
    {
        double min_deg {0.0};
        double max_deg {0.0};
    };

    // A solid angle, replacing the 2D sensor's single AngleRange.
    struct FieldOfView
    {
        AngleSpan horizontal {-180.0, 180.0};
        AngleSpan vertical {-7.0, 52.0};
    };

    enum class ScanPatternType
    {
        // Samples the declared field of view. Depends on nothing but the
        // datasheet, and its floor blind zone is a true circle.
        Uniform,
        // Replays a recorded ray-angle sequence in acquisition order, which is
        // what reproduces non-repetitive scanning.
        Table,
    };

    struct ScanPattern
    {
        ScanPatternType type {ScanPatternType::Uniform};
        int point_rate {200000};
        double frame_rate_hz {10.0};
        bool non_repetitive {false};
        std::string table_file {};
        std::string table_format {};
        std::string table_provenance {};
    };

    struct LiDAR3DConfig
    {
        OutputBinding output {};
        std::string frame_id {"lidar"};
        DetectionDistance detection_distance {};
        FieldOfView field_of_view {};
        ScanPattern scan_pattern {};
        std::vector<DistanceAccuracy> distance_accuracy {};
    };

    // One frame of returns, in the sensor frame. Rays that hit nothing, or hit
    // outside the range gate, are not represented: a frame is the returns, not
    // the rays, so size() is not the ray count. rays_cast records that.
    struct PointCloudFrame
    {
        std::string frame_id {"lidar"};
        std::vector<float> xyz {};        // three floats per point
        std::vector<float> distances {};  // metres, one per point
        std::vector<int> geom_ids {};     // which geom returned each point
        int rays_cast {0};

        size_t size() const { return distances.size(); }
        void clear()
        {
            xyz.clear();
            distances.clear();
            geom_ids.clear();
            rays_cast = 0;
        }
    };

    class ILidar3DSensor : public ISensor
    {
    public:
        virtual ~ILidar3DSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const LiDAR3DConfig& GetConfig() const = 0;
        virtual int GetSamplesPerFrame() const = 0;

        // Cast one frame of rays and collect the returns.
        virtual void Scan(PointCloudFrame& out) = 0;
    };

    class LiDAR3DSensor : public ILidar3DSensor
    {
    public:
        LiDAR3DSensor(
            std::shared_ptr<hako::robots::physics::IWorld> world,
            std::string sensor_body_name = "lidar_mount",
            std::string sensor_site_name = "lidar_site",
            std::string exclude_body_name = "lidar_mount");

        bool LoadConfig(const std::string& config_path) override;
        const LiDAR3DConfig& GetConfig() const override;
        int GetSamplesPerFrame() const override;
        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;
        void Scan(PointCloudFrame& out) override;

        // Fixed seed by default so a scan is reproducible in tests.
        void SetSeed(unsigned int seed);
        void SetApplyNoise(bool enabled);

    private:
        void RebuildNoisePipeline();
        void NextDirections(std::vector<mjtNum>& directions);

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        std::string sensor_body_name_;
        std::string sensor_site_name_;
        std::string exclude_body_name_;
        LiDAR3DConfig config_ {};
        common::UpdateScheduler scheduler_ {};
        noise::RangeNoisePipeline noise_pipeline_;
        std::mt19937 rng_ {12345U};
        bool apply_noise_ {true};

        // Reused between frames so a 10 Hz scan does not reallocate.
        std::vector<mjtNum> directions_ {};
        std::vector<mjtNum> distances_ {};
        std::vector<int> geom_ids_ {};
    };
}

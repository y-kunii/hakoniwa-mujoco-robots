// Hakoniwa asset that publishes the Livox Mid-360S point cloud as a PDU.
//
// The C++ counterpart of livox-mid360s-hakoniwa-asset.py. It exists because
// src/sensors is where the C++ simulator main loop finds its sensors: a
// Python-only Mid-360S cannot be mounted on the C++ robot samples.
//
// Deliberately smaller than ultrasonic-hakoniwa-asset.cpp, which also opens a
// MuJoCo viewer and drives a freejoint from the keyboard. This sensor is a
// static mount and its display is read_point_cloud.py, which receives the PDU
// and does not care which language published it. So this asset only publishes.
//
//     ./src/cmake-build/examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset
//
#include "config/asset_manifest.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/point_cloud2.hpp"
#include "physics/physics_impl.hpp"
#include "runtime/hakoniwa_asset_lifecycle.hpp"
#include "sensors/lidar/lidar_3d_sensor.hpp"

#include <atomic>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

namespace {

class LivoxMid360SHakoniwaAssetApp
{
public:
    int Run(int argc, char** argv)
    {
        if (argc > 1 && std::string(argv[1]) == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }

        if (!LoadManifest(argc, argv) ||
            !InitializeWorld() ||
            !InitializeSensor() ||
            !InitializeLifecycle())
        {
            return 1;
        }

        std::string lifecycle_error;
        if (!asset_lifecycle_->RegisterAndRunAsset(
                [this](hakoniwa::pdu::Endpoint& endpoint) {
                    return RunManualTimingControl(endpoint);
                },
                [this]() { sensor_->Reset(); frames_ = 0; return 0; },
                &lifecycle_error))
        {
            std::cerr << "[ERROR] " << lifecycle_error << std::endl;
            return 1;
        }

        std::cout << "[INFO] published " << frames_ << " frames" << std::endl;
        return 0;
    }

private:
    static constexpr const char* kDefaultManifestPath =
        "config/assets/livox-mid360s-hakoniwa-asset.json";
    static constexpr const char* kComponentId = "mid360s";

    static std::string EnvOrDefault(const char* name, const char* fallback)
    {
        const char* value = std::getenv(name);
        return (value != nullptr && value[0] != '\0') ? std::string(value) : std::string(fallback);
    }

    static void PrintUsage(const char* program)
    {
        std::cout
            << "Usage:\n"
            << "  " << program << " [manifest.json]\n\n"
            << "Defaults:\n"
            << "  manifest.json " << kDefaultManifestPath << "\n\n"
            << "Environment:\n"
            << "  HAKO_LIVOX_MID360S_MANIFEST_PATH  manifest path\n"
            << "  HAKO_LIVOX_MID360S_ASSET_NAME     asset registration name,"
               " default manifest pdu_robot\n"
            << "  HAKO_LIVOX_MID360S_MAX_FRAMES     stop after this many frames, default 0"
               " for unlimited\n\n"
            << "Read the published cloud with:\n"
            << "  python3 examples/sensors/livox_mid360s/read_point_cloud.py\n";
    }

    bool ReadEndpointName(const std::string& path, std::string& out) const
    {
        std::ifstream file(path);
        if (!file) {
            std::cerr << "[ERROR] cannot open endpoint config: " << path << std::endl;
            return false;
        }
        nlohmann::json root;
        file >> root;
        out = root.value("name", std::string("livox_mid360s_endpoint"));
        return true;
    }

    bool LoadManifest(int argc, char** argv)
    {
        const std::string manifest_path = argc > 1
            ? argv[1]
            : EnvOrDefault("HAKO_LIVOX_MID360S_MANIFEST_PATH", kDefaultManifestPath);

        std::string error;
        if (!hako::robots::config::LoadAssetManifestFromJson(manifest_path, manifest_, &error)) {
            std::cerr << "[ERROR] Failed to load manifest: " << error << std::endl;
            return false;
        }

        component_ = manifest_.FindComponent(kComponentId);
        if (component_ == nullptr || component_->pdu_robot.empty()) {
            std::cerr << "[ERROR] Manifest component is missing or has no pdu_robot: "
                      << kComponentId << std::endl;
            return false;
        }

        asset_name_ = EnvOrDefault(
            "HAKO_LIVOX_MID360S_ASSET_NAME", component_->pdu_robot.c_str());
        max_frames_ = std::atoll(EnvOrDefault("HAKO_LIVOX_MID360S_MAX_FRAMES", "0").c_str());
        return ReadEndpointName(manifest_.endpoint, endpoint_name_);
    }

    bool InitializeWorld()
    {
        world_ = std::make_shared<hako::robots::physics::impl::WorldImpl>();
        try {
            world_->loadModel(manifest_.model);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool InitializeSensor()
    {
        // The mount and the exclusion come from the profile's mjcf_binding, so
        // this asset does not restate names the scene and the profile share.
        sensor_ = std::make_unique<hako::robots::sensor::lidar::LiDAR3DSensor>(world_);
        if (!sensor_->LoadConfig(component_->config)) {
            std::cerr << "[ERROR] Failed to load the lidar_3d profile: "
                      << component_->config << std::endl;
            std::cerr << "        A table scan pattern is not supported by the C++ sensor."
                      << std::endl;
            return false;
        }
        return true;
    }

    bool InitializeLifecycle()
    {
        const auto delta_time_usec =
            static_cast<hako_time_t>(world_->getModel()->opt.timestep * 1.0e6);

        asset_lifecycle_ = std::make_unique<hako::robots::runtime::HakoniwaAssetLifecycle>(
            hako::robots::runtime::HakoniwaAssetLifecycleConfig {
                endpoint_name_,
                manifest_.endpoint,
                asset_name_,
                manifest_.pdu_def,
                delta_time_usec,
                HAKO_ASSET_MODEL_PLANT
            });

        std::string error;
        if (!asset_lifecycle_->OpenEndpoint(&error)) {
            std::cerr << "[ERROR] " << error << std::endl;
            return false;
        }

        const auto& config = sensor_->GetConfig();
        const hakoniwa::pdu::PduKey key {component_->pdu_robot, config.output.pdu_name};
        cloud_adapter_ =
            std::make_unique<hako::robots::pdu::adapter::sensor_msgs::PointCloud2PduAdapter>(
                asset_lifecycle_->Endpoint(), key, config.pdu_config.max_points);

        std::cout << "[INFO] manifest : " << manifest_.path << "\n"
                  << "[INFO] model    : " << manifest_.model << "\n"
                  << "[INFO] profile  : " << component_->config << "\n"
                  << "[INFO] channel  : " << component_->pdu_robot << "/"
                  << config.output.pdu_name << "\n"
                  << "[INFO] pattern  : uniform, " << sensor_->GetSamplesPerFrame()
                  << " rays at " << config.scan_pattern.frame_rate_hz << " Hz\n"
                  << "[INFO] budget   : " << config.pdu_config.max_points
                  << " points at point_step " << config.pdu_config.point_step << std::endl;
        return true;
    }

    int RunManualTimingControl(hakoniwa::pdu::Endpoint& endpoint)
    {
        (void)endpoint;
        const double sim_timestep = world_->getModel()->opt.timestep;
        const auto delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1.0e6);
        const auto& config = sensor_->GetConfig();

        std::cout << "[INFO] simulation running; publishing point clouds" << std::endl;

        hako::robots::sensor::lidar::PointCloudFrame frame;
        while (running_.load()) {
            world_->advanceTimeStep();

            if (sensor_->ShouldUpdate(sim_timestep)) {
                sensor_->Scan(frame);
                const double stamp_sec = world_->getData()->time;
                if (!cloud_adapter_->send(config, frame, stamp_sec)) {
                    std::cerr << "[WARN] Failed to send the point cloud PDU." << std::endl;
                }
                ++frames_;
                if (frames_ <= 3 || frames_ % 10 == 0) {
                    std::cout << "[INFO] frame " << frames_ << "  " << frame.size()
                              << " pts of " << frame.rays_cast << " rays" << std::endl;
                }
                if (max_frames_ > 0 && frames_ >= max_frames_) {
                    std::cout << "[INFO] reached max frames " << max_frames_ << std::endl;
                    running_.store(false);
                }
            }

            hako_asset_usleep(delta_time_usec);
        }
        return 0;
    }

    hako::robots::config::AssetManifest manifest_ {};
    const hako::robots::config::AssetManifestComponent* component_ {nullptr};
    std::string asset_name_ {};
    std::string endpoint_name_ {};
    long long max_frames_ {0};
    long long frames_ {0};
    std::atomic_bool running_ {true};

    std::shared_ptr<hako::robots::physics::impl::WorldImpl> world_ {};
    std::unique_ptr<hako::robots::sensor::lidar::LiDAR3DSensor> sensor_ {};
    std::unique_ptr<hako::robots::runtime::HakoniwaAssetLifecycle> asset_lifecycle_ {};
    std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::PointCloud2PduAdapter> cloud_adapter_ {};
};

} // namespace

int main(int argc, char** argv)
{
    LivoxMid360SHakoniwaAssetApp app;
    return app.Run(argc, argv);
}

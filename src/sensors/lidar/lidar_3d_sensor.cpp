#include "sensors/lidar/lidar_3d_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "config/json_config_utils.hpp"
#include "sensors/common/json_utils.hpp"

namespace hako::robots::sensor::lidar
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

noise::NoiseType parse_noise_type_3d(const std::string& value)
{
    if (value == "none" || value == "None") {
        return noise::NoiseType::None;
    }
    if (value == "gaussian_quantized" ||
        value == "GaussianQuantized" ||
        value == "gaussian-quantized" ||
        value == "Gaussian-Quantized")
    {
        return noise::NoiseType::GaussianQuantized;
    }
    return noise::NoiseType::Gaussian;
}

const common::json* find_first(const common::json& node, const char* a, const char* b)
{
    if (node.contains(a)) {
        return &node.at(a);
    }
    if (node.contains(b)) {
        return &node.at(b);
    }
    return nullptr;
}

void read_span(const common::json& node, const char* key, AngleSpan& span)
{
    if (!node.contains(key) || !node.at(key).is_object()) {
        return;
    }
    const auto& entry = node.at(key);
    span.min_deg = common::get_json_number(entry, "Min", span.min_deg);
    span.max_deg = common::get_json_number(entry, "Max", span.max_deg);
}
}

LiDAR3DSensor::LiDAR3DSensor(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    std::string sensor_body_name,
    std::string sensor_site_name,
    std::string exclude_body_name)
    : world_(std::move(world))
    , sensor_body_name_(std::move(sensor_body_name))
    , sensor_site_name_(std::move(sensor_site_name))
    , exclude_body_name_(std::move(exclude_body_name))
{
}

bool LiDAR3DSensor::LoadConfig(const std::string& config_path)
{
    common::json root;
    if (!common::load_json_file(config_path, root)) {
        return false;
    }

    config_ = LiDAR3DConfig {};
    const auto* spec = hako::robots::config::FindObject(root, "spec");
    const auto& spec_root = (spec != nullptr) ? *spec : root;

    config_.output.name = common::get_json_string(spec_root, "name", "point_cloud");
    config_.output.pdu_name = "point_cloud";
    config_.frame_id = spec_root.value("frame_id", std::string("lidar"));

    if (spec_root.contains("DetectionDistance")) {
        const auto& det = spec_root.at("DetectionDistance");
        // Millimetres in the profile, metres in the sensor, as lidar_2d does.
        config_.detection_distance.min = common::get_json_number(det, "Min", 100.0) / 1000.0;
        config_.detection_distance.max = common::get_json_number(det, "Max", 100000.0) / 1000.0;
    }

    if (spec_root.contains("FieldOfView") && spec_root.at("FieldOfView").is_object()) {
        const auto& fov = spec_root.at("FieldOfView");
        read_span(fov, "Horizontal", config_.field_of_view.horizontal);
        read_span(fov, "Vertical", config_.field_of_view.vertical);
    }

    if (spec_root.contains("ScanPattern") && spec_root.at("ScanPattern").is_object()) {
        const auto& pattern = spec_root.at("ScanPattern");
        const std::string type = pattern.value("Type", std::string("uniform"));
        config_.scan_pattern.type =
            (type == "table") ? ScanPatternType::Table : ScanPatternType::Uniform;
        config_.scan_pattern.point_rate =
            common::get_json_int(pattern, "PointRate", config_.scan_pattern.point_rate);
        config_.scan_pattern.frame_rate_hz =
            common::get_json_number(pattern, "FrameRate", config_.scan_pattern.frame_rate_hz);
        if (pattern.contains("NonRepetitive") && pattern.at("NonRepetitive").is_boolean()) {
            config_.scan_pattern.non_repetitive = pattern.at("NonRepetitive").get<bool>();
        }
        config_.scan_pattern.table_file = pattern.value("TableFile", std::string(""));
        config_.scan_pattern.table_format = pattern.value("TableFormat", std::string(""));
        config_.scan_pattern.table_provenance = pattern.value("TableProvenance", std::string(""));
    }

    if (config_.scan_pattern.type == ScanPatternType::Uniform &&
        config_.scan_pattern.non_repetitive)
    {
        // A uniform pattern draws fresh angles every frame; it does not
        // reproduce the structured coverage of a non-repetitive scanner.
        // Accepting the flag and ignoring it would claim behaviour that is
        // not there.
        return false;
    }

    if (config_.scan_pattern.type == ScanPatternType::Table) {
        // Replaying a recorded table is implemented in the Python sensor, which
        // can read the .npy the tooling produces. Whether that belongs in C++,
        // and in what format, is an open question for this repository; failing
        // here is better than silently scanning a different pattern.
        return false;
    }

    config_.output.update_rate_hz = config_.scan_pattern.frame_rate_hz;
    if (spec == nullptr) {
        config_.output.pdu_name =
            common::get_json_string(root, "pdu_name", config_.output.pdu_name);
        config_.output.update_rate_hz =
            common::get_json_number(root, "update_rate_hz", config_.output.update_rate_hz);
    }
    hako::robots::config::ReadPduConfig(root, config_.output.pdu_name, config_.output.update_rate_hz);
    if (config_.output.update_rate_hz > 0.0) {
        // lidar_2d writes the PDU rate back into its scan rate; without this
        // a profile that sets update_rate_hz would publish at one rate and
        // scan at another.
        config_.scan_pattern.frame_rate_hz = config_.output.update_rate_hz;
    }

    config_.distance_accuracy.clear();
    if (spec_root.contains("DistanceAccuracy") && spec_root.at("DistanceAccuracy").is_array()) {
        for (const auto& entry : spec_root.at("DistanceAccuracy")) {
            DistanceAccuracy accuracy {};
            if (entry.contains("Range") && entry.at("Range").is_object()) {
                const auto& range = entry.at("Range");
                accuracy.range.min = common::get_json_number(range, "Min", 0.0) / 1000.0;
                accuracy.range.max = common::get_json_number(range, "Max", 0.0) / 1000.0;
            }
            std::string type = entry.value("type", std::string(""));
            if (type.empty()) {
                type = entry.value("Type", std::string("independent"));
            }
            accuracy.distance_dependent = (type == "dependent");
            // lidar_2d tolerates the misspelled keys that shipped in older
            // profiles. Reading only the correct spelling would drop a
            // profile's noise settings without saying so.
            const auto* band = accuracy.distance_dependent
                ? find_first(entry, "DistanceDependentAccuracy", "DistanceDepedentAccuracy")
                : find_first(entry, "DistanceIndependentAccuracy", "DistanceIndepedentAccuracy");
            if (band != nullptr) {
                if (accuracy.distance_dependent) {
                    accuracy.percentage = common::get_json_number(*band, "Percentage", 0.0);
                } else {
                    accuracy.stddev = common::get_json_number(*band, "StdDev", 0.0);
                }
                accuracy.noise_distribution = band->value("NoiseDistribution", std::string("Gaussian"));
                accuracy.precision = common::get_json_number(*band, "Precision", 0.0);
            }
            config_.distance_accuracy.push_back(std::move(accuracy));
        }
    }

    if (const auto* pdu = hako::robots::config::FindObject(root, "pdu_config"); pdu != nullptr) {
        config_.pdu_config.message_type =
            pdu->value("message_type", config_.pdu_config.message_type);
        config_.pdu_config.max_points = static_cast<size_t>(
            common::get_json_int(*pdu, "max_points", static_cast<int>(config_.pdu_config.max_points)));
        config_.pdu_config.point_step = static_cast<size_t>(
            common::get_json_int(*pdu, "point_step", static_cast<int>(config_.pdu_config.point_step)));
    }

    if (const auto* binding = hako::robots::config::FindObject(root, "mjcf_binding");
        binding != nullptr)
    {
        config_.mjcf_binding.config_style =
            binding->value("config_style", config_.mjcf_binding.config_style);
        config_.mjcf_binding.runtime_source =
            binding->value("runtime_source", config_.mjcf_binding.runtime_source);
        config_.mjcf_binding.source_body =
            binding->value("source_body", config_.mjcf_binding.source_body);
        config_.mjcf_binding.source_site =
            binding->value("source_site", config_.mjcf_binding.source_site);
        config_.mjcf_binding.exclude_body =
            binding->value("exclude_body", config_.mjcf_binding.exclude_body);
    }

    // The profile wins over the constructor arguments, as lidar_2d does: the
    // arguments are the default, and mjcf_binding names the mount the scene
    // actually has.
    if (!config_.mjcf_binding.source_body.empty()) {
        sensor_body_name_ = config_.mjcf_binding.source_body;
    }
    if (!config_.mjcf_binding.source_site.empty()) {
        sensor_site_name_ = config_.mjcf_binding.source_site;
    }
    if (!config_.mjcf_binding.exclude_body.empty()) {
        exclude_body_name_ = config_.mjcf_binding.exclude_body;
    }

    RebuildNoisePipeline();
    // Every other sensor here starts ready, so the first ShouldUpdate scans
    // instead of waiting out a period.
    scheduler_.StartReady(GetUpdatePeriodSec());
    return true;
}

const LiDAR3DConfig& LiDAR3DSensor::GetConfig() const
{
    return config_;
}

int LiDAR3DSensor::GetSamplesPerFrame() const
{
    if (config_.scan_pattern.frame_rate_hz <= 0.0) {
        return 0;
    }
    // Checkable against the datasheet rather than hard coded.
    const double samples =
        static_cast<double>(config_.scan_pattern.point_rate) / config_.scan_pattern.frame_rate_hz;
    return std::max(0, static_cast<int>(std::lround(samples)));
}

void LiDAR3DSensor::Reset()
{
    scheduler_.Reset();
}

double LiDAR3DSensor::GetUpdatePeriodSec() const
{
    if (config_.scan_pattern.frame_rate_hz <= 0.0) {
        return 0.1;
    }
    return 1.0 / config_.scan_pattern.frame_rate_hz;
}

bool LiDAR3DSensor::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
}

void LiDAR3DSensor::SetSeed(unsigned int seed)
{
    rng_.seed(seed);
}

void LiDAR3DSensor::SetApplyNoise(bool enabled)
{
    apply_noise_ = enabled;
}

void LiDAR3DSensor::RebuildNoisePipeline()
{
    noise_pipeline_.Clear();
    for (const auto& accuracy : config_.distance_accuracy) {
        noise::RangeNoiseRule rule {};
        rule.range.min = accuracy.range.min;
        rule.range.max = accuracy.range.max;
        rule.distance_dependent = accuracy.distance_dependent;
        rule.percentage = accuracy.percentage;
        rule.noise.stddev = accuracy.stddev;
        rule.noise.precision = accuracy.precision;
        rule.noise.type = parse_noise_type_3d(accuracy.noise_distribution);
        noise_pipeline_.AddRule(rule);
    }
}

void LiDAR3DSensor::NextDirections(std::vector<mjtNum>& directions)
{
    const int samples = GetSamplesPerFrame();
    directions.resize(static_cast<size_t>(samples) * 3U);

    const auto& fov = config_.field_of_view;
    std::uniform_real_distribution<double> azimuth(
        fov.horizontal.min_deg * kPi / 180.0, fov.horizontal.max_deg * kPi / 180.0);
    std::uniform_real_distribution<double> elevation(
        fov.vertical.min_deg * kPi / 180.0, fov.vertical.max_deg * kPi / 180.0);

    for (int i = 0; i < samples; ++i) {
        const double az = azimuth(rng_);
        const double el = elevation(rng_);
        const double horizontal = std::cos(el);
        directions[static_cast<size_t>(3 * i) + 0] = horizontal * std::cos(az);
        directions[static_cast<size_t>(3 * i) + 1] = horizontal * std::sin(az);
        directions[static_cast<size_t>(3 * i) + 2] = std::sin(el);
    }
}

double LiDAR3DSensor::CastPastSelf(
    const mjModel* model,
    mjData* data,
    const mjtNum* origin,
    const mjtNum* direction,
    int exclude_id,
    double travelled,
    int& geom_id) const
{
    // The same walk lidar_2d does, for one ray: step just past a self geom and
    // cast again, so the ray passes through the mount instead of being lost.
    constexpr int kMaxAttempts = 16;
    constexpr mjtNum kEpsilon = 1.0e-4;
    mjtNum point[3] = {origin[0], origin[1], origin[2]};
    mjtNum dir[3] = {direction[0], direction[1], direction[2]};

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        int hit_geom = -1;
        const mjtNum hit = mj_ray(model, data, point, dir, nullptr, 1, exclude_id,
                                  &hit_geom, nullptr);
        if (hit < 0.0) {
            geom_id = -1;
            return -1.0;
        }
        if (!IsSelfGeom(model, exclude_id, hit_geom)) {
            geom_id = hit_geom;
            return travelled + static_cast<double>(hit);
        }
        const mjtNum step = hit + kEpsilon;
        travelled += static_cast<double>(step);
        if (travelled >= config_.detection_distance.max) {
            geom_id = -1;
            return -1.0;
        }
        point[0] += dir[0] * step;
        point[1] += dir[1] * step;
        point[2] += dir[2] * step;
    }

    geom_id = -1;
    return -1.0;
}

void LiDAR3DSensor::Scan(PointCloudFrame& out)
{
    out.clear();
    out.frame_id = config_.frame_id;

    auto* model = world_->getModel();
    auto* data = world_->getData();
    if (model == nullptr || data == nullptr) {
        return;
    }

    const int site_id = sensor_site_name_.empty()
        ? -1
        : mj_name2id(model, mjOBJ_SITE, sensor_site_name_.c_str());
    const int body_id = mj_name2id(model, mjOBJ_BODY, sensor_body_name_.c_str());
    if (site_id < 0 && body_id < 0) {
        return;
    }

    // The site is the finer origin when the binding names one, as in the
    // profile's mjcf_binding.source_site; otherwise the body origin is used.
    const mjtNum* origin = (site_id >= 0) ? &data->site_xpos[3 * site_id] : &data->xpos[3 * body_id];
    const mjtNum* rotation = (site_id >= 0) ? &data->site_xmat[9 * site_id] : &data->xmat[9 * body_id];

    const int samples = GetSamplesPerFrame();
    if (samples <= 0) {
        return;
    }

    NextDirections(directions_);

    // The rays are cast in world coordinates; the returns are reported in the
    // sensor frame, so the local direction is kept and reused for the point.
    std::vector<mjtNum> world_directions(directions_.size());
    for (int i = 0; i < samples; ++i) {
        const size_t base = static_cast<size_t>(3 * i);
        for (int row = 0; row < 3; ++row) {
            world_directions[base + static_cast<size_t>(row)] =
                rotation[3 * row + 0] * directions_[base + 0] +
                rotation[3 * row + 1] * directions_[base + 1] +
                rotation[3 * row + 2] * directions_[base + 2];
        }
    }

    distances_.assign(static_cast<size_t>(samples), -1.0);
    geom_ids_.assign(static_cast<size_t>(samples), -1);

    const int exclude_id = exclude_body_name_.empty()
        ? -1
        : mj_name2id(model, mjOBJ_BODY, exclude_body_name_.c_str());

    // geomgroup and normal are nullable; surface normals are not published.
    mj_multiRay(model, data, origin, world_directions.data(),
                /*geomgroup=*/nullptr, /*flg_static=*/1, exclude_id,
                geom_ids_.data(), distances_.data(), /*normal=*/nullptr, samples,
                static_cast<mjtNum>(config_.detection_distance.max));

    // mj_multiRay excludes only the named body's own geoms. A mount with child
    // bodies, which is what a sensor on a robot has, is otherwise seen by its
    // own sensor. Re-cast just the rays that hit one.
    if (exclude_id >= 0) {
        for (int i = 0; i < samples; ++i) {
            const int hit_geom = geom_ids_[static_cast<size_t>(i)];
            if (hit_geom < 0 || !IsSelfGeom(model, exclude_id, hit_geom)) {
                continue;
            }
            int resolved = -1;
            const double distance = CastPastSelf(
                model, data, origin, &world_directions[static_cast<size_t>(3 * i)],
                exclude_id, 0.0, resolved);
            distances_[static_cast<size_t>(i)] = distance;
            geom_ids_[static_cast<size_t>(i)] = resolved;
        }
    }

    out.rays_cast = samples;
    out.xyz.reserve(static_cast<size_t>(samples) * 3U);
    out.distances.reserve(static_cast<size_t>(samples));
    out.geom_ids.reserve(static_cast<size_t>(samples));

    for (int i = 0; i < samples; ++i) {
        const double raw = distances_[static_cast<size_t>(i)];
        if (raw < 0.0) {
            continue;  // the ray hit nothing within the cutoff
        }
        const double measured = apply_noise_ ? noise_pipeline_.Apply(raw) : raw;
        if (measured < config_.detection_distance.min || measured >= config_.detection_distance.max) {
            continue;  // outside the declared range gate
        }
        const size_t base = static_cast<size_t>(3 * i);
        out.xyz.push_back(static_cast<float>(directions_[base + 0] * measured));
        out.xyz.push_back(static_cast<float>(directions_[base + 1] * measured));
        out.xyz.push_back(static_cast<float>(directions_[base + 2] * measured));
        out.distances.push_back(static_cast<float>(measured));
        out.geom_ids.push_back(geom_ids_[static_cast<size_t>(i)]);
    }
}
}

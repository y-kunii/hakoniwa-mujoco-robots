#pragma once

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_PointCloud2.hpp"
#include "sensors/lidar/lidar_3d_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    // x, y, z, intensity as float32. Matches pdu_config.point_step in the
    // lidar_3d profile, which is what a publisher budgets the channel against.
    constexpr Hako_uint32 kPointCloud2FloatFields = 4;
    constexpr Hako_uint32 kPointCloud2PointStep = kPointCloud2FloatFields * sizeof(float);
    constexpr Hako_uint8 kPointFieldFloat32 = 7;

    inline HakoCpp_PointCloud2 ToHakoPdu(
        const hako::robots::sensor::lidar::LiDAR3DConfig& config,
        const hako::robots::sensor::lidar::PointCloudFrame& frame,
        double stamp_sec,
        size_t max_points = 0)
    {
        HakoCpp_PointCloud2 out {};

        // A PointCloud2 channel is a fixed size, so a frame that outgrows the
        // budget is truncated here rather than overrunning shared memory.
        const size_t points = (max_points > 0) ? std::min(frame.size(), max_points) : frame.size();

        hako::robots::sensor::MessageHeader header {};
        header.stamp_sec = stamp_sec;
        header.frame_id = config.frame_id.empty() ? frame.frame_id : config.frame_id;
        out.header = ToHakoHeader(header);

        out.height = 1;
        out.width = static_cast<Hako_uint32>(points);
        out.is_bigendian = false;
        out.is_dense = true;
        out.point_step = kPointCloud2PointStep;
        out.row_step = out.point_step * out.width;

        static const char* const names[kPointCloud2FloatFields] = {"x", "y", "z", "intensity"};
        out.fields.resize(kPointCloud2FloatFields);
        for (Hako_uint32 i = 0; i < kPointCloud2FloatFields; ++i) {
            out.fields[i].name = names[i];
            out.fields[i].offset = i * static_cast<Hako_uint32>(sizeof(float));
            out.fields[i].datatype = kPointFieldFloat32;
            out.fields[i].count = 1;
        }

        out.data.resize(points * kPointCloud2PointStep);
        for (size_t i = 0; i < points; ++i) {
            float point[kPointCloud2FloatFields] = {
                frame.xyz[3 * i + 0],
                frame.xyz[3 * i + 1],
                frame.xyz[3 * i + 2],
                1.0F,  // reflectivity is not simulated; a constant marks that
            };
            std::memcpy(out.data.data() + i * kPointCloud2PointStep, point, sizeof(point));
        }

        return out;
    }
}

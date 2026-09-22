#pragma once

#include "hakoniwa/pdu/converter/sensor_msgs/point_cloud2.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_PointCloud2.hpp"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"
#include "sensors/lidar/lidar_3d_sensor.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    class PointCloud2PduAdapter
    {
    public:
        PointCloud2PduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key,
            size_t max_points = 0)
            : endpoint_(endpoint, key)
            , max_points_(max_points)
        {
        }

        bool send(
            const hako::robots::sensor::lidar::LiDAR3DConfig& config,
            const hako::robots::sensor::lidar::PointCloudFrame& frame,
            double stamp_sec)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(
                config, frame, stamp_sec, max_points_);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_PointCloud2& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_PointCloud2,
            hako::pdu::msgs::sensor_msgs::PointCloud2> endpoint_;
        size_t max_points_ {0};
    };
}

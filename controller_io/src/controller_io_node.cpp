#include <controller_io/controller_io_node.h>

#include <cstdint>
#include <cstring>
#include <cmath>

#include <chrono>
#include <fmt/format.h>
#include <functional>
#include <memory>

// ROS2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <controller_io/auto_aim_packet.h>

namespace armor_auto_aim {
using Receive = custom_serial_interfaces::msg::Receive;
using SendPackage = custom_serial_interfaces::srv::SendPackage;
using Target = auto_aim_interfaces::msg::Target;

ControllerIONode::ControllerIONode(const rclcpp::NodeOptions& options)
    : Node("controller_io", options) {
    // Parameter
    auto master_tracker_topic = this->declare_parameter("master_tracker_topic", "/armor_tracker");
    auto slave_tracker_topic  = this->declare_parameter("slave_tracker_topic", "");
    m_gimbal_tf_broad = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Subsciption
    m_serial_sub = this->create_subscription<Receive>("/custom_serial/receive", rclcpp::SensorDataQoS(), 
        std::bind(&ControllerIONode::serialHandle, this, std::placeholders::_1));
    m_target_sub = this->create_subscription<Target>(master_tracker_topic + "/target", rclcpp::SensorDataQoS(), 
        std::bind(&ControllerIONode::trackerCallback, this, std::placeholders::_1));
    m_is_sentry = !slave_tracker_topic.empty();
    m_target_slave_sub = !m_is_sentry
        ? this->create_subscription<Target>(slave_tracker_topic + "/target", rclcpp::SensorDataQoS(), 
            std::bind(&ControllerIONode::trackerCallback, this, std::placeholders::_1))
        : nullptr;
    // Client
    m_serial_cli = this->create_client<SendPackage>("/custom_serial/send");
    // Armor detector target color client
    m_detect_color_cli = std::make_shared<rclcpp::AsyncParametersClient>(this,
        this->declare_parameter("armor_detector_name", "armor_detector_node"));
 

    // Visualization
    m_aim_marker.ns = "aim";
    m_aim_marker.type = visualization_msgs::msg::Marker::CUBE;
    m_aim_marker.scale.x = m_aim_marker.scale.y = m_aim_marker.scale.z = 0.1;
    m_aim_marker.color.a = 1;
    m_aim_marker.color.r = 1;
    m_aim_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    m_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/controll/aim_marker", 10);
    // Debug publish
    m_c_aim_pub = this->create_publisher<auto_aim_interfaces::msg::ControllerAim>("/controll/aim_point", 10);
}

void ControllerIONode::serialHandle(const Receive::SharedPtr serial_msg) {
    RCLCPP_DEBUG(this->get_logger(), "fun: %d; id: %d; len: %d; data: %s;",
        serial_msg->func_code, serial_msg->id, serial_msg->len, serial_msg->data.data());
    if (serial_msg->id == mControllerId || serial_msg->id == mSlaveControllerId) {
        switch (serial_msg->func_code) {
            case mNeedUpdateTimestamp: {
                uint64_t timestamp = static_cast<uint64_t>(this->now().nanoseconds());
                auto request = std::make_shared<SendPackage::Request>();
                request->func_code = mTimestampPackte;
                request->id = serial_msg->id - 10;
                request->len = sizeof(uint64_t);
                request->data.resize(request->len);
                std::memcpy(request->data.data(), &timestamp, sizeof(timestamp));

                using namespace std::chrono_literals;
                while (!m_serial_cli->wait_for_service(500ms)) {
                    if (!rclcpp::ok())
                        RCLCPP_ERROR(this->get_logger(), "Failed wait service;");
                    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
                }
                m_serial_cli->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Sync timerstmap");
                break;
            }
            case mGimbalPose: {
                GimbalPosePacket gimbal_pose_pkt;
                std::memcpy(&gimbal_pose_pkt, serial_msg->data.data(), sizeof(GimbalPosePacket));
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->now();
                // t.header.stamp = rclcpp::Time(static_cast<int64_t>(gimbal_pose_pkt.timestamp));
                t.header.frame_id = "odom";
                t.child_frame_id = serial_msg->id == mControllerId? "gimbal_link": "slave_gimbal_link";
                tf2::Quaternion q(gimbal_pose_pkt.x, gimbal_pose_pkt.y, gimbal_pose_pkt.z, gimbal_pose_pkt.w);
                // RCLCPP_INFO(this->get_logger(), "xyzw: %f %f %f %f", gimbal_pose_pkt.x, gimbal_pose_pkt.y,
                // gimbal_pose_pkt.z, gimbal_pose_pkt.w);
                t.transform.rotation = tf2::toMsg(q);
                t.transform.translation.y = serial_msg->id == mControllerId? 0: 0.23;
                m_gimbal_tf_broad->sendTransform(t);
                // aim info
                geometry_msgs::msg::Point p;
                p.x = gimbal_pose_pkt.px;
                p.y = gimbal_pose_pkt.py;
                p.z = gimbal_pose_pkt.pz;
                auto_aim_interfaces::msg::ControllerAim aim_msg;
                aim_msg.delay = gimbal_pose_pkt.delay;
                aim_msg.point = p;

                m_marker_array.markers.clear();
                m_aim_marker.header = t.header;
                m_aim_marker.action = visualization_msgs::msg::Marker::ADD;
                m_aim_marker.pose.position = p;
                m_aim_marker.pose.orientation = t.transform.rotation;
                m_marker_array.markers.emplace_back(m_aim_marker);

                m_c_aim_pub->publish(aim_msg);
                m_marker_pub->publish(m_marker_array);
                break;
            }
            case mSetTargetColor: {
                uint8_t color;
                std::memcpy(&color, serial_msg->data.data(), sizeof(uint8_t));
                this->setParam(rclcpp::Parameter("detect_color", color? "BLUE": "RED"));
                break;
            }
            default: {
                RCLCPP_WARN(this->get_logger(), "Unknow func_code: %d", serial_msg->func_code);
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknow id: %d", serial_msg->id);
    }
} 

void ControllerIONode::trackerCallback(const Target::SharedPtr target_msg) {
    using namespace std::chrono_literals;

    m_trakcer_state[static_cast<int>(target_msg->is_master)] = target_msg->tracking;

    AutoAimPacket packet;
    packet.x = target_msg->position.x;
    packet.y = target_msg->position.y;
    packet.z = target_msg->position.z;
    packet.v_x = target_msg->velocity.x;
    packet.v_y = target_msg->velocity.y;
    packet.v_z = target_msg->velocity.z;
    packet.theta = target_msg->yaw;
    packet.omega = target_msg->v_yaw;
    packet.r1 = target_msg->r1;
    packet.r2 = target_msg->r2;
    packet.dz = target_msg->dz;
    packet.delay = target_msg->delay;
    packet.is_tracking = target_msg->tracking;
    packet.num = target_msg->num;
    packet.id = target_msg->id;
    packet.is_follow = m_trakcer_state[0] ^ m_trakcer_state[1];

    auto request = std::make_shared<SendPackage::Request>();
    request->func_code = mAimPacket;
    request->id = !static_cast<uint8_t>(target_msg->is_master)/* ? mPCId: mPCId + 1 */;
    request->len = sizeof(AutoAimPacket);
    request->data.resize(request->len);
    std::memcpy(request->data.data(), &packet, request->len);
    
    while (!m_serial_cli->wait_for_service(500ms)) RCLCPP_WARN(this->get_logger(), "wait service timeout!");
    if (rclcpp::ok())
        m_serial_cli->async_send_request(request);
    else
        RCLCPP_ERROR(this->get_logger(), "rclcpp is not ok!");
}

void ControllerIONode::setParam(const rclcpp::Parameter& param) {
  if (!m_detect_color_cli->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }
  if (
    !m_set_param_future.valid() ||
    m_set_param_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %s...", param.as_string().c_str());
    m_set_param_future = m_detect_color_cli->set_parameters(
      {param}, [this, param](const ResultFuturePtr& results) {
        for (const auto& result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %s!", param.as_string().c_str());
        using namespace std::chrono_literals;

        param.get_name();  // TODO: Sentry
        auto request = std::make_shared<SendPackage::Request>();
        request->func_code = mSetTargetColor;
        request->id = mPCId;
        request->len = 0;
        request->data.resize(0);

        while (!m_serial_cli->wait_for_service(500ms)) RCLCPP_WARN(this->get_logger(), "wait service timeout!");
        if (rclcpp::ok()) {
            m_serial_cli->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Set color to noticed controller!");
        }
      });
  }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ControllerIONode)

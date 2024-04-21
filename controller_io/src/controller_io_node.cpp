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
    m_gimbal_tf_broad = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Subsciption
    m_serial_sub = this->create_subscription<Receive>("/custom_serial/receive", rclcpp::SensorDataQoS(), 
        std::bind(&ControllerIONode::serialHandle, this, std::placeholders::_1));
    m_target_sub = this->create_subscription<Target>("/armor_tracker/target", rclcpp::SensorDataQoS(), 
        std::bind(&ControllerIONode::trackerCallback, this, std::placeholders::_1));
    // Client
    m_serial_cli = this->create_client<SendPackage>("/custom_serial/send");

    // visul
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
    if (serial_msg->id == mControllerId) {
        switch (serial_msg->func_code) {
            case mNeedUpdateTimestamp: {
                uint64_t timestamp = std::chrono::duration_cast<
                    std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                auto request = std::make_shared<SendPackage::Request>();
                request->func_code = mTimestampPackte;
                request->id = mPCId;
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

                break;
            }
            case mGimbalPose: {
                GimbalPosePacket gimbal_pose_pkt;
                std::memcpy(&gimbal_pose_pkt, serial_msg->data.data(), sizeof(GimbalPosePacket));
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->now();
                t.header.frame_id = "odom";
                t.child_frame_id = "gimbal_link";
                tf2::Quaternion q(gimbal_pose_pkt.x, gimbal_pose_pkt.y, gimbal_pose_pkt.z, gimbal_pose_pkt.w);
                // tf2::Quaternion q;
                // q.setRPY(gimbal_pose_pkt.roll, gimbal_pose_pkt.pitch, gimbal_pose_pkt.yaw);
                t.transform.rotation = tf2::toMsg(q);
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

    auto request = std::make_shared<SendPackage::Request>();
    request->func_code = mAimPacket;
    request->id = mPCId;
    request->len = sizeof(AutoAimPacket);
    request->data.resize(request->len);
    std::memcpy(request->data.data(), &packet, request->len);

    while (!m_serial_cli->wait_for_service(500ms)) RCLCPP_WARN(this->get_logger(), "wait service timeout!");
    if (rclcpp::ok()) {
        m_serial_cli->async_send_request(request);
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ControllerIONode)
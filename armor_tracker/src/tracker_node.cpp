#include <armor_tracker/tracker_node.h>

// ROS2
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace armor_auto_aim {
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions& options)
    : Node("armor_tracker_node", options) {
    // Parameter
    m_odom_frame = this->declare_parameter("odom_frame", "odom");
    m_is_debug = this->declare_parameter("is_debug", true);
    this->declare_parameter("ekf.r_diagonal", std::vector<double>{1, 1, 1, 1, 1});
    this->declare_parameter("ekf.q_diagonal", std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
    int tt = this->declare_parameter("tracker.tracking_threshold", 5);
    int lt = this->declare_parameter("tracker.lost_threshold", 30);
    double mmd = this->declare_parameter("tracker.max_match_distance", 0.5);
    double mmy = this->declare_parameter("tracker.max_match_yaw", 1.0);
    // set trakcer param
    m_tracker.setMatchDistance(mmd);
    m_tracker.setMatchYaw(mmy);
    m_tracker.getStateMachine()->setTrackingCount(tt);
    m_tracker.getStateMachine()->setlostcount(lt);
    // tf buffer
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    m_tf_buffer->setCreateTimerInterface(timer_interface);
    // tf listensr
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    // Subscript armors
    m_armors_sub.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
    // tf filter
    m_tf_filter = std::make_shared<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>>(
        m_armors_sub, *m_tf_buffer, m_odom_frame, 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(10));
    m_tf_filter->registerCallback(&ArmorTrackerNode::subArmorsCallback, this);
    // Publisher
    m_target_pub = this->create_publisher<
        auto_aim_interfaces::msg::Target>("armor_tracker/target", rclcpp::SensorDataQoS());
    // Compensate for armor detector {{{
    this->declare_parameter("comlex_pattern_mode", 0);
    // }}}
    // Debug Publisher
    // m_odom_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("armor_tracker/debug/odom_pose", 10);
    // m_yaw_pub = this->create_publisher<std_msgs::msg::Float64>("armor_tracker/message_yaw", 10);
    // m_debug_angle = this->create_publisher<auto_aim_interfaces::msg::DebugAngle>("/debug/angle", 10);
    // Marker
    // car center
    m_center_marker.ns = "center";
    m_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    m_center_marker.scale.x = m_center_marker.scale.y = m_center_marker.scale.z = 0.1;
    m_center_marker.color.a = 1.0;
    m_center_marker.color.g = 1.0;
    // linear v
    m_linear_v_marker.ns = "linear_v";
    m_linear_v_marker.type = visualization_msgs::msg::Marker::ARROW;
    m_linear_v_marker.scale.x = 0.03;
    m_linear_v_marker.scale.y = 0.05;
    m_linear_v_marker.color.a = 1.0;
    m_linear_v_marker.color.r = 1.0;
    m_linear_v_marker.color.g = 1.0;
    // angular v
    m_omega_marker.ns = "omega";
    m_omega_marker.type = visualization_msgs::msg::Marker::ARROW;
    m_omega_marker.scale.x = 0.03;
    m_omega_marker.scale.y = 0.1;
    m_omega_marker.color.a = 1.0;
    m_omega_marker.color.g = 1.0;
    m_omega_marker.color.b = 1.0;
    // armors
    m_armors_marker.ns = "armors";
    m_armors_marker.type = visualization_msgs::msg::Marker::CUBE;
    m_armors_marker.scale.x = 0.03;
    m_armors_marker.scale.y = 0.135;
    m_armors_marker.scale.z = 0.125;
    m_armors_marker.color.a = 1.0;
    m_armors_marker.color.g = 1.0;
    // marker publisher
    m_marker_pub = this->create_publisher<
        visualization_msgs::msg::MarkerArray>("armor_tracker/marker", 10);
    //
    initEkf();
}

void ArmorTrackerNode::subArmorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armos_msg) {
    for (auto& armor: armos_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armos_msg->header;
        ps.pose = armor.pose;
        try {
            armor.world_pose = m_tf_buffer->transform(ps, m_odom_frame).pose;
        } catch (const tf2::ExtrapolationException& e) {
            RCLCPP_ERROR(this->get_logger(), "armor pose to odom");
            return;
        }
    }
    // Compensate for armor detector {{{
    // m_tracker.setComlexPatternMode(this->get_parameter(
    //     "comlex_pattern_mode").as_int());
    // }}}
    // target_msg
    rclcpp::Time timestamp = armos_msg->header.stamp;
    auto_aim_interfaces::msg::Target target_msg;
    target_msg.header.stamp = timestamp;
    target_msg.header.frame_id = m_odom_frame;
    target_msg.is_master = armos_msg->header.frame_id == "camera_optical_frame";
    if (m_tracker.state() == TrackerStateMachine::State::Lost) {
        m_last_stamp = this->now();
        m_tracker.initTracker(armos_msg);
        target_msg.tracking = false;
    } else {
        dt = (timestamp - m_last_stamp).seconds();
        m_tracker.updateTracker(armos_msg);

        target_msg.tracking = m_tracker.isTracking();
        if (m_tracker.isTracking()) {
            const auto& state = m_tracker.getTargetPredictSate();
            target_msg.position.x = state(0);
            target_msg.velocity.x = state(1);
            target_msg.position.y = state(2);
            target_msg.velocity.y = state(3);
            target_msg.position.z = state(4);
            target_msg.velocity.z = state(5);
            target_msg.yaw = state(6);
            target_msg.v_yaw = state(7);
            target_msg.r1 = state(8);
            target_msg.r2 = m_tracker.another_r;
            target_msg.dz = m_tracker.dz;
            target_msg.num = m_tracker.getNum();
            target_msg.delay = dt * 1000;
            target_msg.id = m_tracker.tracked_armor.number;
            // message yaw
            auto q = m_tracker.tracked_armor.world_pose.orientation;
            tf2::Quaternion tf_q;
            tf2::fromMsg(q, tf_q);
            double r, p, y;
            tf2::Matrix3x3(tf_q).getRPY(r, p, y);
            std_msgs::msg::Float64 yaw_msg;
            yaw_msg.data = y;
            // Debug angle
            // auto_aim_interfaces::msg::DebugAngle debug_angle;
            // debug_angle.header = armos_msg->header;
            // auto tp = target_msg.position;
            // debug_angle.yaw = std::atan2(tp.y, tp.x) * 180.0 / M_PI;
            // debug_angle.pitch = std::atan2(std::sqrt(tp.y*tp.y + tp.x*tp.x), tp.x) * 180.0 / M_PI;
            // m_odom_pose_pub->publish(m_tracker.tracked_armor.world_pose);
            // m_yaw_pub->publish(yaw_msg);
            // m_debug_angle->publish(debug_angle);
        }
    }

    m_last_stamp = timestamp;
    m_target_pub->publish(target_msg);
    if (m_is_debug)
        this->publishMarkers(target_msg);
}

void ArmorTrackerNode::initEkf() {
    auto f = [this](const Eigen::MatrixXd& x) -> Eigen::MatrixXd {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt;
        x_new(2) += x(3) * dt;
        x_new(4) += x(5) * dt;
        x_new(6) += x(7) * dt;
        return x_new;
    };
    auto j_f = [this](const Eigen::MatrixXd&)->Eigen::MatrixXd {
        Eigen::Matrix<double, 9, 9> F;
        //   x  vx  y  vy   z   yz  yaw v_yaw  r
        F << 1, dt, 0,  0,  0,  0,   0,  0,    0, // x
             0, 1,  0,  0,  0,  0,   0,  0,    0, // vx
             0, 0,  1,  dt, 0,  0,   0,  0,    0, // y
             0, 0,  0,  1,  0,  0,   0,  0,    0, // vy
             0, 0,  0,  0,  1,  dt,  0,  0,    0, // z
             0, 0,  0,  0,  0,  1,   0,  0,    0, // vz
             0, 0,  0,  0,  0,  0,   1,  dt,   0, // yaw
             0, 0,  0,  0,  0,  0,   0,  1,    0, // v_yaw
             0, 0,  0,  0,  0,  0,   0,  0,    1; // r
        return F;
    };

    auto h = [](const Eigen::VectorXd& x)->Eigen::MatrixXd {
        Eigen::VectorXd z(4);
        double xc = x[0], yc = x[2], yaw = x[6], r = x[8];
        z[0] = xc - r*cos(yaw);
        z[1] = yc - r*sin(yaw);
        z[2] = x[4];
        z[3] = x[6];
        return z;
    };
    auto j_h = [](const Eigen::VectorXd& x)->Eigen::MatrixXd {
        Eigen::MatrixXd h(4, 9);
        double yaw = x[6], r = x[8];
        //   x  vx  y  vy   z yz     yaw       v_yaw     r
        h << 1, 0,  0,  0,  0, 0,  r*sin(yaw),   0,   -cos(yaw), // x
             0, 0,  1,  0,  0, 0, -r*cos(yaw),   0,   -sin(yaw), // y
             0, 0,  0,  0,  1, 0,     0      ,   0,      0,      // z
             0, 0,  0,  0,  0, 0,     1      ,   0,      0;      // yaw
        return h;
    };
    auto update_Q = [this]()->Eigen::MatrixXd {
        if (m_tracker.ekf->getQ().size() == 0) {
            Eigen::DiagonalMatrix<double, 9> Q;
            auto q_dio = this->get_parameter("ekf.q_diagonal").as_double_array();
            Q.diagonal() = Eigen::Map<Eigen::VectorXd>(q_dio.data(), 9, 1);
            return Q;
        } else {
            return m_tracker.ekf->getQ();
        }
        double s2qxyz = 0.05,
               s2qyaw = 4.0,
               s2qr   = 80.0;
        Eigen::MatrixXd q(9, 9);
        double t       = dt,
               x       = s2qxyz,
               yaw     = s2qyaw,
               r       = s2qr,
               q_x_x   = pow(t, 4) / 4 * x,
               q_x_vx  = pow(t, 3) / 2 * x,
               q_vx_vx = pow(t, 2) * x,
               q_y_y   = pow(t, 4) / 4 * yaw,
               q_y_vy  = pow(t, 3) / 2 * x,
               q_vy_vy = pow(t, 2) * yaw,
               q_r     = pow(t, 4) / 4 * r;
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q << q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
             q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
             0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
             0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
             0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
             0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
             0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
             0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
             0,      0,      0,      0,      0,      0,      0,      0,      q_r;
        return q;
    };
    auto update_R = [this](const Eigen::MatrixXd& z)->Eigen::MatrixXd {
        if (m_tracker.ekf->getR().size() == 0) {
            Eigen::DiagonalMatrix<double, 4> R;
            auto r_dio = this->get_parameter("ekf.r_diagonal").as_double_array();
            R.diagonal() = Eigen::Map<Eigen::Vector4d>(r_dio.data(), 4, 1);
            return R;
        } else {
            return m_tracker.ekf->getR();
        }
        double r_xyz_factor = 4e-4,
               r_yaw = 4e-4;
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor;
        r.diagonal() << abs(x * z(0)), abs(x * z(1)), abs(x * z(2)), r_yaw;
        return r;
    };
    int p = 1;
    Eigen::Matrix<double, 9, 9> p0;
    //  xa  vxa  ya  vya  za  vza  yaw v_yaw  r
    p0 << p,  0,   0,  0,  0,   0,   0,  0,   0, // xa
          0,  p,   0,  0,  0,   0,   0,  0,   0, // vxa
          0,  0,   p,  0,  0,   0,   0,  0,   0, // ya
          0,  0,   0,  p,  0,   0,   0,  0,   0, // vya
          0,  0,   0,  0,  p,   0,   0,  0,   0, // za
          0,  0,   0,  0,  0,   p,   0,  0,   0, // vza
          0,  0,   0,  0,  0,   0,   p,  0,   0, // yaw
          0,  0,   0,  0,  0,   0,   0,  p,   0, // v_yaw
          0,  0,   0,  0,  0,   0,   0,  0,   p; // r
    m_tracker.ekf = std::make_shared<ExtendedKalmanFilter>(p0, f, h, j_f, j_h, update_Q, update_R);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target& target_msg) {
    m_center_marker.header = m_linear_v_marker.header =
        m_omega_marker.header = m_armors_marker.header = target_msg.header;
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::Point end_point;
    if (target_msg.tracking) {
        double yaw = target_msg.yaw,
               r1  = target_msg.r1,
               r2  = target_msg.r2,
               xc  = target_msg.position.x,
               yc  = target_msg.position.y,
               zc  = target_msg.position.z,
               vx  = target_msg.velocity.x,
               vy  = target_msg.velocity.y,
               vz  = target_msg.velocity.z,
               dz  = target_msg.dz;
        // car center
        m_center_marker.action = visualization_msgs::msg::Marker::ADD;
        m_center_marker.pose.position.x = xc;
        m_center_marker.pose.position.y = yc;
        m_center_marker.pose.position.z = zc;
        // liear v
        m_linear_v_marker.action = visualization_msgs::msg::Marker::ADD;
        m_linear_v_marker.points.clear();
        m_linear_v_marker.points.emplace_back(m_center_marker.pose.position);
        end_point = m_center_marker.pose.position;
        end_point.x += vx;
        end_point.y += vy;
        end_point.z += vz;
        m_linear_v_marker.points.emplace_back(end_point);
        // omega
        m_omega_marker.action = visualization_msgs::msg::Marker::ADD;
        m_omega_marker.points.clear();
        m_omega_marker.points.emplace_back(m_center_marker.pose.position);
        end_point = m_center_marker.pose.position;
        end_point.z += target_msg.v_yaw / M_PI;
        m_omega_marker.points.emplace_back(end_point);
        // armors
        m_armors_marker.action = visualization_msgs::msg::Marker::ADD;
        m_armors_marker.scale.y = m_tracker.tracked_armor.type == "SMALL"? 0.13: 0.5;
        geometry_msgs::msg::Point pa;
        bool is_pair = true;
        int num = m_tracker.getNum();
        double r = 0.0;
        for (int i = 0; i < num; ++i) {
            double tmp_yaw = yaw + i * (2*M_PI/num);
            if (num == 4) {
                r = is_pair? r1: r2;
                pa.z = zc + (is_pair? 0: dz);
                is_pair = !is_pair;
            } else {
                r = r1;
                pa.z = zc;
            }
            pa.x = xc - r*cos(tmp_yaw);
            pa.y = yc - r*sin(tmp_yaw);
            pa.z = zc;
            m_armors_marker.id = i;
            m_armors_marker.pose.position = pa;
            tf2::Quaternion q;
            q.setRPY(0, target_msg.num == 6? -0.26: 0.26, tmp_yaw);
            m_armors_marker.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(m_armors_marker);
        }
    } else {
        m_center_marker.action = m_linear_v_marker.action = m_omega_marker.action =
            m_armors_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.emplace_back(m_armors_marker);
    }
    marker_array.markers.emplace_back(m_center_marker);
    marker_array.markers.emplace_back(m_linear_v_marker);
    marker_array.markers.emplace_back(m_omega_marker); 
    m_marker_pub->publish(marker_array);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorTrackerNode)

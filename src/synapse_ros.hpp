#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include <actuator_msgs/msg/actuators.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <synapse_protobuf/joy.pb.h>

#include <synapse_msgs/msg/status.hpp>
#include <synapse_protobuf/status.pb.h>

#include <synapse_msgs/msg/road_curve_angle.hpp>
#include <synapse_protobuf/road_curve_angle.pb.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>

class UdpClient;

void udp_entry_point();

class SynapseRos : public rclcpp::Node {
public:
    SynapseRos();
    virtual ~SynapseRos();

    void tf_send(int topic, const std::string& data) const;

    void publish_status(const synapse::msgs::Status& msg);
    void publish_uptime(const synapse::msgs::Time& msg);

private:
    std::shared_ptr<TinyFrame> tf_ {};
    builtin_interfaces::msg::Time ros_clock_offset_ {};

    std_msgs::msg::Header compute_header(const synapse::msgs::Header& msg);

    // subscriptions ros -> cerebri
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Subscription<synapse_msgs::msg::RoadCurveAngle>::SharedPtr sub_road_curve_angle_;

    // subscription callbacks
    void joy_callback(const sensor_msgs::msg::Joy& msg) const;
    void road_curve_angle_callback(const synapse_msgs::msg::RoadCurveAngle& msg) const;

    // publications cerebri -> ros
    rclcpp::Publisher<synapse_msgs::msg::Status>::SharedPtr pub_status_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_uptime_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_clock_offset_;

    // callbacks
    std::shared_ptr<std::thread> udp_thread_;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__

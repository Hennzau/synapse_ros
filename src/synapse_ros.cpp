#include "synapse_ros.hpp"
#include "proto/udp_link.hpp"
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

using std::placeholders::_1;
std::shared_ptr<UDPLink> g_udp_link { NULL };

void udp_entry_point()
{
    while (rclcpp::ok()) {
        g_udp_link->run_for(std::chrono::seconds(1));
    }
}

SynapseRos::SynapseRos()
    : Node("synapse_ros")
{
    this->declare_parameter("host", "192.0.2.1");
    this->declare_parameter("port", 4242);

    std::string host = this->get_parameter("host").as_string();
    int port = this->get_parameter("port").as_int();

    // subscriptions ros -> cerebri

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "in/joy", 10, std::bind(&SynapseRos::joy_callback, this, _1));

    // publications cerebri -> ros

    pub_status_ = this->create_publisher<synapse_msgs::msg::Status>("out/status", 10);
    pub_uptime_ = this->create_publisher<builtin_interfaces::msg::Time>("out/uptime", 10);
    pub_clock_offset_ = this->create_publisher<builtin_interfaces::msg::Time>("out/clock_offset", 10);

    // create udp link
    g_udp_link = std::make_shared<UDPLink>(host, port);
    g_udp_link.get()->ros_ = this;
    tf_ = g_udp_link.get()->tf_;
    udp_thread_ = std::make_shared<std::thread>(udp_entry_point);
}

SynapseRos::~SynapseRos()
{
    // join threads
    udp_thread_->join();
}

std_msgs::msg::Header SynapseRos::compute_header(const synapse::msgs::Header& msg)
{
    std_msgs::msg::Header ros_msg;
    ros_msg.frame_id = msg.frame_id();
    if (msg.has_stamp()) {
        int64_t sec = msg.stamp().sec() + ros_clock_offset_.sec;
        int64_t nanos = msg.stamp().nanosec() + ros_clock_offset_.nanosec;
        int extra_sec = nanos / 1e9;
        nanos -= extra_sec * 1e9;
        sec += extra_sec;
        ros_msg.stamp.sec = sec;
        ros_msg.stamp.nanosec = nanos;
    }
    return ros_msg;
}

void SynapseRos::publish_status(const synapse::msgs::Status& msg)
{
    synapse_msgs::msg::Status ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header = compute_header(msg.header());
    }

    ros_msg.arming = msg.arming();
    ros_msg.fuel = msg.fuel();
    ros_msg.joy = msg.joy();
    ros_msg.mode = msg.mode();
    ros_msg.safety = msg.safety();
    ros_msg.fuel_percentage = msg.fuel_percentage();
    ros_msg.power = msg.power();
    ros_msg.status_message = msg.status_message();
    ros_msg.request_rejected = msg.request_rejected();
    ros_msg.request_seq = msg.request_seq();

    pub_status_->publish(ros_msg);
}

void SynapseRos::publish_uptime(const synapse::msgs::Time& msg)
{
    builtin_interfaces::msg::Time ros_uptime;
    rclcpp::Time now = get_clock()->now();

    int64_t uptime_nanos = msg.sec() * 1e9 + msg.nanosec();
    int64_t clock_offset_nanos = now.nanoseconds() - uptime_nanos;

    ros_uptime.sec = msg.sec();
    ros_uptime.nanosec = msg.nanosec();

    ros_clock_offset_.sec = clock_offset_nanos / 1e9;
    ros_clock_offset_.nanosec = clock_offset_nanos - ros_clock_offset_.sec * 1e9;

    pub_uptime_->publish(ros_uptime);
    pub_clock_offset_->publish(ros_clock_offset_);
}

void SynapseRos::joy_callback(const sensor_msgs::msg::Joy& msg) const
{
    synapse::msgs::Joy syn_msg;
    for (auto i = 0u; i < msg.axes.size(); ++i) {
        syn_msg.add_axes(msg.axes[i]);
    }

    for (auto i = 0u; i < msg.buttons.size(); ++i) {
        syn_msg.add_buttons(msg.buttons[i]);
    }

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Joy" << std::endl;
    }
    tf_send(SYNAPSE_JOY_TOPIC, data);
}

void SynapseRos::tf_send(int topic, const std::string& data) const
{
    TF_Msg frame;
    frame.type = topic;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    TF_Send(tf_.get(), &frame);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseRos>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et

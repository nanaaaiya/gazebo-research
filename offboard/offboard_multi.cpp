// offboard_multi_fixed.cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class OffboardMulti : public rclcpp::Node
{
public:
  OffboardMulti()
  : Node("offboard_multi")
  {
    // parameters with defaults
    this->declare_parameter<std::string>("ns", "");
    this->declare_parameter<int>("target_system", 1);
    this->declare_parameter<int>("target_component", 1);
    this->declare_parameter<double>("hover_x", 0.0);
    this->declare_parameter<double>("hover_y", 0.0);
    this->declare_parameter<double>("hover_z", -5.0); // NED (negative = up)
    this->declare_parameter<int>("period_ms", 100);
    this->declare_parameter<int>("setpoints_before_switch", 10);
    this->declare_parameter<int>("qos_depth", 10);

    this->get_parameter("ns", ns_);
    this->get_parameter("target_system", target_system_);
    this->get_parameter("target_component", target_component_);
    this->get_parameter("hover_x", hover_x_);
    this->get_parameter("hover_y", hover_y_);
    this->get_parameter("hover_z", hover_z_);
    this->get_parameter("period_ms", period_ms_);
    this->get_parameter("setpoints_before_switch", sp_before_switch_);
    this->get_parameter("qos_depth", qos_depth_);

    // normalize namespace
    if (!ns_.empty()) {
      if (ns_.front() != '/') ns_ = "/" + ns_;
      if (ns_.size() > 1 && ns_.back() == '/') ns_.pop_back();
    }

    std::string offboard_topic = ns_ + "/fmu/in/offboard_control_mode";
    std::string traj_topic    = ns_ + "/fmu/in/trajectory_setpoint";
    std::string cmd_topic     = ns_ + "/fmu/in/vehicle_command";

    RCLCPP_INFO(this->get_logger(), "Using namespace '%s' -> topics: %s , %s , %s",
                ns_.c_str(), offboard_topic.c_str(), traj_topic.c_str(), cmd_topic.c_str());

    offboard_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_topic, qos_depth_);
    traj_pub_     = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(traj_topic, qos_depth_);
    cmd_pub_      = this->create_publisher<px4_msgs::msg::VehicleCommand>(cmd_topic, qos_depth_);

    // Subscribe to collision_avoidance takeover topic
    std::string takeover_topic = "/collision/takeover_" + ns_.substr(1); // e.g. "/collision/takeover_px4_1"
    takeover_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      takeover_topic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        takeover_active_ = msg->data;
        if (takeover_active_)
          RCLCPP_WARN(this->get_logger(), "[%s] Takeover ACTIVE — pausing mission setpoints", ns_.c_str());
        else
          RCLCPP_INFO(this->get_logger(), "[%s] Takeover CLEARED — resuming mission setpoints", ns_.c_str());
      });

    counter_ = 0;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms_),
      std::bind(&OffboardMulti::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (takeover_active_) return; // ✅ collision_avoidance controls setpoints now

    // publish offboard mode
    px4_msgs::msg::OffboardControlMode off{};
    off.position = true;
    off.velocity = false;
    off.acceleration = false;
    off.attitude = false;
    off.body_rate = false;
    off.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(off);

    // publish trajectory setpoint (position control)
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.position = {static_cast<float>(hover_x_),
                   static_cast<float>(hover_y_),
                   static_cast<float>(hover_z_)};
    sp.yaw = 0.0f;
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(sp);

    // after N setpoints, request mode switch and arm
    if (counter_ == sp_before_switch_) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      rclcpp::sleep_for(100ms);
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
      RCLCPP_INFO(this->get_logger(), "Sent mode switch + arm (ns=%s)", ns_.c_str());
    }

    if (counter_ < sp_before_switch_ + 10) ++counter_;
  }

  void publish_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.command = command;
    cmd.target_system = target_system_;
    cmd.target_component = target_component_;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(cmd);
  }

  // params
  std::string ns_;
  int target_system_{1};
  int target_component_{1};
  double hover_x_{0.0};
  double hover_y_{0.0};
  double hover_z_{-5.0};
  int period_ms_{100};
  int sp_before_switch_{10};
  int qos_depth_{10};

  // publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

  // subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeover_sub_;

  // state
  bool takeover_active_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t counter_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardMulti>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

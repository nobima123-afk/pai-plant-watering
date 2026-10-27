#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace gazebo {
class PaiCanControl : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    node_ = gazebo_ros::Node::Get(sdf);
    joint_ = model->GetJoint("pai_can_tilt_joint");
    if (!joint_) throw std::runtime_error("pai_can_tilt_joint missing");
    target_ = 0.;
    last_command_ = WallSeconds();
    command_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/pai_watering/tilt_cmd", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
        if (std::isfinite(m->data)) {
          target_ = std::clamp(m->data, 0., 0.872664626); // 0..50 degrees
          last_command_ = WallSeconds();
        }
      });
    state_ = node_->create_publisher<std_msgs::msg::String>("/pai_watering/can_state", 10);
    joints_ = node_->create_publisher<sensor_msgs::msg::JointState>("/pai_watering/joint_states", 10);
    last_sim_ = model->GetWorld()->SimTime().Double();
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PaiCanControl::Update, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Measured joint/virtual flow control ready");
  }
  void Reset() override { target_ = 0.; last_sim_ = -1.; }
 private:
  static double WallSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void Update(const common::UpdateInfo &info) {
    const double now = info.simTime.Double();
    double dt = now-last_sim_;
    if (dt < 0 || last_sim_ < 0) { dt = 0.; last_pub_ = now-1.; }
    dt = std::clamp(dt, 0., .05);
    last_sim_ = now;
    const double measured = joint_->Position(0);
    const double target = WallSeconds()-last_command_ > .8 ? 0. : target_.load();
    joint_->SetParam("fmax", 0, .045);
    joint_->SetParam("vel", 0, std::clamp(5.*(target-measured), -.95, .95));
    // Explicit virtual calibration, not a fluid simulation or real flow meter.
    const double degrees = measured*180./M_PI;
    double flow = 1.2*std::max(0., degrees-20.)*std::sqrt(std::max(0., tank_)/750.);
    double amount = std::min(tank_, flow*dt);
    tank_ -= amount;
    total_ += amount;
    if (now-last_pub_ < .0333) return;
    last_pub_ = now;
    sensor_msgs::msg::JointState js;
    js.header.stamp = rclcpp::Time(info.simTime.sec, info.simTime.nsec);
    js.name = {"pai_can_tilt_joint"}; js.position = {measured};
    js.velocity = {joint_->GetVelocity(0)}; joints_->publish(js);
    std::ostringstream s;
    s << std::setprecision(10) << "{\"sim_time\":" << now
      << ",\"angle_rad\":" << measured << ",\"angle_deg\":" << degrees
      << ",\"command_angle_deg\":" << target*180./M_PI
      << ",\"flow_ml_s\":" << flow << ",\"total_ml\":" << total_
      << ",\"tank_ml\":" << tank_ << ",\"source\":\"gazebo_joint_virtual_flow\"}";
    std_msgs::msg::String m; m.data=s.str(); state_->publish(m);
  }
  gazebo_ros::Node::SharedPtr node_;
  physics::JointPtr joint_;
  event::ConnectionPtr update_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::atomic<double> target_{0.}, last_command_{0.};
  double last_sim_{-1.}, last_pub_{-1.}, tank_{750.}, total_{0.};
};
GZ_REGISTER_MODEL_PLUGIN(PaiCanControl)
}

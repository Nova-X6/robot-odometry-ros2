#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/empty.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Odometer node
//
// Subscribes to /bunker_status (bunker_msgs/msg/BunkerStatus).
// BunkerStatus already exposes linear_velocity (m/s) and angular_velocity
// (rad/s) computed by the Agile-X driver from internal motor RPM + gear ratio.
// These fields are identical to the /odom twist in the provided bags, so they
// are the correct and reliable source for integration.
//
// Pose is integrated using 2nd-order Runge-Kutta (midpoint method).
// Odometry starts at (0, 0, 0) and the reset service returns it there.
// ─────────────────────────────────────────────────────────────────────────────

class Odometer : public rclcpp::Node
{
public:
  Odometer()
  : Node("odometer",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    x_(0.0), y_(0.0), theta_(0.0),
    last_time_set_(false)
  {
    // use_sim_time is handled automatically by rclcpp when the parameter is set

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    status_sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&Odometer::status_callback, this, std::placeholders::_1));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Odometer::reset_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Odometer node started (x=0, y=0, theta=0)");
  }

private:
  // ── Integration callback ──────────────────────────────────────────────────
  void status_callback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();

    if (!last_time_set_) {
      last_time_ = current_time;
      last_time_set_ = true;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) return;   // guard stale / bad dt

    // linear_velocity and angular_velocity are provided directly by the
    // Agile-X Bunker driver (already converted from motor RPM + gear ratio).
    double v     = msg->linear_velocity;
    double omega = msg->angular_velocity;

    // 2nd-order Runge-Kutta (midpoint): better accuracy on curved paths
    double theta_mid = theta_ + 0.5 * omega * dt;
    x_     += v * dt * std::cos(theta_mid);
    y_     += v * dt * std::sin(theta_mid);
    theta_ += omega * dt;
    theta_  = std::atan2(std::sin(theta_), std::cos(theta_));  // normalise

    publish_odometry(current_time, v, omega);
    publish_tf(current_time);
  }

  // ── Reset service: set pose to (0, 0, 0) ─────────────────────────────────
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>  /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>       /*res*/)
  {
    x_     = 0.0;
    y_     = 0.0;
    theta_ = 0.0;
    last_time_set_ = false;
    RCLCPP_INFO(this->get_logger(), "Odometry reset to zero");
  }

  // ── Publish nav_msgs/Odometry ─────────────────────────────────────────────
  void publish_odometry(const rclcpp::Time & t, double v, double omega)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = t;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link2";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = omega;

    odom_pub_->publish(odom);
  }

  // ── Broadcast odom -> base_link2 TF ──────────────────────────────────────
  void publish_tf(const rclcpp::Time & t)
  {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp    = t;
    ts.header.frame_id = "odom";
    ts.child_frame_id  = "base_link2";

    ts.transform.translation.x = x_;
    ts.transform.translation.y = y_;
    ts.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(ts);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  double x_, y_, theta_;
  rclcpp::Time last_time_;
  bool last_time_set_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                   tf_broadcaster_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr  status_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                 reset_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}

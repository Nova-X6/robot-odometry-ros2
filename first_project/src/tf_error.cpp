#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "first_project/msg/tf_error_msg.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// TfError node
//
// Computes the Euclidean distance between:
//   odom -> base_link   (GPS ground truth from the bag)
//   odom -> base_link2  (our computed odometry)
//
// Publishes first_project/msg/TfErrorMsg on /tf_error_msg at 20 Hz.
// ─────────────────────────────────────────────────────────────────────────────

class TfError : public rclcpp::Node
{
public:
  TfError()
  : Node("tf_error",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    travelled_distance_(0.0),
    start_time_set_(false),
    prev_set_(false),
    prev_x_(0.0), prev_y_(0.0)
  {
    error_pub_ = this->create_publisher<first_project::msg::TfErrorMsg>(
      "/tf_error_msg", 10);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 20 Hz timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TfError::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TF error node started");
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped tf_base, tf_base2;

    // odom -> base_link (GPS ground truth published by the bag)
    try {
      tf_base = tf_buffer_->lookupTransform(
        "odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Cannot lookup odom->base_link: %s", ex.what());
      return;
    }

    // odom -> base_link2 (our computed odometry)
    try {
      tf_base2 = tf_buffer_->lookupTransform(
        "odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Cannot lookup odom->base_link2: %s", ex.what());
      return;
    }

    rclcpp::Time now = this->now();

    if (!start_time_set_) {
      start_time_    = now;
      start_time_set_ = true;
    }

    // ── Euclidean error between the two frames ────────────────────────────
    double dx = tf_base.transform.translation.x - tf_base2.transform.translation.x;
    double dy = tf_base.transform.translation.y - tf_base2.transform.translation.y;
    float  tf_err = static_cast<float>(std::sqrt(dx * dx + dy * dy));

    // ── Accumulated travelled distance (from base_link2 movement) ─────────
    double bx = tf_base2.transform.translation.x;
    double by = tf_base2.transform.translation.y;
    if (prev_set_) {
      double ddx = bx - prev_x_;
      double ddy = by - prev_y_;
      travelled_distance_ += std::sqrt(ddx * ddx + ddy * ddy);
    }
    prev_x_   = bx;
    prev_y_   = by;
    prev_set_ = true;

    // ── Time from start ───────────────────────────────────────────────────
    int32_t time_from_start =
      static_cast<int32_t>((now - start_time_).seconds());

    // ── Publish ───────────────────────────────────────────────────────────
    first_project::msg::TfErrorMsg out;
    out.header.stamp       = now;
    out.header.frame_id    = "odom";
    out.tf_error           = tf_err;
    out.time_from_start    = time_from_start;
    out.travelled_distance = static_cast<float>(travelled_distance_);

    error_pub_->publish(out);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  double travelled_distance_;
  double prev_x_, prev_y_;
  bool prev_set_;
  rclcpp::Time start_time_;
  bool start_time_set_;

  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr error_pub_;
  std::shared_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
  rclcpp::TimerBase::SharedPtr                 timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfError>());
  rclcpp::shutdown();
  return 0;
}

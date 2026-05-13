#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>
#include <vector>
#include <map>

class ColorDetector : public rclcpp::Node
{
public:
  ColorDetector()
  : Node("color_detector")
  {
    // Subscriber
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));

    // Publisher
    coords_pub_ = this->create_publisher<std_msgs::msg::String>("/color_coordinates", 10);

    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Color Detector Node Started with TF2 lookup transform");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Define color ranges (HSV)
    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_ranges = {
      {"R", {cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)}},
      {"G", {cv::Scalar(55, 200, 200), cv::Scalar(60, 255, 255)}},
      {"B", {cv::Scalar(90, 200, 200), cv::Scalar(128, 255, 255)}}
    };

    for (const auto& [color_id, range] : color_ranges) {
      cv::Mat mask;
      cv::inRange(hsv, range.first, range.second, mask);

      // Noise removal
      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

      // Find contours
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (const auto& cnt : contours) {
        if (cv::contourArea(cnt) > 1) { // Increased minimum area threshold
          cv::Rect boundRect = cv::boundingRect(cnt);
          int x = boundRect.x;
          int y = boundRect.y;
          int w = boundRect.width;
          int h = boundRect.height;
          int cx_pix = x + w / 2;
          int cy_pix = y + h / 2;

          // Draw bounding box + label
          cv::rectangle(frame, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 255), 2);
          cv::putText(frame, color_id, cv::Point(x, y - 10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

          // Convert pixel -> camera frame
          double Z = 0.1; // Assumed depth/distance
          double Y = (cx_pix - cx_) * Z / fx_ * -10.0;
          double X = (cy_pix - cy_) * Z / fy_;

          try {
            // Lookup transform camera_link -> panda_link0
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
              "panda_link0",
              "camera_link",
              tf2::TimePointZero,
              tf2::durationFromSec(1.0));

            // Transform point from camera frame to base frame
            geometry_msgs::msg::Point pt_cam;
            pt_cam.x = X;
            pt_cam.y = Y;
            pt_cam.z = Z;

            geometry_msgs::msg::Point pt_base;
            tf2::doTransform(pt_cam, pt_base, t);

            // Adjust X coordinate for blue and green (which map to Y axis shift in script)
            if (color_id == "B") {
              pt_base.y -= 0.0215;
            } else if (color_id == "G") {
              pt_base.y += 0.02;
            }

            // Publish color ID + coordinates in panda_link0 frame
            char msg_str[256];
            snprintf(msg_str, sizeof(msg_str), "%s,%.3f,%.3f,%.3f", color_id.c_str(), pt_base.x, pt_base.y, pt_base.z);
            auto msg_pub = std::make_unique<std_msgs::msg::String>();
            msg_pub->data = msg_str;
            coords_pub_->publish(std::move(msg_pub));
            RCLCPP_INFO(this->get_logger(), "%s", msg_str);

          } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
          } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected error in TF transform: %s", e.what());
          }
        }
      }
    }

    // Show image in window (Optional for C++ as OpenCV HighGUI may block if not careful, using waitKey(1))
    try {
      cv::namedWindow("Color Detection", cv::WINDOW_NORMAL);
      cv::resizeWindow("Color Detection", 640, 320);
      cv::imshow("Color Detection", frame);
      cv::waitKey(1);
    } catch (const cv::Exception& e) {
      RCLCPP_WARN(this->get_logger(), "OpenCV display error: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr coords_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera intrinsic parameters (from your SDF)
  const double fx_ = 585.0;
  const double fy_ = 588.0;
  const double cx_ = 320.0;
  const double cy_ = 160.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

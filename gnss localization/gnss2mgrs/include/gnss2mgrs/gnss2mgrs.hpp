#ifndef GNSS2MGRS__GNSS2MGRS_HPP_
#define GNSS2MGRS__GNSS2MGRS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geodesy/utm.h>  
#include "geodesy/mgrs.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class GnssToMgrs : public rclcpp::Node
{
public:
    GnssToMgrs();

private:
    void gnss_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mgrs_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string target_frame_;
    std::string gnss_frame_;
};

#endif  // GNSS2MGRS__GNSS2MGRS_HPP_

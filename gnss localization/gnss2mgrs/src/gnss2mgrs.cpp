#include <proj.h> 
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <geodesy/utm.h>
#include "geodesy/mgrs.hpp"
#include <geographic_msgs/msg/geo_point.hpp>

class GnssToMgrs : public rclcpp::Node {
public:
    GnssToMgrs()
    : Node("gnss_to_mgrs_node")
    {
        mgrs_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mgrs_pose", 10);
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gnss", 10, std::bind(&GnssToMgrs::gnss_callback, this, std::placeholders::_1));
    }

private:
void gnss_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
    
    // GNSS(WGS84)
    geographic_msgs::msg::GeoPoint geo_point;
    geo_point.latitude = msg->latitude;
    geo_point.longitude = msg->longitude;
    geo_point.altitude = msg->altitude;

    // GPS -> UTM 
    geodesy::UTMPoint utm_point(geo_point);

    // UTM -> MGRS
    geodesy::MGRSPoint mgrs_point(utm_point);

    // // 상대 easting, northing 가져오기
    // double mgrs_easting = mgrs_point.easting;  
    // double mgrs_northing = mgrs_point.northing; 

    // 소숫점 셋째 자리까지 fmod로 나머지 계산 (보정된 UTM 좌표로 변환)
    // double adjusted_easting = std::fmod(utm_point.easting, 100000.0) + mgrs_easting;
    // double adjusted_northing = std::fmod(utm_point.northing, 100000.0) + mgrs_northing;
    double adjusted_easting = std::fmod(utm_point.easting, 100000.0);
    double adjusted_northing = std::fmod(utm_point.northing, 100000.0);

    geometry_msgs::msg::PoseWithCovarianceStamped mgrs_msg;
    mgrs_msg.header.stamp = msg->header.stamp;
    mgrs_msg.header.frame_id = "map";

    mgrs_msg.pose.pose.position.x = adjusted_easting;  // 보정된 Easting
    mgrs_msg.pose.pose.position.y = adjusted_northing; // 보정된 Northing
    mgrs_msg.pose.pose.position.z = msg->altitude;   

    // Covariance 설정
    for (int i = 0; i < 36; ++i) {
        mgrs_msg.pose.covariance[i] = msg->position_covariance[i];
    }

    // 소숫점 셋째 자리까지 출력하도록 조정
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3);
    stream << "Adjusted Easting: " << adjusted_easting << ", Adjusted Northing: " << adjusted_northing;
    
    RCLCPP_INFO(this->get_logger(), "%s", stream.str().c_str());

    mgrs_pub_->publish(mgrs_msg);
}

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mgrs_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssToMgrs>());
    rclcpp::shutdown();
    return 0;
}
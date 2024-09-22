#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

class PoseFusionNode : public rclcpp::Node
{
public:
    PoseFusionNode()
        : Node("pose_fusion_node")
    {
        // Subscribers for LiDAR and GNSS pose
        lidar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose_with_covariance", 10,
            std::bind(&PoseFusionNode::lidarPoseCallback, this, std::placeholders::_1));

        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mgrs_pose", 10,
            std::bind(&PoseFusionNode::gnssPoseCallback, this, std::placeholders::_1));

        // Subscribers for EKF and GNSS TwistWithCovarianceStamped
        ekf_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/localization/pose_twist_fusion_filter/twist_with_covariance", 10,
            std::bind(&PoseFusionNode::ekfTwistCallback, this, std::placeholders::_1));

        gnss_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/gnss_twist_cov", 10,
            std::bind(&PoseFusionNode::gnssTwistCallback, this, std::placeholders::_1));
        
        // gnss twist topic needs to change
        // gnss_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        //     "/gnss_twist_cov", 10,
        //     std::bind(&PoseFusionNode::gnssTwistCallback, this, std::placeholders::_1));

        final_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/final/pose_with_covariance", 10);
        fused_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/fused_twist_with_covariance", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void lidarPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr lidar_msg)
    {
        last_lidar_msg_ = lidar_msg;

        if (last_gnss_msg_)
        {
            fusePoses();
        }
    }

    void gnssPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gnss_msg)
    {
        last_gnss_msg_ = gnss_msg;

        if (last_lidar_msg_)
        {
            fusePoses();
        }
    }

    void ekfTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr ekf_twist_msg)
    {
        last_ekf_twist_msg_ = ekf_twist_msg;

        if (last_gnss_twist_msg_)
        {
            fuseTwists();
        }
    }

    void gnssTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr gnss_twist_msg)
    {
        last_gnss_twist_msg_ = gnss_twist_msg;

        if (last_ekf_twist_msg_)
        {
            fuseTwists();
        }
    }

    void fusePoses()
    {
        Eigen::Vector3d lidar_pos(last_lidar_msg_->pose.pose.position.x, last_lidar_msg_->pose.pose.position.y, last_lidar_msg_->pose.pose.position.z);
        Eigen::Vector3d gnss_pos(last_gnss_msg_->pose.pose.position.x, last_gnss_msg_->pose.pose.position.y, last_gnss_msg_->pose.pose.position.z);

        // Covariance trace를 사용하여 불확실성 계산
        double lidar_cov_trace = calculateCovarianceTrace(last_lidar_msg_->pose.covariance);
        double gnss_cov_trace = calculateCovarianceTrace(last_gnss_msg_->pose.covariance);

        // Covariance에 기반한 동적 가중치 계산 (trace가 작을수록 신뢰도 증가)
        double total_cov_trace = lidar_cov_trace + gnss_cov_trace;
        double lidar_weight = 1.0 - (lidar_cov_trace / total_cov_trace);
        double gnss_weight = 1.0 - (gnss_cov_trace / total_cov_trace);

        geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
        fused_pose.header.stamp = this->now();
        fused_pose.header.frame_id = "base_link";

        fused_pose.pose.pose.position.x = lidar_weight * lidar_pos.x() + gnss_weight * gnss_pos.x();
        fused_pose.pose.pose.position.y = lidar_weight * lidar_pos.y() + gnss_weight * gnss_pos.y();
        fused_pose.pose.pose.position.z = lidar_weight * lidar_pos.z() + gnss_weight * gnss_pos.z();

        fused_pose.pose.pose.orientation = last_lidar_msg_->pose.pose.orientation;

        for (size_t i = 0; i < 36; ++i)
        {
            fused_pose.pose.covariance[i] = lidar_weight * last_lidar_msg_->pose.covariance[i] +
                                            gnss_weight * last_gnss_msg_->pose.covariance[i];
        }

        final_pose_pub_->publish(fused_pose);

        // Broadcast the transform
        broadcastTransform(fused_pose);
    }

    double calculateCovarianceTrace(const std::array<double, 36> &covariance)
    {
        // Covariance 행렬의 trace를 계산 (불확실성 측정)
        double trace = covariance[0] + covariance[7] + covariance[14]; // X, Y, Z의 variance만 고려
        return trace;
    }

    void fuseTwists()
    {
        geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
        fused_twist.header.stamp = this->now();
        fused_twist.header.frame_id = "base_link";  // Adjust frame_id as needed

        // Twist covariance trace를 사용하여 불확실성 측정
        double ekf_twist_cov_trace = calculateCovarianceTrace(last_ekf_twist_msg_->twist.covariance);
        double gnss_twist_cov_trace = calculateCovarianceTrace(last_gnss_twist_msg_->twist.covariance);

        // Covariance trace에 기반한 동적 가중치 계산
        double total_twist_cov_trace = ekf_twist_cov_trace + gnss_twist_cov_trace;
        double ekf_twist_weight = 1.0 - (ekf_twist_cov_trace / total_twist_cov_trace);
        double gnss_twist_weight = 1.0 - (gnss_twist_cov_trace / total_twist_cov_trace);

        // 선형 속도 및 각속도를 EKF와 GNSS 데이터를 사용해 가중 평균
        for (size_t i = 0; i < 36; ++i)
        {
            fused_twist.twist.covariance[i] = ekf_twist_weight * last_ekf_twist_msg_->twist.covariance[i] +
                                              gnss_twist_weight * last_gnss_twist_msg_->twist.covariance[i];
        }

        fused_twist.twist.twist.linear.x = ekf_twist_weight * last_ekf_twist_msg_->twist.twist.linear.x + gnss_twist_weight * last_gnss_twist_msg_->twist.twist.linear.x;
        fused_twist.twist.twist.linear.y = ekf_twist_weight * last_ekf_twist_msg_->twist.twist.linear.y + gnss_twist_weight * last_gnss_twist_msg_->twist.twist.linear.y;
        fused_twist.twist.twist.linear.z = ekf_twist_weight * last_ekf_twist_msg_->twist.twist.linear.z + gnss_twist_weight * last_gnss_twist_msg_->twist.twist.linear.z;

        fused_twist.twist.twist.angular.x = ekf_twist_weight * last_ekf_twist_msg_->twist.twist.angular.x + gnss_twist_weight * last_gnss_twist_msg_->twist.twist.angular.x;
        fused_twist.twist.twist.angular.y = ekf_twist_weight * last_ekf_twist_msg_->twist.twist.angular.y + gnss_twist_weight * last_gnss_twist_msg_->twist.twist.angular.y;
        fused_twist.twist.twist.angular.z = ekf_twist_weight * last_ekf_twist_msg_->twist.twist.angular.z + gnss_twist_weight * last_gnss_twist_msg_->twist.twist.angular.z;

        fused_twist_pub_->publish(fused_twist);
    }

    void broadcastTransform(const geometry_msgs::msg::PoseWithCovarianceStamped &fused_pose)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = fused_pose.header.stamp;
        transformStamped.header.frame_id = "map";  // Adjust this to the correct reference frame as needed
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = fused_pose.pose.pose.position.x;
        transformStamped.transform.translation.y = fused_pose.pose.pose.position.y;
        transformStamped.transform.translation.z = fused_pose.pose.pose.position.z;

        transformStamped.transform.rotation.x = fused_pose.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = fused_pose.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = fused_pose.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = fused_pose.pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr ekf_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr final_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fused_twist_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_lidar_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_gnss_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr last_ekf_twist_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr last_gnss_twist_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFusionNode>());
    rclcpp::shutdown();
    return 0;
}

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import math

class GnssTwistPublisher(Node):

    def __init__(self):
        super().__init__('gnss_twist_publisher')

        self.subscription_mgrs = self.create_subscription(
            PoseWithCovarianceStamped,
            '/mgrs_pose',
            self.listener_mgrs_callback,
            10)

        self.publisher_twist_with_cov = self.create_publisher(TwistWithCovarianceStamped, '/gnss_twist_cov', 10)

        self.previous_x = None
        self.previous_y = None
        self.previous_time = None

    def listener_mgrs_callback(self, msg):
        current_time = self.get_clock().now()

        if self.previous_x is not None and self.previous_y is not None and self.previous_time is not None:

            delta_x = msg.pose.pose.position.x - self.previous_x
            delta_y = msg.pose.pose.position.y - self.previous_y

            distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

            time_delta = (current_time - self.previous_time).nanoseconds * 1e-9  

            if time_delta > 0:
                velocity = distance / time_delta

                twist_cov_msg = TwistWithCovarianceStamped()
                twist_cov_msg.header.stamp = current_time.to_msg()
                twist_cov_msg.header.frame_id = 'mgrs'

                twist_cov_msg.twist.twist.linear.x = velocity
                twist_cov_msg.twist.twist.linear.y = 0.0
                twist_cov_msg.twist.twist.linear.z = 0.0

                twist_cov_msg.twist.twist.angular.x = 0.0
                twist_cov_msg.twist.twist.angular.y = 0.0
                twist_cov_msg.twist.twist.angular.z = 0.0

                twist_cov_msg.twist.covariance[0] = msg.pose.covariance[0] / (time_delta ** 2)  # xx -> vx vx
                twist_cov_msg.twist.covariance[7] = msg.pose.covariance[7] / (time_delta ** 2)  # yy -> vy vy
                twist_cov_msg.twist.covariance[14] = msg.pose.covariance[14] / (time_delta ** 2)  # zz -> vz vz

                twist_cov_msg.twist.covariance[21] = msg.pose.covariance[21]  # orientation covariance -> angular velocity covariance
                twist_cov_msg.twist.covariance[28] = msg.pose.covariance[28]
                twist_cov_msg.twist.covariance[35] = msg.pose.covariance[35]

                self.publisher_twist_with_cov.publish(twist_cov_msg)

        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y
        self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    gnss_twist_publisher = GnssTwistPublisher()
    rclpy.spin(gnss_twist_publisher)

    gnss_twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

class GpsAndDEg : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix curr_gps;
        std_msgs::msg::Float32 angle_deg;
        std::string gps_corr_topic;
        std::string angle_deg_topic;

        std::string name;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_corr_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_deg_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr deg_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr rad_;

    public:
        GpsAndDEg() : Node(
            "gps_and_deg",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting GPS & DEG Node");
            rclcpp::Parameter param_val = this->get_parameter("name"); 
            rclcpp::Parameter topic_prefix_param = this->get_parameter("topic_prefix");
            try{
                rclcpp::Parameter gps_corr_param = this->get_parameter("gps_corr");
                gps_corr_topic = gps_corr_param.as_string();
                rclcpp::Parameter angle_gpses_param = this->get_parameter("angle_deg");
                angle_deg_topic = angle_gpses_param.as_string();
            } catch(const std::exception& e) {
                RCLCPP_INFO(this->get_logger(), "Error: %s", e.what());
                gps_corr_topic = topic_prefix_param.as_string() + "/gps_corr";
                angle_deg_topic = topic_prefix_param.as_string() + "/angle_deg";
            }
            RCLCPP_INFO(this->get_logger(), "Subscribing to %s and %s", gps_corr_topic.c_str(), angle_deg_topic.c_str());

            gps_corr_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_corr_topic, 10, std::bind(&GpsAndDEg::gps_corr_callback, this, std::placeholders::_1));
            angle_deg_ = this->create_subscription<std_msgs::msg::Float32>(angle_deg_topic, 10, std::bind(&GpsAndDEg::angle_deg_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&GpsAndDEg::timer_callback, this));
            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix_param.as_string() + "/loc/fix", 10);
            deg_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>(topic_prefix_param.as_string() + "/loc/deg", 10);
            rad_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>(topic_prefix_param.as_string() + "/loc/rad", 10);
        }

    private:

        void gps_corr_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_corr_msg) {
            curr_gps = *gps_corr_msg;
        }

        void angle_deg_callback(const std_msgs::msg::Float32::ConstSharedPtr& angle_deg_msg) {
            angle_deg = *angle_deg_msg;
        }

        void timer_callback() {
            sensor_msgs::msg::NavSatFix curr_pose;
            curr_pose = curr_gps;
            curr_pose.header.frame_id = "gps";
            curr_pose.header.stamp = this->now();
            gps_pub_->publish(curr_pose);
            farmbot_interfaces::msg::Float32Stamped deg_msg;
            deg_msg.header = curr_pose.header;
            deg_msg.data = angle_deg.data;
            deg_->publish(deg_msg);
            farmbot_interfaces::msg::Float32Stamped rad_msg;
            rad_msg.header = curr_pose.header;
            rad_msg.data = toRadians(angle_deg.data);
            rad_->publish(rad_msg);
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<GpsAndDEg>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}

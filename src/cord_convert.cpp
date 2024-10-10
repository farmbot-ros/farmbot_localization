#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/ecef2_enu.hpp"
#include "farmbot_interfaces/srv/ecef2_gps.hpp"
#include "farmbot_interfaces/srv/enu2_ecef.hpp"
#include "farmbot_interfaces/srv/enu2_gps.hpp"
#include "farmbot_interfaces/srv/gps2_ecef.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_localization/utils/wgs_to_enu.hpp"

using namespace std::placeholders;

class CoordinateTransformer : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix geo_datum;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr geo_datum_sub_;
        nav_msgs::msg::Odometry ecef_datum;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ecef_datum_sub_;
        rclcpp::Service<farmbot_interfaces::srv::Ecef2Enu>::SharedPtr ecef2enu_service_;
        rclcpp::Service<farmbot_interfaces::srv::Ecef2Gps>::SharedPtr ecef2gps_service_;
        rclcpp::Service<farmbot_interfaces::srv::Enu2Ecef>::SharedPtr enu2ecef_service_;
        rclcpp::Service<farmbot_interfaces::srv::Enu2Gps>::SharedPtr enu2gps_service_;
        rclcpp::Service<farmbot_interfaces::srv::Gps2Ecef>::SharedPtr gps2ecef_service_;
        rclcpp::Service<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_service_;
        std::string name;
        std::string topic_prefix_param;
        bool datum_set;
    public:
        CoordinateTransformer() : Node(
            "coordinate_transformer",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            name = this->get_parameter_or<std::string>("name", "using_enu");
            topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");


            ecef2enu_service_ = this->create_service<farmbot_interfaces::srv::Ecef2Enu>(
                topic_prefix_param + "/loc/ecef2enu", std::bind(&CoordinateTransformer::ecef2enuCallback, this, _1, _2));
            
            ecef2gps_service_ = this->create_service<farmbot_interfaces::srv::Ecef2Gps>(
                topic_prefix_param + "/loc/ecef2gps", std::bind(&CoordinateTransformer::ecef2gpsCallback, this, _1, _2));
            
            enu2ecef_service_ = this->create_service<farmbot_interfaces::srv::Enu2Ecef>(
                topic_prefix_param + "/loc/enu2ecef", std::bind(&CoordinateTransformer::enu2ecefCallback, this, _1, _2));
            
            enu2gps_service_ = this->create_service<farmbot_interfaces::srv::Enu2Gps>(
                topic_prefix_param + "/loc/enu2gps", std::bind(&CoordinateTransformer::enu2gpsCallback, this, _1, _2));
            
            gps2ecef_service_ = this->create_service<farmbot_interfaces::srv::Gps2Ecef>(
                topic_prefix_param + "/loc/gps2ecef", std::bind(&CoordinateTransformer::gps2ecefCallback, this, _1, _2));
            
            gps2enu_service_ = this->create_service<farmbot_interfaces::srv::Gps2Enu>(
                topic_prefix_param + "/loc/gps2enu", std::bind(&CoordinateTransformer::gps2enuCallback, this, _1, _2));

            geo_datum_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                topic_prefix_param + "/loc/ref/geo", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                    datum_set = true;
                    geo_datum = *msg;
                });
            
            ecef_datum_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic_prefix_param + "/loc/ref", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    datum_set = true;
                    ecef_datum = *msg;
                });
            
            RCLCPP_INFO(this->get_logger(), "Coordinate Transformer Node started");
        }

    private:
        void ecef2enuCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Ecef2Enu::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Ecef2Enu::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Converting ECEF to ENU...");
            // Placeholder logic: Copy input directly for demonstration
            response->message = "Converted ECEF to ENU";
            response->enu = request->ecef; // Replace with actual conversion logic
        }

        void ecef2gpsCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Ecef2Gps::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Ecef2Gps::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Converting ECEF to GPS...");
            response->message = "Converted ECEF to GPS";
            for (const auto &pose : request->ecef) {
                sensor_msgs::msg::NavSatFix gps;
                gps.latitude = 0.0;  // Replace with actual latitude calculation
                gps.longitude = 0.0; // Replace with actual longitude calculation
                gps.altitude = 0.0;  // Replace with actual altitude calculation
                response->gps.push_back(gps);
            }
        }

        void enu2ecefCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Enu2Ecef::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Enu2Ecef::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Converting ENU to ECEF...");
            response->message = "Converted ENU to ECEF";
            response->ecef = request->enu; // Replace with actual conversion logic
        }

        void enu2gpsCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Enu2Gps::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Enu2Gps::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Converting ENU to GPS...");
            response->message = "Converted ENU to GPS";
            for (const auto &pose : request->enu) {
                sensor_msgs::msg::NavSatFix gps;
                gps.latitude = 0.0;  // Replace with actual latitude calculation
                gps.longitude = 0.0; // Replace with actual longitude calculation
                gps.altitude = 0.0;  // Replace with actual altitude calculation
                response->gps.push_back(gps);
            }
        }

        void gps2ecefCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Gps2Ecef::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Gps2Ecef::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Converting GPS to ECEF...");
            response->message = "Converted GPS to ECEF";
            for (const auto &gps : request->gps) {
                geometry_msgs::msg::Pose ecef_pose;
                ecef_pose.position.x = 0.0; // Replace with actual X calculation
                ecef_pose.position.y = 0.0; // Replace with actual Y calculation
                ecef_pose.position.z = 0.0; // Replace with actual Z calculation
                response->ecef.push_back(ecef_pose);
            }
        }

        void gps2enuCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Gps2Enu::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Gps2Enu::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Converting GPS to ENU...");
            response->message = "Converted GPS to ENU";
            for (const auto &gps : request->gps) {
                geometry_msgs::msg::Pose enu_pose;
                enu_pose.position.x = 0.0; // Replace with actual X calculation
                enu_pose.position.y = 0.0; // Replace with actual Y calculation
                enu_pose.position.z = 0.0; // Replace with actual Z calculation
                response->enu.push_back(enu_pose);
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformer>());
    rclcpp::shutdown();
    return 0;
}

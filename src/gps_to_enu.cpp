#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/msg/float64_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

const double R = 6378137.0;                // Earth's radius in meters (equatorial radius)
const double f = 1.0 / 298.257223563;      // Flattening factor
const double e2 = 2 * f - f * f;           // Square of eccentricity

// Function to convert GPS (lat, lon, alt) to ECEF coordinates
std::tuple<double, double, double> gps_to_ecef(double latitude, double longitude, double altitude) {
    // Convert latitude and longitude to radians
    double cosLat = std::cos(latitude * M_PI / 180.0);
    double sinLat = std::sin(latitude * M_PI / 180.0);
    double cosLong = std::cos(longitude * M_PI / 180.0);
    double sinLong = std::sin(longitude * M_PI / 180.0);
    // Prime vertical radius of curvature
    double N = R / std::sqrt(1.0 - e2 * sinLat * sinLat);
    // Calculate ECEF coordinates
    double x = (N + altitude) * cosLat * cosLong;
    double y = (N + altitude) * cosLat * sinLong;
    double z = (N * (1 - e2) + altitude) * sinLat;
    // Return the ECEF coordinates
    return std::make_tuple(x, y, z);
}

// Function to convert ECEF coordinates to ENU (East, North, Up) with respect to a datum
std::tuple<double, double, double> ecef_to_enu(std::tuple<double, double, double> ecef, std::tuple<double, double, double> datum) {
    double x, y, z;
    std::tie(x, y, z) = ecef;
    double latRef, longRef, altRef;
    std::tie(latRef, longRef, altRef) = datum;
    // Convert the reference latitude and longitude to radians
    double cosLatRef = std::cos(latRef * M_PI / 180.0);
    double sinLatRef = std::sin(latRef * M_PI / 180.0);
    double cosLongRef = std::cos(longRef * M_PI / 180.0);
    double sinLongRef = std::sin(longRef * M_PI / 180.0);
    // Prime vertical radius of curvature at the reference point
    double NRef = R / std::sqrt(1.0 - e2 * sinLatRef * sinLatRef);
    // Calculate the reference ECEF coordinates (datum)
    double x0 = (NRef + altRef) * cosLatRef * cosLongRef;
    double y0 = (NRef + altRef) * cosLatRef * sinLongRef;
    double z0 = (NRef * (1 - e2) + altRef) * sinLatRef;
    // Calculate the differences between the ECEF coordinates and the reference ECEF coordinates
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;
    // Calculate the ENU coordinates
    double xEast = -sinLongRef * dx + cosLongRef * dy;
    double yNorth = -cosLongRef * sinLatRef * dx - sinLatRef * sinLongRef * dy + cosLatRef * dz;
    double zUp = cosLatRef * cosLongRef * dx + cosLatRef * sinLongRef * dy + sinLatRef * dz;
    // Return the ENU coordinates
    return std::make_tuple(xEast, yNorth, zUp);
}

// Function to convert GPS (lat, lon, alt) to ENU (East, North, Up) with respect to a datum
std::tuple<double, double, double> gps_to_enu(double latitude, double longitude, double altitude, double latRef, double longRef, double altRef) {
    // Convert GPS coordinates to ECEF
    std::tuple<double, double, double> ecef = gps_to_ecef(latitude, longitude, altitude);
    // Convert reference GPS coordinates to ECEF
    std::tuple<double, double, double> datum = gps_to_ecef(latRef, longRef, altRef);
    // Return the ENU coordinates
    return ecef_to_enu(ecef, datum);
}

std::tuple<double, double, double> enu_to_ecef(std::tuple<double, double, double> enu, std::tuple<double, double, double> datum) {
    // Extract ENU and datum coordinates
    double xEast, yNorth, zUp;
    std::tie(xEast, yNorth, zUp) = enu;
    double latRef, longRef, altRef;
    std::tie(latRef, longRef, altRef) = datum;
    // Compute trigonometric values for the reference latitude and longitude
    double cosLatRef = std::cos(latRef * M_PI / 180);
    double sinLatRef = std::sin(latRef * M_PI / 180);
    double cosLongRef = std::cos(longRef * M_PI / 180);
    double sinLongRef = std::sin(longRef * M_PI / 180);
    // Compute reference ECEF coordinates for the datum
    double cRef = 1 / std::sqrt(cosLatRef * cosLatRef + (1 - f) * (1 - f) * sinLatRef * sinLatRef);
    double x0 = (R * cRef + altRef) * cosLatRef * cosLongRef;
    double y0 = (R * cRef + altRef) * cosLatRef * sinLongRef;
    double z0 = (R * cRef * (1 - e2) + altRef) * sinLatRef;
    // Reverse the ENU to ECEF transformation
    double x = x0 + (-sinLongRef * xEast) - (sinLatRef * cosLongRef * yNorth) + (cosLatRef * cosLongRef * zUp);
    double y = y0 + (cosLongRef * xEast) - (sinLatRef * sinLongRef * yNorth) + (cosLatRef * sinLongRef * zUp);
    double z = z0 + (cosLatRef * yNorth) + (sinLatRef * zUp);
    // Return the ECEF coordinates
    return std::make_tuple(x, y, z);
}

std::tuple<double, double, double> ecef_to_gps(double x, double y, double z) {
    const double e2 = f * (2 - f); // Square of eccentricity
    const double eps = 1e-12; // Convergence threshold
    double longitude = std::atan2(y, x) * 180 / M_PI;
    // Compute initial latitude and height guesses
    double p = std::sqrt(x * x + y * y);
    double latitude = std::atan2(z, p * (1 - e2));
    double N = R / std::sqrt(1 - e2 * std::sin(latitude) * std::sin(latitude)); // Prime vertical radius of curvature
    double altitude = p / std::cos(latitude) - N;
    double latOld;
    // Iterate to refine the latitude and height
    do {
        latOld = latitude;
        N = R / std::sqrt(1 - e2 * std::sin(latitude) * std::sin(latitude));
        altitude = p / std::cos(latitude) - N;
        latitude = std::atan2(z, p * (1 - e2 * N / (N + altitude)));
    } while (std::abs(latitude - latOld) > eps);
    // Return the GPS coordinates
    return std::make_tuple(latitude * 180 / M_PI, longitude, altitude);
}

std::tuple<double, double, double> enu_to_gps(double xEast, double yNorth, double zUp, double latRef, double longRef, double altRef) {
    // Convert ENU to ECEF
    std::tuple<double, double, double> ecef = enu_to_ecef(std::make_tuple(xEast, yNorth, zUp), std::make_tuple(latRef, longRef, altRef));
    // Convert ECEF to GPS
    return ecef_to_gps(std::get<0>(ecef), std::get<1>(ecef), std::get<2>(ecef));
}

class Gps2Enu : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix curr_gps;
        sensor_msgs::msg::NavSatFix datum;
        nav_msgs::msg::Odometry ecef_datum;
        bool datum_set = false;
        int gps_lock_time = 10;

        std::string name;
        std::string topic_prefix_param;
        bool autodatum;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr enu_pub_;        
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr dat_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_datum_pub_;

        rclcpp::Service<farmbot_interfaces::srv::Datum>::SharedPtr datum_gps_;
        rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr datum_set_;

        rclcpp::TimerBase::SharedPtr info_timer_;
        rclcpp::TimerBase::SharedPtr datum_timer_;

    public:
        Gps2Enu() : Node(
            "gps_to_enu",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting GPS2ENU Node");

            try {
                name = this->get_parameter("name").as_string(); 
                topic_prefix_param = this->get_parameter("topic_prefix").as_string();
            } catch (...) {
                name = "gps_to_enu";
                topic_prefix_param = "/fb";
            }

            try {
                autodatum = this->get_parameter("autodatum").as_bool();
            } catch (...) {
                autodatum = false;
                RCLCPP_WARN(this->get_logger(), "Autodatum parameter not found, using default value of false");
            }

            fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(topic_prefix_param + "/loc/fix", 10, std::bind(&Gps2Enu::callback, this, std::placeholders::_1));
            
            ecef_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param + "/loc/ecef", 10);
            enu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param + "/loc/enu", 10);
            ecef_datum_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param + "/loc/ref/ecef", 10);
            dat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix_param + "/loc/ref", 10);

            datum_gps_ = this->create_service<farmbot_interfaces::srv::Datum>(topic_prefix_param + "/datum", std::bind(&Gps2Enu::datum_gps_callback, this, std::placeholders::_1, std::placeholders::_2));
            datum_set_ = this->create_service<farmbot_interfaces::srv::Trigger>(topic_prefix_param + "/datum/set", std::bind(&Gps2Enu::datum_set_callback, this, std::placeholders::_1, std::placeholders::_2));

            info_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&Gps2Enu::info_timer_callback, this));
            datum_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Gps2Enu::datum_timer_callback, this));
        }

    private:

        void info_timer_callback() {
            if (!datum_set) {
                RCLCPP_WARN(this->get_logger(), "NO DATUM SET, PLEASE SET DATUM FIRST!");
            }
        }

        void datum_timer_callback() {
            gps_lock_time--;
            if (gps_lock_time <= 0) {
                datum_timer_.reset();
            }
        }

        void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix) {
            curr_gps = *fix;
            if (!datum_set) {
                if (gps_lock_time <= 0 && autodatum) {
                    set_datum(fix);
                } else {
                    return;
                }
            }

            ecef_datum.header = fix->header;
            ecef_datum_pub_->publish(ecef_datum);
            datum.header = fix->header;
            dat_pub_->publish(datum);

            double lat = fix->latitude, lon = fix->longitude, alt = fix->altitude;
            nav_msgs::msg::Odometry ecef_msg;
            ecef_msg.header = fix->header;
            double ecef_x, ecef_y, ecef_z;
            std::tie(ecef_x, ecef_y, ecef_z) = gps_to_ecef(lat, lon, alt);
            ecef_msg.pose.pose.position.x = ecef_x;
            ecef_msg.pose.pose.position.y = ecef_y;
            ecef_msg.pose.pose.position.z = ecef_z;

            nav_msgs::msg::Odometry enu_msg;
            enu_msg.header = fix->header;
            double d_lat = datum.latitude, d_lon = datum.longitude, d_alt = datum.altitude;
            double enu_x, enu_y, enu_z;
            std::tie(enu_x, enu_y, enu_z) = ecef_to_enu(std::make_tuple(ecef_x, ecef_y, ecef_z), std::make_tuple(d_lat, d_lon, d_alt));
            enu_msg.pose.pose.position.x = enu_x;
            enu_msg.pose.pose.position.y = enu_y;
            enu_msg.pose.pose.position.z = enu_z;

            ecef_pub_->publish(ecef_msg);
            enu_pub_->publish(enu_msg);
        }

        void datum_gps_callback(const std::shared_ptr<farmbot_interfaces::srv::Datum::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Datum::Response> _response) {
            RCLCPP_INFO(this->get_logger(), "Datum Set Request -> SPECIFIED");
            set_datum(std::make_shared<sensor_msgs::msg::NavSatFix>(_request->gps));
            _response->message = "DATUM SET TO: " + std::to_string(datum.latitude) + ", " + std::to_string(datum.longitude) + ", " + std::to_string(datum.altitude);
            return;
        }

        void datum_set_callback(const std::shared_ptr<farmbot_interfaces::srv::Trigger::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Trigger::Response> _response) {
            RCLCPP_INFO(this->get_logger(), "Datum Set Request -> CURRENT");
            auto req = _request;  // to avoid unused parameter warning
            auto res = _response; // to avoid unused parameter warning
            set_datum(std::make_shared<sensor_msgs::msg::NavSatFix>(curr_gps));
            return;
        }

        void set_datum(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& ref) {
            datum = *ref;
            datum.header = ref->header;
            ecef_datum.header = ref->header;
            double lat = ref->latitude;
            double lon = ref->longitude;
            double alt = ref->altitude;
            double x, y, z;
            std::tie(x, y, z) = gps_to_ecef(lat, lon, alt);
            ecef_datum.pose.pose.position.x = x;
            ecef_datum.pose.pose.position.y = y;
            ecef_datum.pose.pose.position.z = z;
            datum_set = true;
        }



};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<Gps2Enu>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}

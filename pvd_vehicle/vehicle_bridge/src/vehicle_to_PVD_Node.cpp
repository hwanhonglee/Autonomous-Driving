#include "rclcpp/rclcpp.hpp"

#include "pvd_msgs/msg/pv_ddata.hpp"

#include "novatel_gps_msgs/msg/novatel_position.hpp"
#include "novatel_gps_msgs/msg/novatel_heading2.hpp"

#include <string>
#include <vector>

#include "Model.hpp"
#include "vehicle_type.hpp"

#include <std_msgs/msg/u_int8.hpp>
#include "std_msgs/msg/bool.hpp"
#include <std_msgs//msg/float64.hpp>

#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
// #include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

typedef struct PVDdata{
    //probe ID
    std::string vehicle_name;
    std::string vehicle_id;

    //vehicle type
    uint32_t vehicle_type;

    //vehicle status
    int steering_wheel;
    float acceleration;
    
    uint8_t velocity;
    uint8_t gear_status;

    uint8_t lat_approved;   //Only kona, Ioniq phev     eps_enable
    uint8_t long_approved;  //Only kona, Ioniq phev     acc_enable
    uint8_t vehicle_mode;
    uint32_t vehicle_mile;

    //GNSS data
    double latitude;
    double longitude;
    float elevation;
    float heading;

    //sensor status
    uint8_t gnss_status;
    std::vector<uint8_t> camera_status;
    std::vector<uint8_t> lidar_status;
    std::vector<uint8_t> radar_status;

    PVDdata() :
        vehicle_name(""), vehicle_id(""), vehicle_type(0), steering_wheel(0), 
        acceleration(0), velocity(0), latitude(0.0),  longitude(0.0), elevation(0.0),
        heading(0.0), lat_approved(0), long_approved(0), vehicle_mode(0), vehicle_mile(0), gnss_status(0)
        {   
            // camera_status.emplace_back(0xff);
            lidar_status.emplace_back(0xff);
            // radar_status.emplace_back(0xff);
        }
} tPVDdata;
    
tPVDdata temporaryPVDdata;

class VehicleDataToPVD : public rclcpp::Node
{
public:
    VehicleDataToPVD()
        : Node("VehicleDataToPVD")
    {
        // 1. Novatel GNSS subscriber
        sub_gps_status = create_subscription<novatel_gps_msgs::msg::NovatelPosition>(
            "/sensing/gnss/bestpos", 1, std::bind(&VehicleDataToPVD::callback_gps_status, this, std::placeholders::_1));

        // 2. Novatel Heading subscriber
        sub_heading_status = create_subscription<novatel_gps_msgs::msg::NovatelHeading2>(
            "/sensing/gnss/heading2", 1, std::bind(&VehicleDataToPVD::callback_heading_status, this, std::placeholders::_1));

        // 3. Velocity subscriber
        sub_velocity_status = create_subscription<std_msgs::msg::Float64>(
            "/current_velocity_status", 1, std::bind(&VehicleDataToPVD::callback_velocity_status, this, std::placeholders::_1));

        // 4. Steering_tire_angle status subscriber (rad to deg, tire to steering handle)
        sub_steering_status = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 1, std::bind(&VehicleDataToPVD::callback_steering_status, this, std::placeholders::_1));

        // 5. Gear data  subscriber
        sub_gear_status = create_subscription<std_msgs::msg::UInt8>(
            "/current_gear_state", 1, std::bind(&VehicleDataToPVD::callback_gear_status, this, std::placeholders::_1));
        
        // 6. EPS status subscriber
        sub_eps_status = create_subscription<std_msgs::msg::Bool>(
            "/current_EPS_En_status", 1, std::bind(&VehicleDataToPVD::callback_eps_status, this, std::placeholders::_1));
        
        // 7. ACC status subscriber
        sub_acc_status = create_subscription<std_msgs::msg::Bool>(
            "/current_ACC_En_status", 1, std::bind(&VehicleDataToPVD::callback_acc_status, this, std::placeholders::_1));

        //  8. Cruise mode status subscriber
        sub_cruise_mode_status = create_subscription<std_msgs::msg::Bool>(
            "/current_cruise_mode_status", 1, std::bind(&VehicleDataToPVD::callback_cruise_mode_status, this, std::placeholders::_1));

        // PVD msg publisher
        pub_pvd_data = create_publisher<pvd_msgs::msg::PVDdata>("PVD_data", 1);

        // Initialize vehicle parameters (you can load them from a config file)
        vehicle_name = "IONIQ_HEV";
        vehicle_id = "03_9608";
        vehicle_type = 4; // Assuming CAR as the vehicle type

        // Initialize PVDdata
        temporaryPVDdata.vehicle_name = vehicle_name;
        temporaryPVDdata.vehicle_id = vehicle_id;
        temporaryPVDdata.vehicle_type = vehicle_type;

        // Set some default values for other fields
        temporaryPVDdata.gnss_status = 1;
        // temporaryPVDdata.camera_status = {1, 1, 1, 1};

        prev_time = this->now();
        timer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&VehicleDataToPVD::timer_callback, this));
    }

private:
    // 1. Callback gps_status
    void callback_gps_status(const novatel_gps_msgs::msg::NovatelPosition::SharedPtr gps_msg)
    {
        temporaryPVDdata.latitude = gps_msg->lat;
        temporaryPVDdata.longitude = gps_msg->lon;
        temporaryPVDdata.elevation = gps_msg->height;
    }

    // 2. Callback heading_status
    void callback_heading_status(const novatel_gps_msgs::msg::NovatelHeading2::SharedPtr heading_msg)
    {
        temporaryPVDdata.heading = heading_msg->heading;
    }

    // 3. Callback velocity 
    void callback_velocity_status(const std_msgs::msg::Float64::SharedPtr msg)
    {
        temporaryPVDdata.velocity = msg->data;
    }

    // 4. Callback steering angle status
    void callback_steering_status(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
    {
        temporaryPVDdata.steering_wheel = pvd_vehicle::helper::Vehicle::Model::rad2deg(msg->steering_tire_angle) * 16.16667;
    }

    // 5. Callback gear status
    void callback_gear_status(const std_msgs::msg::UInt8::SharedPtr msg) 
    {
        temporaryPVDdata.gear_status = msg->data;
    }

    // 6. Callback eps_status
    void callback_eps_status(const std_msgs::msg::Bool::SharedPtr msg)
    {
        temporaryPVDdata.lat_approved = msg->data;
        
    }

    // 7. Callback acc_status
    void callback_acc_status(const std_msgs::msg::Bool::SharedPtr msg)
    {
       temporaryPVDdata.long_approved = msg->data;
    }

    // 8. Callback Vehicle Mode
    void callback_cruise_mode_status(const std_msgs::msg::Bool::SharedPtr msg)
    {
        temporaryPVDdata.vehicle_mode = msg->data;
    }
    
    void temporary_data_to_pvd_msg(const tPVDdata &tempData, pvd_msgs::msg::PVDdata &msg)
    {
        msg.vehicle_name = tempData.vehicle_name;
        msg.vehicle_id = tempData.vehicle_id;
        msg.vehicle_type = tempData.vehicle_type;

        msg.steering_wheel = tempData.steering_wheel;
        msg.acceleration = tempData.acceleration;
        msg.velocity = tempData.velocity;
        msg.gear_status = tempData.gear_status;

        msg.latitude = tempData.latitude;
        msg.longitude = tempData.longitude;
        msg.elevation = tempData.elevation;

        msg.heading = tempData.heading;

        msg.vehicle_mile = tempData.vehicle_mile;

        msg.lat_approved = tempData.lat_approved;   // Only KONA, IONIQ PHEV    // steering Enable
        msg.long_approved = tempData.long_approved; // Only KONA, IONIQ PHEV    // acc Enable

        msg.vehicle_mode = tempData.vehicle_mode;

        msg.gnss_status = tempData.gnss_status;
        // msg.camera_status = tempData.camera_status;
        msg.lidar_status = tempData.lidar_status;
        // msg.radar_status = tempData.radar_status;
    }

    float dist = 0.0;
    rclcpp::Time prev_time;
    rclcpp::TimerBase::SharedPtr timer;

    void timer_callback()
    {
        // 9. Calculate vehicle mileage
        rclcpp::Time current_time = this->now();
        float velocity_kph = temporaryPVDdata.velocity;
        float velocity_mps = pvd_vehicle::helper::Vehicle::Model::kph2mps(velocity_kph);

        // Integrate distance
        float dt = (current_time - prev_time).seconds();
        prev_time = current_time;
        dist += dt * velocity_mps;

        temporaryPVDdata.vehicle_mile = static_cast<uint32_t>(dist);

        // Publish PVDdata
        pvd_msgs::msg::PVDdata pvd_data_msg;
        temporary_data_to_pvd_msg(temporaryPVDdata, pvd_data_msg);
        pub_pvd_data->publish(pvd_data_msg);
    }

    std::string vehicle_name;
    std::string vehicle_id;
    uint32_t vehicle_type;
    
    rclcpp::Subscription<novatel_gps_msgs::msg::NovatelPosition>::SharedPtr sub_gps_status;  // 1 
    rclcpp::Subscription<novatel_gps_msgs::msg::NovatelHeading2>::SharedPtr sub_heading_status; // 2 
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_velocity_status; // 3
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_status; // 4
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_status; // 5
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_eps_status; // 6
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_acc_status; // 7
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cruise_mode_status; // 8 
 
    rclcpp::Publisher<pvd_msgs::msg::PVDdata>::SharedPtr pub_pvd_data;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleDataToPVD>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

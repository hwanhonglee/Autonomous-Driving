#include "vehicle_bridge/vehicle_tyep.hpp"
#include "vehicle_bridge/Model.hpp"


#include "rclcpp/rclcpp.hpp"
// #include "pvd_msgs/msg/PVD.hpp"
#include "pvd_msgs/msg/pv_ddata.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <string>

#include <std_msgs/msg/u_int8.hpp>
#include "std_msgs/msg/bool.hpp"
#include <std_msgs//msg/float64.hpp>

#include "novatel_gps_msgs/msg/novatel_position.hpp"
#include "novatel_gps_msgs/msg/novatel_heading2.hpp"

#include <autoware_vehicle_msgs/msg/steering_report.hpp>

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

    uint8_t lat_approved;   //Only kona, Ioniq phev
    uint8_t long_approved;  //Only kona, Ioniq phev
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
            camera_status.emplace_back(0xff);
            lidar_status.emplace_back(0xff);
            radar_status.emplace_back(0xff);
        }
} tPVDdata;

using namespace std::chrono_literals;
using std::placeholders::_1;
using PVD = pvd_msgs::msg::PVDdata;

class VehicleDataToPVD : public rclcpp::Node
{
public:
    VehicleDataToPVD() : Node("VehicleDataToPVD")
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
        sub_steering_status = create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
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


        pvd_pub_ = this->create_publisher<PVD>("pvd_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VehicleDataToPVD::timer_callback, this));
        timer2_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VehicleDataToPVD::calculate_vehicle_mile, this));
        

        // vehicle_sub_ = this->create_subscription<VehicleStatus>("topic_name", 1,
        //     std::bind(&VehicleDataToPVD::callback_vehicle_data, this, _1));
        // gps_sub_ = this->create_subscription<Gps>("topic_name", 1,
        //     std::bind(&VehicleDataToPVD::callback_gps_status, this, _1));

        vehicle_name = declare_parameter("vehicleName", "IONIQ_PHEV");
        vehicle_id = declare_parameter("vehicleID", "03_9608");
        
        temporaryPVDdata.lidar_status.clear();
        temporaryPVDdata.lidar_status.emplace_back(1);
        
        temporaryPVDdata.gnss_status = 1;
        
        // Initialize PVDdata
        temporaryPVDdata.vehicle_name = vehicle_name;
        temporaryPVDdata.vehicle_id = vehicle_id;
        temporaryPVDdata.vehicle_type = CAR;
    }
    rclcpp::Publisher<PVD>::SharedPtr pvd_pub_;
    
    //Subscriber for vehicle data, gps, etc
    /* 
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_sub_;
    rclcpp::Subscription<Gps>::SharedPtr gps_sub_;
    rclcpp::Subscription<Heading>::SharedPtr heading_sub_;
    */
    private:
        tPVDdata temporaryPVDdata;
        PVD pvd_msg;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        float dist=0.0;

        std::string vehicle_name{"IONIQ_PHEV"};
        std::string vehicle_id{"03_9608"};

        uint8_t gnss_status;
        std::vector<uint8_t> camera_status;
        std::vector<uint8_t> lidar_status;
        std::vector<uint8_t> radar_status;

        rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};

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
        void callback_steering_status(const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
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

        void timer_callback() {

            pvd_msg.vehicle_name = temporaryPVDdata.vehicle_name;
            pvd_msg.vehicle_id = temporaryPVDdata.vehicle_id;
            pvd_msg.vehicle_type = temporaryPVDdata.vehicle_type;

            pvd_msg.steering_wheel = temporaryPVDdata.steering_wheel;
            pvd_msg.acceleration = temporaryPVDdata.acceleration;
            pvd_msg.velocity = temporaryPVDdata.velocity;
            pvd_msg.gear_status = temporaryPVDdata.gear_status;

            pvd_msg.latitude = temporaryPVDdata.latitude;
            pvd_msg.longitude = temporaryPVDdata.longitude;
            pvd_msg.elevation = temporaryPVDdata.elevation;

            pvd_msg.heading = temporaryPVDdata.heading;

            pvd_msg.vehicle_mile = temporaryPVDdata.vehicle_mile;

            pvd_msg.lat_approved = temporaryPVDdata.lat_approved;   // Only KONA,IONIQ PHEV 
            pvd_msg.long_approved = temporaryPVDdata.long_approved; // Only KONA,IONIQ PHEV

            pvd_msg.vehicle_mode = temporaryPVDdata.vehicle_mode;

            pvd_msg.gnss_status = temporaryPVDdata.gnss_status;
            pvd_msg.camera_status = temporaryPVDdata.camera_status;
            pvd_msg.lidar_status = temporaryPVDdata.lidar_status;
            pvd_msg.radar_status = temporaryPVDdata.radar_status;
            pvd_pub_->publish(pvd_msg);
        }

        //calculate vehicle mileage
        uint32_t calculate_vehicle_mile()
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
        }

        
        rclcpp::Subscription<novatel_gps_msgs::msg::NovatelPosition>::SharedPtr sub_gps_status;  // 1 
        rclcpp::Subscription<novatel_gps_msgs::msg::NovatelHeading2>::SharedPtr sub_heading_status; // 2 
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_velocity_status; // 3
        rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_status; // 4
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_status; // 5
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_eps_status; // 6
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_acc_status; // 7
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cruise_mode_status; // 8 
    
        rclcpp::Publisher<pvd_msgs::msg::PVDdata>::SharedPtr pub_pvd_data;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pvd_node = std::make_shared<VehicleDataToPVD>();

  rclcpp::spin(pvd_node);
  rclcpp::shutdown();
  return 0;
}

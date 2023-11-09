#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <masnrd_msgs/msg/reached_waypoint.hpp>
#include <masnrd_msgs/msg/goto_waypoint.hpp>
#include <masnrd_msgs/msg/detected.hpp>

using namespace std::chrono_literals;

using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::OffboardControlMode; // Heartbeat

using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::TrajectorySetpoint;

using masnrd_msgs::msg::ReachedWaypoint;
using masnrd_msgs::msg::GotoWaypoint;
using masnrd_msgs::msg::Detected;

class Point
{
    public:
    float x, y;
    float ref_lat, ref_lon;
    Point() { x = 0; y = 0; ref_lat = 0; ref_lon = 0;}
    Point(const float initX, const float initY, const float initRefLat, const float initRefLon) {
        x = initX;
        y = initY;
        ref_lat = initRefLat;
        ref_lon = initRefLon;
    }
};


class CentralNode : public rclcpp::Node 
{
    private:
    uint64_t counter_; // current cycle

    // Position Tracking
    Point pos; // current X/Y
    float alt; // current altitude

    TrajectorySetpoint tgt; // target point
    bool operating; // if the drone is in operation
    
    // Closeness to target (in metres) to which the drone considers itself as having "reached" the target
    const float tolerance = 0.25;

    // Target Altitude, in metres
    const float targetAlt = 5;

    // Velocity Targets (m/s)
    const float fullSpeed = 5.0;
    const float scanSpeed = 2.8;

    // Publisher Interval
    const std::chrono::milliseconds pubIntv = 100ms;

    rclcpp::TimerBase::SharedPtr timer_;

    // Communication with PX4
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehcom_pub;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr ocm_pub;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr localpos_sub;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr tsp_pub;

    // Communication with Pathfinder
    rclcpp::Publisher<ReachedWaypoint>::SharedPtr reachedwp_pub;
    rclcpp::Publisher<Detected>::SharedPtr detected_pub;
    rclcpp::Subscription<GotoWaypoint>::SharedPtr gotowp_sub;

    void set_target(const Point pt);
    void process_pos();
    void pub_reached();

    void pub_heartbeat();
    void pub_target();
    void pub_vehcom(uint16_t cmd, float p1 = 0.0, float p2 = 0.0, float p3 = 0.0);

    public:
    CentralNode();
    void arm();
    void disarm();
    void log(const char* msg);
};

CentralNode::CentralNode() : Node("central_node")
{
    // Initial Setup
    counter_ = 0;
    pos = Point(0, 0, 0, 0);
    alt = 0;
    operating = false;
    tgt = TrajectorySetpoint{}; // Setpoint message to be sent
    tgt.position = {0.0, 0.0, -targetAlt};

    // Setup Subscriber
    // 1. QOS Setup (ROS2-PX4 interfacing issues)
    rmw_qos_profile_t qos_sub_prof = rmw_qos_profile_sensor_data;
    auto qos_sub = rclcpp::QoS(rclcpp::QoSInitialization(qos_sub_prof.history, 5), qos_sub_prof);

    // 2. Create subscriptions
    // 2.1. Subscriber to GPS Updates
    localpos_sub = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_sub, [this](const VehicleLocalPosition::UniquePtr msg) {
        // Update local position every time PX4 publishes to this
        pos.x = msg->x;
        pos.y = msg->y;
        pos.ref_lat = msg->ref_lat;
        pos.ref_lon = msg->ref_lon;
        alt = -msg->z;

        // std::cout << "Position: " << pos.x << "," << pos.y << std::endl;

        // Process the position to see if we've reached it.
        if (abs(pos.x - tgt.position[0]) > tolerance || abs(pos.y - tgt.position[1]) > tolerance)
            return;
        if (!operating) return;

        // If we've reached the target, report it.
        pub_reached();
        log("Reached target.");
        operating = false; 
    });
    // 2.2. Subscriber to GOTO position updates from pathfinder
    gotowp_sub = this->create_subscription<GotoWaypoint>("/pathfinder/out/goto_waypoint", 10, [this](const GotoWaypoint::UniquePtr msg) {
        // Update target
        //TODO: take timestamp into account?
        //TODO: validate no change in reference point?
        log("Received instruction");
        set_target(Point(msg->x, msg->y, pos.ref_lat, pos.ref_lon));
        std::cout << "Goto: " << tgt.position[0] << "," << tgt.position[1] << std::endl;
    });

    // Setup Publishers
    ocm_pub = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    tsp_pub = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehcom_pub = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    reachedwp_pub = this->create_publisher<ReachedWaypoint>("/pathfinder/in/reached_waypoint", 10);
    detected_pub = this->create_publisher<Detected>("/pathfinder/in/detected", 10);

    // Setup timed publishing
    auto timer_callback = [this]() -> void {
        counter_++;
        if (counter_ <= 10) {
            // Continuous sending for the first 1 second
            this->arm();
            this->pub_vehcom(
                VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1, 6 // Set to OFFBOARD mode
            );

            // TODO: change to only change speed to scanSpeed when actually initiating scan
            this->pub_vehcom(
                VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED,
                scanSpeed, -1
            );
        }
        pub_heartbeat();

        if (operating)
            pub_target();
    };
    timer_ = this->create_wall_timer(pubIntv, timer_callback);
}

void CentralNode::process_pos() {
    if (abs(pos.x - tgt.position[0]) > tolerance)
        return;
    if (abs(pos.y - tgt.position[1]) > tolerance)
        return;
    if (!operating)
        return;
    
    // If we've reached the target, report in.
    // log("Reached target.");
    std::cout << "Position is (" << pos.x << "," << pos.y << "), reached (" << tgt.position[0] << "," << tgt.position[1] << ")" << std::endl;
    pub_reached();
    operating = false;
}

void CentralNode::set_target(const Point pt) {
    operating = false;
    tgt.position = {pt.x, pt.y, -targetAlt};
    operating = true;
}

void CentralNode::log(const char* msg) {
    RCLCPP_INFO(this->get_logger(), msg);
}

void CentralNode::arm() {
    pub_vehcom(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    log("Arm command sent.");
}

void CentralNode::disarm() {
    pub_vehcom(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    log("Disarm command sent.");
}

void CentralNode::pub_reached() {
    ReachedWaypoint rwp{};
    std::cout << "Reached: " << tgt.position[0] << "," << tgt.position[1] << std::endl;
    rwp.x = tgt.position[0]; rwp.y = tgt.position[1];
    rwp.ref_lat = pos.ref_lat; rwp.ref_lon = pos.ref_lon;
    rwp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    reachedwp_pub->publish(rwp);
}

void CentralNode::pub_heartbeat() {
    OffboardControlMode hb{};
    hb.position = true;
    hb.velocity = false; hb.acceleration = false; hb.attitude = false; hb.body_rate = false;
    hb.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    ocm_pub->publish(hb);
}

void CentralNode::pub_vehcom(uint16_t cmd, float p1, float p2, float p3) {
    VehicleCommand msg{};
    msg.param1 = p1; msg.param2 = p2; msg.param3 = p3;
    msg.command = cmd;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehcom_pub->publish(msg);
}

void CentralNode::pub_target() {
    // Continuously publish to this to ensure drone knows where to go
    tgt.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    tsp_pub->publish(tgt);
}

int main(int argc, char** argv)
{
    std::cout << "Starting CentralNode." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}
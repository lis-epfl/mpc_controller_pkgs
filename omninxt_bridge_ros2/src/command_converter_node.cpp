/**
 * @file command_converter_node.cpp
 * @brief Converts MPC controller output to swarmnxt_msgs/ControllerCommand
 * Handles both VehicleRatesSetpoint and ActuatorMotors messages
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <swarmnxt_msgs/msg/controller_command.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

class CommandConverter : public rclcpp::Node {
public:
    CommandConverter() : Node("command_converter") {
        // Declare parameters
        this->declare_parameter<std::string>("rates_input_topic", "/fmu/in/vehicle_rates_setpoint");
        this->declare_parameter<std::string>("motors_input_topic", "/fmu/in/actuator_motors");
        this->declare_parameter<std::string>("output_topic", "/controller/cmd");
        
        // Get parameters
        std::string rates_input = this->get_parameter("rates_input_topic").as_string();
        std::string motors_input = this->get_parameter("motors_input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // Create subscribers
        rates_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
            rates_input, 10,
            std::bind(&CommandConverter::ratesCallback, this, std::placeholders::_1));
        
        motors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
            motors_input, 10,
            std::bind(&CommandConverter::motorsCallback, this, std::placeholders::_1));
        
        // Create publisher
        pub_ = this->create_publisher<swarmnxt_msgs::msg::ControllerCommand>(
            output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "Command Converter initialized");
        RCLCPP_INFO(this->get_logger(), "  Rates input topic: %s", rates_input.c_str());
        RCLCPP_INFO(this->get_logger(), "  Motors input topic: %s", motors_input.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    }

private:
    void ratesCallback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg) {
        // Create output message
        auto out_msg = swarmnxt_msgs::msg::ControllerCommand();
        
        // Set header
        out_msg.header.stamp = this->get_clock()->now();
        out_msg.header.frame_id = "base_link";
        
        // Set command type to RATE_SETPOINT
        out_msg.command_type_mask = swarmnxt_msgs::msg::ControllerCommand::RATE_SETPOINT;
        
        // Fill in rate command (AttitudeTarget)
        out_msg.rate_cmd.header = out_msg.header;
        
        // Type mask for attitude rate control
        // bit 7: ignore attitude (use rates)
        out_msg.rate_cmd.type_mask = 
            mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
        
        // Body rates from PX4 message
        // PX4 uses: roll, pitch, yaw rates
        out_msg.rate_cmd.body_rate.x = msg->roll;   // roll rate
        out_msg.rate_cmd.body_rate.y = msg->pitch;  // pitch rate
        out_msg.rate_cmd.body_rate.z = msg->yaw;    // yaw rate
        
        // Thrust - PX4 uses thrust_body[2] (normalized [-1, 1])
        // Convert to [0, 1] range expected by MAVROS
        double thrust_normalized = -msg->thrust_body[2];  // Negative because PX4 NED has Z down
        thrust_normalized = std::clamp(thrust_normalized, 0.0, 1.0);
        out_msg.rate_cmd.thrust = thrust_normalized;
        
        // Orientation is not used for rate control but set identity quaternion
        out_msg.rate_cmd.orientation.w = 1.0;
        out_msg.rate_cmd.orientation.x = 0.0;
        out_msg.rate_cmd.orientation.y = 0.0;
        out_msg.rate_cmd.orientation.z = 0.0;
        
        // Publish converted message
        pub_->publish(out_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Converted rate command: roll=%.3f, pitch=%.3f, yaw=%.3f, thrust=%.3f",
                    out_msg.rate_cmd.body_rate.x,
                    out_msg.rate_cmd.body_rate.y,
                    out_msg.rate_cmd.body_rate.z,
                    out_msg.rate_cmd.thrust);
    }
    
    void motorsCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
        // Create output message
        auto out_msg = swarmnxt_msgs::msg::ControllerCommand();
        
        // Set header
        out_msg.header.stamp = this->get_clock()->now();
        out_msg.header.frame_id = "base_link";
        
        // Set command type to MOTOR_SETPOINT
        out_msg.command_type_mask = swarmnxt_msgs::msg::ControllerCommand::MOTOR_SETPOINT;
        
        // Convert motor commands
        // PX4 ActuatorMotors has control[12] array with normalized values [0, 1]
        // or NaN for unused motors
        out_msg.motor_cmd.clear();
        
        for (size_t i = 0; i < 12; ++i) {
            float motor_value = msg->control[i];
            
            // Skip NaN values (unused motors)
            if (std::isnan(motor_value)) {
                break;  // Assume all motors after first NaN are also unused
            }
            
            // Keep normalized [0, 1] values, just clamp to valid range
            motor_value = std::clamp(motor_value, 0.0f, 1.0f);
            out_msg.motor_cmd.push_back(motor_value);
        }
        
        // Publish converted message
        pub_->publish(out_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Converted motor command: %zu motors",
                    out_msg.motor_cmd.size());
    }
    
    rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_sub_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_sub_;
    rclcpp::Publisher<swarmnxt_msgs::msg::ControllerCommand>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandConverter>());
    rclcpp::shutdown();
    return 0;
}

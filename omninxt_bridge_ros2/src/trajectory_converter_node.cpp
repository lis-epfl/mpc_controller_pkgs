/**
 * @file trajectory_converter_node.cpp
 * @brief Converts multi_agent_planner_msgs/Trajectory to mpc_controller_ros2_msgs/Trajectory
 */

#include <rclcpp/rclcpp.hpp>
#include <multi_agent_planner_msgs/msg/trajectory.hpp>
#include <mpc_controller_ros2_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

class TrajectoryConverter : public rclcpp::Node {
public:
    TrajectoryConverter() : Node("trajectory_converter") {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/planner/trajectory");
        this->declare_parameter<std::string>("output_topic", "/mpc/trajectory");
        
        // Get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // Create subscriber and publisher
        sub_ = this->create_subscription<multi_agent_planner_msgs::msg::Trajectory>(
            input_topic, 10,
            std::bind(&TrajectoryConverter::trajectoryCallback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<mpc_controller_ros2_msgs::msg::Trajectory>(
            output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "Trajectory Converter initialized");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    }

private:
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.w = std::cos(yaw / 2.0);
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        return q;
    }
    
    void trajectoryCallback(const multi_agent_planner_msgs::msg::Trajectory::SharedPtr msg) {
        // Create output message
        auto out_msg = mpc_controller_ros2_msgs::msg::Trajectory();
        
        // Convert header
        out_msg.header.stamp = msg->stamp;
        out_msg.header.frame_id = "world";
        
        // Convert timing information
        out_msg.t_0 = msg->planning_start_time;
        out_msg.dt = msg->dt;
        
        // Convert yaw to quaternion (will be applied to all states)
        auto orientation = yawToQuaternion(msg->yaw);
        
        // Convert each state
        for (const auto& in_state : msg->states) {
            mpc_controller_ros2_msgs::msg::TrajectoryState out_state;
            
            // Convert position (array to Point)
            if (in_state.position.size() >= 3) {
                out_state.position.x = in_state.position[0];
                out_state.position.y = in_state.position[1];
                out_state.position.z = in_state.position[2];
            } else {
                RCLCPP_WARN(this->get_logger(), "Position array too short, padding with zeros");
                out_state.position.x = in_state.position.size() > 0 ? in_state.position[0] : 0.0;
                out_state.position.y = in_state.position.size() > 1 ? in_state.position[1] : 0.0;
                out_state.position.z = in_state.position.size() > 2 ? in_state.position[2] : 0.0;
            }
            
            // Convert velocity (array to Vector3)
            if (in_state.velocity.size() >= 3) {
                out_state.velocity.x = in_state.velocity[0];
                out_state.velocity.y = in_state.velocity[1];
                out_state.velocity.z = in_state.velocity[2];
            } else {
                RCLCPP_WARN(this->get_logger(), "Velocity array too short, padding with zeros");
                out_state.velocity.x = in_state.velocity.size() > 0 ? in_state.velocity[0] : 0.0;
                out_state.velocity.y = in_state.velocity.size() > 1 ? in_state.velocity[1] : 0.0;
                out_state.velocity.z = in_state.velocity.size() > 2 ? in_state.velocity[2] : 0.0;
            }
            
            // Set orientation from yaw
            out_state.orientation = orientation;
            
            // Set angular velocity to zero (not provided in input)
            out_state.angular_velocity.x = 0.0;
            out_state.angular_velocity.y = 0.0;
            out_state.angular_velocity.z = 0.0;
            
            out_msg.states.push_back(out_state);
        }
        
        // Publish converted message
        pub_->publish(out_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Converted trajectory: %zu states, dt=%.3f, t_0=%.3f",
                    out_msg.states.size(), out_msg.dt, out_msg.t_0);
    }
    
    rclcpp::Subscription<multi_agent_planner_msgs::msg::Trajectory>::SharedPtr sub_;
    rclcpp::Publisher<mpc_controller_ros2_msgs::msg::Trajectory>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryConverter>());
    rclcpp::shutdown();
    return 0;
}

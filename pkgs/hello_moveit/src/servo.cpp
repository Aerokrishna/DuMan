#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "std_msgs/msg/header.hpp"

class JointJogPublisher : public rclcpp::Node
{
public:
    JointJogPublisher()
    : Node("joint_jog_publisher")
    {
        // Publisher for JointJog messages
        pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_joint_cmds", 10);

        // Timer to publish at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointJogPublisher::publish_cmd, this)
        );
    }

private:
    void publish_cmd()
    {
        auto msg = control_msgs::msg::JointJog();

        // Header with current timestamp
        msg.header.stamp = this->now();
        msg.header.frame_id = "base";  // change to your robot frame

        // Joint names to control
        msg.joint_names = {"hip", "shoulder", "elbow", "wrist_yaw", "wrist"};

        // Joint position increments (optional)
        msg.displacements = {0.0, 0.0, 0.0, 0.0, 0.0};  

        // Joint velocities (rad/s)
        msg.velocities = {0.1, 0.1, 0.1, 0.1, 0.1};

        // Duration for this command
        msg.duration = 0.1;  // seconds, should match your publish period

        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published JointJog command with timestamp %d.%d",
                    msg.header.stamp.sec, msg.header.stamp.nanosec);
    }

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointJogPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"

// class TwistStampedPublisher : public rclcpp::Node
// {
// public:
//     TwistStampedPublisher()
//     : Node("twist_stamped_publisher")
//     {
//         // Publisher on the desired topic
//         pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

//         // Timer period in milliseconds (adjust for publish rate)
//         auto timer_period = std::chrono::milliseconds(100); // 10 Hz
//         timer_ = this->create_wall_timer(
//             timer_period,
//             std::bind(&TwistStampedPublisher::publish_cmd, this)
//         );
//     }

// private:
//     void publish_cmd()
//     {
//         auto msg = geometry_msgs::msg::TwistStamped();

//         // Fill in header with current timestamp
//         msg.header.stamp = this->now();
//         msg.header.frame_id = "base"; // Replace with your robot's frame

//         // Set linear velocities (m/s)
//         msg.twist.linear.x = 0.1;
//         msg.twist.linear.y = 0.1;
//         msg.twist.linear.z = 0.0;

//         // Set angular velocities (rad/s)
//         msg.twist.angular.x = 0.0;
//         msg.twist.angular.y = 0.0;
//         msg.twist.angular.z = 0.0;

//         pub_->publish(msg);
//         // RCLCPP_INFO(this->get_logger(), "Published TwistStamped with timestamp: %d.%d",
//         //             msg.header.stamp.sec, msg.header.stamp.nanosec);
//     }

//     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TwistStampedPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

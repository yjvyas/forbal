/*  Position Rhombus Controller for a 2-DOF kinematic closed-chain planar mechanism
    Package: forbal_controllers (Force Balanced Manipulator Controllers)
    
    Written by Yash Vyas (2024), University of Padua
    Reuse allowed under MIT Licence.
*/

#ifndef FORBAL_CONTROLLERS_FORBAL5_CONTROLLER_H
#define FORBAL_CONTROLLERS_FORBAL5_CONTROLLER_H

#define _USE_MATH_DEFINES
#include <array>
#include <vector>
#include <cmath>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "forbal_interfaces/msg/position_trajectory.hpp"
#include "forbal_interfaces/action/follow_position_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "spline.h"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

class Forbal5Controller : public rclcpp::Node {
public:
    using FollowPositionTrajectory = forbal_interfaces::action::FollowPositionTrajectory;
    using GoalHandleFollowPositionTrajectory = rclcpp_action::ServerGoalHandle<FollowPositionTrajectory>;
    
    Forbal5Controller();
    
    struct JointAngles {
        double theta0;
        double theta1a;
        double theta1p;
        double theta2a;
        double theta2p;
        double theta3;
        double theta4;
    };

    struct Point {
        double x;
        double y;
        double z;
    };

    struct PointAngle5D {
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
    };

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fixed_pub_;

    void joint_state_callback_(const sensor_msgs::msg::JointState::SharedPtr msg);
    void robot_description_callback_(const std_msgs::msg::String::SharedPtr msg);
};

#endif // FORBAL2_CONTROLLER_H
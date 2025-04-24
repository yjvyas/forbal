/*  Position Rhombus Controller for a 2-DOF kinematic closed-chain planar mechanism
    Package: forbal_controllers (Force Balanced Manipulator Controllers)
    
    Written by Yash Vyas (2024), University of Padua
    Reuse allowed under MIT Licence.
*/

#ifndef FORBAL_CONTROLLERS_FORBAL2_CONTROLLER_H
#define FORBAL_CONTROLLERS_FORBAL2_CONTROLLER_H

#define _USE_MATH_DEFINES
#include <array>
#include <vector>
#include <cmath>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "forbal_interfaces/msg/position_trajectory.hpp"
#include "forbal_interfaces/action/follow_position_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "spline.h"

class Forbal2Controller : public rclcpp::Node {
public:
    using FollowPositionTrajectory = forbal_interfaces::action::FollowPositionTrajectory;
    using GoalHandleFollowPositionTrajectory = rclcpp_action::ServerGoalHandle<FollowPositionTrajectory>;
    
    Forbal2Controller();
    
    struct JointAngles {
        double theta1a;
        double theta1p;
        double theta2a;
        double theta2p;
    };

    struct Point {
        double x;
        double y;
        double z;
    };

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<forbal_interfaces::msg::PositionTrajectory>::SharedPtr position_traj_sub_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_controller_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fixed_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ee_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ee_ref_pub_;
    rclcpp_action::Server<FollowPositionTrajectory>::SharedPtr position_trajectory_action_server_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joint_trajectory_client_;

    void point_callback_(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void position_traj_callback_(const forbal_interfaces::msg::PositionTrajectory::SharedPtr msg);
    void joint_controller_callback_(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void joint_state_callback_(const sensor_msgs::msg::JointState::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, 
        const std::shared_ptr<const FollowPositionTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowPositionTrajectory> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFollowPositionTrajectory> goal_handle);

    void execute(const std::shared_ptr<GoalHandleFollowPositionTrajectory> goal_handle);

    JointAngles forward_kinematics(double theta1a, double theta2a);
    Point ee_position(JointAngles q);
    JointAngles inverse_kinematics(double x, double z);
    bool check_joint_limits(JointAngles ja);
    bool check_trajectory(const std::vector<double>& time, const std::vector<double>& x, const std::vector<double> z);
    double total_spline_distance(const tk::spline& sx, const tk::spline& sz, double tMax, float dt);

    std::string joint_1_act_;
    std::string joint_2_act_;
    std::string joint_1_psv_;
    std::string joint_2_psv_;
    bool trajectory_active;
    float l_;
    float le_;
    float jz_;
    float jx_;
    float j1a_max_;
    float j1a_min_;
    float j2a_max_;
    float j2a_min_;
    float j1p_max_;
    float j1p_min_;
    float j2p_max_;
    float j2p_min_;
};

#endif // FORBAL2_CONTROLLER_H
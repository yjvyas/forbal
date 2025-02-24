/*  Position Rhombus Controller for a 2-DOF kinematic closed-chain planar mechanism
    Package: forbal_controllers (Force Balanced Manipulator Controllers)
    
    Written by Yash Vyas (2024), University of Padua
    Reuse allowed under MIT Licence.
*/

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
  
  Forbal5Controller() : Node("forbal5_controller") {
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot_description",
        rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
        std::bind(&Forbal5Controller::robot_description_callback_, this, std::placeholders::_1));

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&Forbal5Controller::joint_state_callback_, this, std::placeholders::_1)
    );

    joint_states_fixed_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states_fixed",
        10
    );

    RCLCPP_INFO(this->get_logger(), "Initialised Forbal5Controller.");
  }
  
  struct JointAngles {
      double _0;
      double _11;
      double _12;
      double _21;
      double _22;
      double _3;
      double _4;
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
  void robot_description_callback_(const std_msgs::msg::String& msg) {
    // Construct KDL tree from URDF
    const std::string urdf = msg.data;
    if (!kdl_parser::treeFromString(urdf, tree_)) RCLCPP_ERROR(this->get_logger(),"Unable to parse robot description!");
    // Print basic information about the tree
    std::cout << "nb joints:        " << tree_.getNrOfJoints() << std::endl;
    std::cout << "nb segments:      " << tree_.getNrOfSegments() << std::endl;
    std::cout << "root segment:     " << tree_.getRootSegment()->first << std::endl;

    RCLCPP_INFO(this->get_logger(),"Loaded URDF");
  }

  void joint_state_callback_(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // if (msg) RCLCPP_INFO(this->get_logger(),"got message");
    JointAngles q;

    for (size_t i=0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "joint0") q._0 = msg->position[i];
      if (msg->name[i] == "joint11") q._11 = msg->position[i];
      if (msg->name[i] == "joint21") q._21 = msg->position[i];
      if (msg->name[i] == "joint3") q._3 = msg->position[i];
      if (msg->name[i] == "joint4") q._4 = msg->position[i];
    }

  if (q._11 && q._21) {
    resolve_passive_joints(q);

    auto msg_new = sensor_msgs::msg::JointState();
    msg_new.header.stamp = this->get_clock()->now();
    msg_new.name = {"joint0","joint11","joint12","joint21","joint22","joint3","joint4"};
    msg_new.position = {q._0, q._11, q._12, q._21, q._22, q._3, q._4};
    joint_states_fixed_pub_->publish(msg_new);
    }
  }

  void resolve_passive_joints(JointAngles &q) {
    // assumes all lengths of the closed chain 5BL are the same, i.e. it's a rhombus
    double alpha = q._11+q._21;
    double beta = M_PI-alpha;
    q._12 = M_PI-beta;
    q._22 = M_PI-beta;
  }

  KDL::Tree tree_;
  KDL::Chain chain_;

  float l_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fixed_pub_;
};



int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Forbal5Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

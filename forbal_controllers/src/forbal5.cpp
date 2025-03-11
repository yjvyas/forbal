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
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "forbal_interfaces/action/follow5_dof_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

class Forbal5Controller : public rclcpp::Node {
public:
  using Follow5DOFTrajectory = forbal_interfaces::action::Follow5DOFTrajectory;
  using GoalHandleFollow5DOFTrajectory = rclcpp_action::ServerGoalHandle<Follow5DOFTrajectory>;
  
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

    joint_controller_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/joint_trajectory_controller/controller_state",
      10,
      std::bind(&Forbal5Controller::joint_controller_callback_, this, std::placeholders::_1)
    );

    joint_states_fixed_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states_fixed",
        10
    );

    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "ee_pose",
      10
    );

    ee_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "ee_ref",
      10
    );

    ee_traj_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "ee_traj",
      10
    );

    follow_5DOF_trajectory_action_server_ = rclcpp_action::create_server<Follow5DOFTrajectory>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "follow_5dof_trajectory",
      std::bind(&Forbal5Controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Forbal5Controller::handle_cancel, this, std::placeholders::_1),
      std::bind(&Forbal5Controller::handle_accepted, this, std::placeholders::_1)
    );

    joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this,
      "/joint_trajectory_controller/follow_joint_trajectory"
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

  struct Point5D {
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
    if (kdl_parser::treeFromString(urdf, tree_)) {
      RCLCPP_INFO(this->get_logger(),"Loaded URDF");
      RCLCPP_INFO(this->get_logger(),"Joints: %u, Segments: %u", tree_.getNrOfJoints(), tree_.getNrOfSegments());
      if (tree_.getChain("world","ee", chain_world_ee_)) {
        RCLCPP_INFO(this->get_logger(),"Joints(world->ee): %u", chain_world_ee_.getNrOfJoints());
        solver_ee_pos_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_world_ee_);
      } else {
        RCLCPP_ERROR(this->get_logger(),"Unable to extract chain from world to ee!");
      }
      if (tree_.getChain("world","ee_mkr", chain_world_eemkr_)) {
        RCLCPP_INFO(this->get_logger(),"Joints(world->ee_mkr): %u", chain_world_eemkr_.getNrOfJoints());
        solver_eemkr_pos_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_world_eemkr_);
        // for (size_t i = 0; i < chain_world_eemkr_.getNrOfSegments(); ++i) {
        //   const KDL::Segment& segment = chain_world_eemkr_.getSegment(i);
        //   if (segment.getJoint().getType() != KDL::Joint::None) { // Ignore fixed joints
        //       std::cout << i << ": " << segment.getJoint().getName() << std::endl;
        //   }
        // }
      } else {
        RCLCPP_ERROR(this->get_logger(),"Unable to extract chain from world to ee_mkr!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),"Unable to parse robot description!");
    }
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

    KDL::JntArray joint_positions(5);
    joint_positions(0) = q._0;
    joint_positions(1) = q._21;
    joint_positions(2) = q._22;
    joint_positions(3) = q._3;
    joint_positions(4) = q._4;

    int result = solver_eemkr_pos_->JntToCart(joint_positions, eemkr_pose_);
    if (result==0) {
      // RCLCPP_INFO(this->get_logger(),"ee_mkr x: %f, y: %f, z: %f",eemkr_pose_.p.x(),eemkr_pose_.p.y(),eemkr_pose_.p.z());
      auto msg_ee = geometry_msgs::msg::PoseStamped();
      msg_ee.header.stamp = this->get_clock()->now();
      msg_ee.header.frame_id = "world";
      msg_ee.pose.position.x = eemkr_pose_.p.x();
      msg_ee.pose.position.y = eemkr_pose_.p.y();
      msg_ee.pose.position.z = eemkr_pose_.p.z();
 
      double qx, qy, qz, qw;
      eemkr_pose_.M.GetQuaternion(qx, qy, qz, qw);
      msg_ee.pose.orientation.x = qx;
      msg_ee.pose.orientation.y = qy;
      msg_ee.pose.orientation.z = qz;
      msg_ee.pose.orientation.w = qw;
      ee_pose_pub_->publish(msg_ee);
      } else {
        RCLCPP_ERROR(this->get_logger(),"Pose estimate failed, code: %d.",result);
      }
    }
  }
  
  void joint_controller_callback_(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(),"controller state message received, time: %i",msg->header.stamp.sec);

    JointAngles q;

    for (size_t i=0; i < msg->joint_names.size(); ++i) {
      if (msg->joint_names[i] == "joint0") q._0 = msg->reference.positions[i];
      if (msg->joint_names[i] == "joint11") q._11 = msg->reference.positions[i];
      if (msg->joint_names[i] == "joint21") q._21 = msg->reference.positions[i];
      if (msg->joint_names[i] == "joint3") q._3 = msg->reference.positions[i];
      if (msg->joint_names[i] == "joint4") q._4 = msg->reference.positions[i];
    }

    if (q._11 && q._21) {
      resolve_passive_joints(q);
      
      auto msg_new = sensor_msgs::msg::JointState();
      msg_new.header.stamp = this->get_clock()->now();
      msg_new.name = {"joint0","joint11","joint12","joint21","joint22","joint3","joint4"};
      msg_new.position = {q._0, q._11, q._12, q._21, q._22, q._3, q._4};
      joint_states_fixed_pub_->publish(msg_new);

      KDL::JntArray joint_positions(5);
      joint_positions(0) = q._0;
      joint_positions(1) = q._21;
      joint_positions(2) = q._22;
      joint_positions(3) = q._3;
      joint_positions(4) = q._4;

      int result = solver_eemkr_pos_->JntToCart(joint_positions, eemkr_pose_);
      if (result==0) {
        // RCLCPP_INFO(this->get_logger(),"ee_mkr x: %f, y: %f, z: %f",eemkr_pose_.p.x(),eemkr_pose_.p.y(),eemkr_pose_.p.z());
        auto msg_ee = geometry_msgs::msg::PoseStamped();
        msg_ee.header.stamp = this->get_clock()->now();
        msg_ee.header.frame_id = "world";
        msg_ee.pose = frame_to_PoseMsg(eemkr_pose_);
        ee_ref_pub_->publish(msg_ee);
        } else {
          RCLCPP_ERROR(this->get_logger(),"Pose estimate failed, code: %d.",result);
        }
    }
  }

  void resolve_passive_joints(JointAngles &q) {
    // assumes all lengths of the closed chain 5BL are the same, i.e. it's a rhombus
    double alpha = q._11+q._21;
    double beta = M_PI-alpha;
    q._12 = M_PI-beta;
    q._22 = M_PI-beta;
  }

  KDL::Frame point5D_to_Frame(const Point5D& point) {
    return KDL::Frame(KDL::Rotation::RotY(point.pitch)*KDL::Rotation::RotZ(point.yaw),
                      KDL::Vector(point.x, point.y, point.z));
  }

  geometry_msgs::msg::Pose frame_to_PoseMsg(const KDL::Frame& frame) {
    auto pose = geometry_msgs::msg::Pose();
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();

    double qx, qy, qz, qw;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
  }

  JointAngles inverse_kinematics(KDL::Frame) {
    JointAngles q;
    return q;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid, 
      const std::shared_ptr<const Follow5DOFTrajectory::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received a goal request with UUID: %s", 
    rclcpp_action::to_string(uuid).c_str());
    bool array_size = true;

    if (goal->x.size() != goal->time.size()) array_size = false;
    if (goal->y.size() != goal->time.size()) array_size = false;
    if (goal->z.size() != goal->time.size()) array_size = false;
    if (goal->pitch.size() != goal->time.size()) array_size = false;
    if (goal->yaw.size() != goal->time.size()) array_size = false;

    if (!array_size) {
      RCLCPP_WARN(this->get_logger(), "Goal arrays have mismatched sizes");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // check trajectory here
    auto ee_traj_msg = geometry_msgs::msg::PoseArray();
    ee_traj_msg.header.stamp = this->get_clock()->now();
    ee_traj_msg.header.frame_id = "world";
    std::vector<geometry_msgs::msg::Pose> traj_poses;
    for (size_t i=0; i < goal->time.size(); ++i) {
      Point5D point_5D;
      point_5D.x = goal->x[i]; 
      point_5D.y = goal->y[i]; 
      point_5D.z = goal->z[i];
      point_5D.pitch = goal->pitch[i];
      point_5D.yaw = goal->yaw[i];

      traj_poses.push_back(frame_to_PoseMsg(point5D_to_Frame(point_5D)));
    }
    ee_traj_msg.poses = traj_poses;
    ee_traj_pub_->publish(ee_traj_msg);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollow5DOFTrajectory> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Canceling goal");
    goal_handle->canceled(std::make_shared<Follow5DOFTrajectory::Result>());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollow5DOFTrajectory> goal_handle) {
    std::thread([this, goal_handle]() {execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollow5DOFTrajectory> goal_handle) {
    auto goal = goal_handle->get_goal();
    // auto trajectory_goal = control_msgs::action::FollowJointTrajectory::Goal();
    // trajectory_goal.trajectory.joint_names = {"joint0", "joint11","joint21","joint3","joint4"};
    
    auto result = std::make_shared<Follow5DOFTrajectory::Result>();
    result->error_code = result->SUCCESSFUL;
    goal_handle->succeed(result);
  }

  KDL::Tree tree_;
  KDL::Chain chain_world_ee_;
  KDL::Chain chain_world_eemkr_;
  KDL::Chain chain_ee_eeimp_;
  KDL::Frame eemkr_pose_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_ee_pos_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_eemkr_pos_;

  float l_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_controller_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fixed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_ref_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ee_traj_pub_;

  rclcpp_action::Server<Follow5DOFTrajectory>::SharedPtr follow_5DOF_trajectory_action_server_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joint_trajectory_client_;
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Forbal5Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

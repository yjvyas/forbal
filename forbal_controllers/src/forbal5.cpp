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
#include "forbal_controllers/spline.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <map>
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
#include <urdf/model.h>

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

    joint_names_ = {"joint0", "joint11", "joint12", "joint21", "joint22", "joint3", "joint4"};

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

    try {
      KDL::Frame T_O_j21 = computeRelativeTransform(tree_, "world", "link21");
      KDL::Frame T_j11_jee = computeRelativeTransform(tree_, "link21", "ee");
      T_ee_j3 = computeRelativeTransform(tree_, "ee", "ee_motor");
      T_j3_j4 = computeRelativeTransform(tree_, "ee_motor", "ee_imp");
      RCLCPP_INFO(this->get_logger(),"T_ee_j3: %f, %f, %f",T_ee_j3.p.x(), T_ee_j3.p.y(), T_ee_j3.p.z());
      RCLCPP_INFO(this->get_logger(),"T_j3_j4: %f, %f, %f",T_j3_j4.p.x(), T_j3_j4.p.y(), T_j3_j4.p.z());
      l_ = T_j11_jee.p.x(); // length of the span of the manipulator up to end effector
      h_ = T_O_j21.p.z(); // sets height of the joint base
      x_off_ = T_O_j21.p.x(); // x offset of the joint base
      RCLCPP_INFO(this->get_logger(),"Manipulator span: %f, height: %f", l_, h_);
  } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform computation error: %s", ex.what());
  }
  
  urdf::Model urdf_model;
  if (!urdf_model.initString(msg.data)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF model. All joint limits will be set to bounds.");
    for (const std::string& name : joint_names_) {
      joint_limits_[name] = { -M_PI, M_PI }; // Default limits if URDF parsing fails
      }
  } else {
  //   // Set joint limits based on URDF
    for (const std::string& name : joint_names_) {
      urdf::JointConstSharedPtr joint = urdf_model.getJoint(name);
      if (!joint) {
        RCLCPP_WARN(this->get_logger(), "Joint %s not found in URDF", name.c_str());
        joint_limits_[name] = { -M_PI, M_PI }; // Default limits if URDF parsing fails
        continue;
      } else {
        joint_limits_[name] = {joint->limits->lower, joint->limits->upper};
      }
    }
  
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
        ee_pose_pub_->publish(msg_ee);
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
  // Converts a Point5D to a KDL::Frame
  // find the position of the end-effector in the world frame and it's yaw
  float ei_angle = atan2(point.y,point.x);

  return KDL::Frame(KDL::Rotation::RotZ(ei_angle)*KDL::Rotation::RotY(point.pitch)*KDL::Rotation::RotZ(point.yaw-ei_angle),
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

  bool check_joint_limits(JointAngles& q) {
    // Checks if the joint angles are within the limits
    bool limits = true;
    if (q._0 < joint_limits_["joint0"].first || q._0 > joint_limits_["joint0"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 0 angle %f out of limits [%f, %f]", q._0, joint_limits_["joint0"].first, joint_limits_["joint0"].second);
      limits = false;
    }
    if (q._11 < joint_limits_["joint11"].first || q._11 > joint_limits_["joint11"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 11 angle %f out of limits [%f, %f]", q._11, joint_limits_["joint11"].first, joint_limits_["joint11"].second);
      limits = false;
    }
    if (q._12 < joint_limits_["joint12"].first || q._12 > joint_limits_["joint12"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 12 angle %f out of limits [%f, %f]", q._12, joint_limits_["joint12"].first, joint_limits_["joint12"].second);
      limits = false;
    }
    if (q._21 < joint_limits_["joint21"].first || q._21 > joint_limits_["joint21"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 21 angle %f out of limits [%f, %f]", q._21, joint_limits_["joint21"].first, joint_limits_["joint21"].second);
      limits = false;
    }
    if (q._22 < joint_limits_["joint22"].first || q._22 > joint_limits_["joint22"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 22 angle %f out of limits [%f, %f]", q._22, joint_limits_["joint22"].first, joint_limits_["joint22"].second);
      limits = false;
    }
    if (q._3 < joint_limits_["joint3"].first || q._3 > joint_limits_["joint3"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 3 angle %f out of limits [%f, %f]", q._3, joint_limits_["joint3"].first, joint_limits_["joint3"].second);
      limits = false;
    }
    if (q._4 < joint_limits_["joint4"].first || q._4 > joint_limits_["joint4"].second) {
      RCLCPP_ERROR(this->get_logger(), "Joint 4 angle %f out of limits [%f, %f]", q._4, joint_limits_["joint4"].first, joint_limits_["joint4"].second);
      limits = false;
    }
    return limits;
  }

  bool inverse_kinematics(const Point5D& point, JointAngles& q) {
    // Computes the inverse kinematics for the end-effector position and orientation
    // returns true if an IK solution exists, false otherwise.
    float ei_angle = atan2(point.y,point.x);
    // RCLCPP_INFO(this->get_logger(), "End-effector angle: %f", ei_angle);

    KDL::Frame T_ei(KDL::Rotation::RotZ(ei_angle),
                    KDL::Vector(point.x, point.y, point.z));
    // RCLCPP_INFO(this->get_logger(), "End-effector position: [%f, %f, %f]", T_ei.p.x(), T_ei.p.y(), T_ei.p.z());

    KDL::Frame T_ei_proj = KDL::Frame(KDL::Rotation::RotZ(-ei_angle))*T_ei; // projected in the plane
    // RCLCPP_INFO(this->get_logger(), "proj ei position: [%f, %f, %f]",T_ei_proj.p.x(), T_ei_proj.p.y(), T_ei_proj.p.z());

    KDL::Frame T_j3_j4_inv = KDL::Frame(KDL::Rotation::RotY(point.pitch),KDL::Vector(-T_j3_j4.p.y(),0.0,T_j3_j4.p.x())).Inverse();
    KDL::Frame T_em_proj = T_ei_proj * T_j3_j4_inv; // end-effector mount position in the projected frame
    // RCLCPP_INFO(this->get_logger(), "proj em position: [%f, %f, %f]", T_em_proj.p.x(), T_em_proj.p.y(), T_em_proj.p.z());
    KDL::Vector pd = KDL::Vector(T_em_proj.p.x()-x_off_,T_em_proj.p.y(), T_em_proj.p.z()-h_); // desired ee position from the height location
    // RCLCPP_INFO(this->get_logger(), "proj em position relative to height: [%f, %f, %f]", pd.x(), pd.y(), pd.z());

    q._0 = ei_angle; // angle of the joint 0 to hit the required angle
    // RCLCPP_INFO(this->get_logger(), "joint 0 angle: %f", q._0);
    float d_ee = pd.Norm();
    float le = T_ee_j3.p.x(); // length of the end-effector mount to ee motor
    // RCLCPP_INFO(this->get_logger(), "distance = %f", d_ee);
    float l = l_/2;
    if (d_ee < l_+le) {
      double thetaE0 = atan(pd.z()/pd.x());
      double lE0 = sqrt(pow(pd.x(),2)+pow(pd.z(),2)); // distance from the base to the end-effector in the projected plane
      double beta = acos((pow(l,2)+pow(l+le,2)-pow(lE0,2))/(2*l*(l+le)));
      double gamma = acos((pow(lE0,2)+pow(l,2)-pow(l+le,2))/(2*l*lE0));
      double alpha = M_PI-beta;

      q._21 = gamma+thetaE0;
      q._11 = alpha-q._21;
      q._12 = M_PI-beta;
      q._22 = M_PI-beta;
      q._3 = q._22-q._21-point.pitch; // pitch relative to the end-effector
      q._4 = -q._0+point.yaw;

      // RCLCPP_INFO(this->get_logger(), "Joint angles: q0 = %f, q11 = %f, q12 = %f, q21 = %f, q22 = %f, q3 = %f, q4 = %f", q._0, q._11, q._12, q._21, q._22, q._3, q._4);
      if (!check_joint_limits(q)) {
        // RCLCPP_ERROR(this->get_logger(), "Joint angles out of limits, no IK solution exists.");
        return false;
      }
    } else {
      // RCLCPP_ERROR(this->get_logger(), "End-effector mount position is %f m, too far from the base, no IK solution exists.", d_ee);
      return false;
    }

    return true;
  }

  // Utility function
  KDL::Frame computeRelativeTransform(const KDL::Tree& tree, const std::string& from, const std::string& to) {
    KDL::Chain chain;
    if (!tree.getChain(from, to, chain)) {
        throw std::runtime_error("Failed to extract chain from " + from + " to " + to);
    }

    KDL::JntArray joint_positions(chain.getNrOfJoints());
    SetToZero(joint_positions);

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame transform;

    if (fk_solver.JntToCart(joint_positions, transform) < 0) {
      throw std::runtime_error("Failed to compute FK from " + from + " to " + to);
    }

    return transform;
  }

  double total_spline_distance(const tk::spline& sx, const tk::spline&sy, const tk::spline& sz, double tMax, float dt) {
    double d_total = 0.0;
    double x_prev = sx(0.0);
    double y_prev = sy(0.0);
    double z_prev = sz(0.0);
    double x_curr;
    double y_curr;
    double z_curr;
  
    for (double t = 0.0 + dt; t <= tMax; t += dt) {
        x_curr = sx(t);
        y_curr = sy(t);
        z_curr = sz(t);
        d_total += sqrt(pow(x_curr - x_prev, 2) + pow(y_curr - y_prev, 2) + pow(z_curr - z_prev, 2));
        x_prev = x_curr;
        y_prev = y_curr;
        z_prev = z_curr;
    }
    return d_total;
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

      JointAngles q;
      if (!inverse_kinematics(point_5D, q)) {
        RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed for point %zu", i);
        return rclcpp_action::GoalResponse::REJECT;
      } else {
        traj_poses.push_back(frame_to_PoseMsg(point5D_to_Frame(point_5D)));
      }
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

    tk::spline sx(goal->time, goal->x,
                  tk::spline::cspline_hermite, false,
                  tk::spline::first_deriv, 0.0,
                  tk::spline::first_deriv, 0.0);

    tk::spline sy(goal->time, goal->y,
                  tk::spline::cspline_hermite, false,
                  tk::spline::first_deriv, 0.0,
                  tk::spline::first_deriv, 0.0);

    tk::spline sz(goal->time, goal->z,  
                  tk::spline::cspline_hermite, false,
                  tk::spline::first_deriv, 0.0,
                  tk::spline::first_deriv, 0.0);

    tk::spline spitch(goal->time, goal->pitch,
                      tk::spline::cspline_hermite, false,
                      tk::spline::first_deriv, 0.0,
                      tk::spline::first_deriv, 0.0);

    tk::spline syaw(goal->time, goal->yaw,
                      tk::spline::cspline_hermite, false,
                      tk::spline::first_deriv, 0.0,
                      tk::spline::first_deriv, 0.0);

    float dt;
    if (goal->dt <= 0.0f) {
      RCLCPP_WARN(this->get_logger(),"FollowPositionTrajectory: dt not specified or incorrent, setting to 0.01");
      dt = 0.01;
    } else {
      dt = goal->dt;
    }
    double tMax = *std::max_element(goal->time.begin(),goal->time.end());

    auto trajectory_goal = control_msgs::action::FollowJointTrajectory::Goal();
    trajectory_goal.trajectory.joint_names = {"joint0", "joint11","joint21","joint3","joint4"};

    JointAngles q;
    Point5D p;

    auto ee_traj_msg = geometry_msgs::msg::PoseArray();
    ee_traj_msg.header.stamp = this->get_clock()->now();
    ee_traj_msg.header.frame_id = "world";
    std::vector<geometry_msgs::msg::Pose> traj_poses;

    if (goal->type == "joint_trajectory") {
      for (size_t i = 0; i < goal->time.size(); ++i) {
        p.x = goal->x[i];
        p.y = goal->y[i];
        p.z = goal->z[i];
        p.pitch = goal->pitch[i];
        p.yaw = goal->yaw[i];

        if (inverse_kinematics(p, q)) {
          trajectory_goal.trajectory.points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
          trajectory_goal.trajectory.points.back().positions = {q._0, q._11, q._21, q._3, q._4};
          trajectory_goal.trajectory.points.back().time_from_start = rclcpp::Duration::from_seconds(goal->time[i]);
          traj_poses.push_back(frame_to_PoseMsg(point5D_to_Frame(p)));
        };
      }
    } else if (goal->type == "constant_waypoints") {
      for (float t = 0.0; t <= tMax; t += dt) {
        p.x = sx(t);
        p.y = sy(t);
        p.z = sz(t);
        p.pitch = spitch(t);
        p.yaw = syaw(t);

        if (inverse_kinematics(p, q)) {
          trajectory_goal.trajectory.points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
          trajectory_goal.trajectory.points.back().positions = {q._0, q._11, q._21, q._3, q._4};
          trajectory_goal.trajectory.points.back().time_from_start = rclcpp::Duration::from_seconds(t);
          traj_poses.push_back(frame_to_PoseMsg(point5D_to_Frame(p)));
        } else {
          auto result = std::make_shared<Follow5DOFTrajectory::Result>();
          result->error_code = result->INVALID_GOAL;
          result->error_string = "Trajectory out of bounds!";
          goal_handle->abort(result);
          return;
        }
      }
    } else if (goal->type == "trapezoidal_waypoints") {
      double d_total = total_spline_distance(sx, sy, sz, tMax, dt);
      float acc_time_ = goal->acc_time;
      RCLCPP_INFO(this->get_logger(), "Total spline distance: %f", d_total);
      RCLCPP_INFO(this->get_logger(),"Acceleration time in goal: %f", acc_time_);

      double t_const = tMax - 2 * acc_time_; // Time at max velocity
      double v_max = 2.0*d_total / (tMax + t_const); // using trapezoid area as d_total
      double a_max = v_max / acc_time_; // gradient of acceleration
      RCLCPP_INFO(this->get_logger(), "t_const: %f, d_total: %f, v_max: %f, a_max: %f",t_const,d_total,v_max,a_max);

      if (acc_time_ >= tMax / 2.0) {
        RCLCPP_ERROR(this->get_logger(), "acc_time is too large for the given tMax!");
        goal_handle->abort(std::make_shared<Follow5DOFTrajectory::Result>());
        return;
      }

      for (double t = 0.0; t <= tMax+dt; t += dt) {
        double s;

        // Calculate s(t) based on the trapezoidal speed law
        if (t <= acc_time_) {
            s = 0.5 * a_max * t * t; // Acceleration phase
        } else if (t <= acc_time_ + t_const) {
            s = 0.5 * a_max * acc_time_ * acc_time_ + v_max * (t - acc_time_); // Constant velocity phase
        } else if (t >= acc_time_ + t_const) {
            s = a_max * acc_time_ * acc_time_ + v_max * t_const 
                - 0.5 * a_max * (tMax - t) * (tMax - t); // Deceleration phase
        }

      // Map s(t) to the splines
      double x = sx(s*tMax/d_total);
      double y = sy(s*tMax/d_total);
      double z = sz(s*tMax/d_total);
      // RCLCPP_INFO(this->get_logger(), "t: %f, st: %f, x: %f, z: %f",t,s*tMax/d_total,x,z);

      p.x = x;
      p.y = y;
      p.z = z;
      p.pitch = spitch(t);
      p.yaw = syaw(t);

      if (inverse_kinematics(p, q)) {
        trajectory_goal.trajectory.points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
        trajectory_goal.trajectory.points.back().positions = {q._0, q._11, q._21, q._3, q._4};
        trajectory_goal.trajectory.points.back().time_from_start = rclcpp::Duration::from_seconds(t);
        traj_poses.push_back(frame_to_PoseMsg(point5D_to_Frame(p)));
      } else {
        auto result = std::make_shared<Follow5DOFTrajectory::Result>();
        result->error_code = result->INVALID_GOAL;
        result->error_string = "Trajectory out of bounds!";
        goal_handle->abort(result);
        return;
      }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported trajectory type: %s", goal->type.c_str());
      auto result = std::make_shared<Follow5DOFTrajectory::Result>();
      result->error_code = result->INVALID_GOAL;
      result->error_string = "Unsupported trajectory type";
      goal_handle->abort(result);
      return;
    }

    ee_traj_msg.poses = traj_poses;
    ee_traj_pub_->publish(ee_traj_msg);

    if (!joint_trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server not available");
      auto result = std::make_shared<Follow5DOFTrajectory::Result>();
      result->error_code = result->JOINT_TRAJECTORY_ERROR;
      result->error_string = "Action server not available";
      goal_handle->abort(result);
      return;
    }

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    auto time_start = this->now();
    send_goal_options.feedback_callback = [goal_handle, this, tMax, time_start](auto, auto feedback) {
      auto fb = std::make_shared<Follow5DOFTrajectory::Feedback>();
      fb->header = feedback->header;
      fb->position_error = 0.0;
      goal_handle->publish_feedback(fb);
    };

    auto future = joint_trajectory_client_->async_send_goal(trajectory_goal, send_goal_options);
    auto traj_goal_handle = future.get();
    if (!traj_goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory goal was rejected");
      auto result = std::make_shared<Follow5DOFTrajectory::Result>();
      result->error_code = result->JOINT_TRAJECTORY_ERROR;
      result->error_string = "Goal rejected by server";
      goal_handle->abort(result);
      return;
    }

    auto result_future = joint_trajectory_client_->async_get_result(future.get());

    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
      auto result = std::make_shared<Follow5DOFTrajectory::Result>();
      result->error_code = result->JOINT_TRAJECTORY_ERROR;
      result->error_string = "Trajectory failed.";
      goal_handle->abort(std::make_shared<Follow5DOFTrajectory::Result>());
    } else {
      RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded");
      auto result = std::make_shared<Follow5DOFTrajectory::Result>();
      result->error_code = result->SUCCESSFUL;
      goal_handle->succeed(result);
    }
  }

  KDL::Tree tree_;
  KDL::Chain chain_world_ee_;
  KDL::Chain chain_world_eemkr_;
  KDL::Chain chain_ee_eeimp_;
  KDL::Frame eemkr_pose_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_ee_pos_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_eemkr_pos_;

  float l_; // length of the link
  float x_off_; // offset of the base in the x direction
  float h_; // height of the manipulator joints 11 and 21
  float span_;
  KDL::Frame T_j3_j4; // transform from joint 3 to joint 4
  KDL::Frame T_ee_j3; // transform from end-effector to joint 3
  std::vector<std::string> joint_names_;
  std::map<std::string, std::pair<float, float>> joint_limits_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_controller_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fixed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
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

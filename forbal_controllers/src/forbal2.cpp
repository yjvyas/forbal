/*  Forbal2 Controller for a 2-DOF kinematic closed-chain planar mechanism
    Package: forbal_controllers (Force Balanced Manipulator Controllers)
    
    Written by Yash Vyas (2024), University of Padua
    Reuse allowed under MIT Licence.
*/

#include "forbal_controllers/forbal2.h"

using namespace std;

Forbal2Controller::Forbal2Controller() : Node("forbal2_controller") {
  this->declare_parameter<std::string>("joint_1_act","joint11");
  this->declare_parameter<std::string>("joint_2_act","joint21");
  this->declare_parameter<std::string>("joint_1_psv","joint12");
  this->declare_parameter<std::string>("joint_2_psv","joint22");
  this->declare_parameter<float>("link_length",0.2);
  this->declare_parameter<float>("ee_length",0.025);
  this->declare_parameter<float>("joints_origin_z",0.0);
  this->declare_parameter<float>("joints_origin_x",0.0);
  this->declare_parameter<float>("joint_1_act_max",M_PI);
  this->declare_parameter<float>("joint_1_act_min",-M_PI);
  this->declare_parameter<float>("joint_2_act_max",M_PI);
  this->declare_parameter<float>("joint_2_act_min",-M_PI);
  this->declare_parameter<float>("joint_1_psv_max",M_PI);
  this->declare_parameter<float>("joint_1_psv_min",-M_PI);
  this->declare_parameter<float>("joint_2_psv_max",M_PI);
  this->declare_parameter<float>("joint_2_psv_min",-M_PI);

  this->get_parameter("joint_1_act", joint_1_act_);
  this->get_parameter("joint_2_act", joint_2_act_);
  this->get_parameter("joint_1_psv", joint_1_psv_);
  this->get_parameter("joint_2_psv", joint_2_psv_);
  this->get_parameter("link_length",l_);
  this->get_parameter("ee_length",le_);
  this->get_parameter("joints_origin_x",jx_);
  this->get_parameter("joints_origin_z",jz_);
  this->get_parameter("joint_1_act_max",j1a_max_);
  this->get_parameter("joint_1_act_min",j1a_min_);
  this->get_parameter("joint_2_act_max",j2a_max_);
  this->get_parameter("joint_2_act_min",j2a_min_);
  this->get_parameter("joint_1_psv_max",j1p_max_);
  this->get_parameter("joint_1_psv_min",j1p_min_);
  this->get_parameter("joint_2_psv_max",j2p_max_);
  this->get_parameter("joint_2_psv_min",j2p_min_);

  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_1_act: %s",joint_1_act_.c_str());
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_2_act: %s",joint_2_act_.c_str());
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_1_psv: %s",joint_1_psv_.c_str());
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_2_psv: %s",joint_2_psv_.c_str());
  RCLCPP_INFO(this->get_logger(),"Forbal2 link_length: %f",l_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 ee_length: %f",le_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joints_origin_x: %f",jx_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joints_origin_z: %f",jz_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_1_act_max: %f",j1a_max_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_1_act_min: %f",j1a_min_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_2_act_max: %f",j2a_max_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_2_act_min: %f",j2a_min_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_1_psv_max: %f",j1p_max_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_1_psv_min: %f",j1p_min_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_2_psv_max: %f",j2p_max_);
  RCLCPP_INFO(this->get_logger(),"Forbal2 joint_2_psv_min: %f",j2p_min_);

  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "point_reference",
    10,
    std::bind(&Forbal2Controller::point_callback_, this, std::placeholders::_1)
  );

  position_traj_sub_ = this->create_subscription<forbal_interfaces::msg::PositionTrajectory>(
    "position_trajectory",
    10,
    std::bind(&Forbal2Controller::position_traj_callback_, this, std::placeholders::_1)
  );

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&Forbal2Controller::joint_state_callback_, this, std::placeholders::_1)
  );

  joint_states_fixed_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states_fixed",
    10
  );

  joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory",
    10
  );

  ee_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "position_trajectory/ee_position",
    10
  );

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/position_trajectory/markers",
    10
  );

  position_trajectory_action_server_ = rclcpp_action::create_server<FollowPositionTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "follow_position_trajectory",
    std::bind(&Forbal2Controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Forbal2Controller::handle_cancel, this, std::placeholders::_1),
    std::bind(&Forbal2Controller::handle_accepted, this, std::placeholders::_1)
  );

  joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this,
      "/joint_trajectory_controller/follow_joint_trajectory"
  );

  RCLCPP_INFO(this->get_logger(),
        "Initialised Forbal2Controller.");
}

void Forbal2Controller::point_callback_(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Point: '%f','%f','%f'", msg->point.x, msg->point.y, msg->point.z);
      
  JointAngles ja = inverse_kinematics(msg->point.x,msg->point.z);
  if (std::isnan(ja.theta1a) || std::isnan(ja.theta2a)) {
    RCLCPP_ERROR(this->get_logger(),"Reference rejected.");
  }
  // RCLCPP_INFO(this->get_logger(), "Joint Angles: 11 = '%f', 21 = '%f', 12='%f', 22='%f'", ja.theta1a, ja.theta2a, ja.theta1p, ja.theta2p);

  if (check_joint_limits(ja)) {
    auto msg_jt = trajectory_msgs::msg::JointTrajectory();
    msg_jt.joint_names = {joint_1_act_,joint_2_act_};

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = {ja.theta1a, ja.theta2a};
    point.time_from_start = rclcpp::Duration::from_seconds(0.01); // immediate tracking

    msg_jt.points.push_back(point);

    msg_jt.header.stamp = this->get_clock()->now();
    joint_trajectory_pub_->publish(msg_jt);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Reference rejected.");
  }

  trajectory_active = false;
}

void Forbal2Controller::position_traj_callback_(const forbal_interfaces::msg::PositionTrajectory::SharedPtr msg) {
  auto msg_jt = trajectory_msgs::msg::JointTrajectory();
  msg_jt.joint_names = {joint_1_act_,joint_2_act_};
  // RCLCPP_INFO(this->get_logger(),"Received message");
  if ((msg->time.size() != msg->x.size()) || (msg->time.size() != msg->z.size())) {
    RCLCPP_WARN(this->get_logger(),"PositionTrajectory message does not have same size arrays.");
    return;
  }

  for (size_t i = 0; i < msg->time.size(); ++i) {
    JointAngles ja = inverse_kinematics(msg->x[i],msg->z[i]);
    if (std::isnan(ja.theta1a) || std::isnan(ja.theta2a)) {
      RCLCPP_WARN(this->get_logger(),"Point %i rejected.",int(i));
      continue;
    }

    if (check_joint_limits(ja)) {
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      point.positions = {ja.theta1a, ja.theta2a};
      point.time_from_start = rclcpp::Duration::from_seconds(msg->time[i]);

      msg_jt.points.push_back(point);
    } else {
      RCLCPP_WARN(this->get_logger(), "Point %i rejected.",int(i));
      continue;
    }
  }
  msg_jt.header.stamp = this->get_clock()->now();
  joint_trajectory_pub_->publish(msg_jt);
}

void Forbal2Controller::joint_state_callback_(const sensor_msgs::msg::JointState::SharedPtr msg) {
  double theta1a = NAN;
  double theta2a = NAN;

  for (size_t i=0; i < msg->name.size(); ++i) {
    if (msg->name[i] == joint_1_act_) theta1a = msg->position[i];
    if (msg->name[i] == joint_2_act_) theta2a = msg->position[i];
  }

  if (theta1a && theta2a) {
    JointAngles ja = forward_kinematics(theta1a, theta2a);

    auto msg_new = sensor_msgs::msg::JointState();
    msg_new.header.stamp = this->get_clock()->now();
    msg_new.name = {joint_1_act_,joint_1_psv_,joint_2_act_,joint_2_psv_};
    msg_new.position = {ja.theta1a, ja.theta1p, ja.theta2a, ja.theta2p};
    joint_states_fixed_pub_->publish(msg_new);

    auto msg_ee = geometry_msgs::msg::PointStamped();
    msg_ee.header.stamp = this->get_clock()->now();
    Forbal2Controller::Point p = ee_position(ja); 
    msg_ee.point.x = p.x;
    msg_ee.point.y = p.y;
    msg_ee.point.z = p.z;
    ee_position_pub_->publish(msg_ee);

    // RCLCPP_INFO(this->get_logger(),"1a: %f 2a: %f 1p: %f 2p:%f", ja.theta1a, ja.theta2a, ja.theta1p, ja.theta2p);
  }
}

rclcpp_action::GoalResponse Forbal2Controller::handle_goal(
        const rclcpp_action::GoalUUID & uuid, 
        const std::shared_ptr<const FollowPositionTrajectory::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received a goal request with UUID: %s", 
              rclcpp_action::to_string(uuid).c_str());

  if (goal->x.size() != goal->time.size() || goal->z.size() != goal->time.size()) {
      RCLCPP_WARN(this->get_logger(), "Goal time, x and z arrays have mismatched sizes");
      return rclcpp_action::GoalResponse::REJECT;
  } else {
    if (goal->type == "constant_waypoints") {
      RCLCPP_INFO(this->get_logger(),"Trajectory type is constant_waypoints.");
      if (!check_trajectory(goal->time,goal->x,goal->z)) {
        RCLCPP_WARN(this->get_logger(),"FollowPositionTrajectory: trajectory invalid.");
        return rclcpp_action::GoalResponse::REJECT; 
      }
    } 
    else if (goal->type == "trapezoidal_waypoints") {
      RCLCPP_INFO(this->get_logger(),"Trajectory type is trapezoidal_waypoints.");
      if (!check_trajectory(goal->time,goal->x,goal->z)) {
        RCLCPP_WARN(this->get_logger(),"FollowPositionTrajectory: trajectory invalid.");
        return rclcpp_action::GoalResponse::REJECT; 
      }
    } 
    else {
      RCLCPP_WARN(this->get_logger(),"FollowPositionTrajectory: type %s unsupported.",goal->type.c_str());
      return rclcpp_action::GoalResponse::REJECT; 
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Forbal2Controller::handle_cancel(const std::shared_ptr<GoalHandleFollowPositionTrajectory> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Canceling goal");
  goal_handle->canceled(std::make_shared<FollowPositionTrajectory::Result>());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Forbal2Controller::handle_accepted(const std::shared_ptr<GoalHandleFollowPositionTrajectory> goal_handle) {
  std::thread([this, goal_handle]() {execute(goal_handle); }).detach();
}

void Forbal2Controller::execute(const std::shared_ptr<GoalHandleFollowPositionTrajectory> goal_handle) {
  // const auto goal = goal_handle->get_goal();
  // RCLCPP_INFO(this->get_logger(), "Executing goal");
  auto goal = goal_handle->get_goal();
  tk::spline sx(goal->time,goal->x,
                tk::spline::cspline_hermite,false,
                tk::spline::first_deriv,0.0,
                tk::spline::first_deriv,0.0);
  tk::spline sz(goal->time,goal->z,
                tk::spline::cspline_hermite,false,
                tk::spline::first_deriv,0.0,
                tk::spline::first_deriv,0.0);

  float dt;
  if (goal->dt <= 0.0f) {
    RCLCPP_WARN(this->get_logger(),"FollowPositionTrajectory: dt not specified or incorrent, setting to 0.01");
    dt = 0.01;
  } else {
    dt = goal->dt;
  }
  auto trajectory_goal = control_msgs::action::FollowJointTrajectory::Goal();
  trajectory_goal.trajectory.joint_names = {joint_1_act_, joint_2_act_};

  double tMax = *std::max_element(goal->time.begin(),goal->time.end());
  double x;
  double z;
  JointAngles q;

  rclcpp::Time stamp = this->now();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = stamp;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.ns = "position_trajectory";
  marker.scale.x = 0.002;
  marker.color.r = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(tMax+2.0);
  
  if (goal->type == "constant_waypoints") {
    for (double t = 0.0; t <= tMax; t += dt) {
      x = sx(t);
      z = sz(t);
      q = inverse_kinematics(x,z);
      geometry_msgs::msg::Point point;

      point.x = x;
      point.y = 0;
      point.z = z;
      marker.points.push_back(point);
      // RCLCPP_INFO(this->get_logger(),"t = %f, x = %f, z = %f",t,x,sz(t));

      auto joint = trajectory_msgs::msg::JointTrajectoryPoint();
      joint.positions = {q.theta1a, q.theta2a};
      joint.time_from_start = rclcpp::Duration::from_seconds(t);
      trajectory_goal.trajectory.points.push_back(joint);
    }
  }
  else if (goal->type == "trapezoidal_waypoints") {
    double d_total = total_spline_distance(sx, sz, tMax, dt);
    float acc_time_ = goal->acc_time;
    RCLCPP_INFO(this->get_logger(), "Total spline distance: %f", d_total);
    RCLCPP_INFO(this->get_logger(),"Acceleration time in goal: %f", acc_time_);

    double t_const = tMax - 2 * acc_time_; // Time at max velocity
    double v_max = 2.0*d_total / (tMax + t_const); // using trapezoid area as d_total
    double a_max = v_max / acc_time_; // gradient of acceleration
    RCLCPP_INFO(this->get_logger(), "t_const: %f, d_total: %f, v_max: %f, a_max: %f",t_const,d_total,v_max,a_max);

    if (acc_time_ >= tMax / 2.0) {
      RCLCPP_ERROR(this->get_logger(), "acc_time is too large for the given tMax!");
      goal_handle->abort(std::make_shared<FollowPositionTrajectory::Result>());
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
      double z = sz(s*tMax/d_total);
      RCLCPP_INFO(this->get_logger(), "t: %f, st: %f, x: %f, z: %f",t,s*tMax/d_total,x,z);

      // Compute joint angles using inverse kinematics
      JointAngles q = inverse_kinematics(x, z);

      // Add point to visualization marker
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = 0;
      point.z = z;
      marker.points.push_back(point);

      // Add joint positions to trajectory
      auto joint = trajectory_msgs::msg::JointTrajectoryPoint();
      joint.positions = {q.theta1a, q.theta2a};
      joint.time_from_start = rclcpp::Duration::from_seconds(t);
      trajectory_goal.trajectory.points.push_back(joint);
    }
  } 

  marker_pub_->publish(marker);

  // if (!joint_trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
  //     RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server unavailable");
  //     goal_handle->abort(std::make_shared<FollowPositionTrajectory::Result>());
  //     return;
  // }

  if (!joint_trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server not available");
    auto result = std::make_shared<FollowPositionTrajectory::Result>();
    result->error_code = result->JOINT_TRAJECTORY_ERROR;
    result->error_string = "Action server not available";
    goal_handle->abort(result);
    return;
  }

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  auto time_start = this->now();
  send_goal_options.feedback_callback = [goal_handle, this, tMax, time_start](auto, auto feedback) {
      auto fb = std::make_shared<FollowPositionTrajectory::Feedback>();
      fb->header = feedback->header;
      fb->position_error = 0.0;
      goal_handle->publish_feedback(fb);
  };

  auto future = joint_trajectory_client_->async_send_goal(trajectory_goal, send_goal_options);
  auto traj_goal_handle = future.get();
  if (!traj_goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory goal was rejected");
      auto result = std::make_shared<FollowPositionTrajectory::Result>();
      result->error_code = result->JOINT_TRAJECTORY_ERROR;
      result->error_string = "Goal rejected by server";
      goal_handle->abort(result);
      return;
  }

  auto result_future = joint_trajectory_client_->async_get_result(future.get());

  if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
      auto result = std::make_shared<FollowPositionTrajectory::Result>();
      result->error_code = result->JOINT_TRAJECTORY_ERROR;
      result->error_string = "Trajectory failed.";
      goal_handle->abort(std::make_shared<FollowPositionTrajectory::Result>());
  } else {
      RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded");
      auto result = std::make_shared<FollowPositionTrajectory::Result>();
      result->error_code = result->SUCCESSFUL;
      goal_handle->succeed(result);
  }
}

double Forbal2Controller::total_spline_distance(const tk::spline& sx, const tk::spline& sz, double tMax, float dt) {
  double d_total = 0.0;
  double x_prev = sx(0.0);
  double z_prev = sz(0.0);
  double x_curr;
  double z_curr;

  for (double t = 0.0 + dt; t <= tMax; t += dt) {
      x_curr = sx(t);
      z_curr = sz(t);
      d_total += sqrt(pow(x_curr - x_prev, 2) + pow(z_curr - z_prev, 2));
      x_prev = x_curr;
      z_prev = z_curr;
  }
  return d_total;
}

Forbal2Controller::JointAngles Forbal2Controller::forward_kinematics(double theta1a, double theta2a) {
  double alpha = theta2a+theta1a;
  double beta = M_PI-alpha;
  JointAngles ja;
  ja.theta1a = theta1a;
  ja.theta2a = theta2a;
  ja.theta1p = M_PI-beta;
  ja.theta2p = M_PI-beta;
  return ja;
}

Forbal2Controller::Point Forbal2Controller::ee_position(Forbal2Controller::JointAngles q) {
  Forbal2Controller::Point p;
  p.x = l_*cos(q.theta1a)-(l_+le_)*cos(q.theta1a+q.theta1p);
  p.z = l_*sin(q.theta1a)-(l_+le_)*sin(q.theta1a+q.theta1p);
  p.y = 0;
  return p;
}

Forbal2Controller::JointAngles Forbal2Controller::inverse_kinematics(double x, double z) {
  double xd = x-jx_;
  double zd = z-jz_;
  
  JointAngles ja;
  double lE0 = sqrt(pow(xd,2)+pow(zd,2));
  if (lE0 > 2*l_+le_) {
    // RCLCPP_WARN(this->get_logger(),"Overextension for reference '%f' '%f'.",x,z);
    ja.theta1a = NAN;
    ja.theta2a = NAN;
    ja.theta1p = NAN;
    ja.theta2p = NAN;
    return ja;
    }
  double thetaE0 = atan(zd/xd);
  double beta = acos((pow(l_,2)+pow(l_+le_,2)-pow(lE0,2))/(2*l_*(l_+le_)));
  double gamma = acos((pow(lE0,2)+pow(l_,2)-pow(l_+le_,2))/(2*l_*lE0));
  double alpha = M_PI-beta;
  
  ja.theta2a = gamma+thetaE0;
  ja.theta1a = alpha-ja.theta2a;
  ja.theta1p = M_PI-beta;
  ja.theta2p = M_PI-beta;
  return ja;
}

bool Forbal2Controller::check_joint_limits(JointAngles ja) {
  bool success = true;
  if (ja.theta1a > j1a_max_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 1a of %f is above than the maximum limit of %f",ja.theta1a,j1a_max_);
    success = false;
  }
  else if (ja.theta1a < j1a_min_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 1a of %f is below than the minimum limit of %f",ja.theta1a,j1a_min_);
  };
  if (ja.theta2a > j2a_max_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 2a of %f is above than the maximum limit of %f",ja.theta2a,j2a_max_);
    success = false;
  }
  else if (ja.theta2a < j2a_min_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 2a of %f is below than the minimum limit of %f",ja.theta2a,j2a_min_);
    success = false;
  };
  if (ja.theta1p > j1p_max_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 1p of %f is above than the maximum limit of %f",ja.theta1p,j1p_max_);
    success = false;
  }
  else if (ja.theta1p < j1p_min_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 1p of %f is below than the minimum limit of %f",ja.theta1p,j1p_min_);
    success = false;
  };
  if (ja.theta2p > j2p_max_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 2p of %f is above than the maximum limit of %f",ja.theta2p,j2p_max_);
    success = false;
  }
  else if (ja.theta2p < j2p_min_) {
    RCLCPP_ERROR(this->get_logger(),"Angle 2p of %f is below than the minimum limit of %f",ja.theta2p,j2p_min_);
    success = false;
  };
  return success;
}

bool Forbal2Controller::check_trajectory(
  const std::vector<double>& time, 
  const std::vector<double>& x, 
  const std::vector<double> z) {

    for (size_t i = 0; i < time.size(); ++i) {
          JointAngles ja = inverse_kinematics(x[i],z[i]);
          if (std::isnan(ja.theta1a) || std::isnan(ja.theta2a)) {
            RCLCPP_WARN(this->get_logger(),"Point %i overextends.",int(i));
            return false;
          }
          if (check_joint_limits(ja)) {
            continue;
          } else {
            RCLCPP_WARN(this->get_logger(), "Point %i exceeds joint limits.",int(i));
            return false;
          }
        }
    return true;
  }

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Forbal2Controller>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
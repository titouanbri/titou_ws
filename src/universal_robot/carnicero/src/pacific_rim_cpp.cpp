#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <controller_manager_msgs/SwitchController.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

class PRControl
{
public:
  PRControl();
  void run();
  bool switchControllers(const std::vector<std::string>& start_list,
                         const std::vector<std::string>& stop_list);
  void initOffsetAndLatch();

private:
  void wristCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  bool getPose(const std::string& frame, Eigen::Matrix<double,7,1>& pose);

  ros::NodeHandle nh_;
  std::string tool_frame_;
  std::string base_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber wrist_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_vel_pub_;

  sensor_msgs::JointStateConstPtr joint_state_;
  Eigen::Vector3d wrist_real_;
  bool has_wrist_real_;

  Eigen::Vector3d ref_pos_;
  Eigen::Vector3d offset_;
  Eigen::Vector3d I_pos_;

  double freq_;
  double dt_;
  double Kp_pos_;
  double Kd_pos_;
  double Ki_pos_;
  double I_term_max_;
  double I_leak_;
  double lambda_dls_;
  double V_alpha_;
  double v_lin_max_;
  double qdot_max_;
  double pos_tol_update_;
  double pos_deadband_;

  std::vector<std::string> joint_names_;
  std_msgs::Float64MultiArray vel_msg_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> kdl_jnt_to_jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;

  Eigen::Vector3d last_p_real_;
  bool has_last_p_real_;
  Eigen::VectorXd qdot_filt_;
};

PRControl::PRControl() :
  tf_listener_(tf_buffer_),
  has_wrist_real_(false),
  has_last_p_real_(false)
{
  tool_frame_ = "wrist_3_link";
  base_frame_ = "base_link";

  wrist_sub_ = nh_.subscribe("right_arm/wrist", 1, &PRControl::wristCallback, this);
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &PRControl::jointStateCallback, this);

  ref_pos_.setZero();
  offset_.setZero();
  I_pos_.setZero();

  freq_ = 350.0;
  dt_ = 1.0 / freq_;
  Kp_pos_ = 0.25;
  Kd_pos_ = 0.8;
  Ki_pos_ = 0.02;
  I_term_max_ = 0.10;
  I_leak_ = 0.0;
  lambda_dls_ = 0.18;
  V_alpha_ = 0.1;
  v_lin_max_ = 0.08;
  qdot_max_ = 0.8;
  pos_tol_update_ = 0.005;
  pos_deadband_ = 0.005;

  urdf::Model robot;
  robot.initParam("/robot_description");
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(robot, tree);
  tree.getChain(base_frame_, tool_frame_, kdl_chain_);
  kdl_jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  joint_names_ = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                  "wrist_1_joint","wrist_2_joint","wrist_3_joint"};

  joint_vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_vel_controller/command",1);
  vel_msg_.data.assign(6,0.0);

  qdot_filt_.resize(6);
  qdot_filt_.setZero();
}

void PRControl::wristCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  wrist_real_(0) = msg->point.x;
  wrist_real_(1) = -msg->point.z;
  wrist_real_(2) = msg->point.y;
  has_wrist_real_ = true;
}

void PRControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state_ = msg;
}

bool PRControl::getPose(const std::string& frame, Eigen::Matrix<double,7,1>& pose)
{
  try
  {
    geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
      base_frame_, frame, ros::Time(0), ros::Duration(0.1));
    pose << tf_stamped.transform.translation.x,
            tf_stamped.transform.translation.y,
            tf_stamped.transform.translation.z,
            tf_stamped.transform.rotation.x,
            tf_stamped.transform.rotation.y,
            tf_stamped.transform.rotation.z,
            tf_stamped.transform.rotation.w;
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("TF fail %s: %s", frame.c_str(), ex.what());
    return false;
  }
}

void PRControl::initOffsetAndLatch()
{
  std::vector<Eigen::Vector3d> buf;
  int i = 0;
  ros::Rate r(100);
  while (i < 5 && ros::ok())
  {
    if (!has_wrist_real_)
    {
      r.sleep();
      ros::spinOnce();
      continue;
    }
    Eigen::Matrix<double,7,1> x;
    if (!getPose(tool_frame_, x))
    {
      r.sleep();
      ros::spinOnce();
      continue;
    }
    buf.push_back(-wrist_real_ + x.head<3>());
    i++;
    r.sleep();
    ros::spinOnce();
  }
  if (!buf.empty())
  {
    offset_.setZero();
    for (const auto& v : buf)
      offset_ += v;
    offset_ /= static_cast<double>(buf.size());
  }
  ROS_INFO_STREAM("Offset estimé: " << offset_.transpose());
  ref_pos_ = wrist_real_ + offset_;
  I_pos_.setZero();
  ROS_INFO_STREAM("Consigne latched (pos=" << ref_pos_.transpose() << ")");
}

bool PRControl::switchControllers(const std::vector<std::string>& start_list,
                                  const std::vector<std::string>& stop_list)
{
  ros::ServiceClient client = nh_.serviceClient<controller_manager_msgs::SwitchController>(
    "controller_manager/switch_controller");
  client.waitForExistence();
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = start_list;
  srv.request.stop_controllers = stop_list;
  srv.request.strictness = 1;
  if (client.call(srv))
    return srv.response.ok;
  ROS_ERROR("Switch controller fail");
  return false;
}

void PRControl::run()
{
  ros::Rate rate(freq_);
  std_msgs::Float64MultiArray vel_msg;
  vel_msg.data.assign(6,0.0);

  while (ros::ok())
  {
    if (!joint_state_ || !has_wrist_real_)
    {
      rate.sleep();
      ros::spinOnce();
      continue;
    }
    std::vector<double> q(joint_names_.size());
    bool ok = true;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto it = std::find(joint_state_->name.begin(), joint_state_->name.end(), joint_names_[i]);
      if (it == joint_state_->name.end())
      {
        ok = false;
        break;
      }
      q[i] = joint_state_->position[it - joint_state_->name.begin()];
    }
    if (!ok)
    {
      rate.sleep();
      ros::spinOnce();
      continue;
    }

    KDL::JntArray jnt_array(q.size());
    for (size_t i = 0; i < q.size(); ++i)
      jnt_array(i) = q[i];
    KDL::Jacobian Jkdl(q.size());
    kdl_jnt_to_jac_solver_->JntToJac(jnt_array, Jkdl);
    Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, q.size());
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < (int)q.size(); ++c)
        J6(r,c) = Jkdl(r,c);
    Eigen::MatrixXd J = J6.topRows(3);

    Eigen::Matrix<double,7,1> x_real;
    if (!getPose(tool_frame_, x_real))
    {
      rate.sleep();
      ros::spinOnce();
      continue;
    }
    Eigen::Vector3d p_real = x_real.head<3>();

    Eigen::Vector3d p_tgt = wrist_real_ + offset_;
    if ((p_tgt - ref_pos_).norm() > pos_tol_update_)
    {
      ref_pos_ = p_tgt;
      I_pos_.setZero();
      ROS_INFO("Nouvelle consigne (Δpos: %.1f mm)", 1000.0*(p_tgt - p_real).norm());
    }

    Eigen::Vector3d e_pos = ref_pos_ - p_real;

    Eigen::Vector3d v_meas;
    if (!has_last_p_real_)
      v_meas.setZero();
    else
      v_meas = (p_real - last_p_real_) / dt_;

    Eigen::Vector3d v_cmd;
    if (e_pos.norm() < pos_deadband_)
    {
      v_cmd.setZero();
      I_pos_.setZero();
    }
    else
    {
      I_pos_ += e_pos * dt_;
      if (I_leak_ > 0.0)
        I_pos_ *= (1.0 - I_leak_ * dt_);
      Eigen::Vector3d I_term = Eigen::Vector3d::Zero();
      if (Ki_pos_ > 0.0)
      {
        I_term = Ki_pos_ * I_pos_;
        for (int i = 0; i < 3; ++i)
        {
          I_term[i] = std::max(-I_term_max_, std::min(I_term[i], I_term_max_));
          double limit = I_term_max_ / Ki_pos_;
          I_pos_[i] = std::max(-limit, std::min(I_pos_[i], limit));
        }
      }
      v_cmd = Kp_pos_ * e_pos + I_term - Kd_pos_ * v_meas;
      for (int i = 0; i < 3; ++i)
        v_cmd[i] = std::max(-v_lin_max_, std::min(v_cmd[i], v_lin_max_));
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd S = svd.singularValues();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::VectorXd S_damped = S.array() / (S.array().square() + lambda_dls_*lambda_dls_);
    Eigen::MatrixXd J_pinv = V * S_damped.asDiagonal() * U.transpose();
    Eigen::VectorXd qdot = J_pinv * v_cmd;

    if (qdot_filt_.size() != qdot.size())
      qdot_filt_ = qdot;
    else
      qdot_filt_ = V_alpha_ * qdot + (1.0 - V_alpha_) * qdot_filt_;
    for (int i = 0; i < qdot_filt_.size(); ++i)
      qdot_filt_[i] = std::max(-qdot_max_, std::min(qdot_filt_[i], qdot_max_));

    vel_msg.data.clear();
    for (int i = 0; i < qdot_filt_.size(); ++i)
      vel_msg.data.push_back(qdot_filt_[i]);
    joint_vel_pub_.publish(vel_msg);

    last_p_real_ = p_real;
    has_last_p_real_ = true;
    ROS_INFO_STREAM_THROTTLE(0.5,
      "||e_pos||=" << (e_pos.norm()*1000.0) << " mm | ||v_cmd||=" << (v_cmd.norm()*1000.0) << " mm/s");

    rate.sleep();
    ros::spinOnce();
  }

  Eigen::VectorXd vals = qdot_filt_;
  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    vals /= 1.03;
    vel_msg.data.clear();
    for (int j = 0; j < vals.size(); ++j)
      vel_msg.data.push_back(vals[j]);
    joint_vel_pub_.publish(vel_msg);
    ros::Duration(0.0025).sleep();
  }
  vel_msg.data.assign(6,0.0);
  joint_vel_pub_.publish(vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PR");
  try
  {
    PRControl ctrl;
    ctrl.switchControllers({"joint_group_vel_controller"}, {"scaled_pos_joint_traj_controller"});
    ROS_INFO("Init offset + latch...");
    ctrl.initOffsetAndLatch();
    ROS_INFO("Go.");
    ctrl.run();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception: %s", e.what());
  }
  return 0;
}


// file: src/admittance_control.cpp

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager_msgs/SwitchController.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <cstdlib>   // system()
#include <algorithm>

class AdmittanceControl {
public:
  AdmittanceControl(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    tf_buffer_(),
    tf_listener_(tf_buffer_) {

    // Frames
    tool_frame_ = "tool0";
    base_frame_ = "base_link";

    // Param capteur
    pnh_.param<std::string>("sensor", which_sensor_, std::string("serial_rkb"));

    // Abonnements force selon le capteur
    if (which_sensor_ == "serial_rkb" || which_sensor_ == "serial_big") {
      sub_wrench_ = nh_.subscribe("/force_sensor", 1, &AdmittanceControl::forceCb, this);
    } else if (which_sensor_ == "UR") {
      sub_wrench_ = nh_.subscribe("/wrench", 1, &AdmittanceControl::forceCb, this);
    } else if (which_sensor_ == "ethercat") {
      sub_wrench_ = nh_.subscribe("/force_sensor_eth", 1, &AdmittanceControl::forceCb, this);
    } else {
      ROS_WARN_STREAM("Unknown sensor '" << which_sensor_ << "', defaulting to /force_sensor");
      sub_wrench_ = nh_.subscribe("/force_sensor", 1, &AdmittanceControl::forceCb, this);
    }

    sub_jstate_ = nh_.subscribe("/joint_states", 1, &AdmittanceControl::jointStateCb, this);

    pub_joint_vel_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    // Correction rotation selon le capteur
    double zcorr = 0.0;
    if (which_sensor_ == "serial_rkb" || which_sensor_ == "ethercat" || which_sensor_ == "serial_big") {
      zcorr = M_PI/2.0;
    } // else "UR" => 0
    correction_ = rotZ(zcorr);

    // KDL: construction de la chaîne base -> tool
    std::string robot_description;
    if (!nh_.getParam("/robot_description", robot_description)) {
      ROS_ERROR("Cannot read /robot_description from parameter server.");
      throw std::runtime_error("No robot_description");
    }
    urdf::Model model;
    if (!model.initString(robot_description)) {
      ROS_ERROR("Failed to parse URDF.");
      throw std::runtime_error("URDF parse error");
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      ROS_ERROR("Failed to construct KDL tree from URDF.");
      throw std::runtime_error("KDL tree error");
    }
    if (!tree.getChain(base_frame_, tool_frame_, kdl_chain_)) {
      ROS_ERROR_STREAM("Failed to get KDL chain from " << base_frame_ << " to " << tool_frame_);
      throw std::runtime_error("KDL chain error");
    }
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Noms d'articulations (UR typique)
    joint_names_ = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    // États init 6D
    v_k_.setZero(6);
    x_k_.setZero(6);
    a_k_.setZero(6);

    last_R_valid_ = false;

    ROS_INFO_STREAM("Admittance node ready. Sensor='" << which_sensor_ << "'");
  }

  // Switch contrôleurs
  bool switchControllers(const std::vector<std::string>& start_list,
                         const std::vector<std::string>& stop_list) {
    ros::ServiceClient cl =
        nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = start_list;
    srv.request.stop_controllers = stop_list;
    srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    if (!cl.call(srv)) {
      ROS_ERROR("SwitchController call failed.");
      return false;
    }
    if (!srv.response.ok) {
      ROS_WARN("SwitchController returned not ok.");
      return false;
    }
    return true;
  }

  // Boucle principale d’admittance
  void run() {
    ROS_INFO("Starting full-body admittance control (6D)...");
    const double c = 15.0;
    const double M = 8.0, B = c*M, K = 0.0;          // translation
    const double M_rot = 0.07, B_rot = c*M_rot, K_rot = 0.0; // rotation
    const double freq = 350.0; // Hz
    const double dt = 1.0 / freq;

    const double F_alpha = 0.015;   // filtre expo forces
    const double V_alpha = 0.15;    // filtre expo vitesses articulaires
    const double force_dead_cart = 0.05;
    const double force_dead_rot  = 0.003;

    const double lambda_dls = 0.015; // amortissement pseudo-inverse

    Eigen::VectorXd filtered_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd smoothed_rel   = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_velocity_smoothed; // non initialisé au départ
    bool vel_init = false;

    Eigen::VectorXd B_vec(6), K_vec(6), M_vec(6);
    B_vec << B, B, B, B_rot, B_rot, B_rot;
    K_vec << K, K, K, K_rot, K_rot, K_rot;
    M_vec << M, M, M, M_rot, M_rot, M_rot;

    ros::Rate rate(freq);

    while (ros::ok()) {
      ros::spinOnce();

      if (!has_wrench_ || !has_joint_state_) {
        rate.sleep();
        continue;
      }

      geometry_msgs::WrenchStamped wrench_base;
      if (!transformWrenchToBaseFrame(last_wrench_, wrench_base)) {
        rate.sleep();
        continue;
      }

      // Force/torque actuelles
      Eigen::VectorXd f(6);
      f << wrench_base.wrench.force.x,
           wrench_base.wrench.force.y,
           wrench_base.wrench.force.z,
           wrench_base.wrench.torque.x,
           wrench_base.wrench.torque.y,
           wrench_base.wrench.torque.z;

      // Filtre exponentiel sur les forces
      smoothed_rel = (1.0 - F_alpha) * smoothed_rel + F_alpha * f;
      filtered_force = smoothed_rel;

      // Zones mortes
      if (filtered_force.head<3>().norm() < force_dead_cart) {
        filtered_force(0) = 0.0; filtered_force(1) = 0.0; filtered_force(2) = 0.0;
      }
      if (filtered_force.tail<3>().norm() < force_dead_rot) {
        filtered_force(3) = 0.0; filtered_force(4) = 0.0; filtered_force(5) = 0.0;
      }

      // Récup joint positions dans l’ordre joint_names_
      std::vector<double> q_vec(joint_names_.size(), 0.0);
      {
        // map name -> index
        std::map<std::string, size_t> name_to_idx;
        for (size_t i = 0; i < last_jstate_.name.size(); ++i)
          name_to_idx[last_jstate_.name[i]] = i;
        for (size_t j = 0; j < joint_names_.size(); ++j) {
          auto it = name_to_idx.find(joint_names_[j]);
          if (it != name_to_idx.end()) {
            q_vec[j] = last_jstate_.position[it->second];
          } else {
            // si introuvable, on garde 0
          }
        }
      }

      // Jacobienne via KDL
      KDL::JntArray q(kdl_chain_.getNrOfJoints());
      for (size_t i = 0; i < q_vec.size(); ++i) q(i) = q_vec[i];
      KDL::Jacobian jac(kdl_chain_.getNrOfJoints());
      if (jac_solver_->JntToJac(q, jac) < 0) {
        rate.sleep(); continue;
      }
      Eigen::MatrixXd J(6, kdl_chain_.getNrOfJoints());
      for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < kdl_chain_.getNrOfJoints(); ++j)
          J(i, j) = jac(i, j);

      // Admittance discrète
      a_k_ = (filtered_force - B_vec.cwiseProduct(v_k_) - K_vec.cwiseProduct(x_k_)).cwiseQuotient(M_vec);
      v_k_ = v_k_ + a_k_ * dt;
      x_k_ = v_k_ * dt; // K=0 => déplacement non utilisé ici

      Eigen::VectorXd ee_vel = v_k_; // 6D

      // Pseudo-inverse amortie par SVD
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::VectorXd S = svd.singularValues();
      Eigen::MatrixXd U = svd.matrixU();
      Eigen::MatrixXd V = svd.matrixV();

      Eigen::VectorXd Sd = S;
      for (int i = 0; i < Sd.size(); ++i) {
        Sd(i) = S(i) / (S(i)*S(i) + lambda_dls*lambda_dls);
      }
      Eigen::MatrixXd J_pinv = V * Sd.asDiagonal() * U.transpose();

      Eigen::VectorXd qdot = J_pinv * ee_vel;

      // Filtre expo vitesse articulations
      if (!vel_init) {
        joint_velocity_smoothed = qdot;
        vel_init = true;
      } else {
        joint_velocity_smoothed = V_alpha * qdot + (1.0 - V_alpha) * joint_velocity_smoothed;
      }

      // Publication
      std_msgs::Float64MultiArray cmd;
      cmd.data.resize(joint_velocity_smoothed.size());
      for (int i = 0; i < joint_velocity_smoothed.size(); ++i) cmd.data[i] = joint_velocity_smoothed(i);
      pub_joint_vel_.publish(cmd);

      rate.sleep();
    }

    // Arrêt propre: rampe à zéro
    std_msgs::Float64MultiArray cmd;
    cmd.data = {0,0,0,0,0,0};
    for (int k = 0; k < 100; ++k) {
      for (double &v : cmd.data) v /= 1.03;
      pub_joint_vel_.publish(cmd);
      ros::Duration(0.0025).sleep();
    }
    cmd.data = {0,0,0,0,0,0};
    pub_joint_vel_.publish(cmd);
  }

  // Lancement/kill des publishers capteurs (optionnel – comme dans le Python)
  void manageSensorPublishers() {
    if (which_sensor_ == "ethercat") {
      std::ignore = std::system("pkill -f force_sensor_eth_publisher.py");
      std::ignore = std::system("rosrun carnicero force_sensor_eth_publisher.py &");
    } else if (which_sensor_ == "serial_rkb" || which_sensor_ == "serial_big") {
      std::ignore = std::system("pkill -f force_sensor_publisher.py");
      std::ignore = std::system("rosrun carnicero force_sensor_publisher.py &");
    }
  }

  const std::string& sensorKind() const { return which_sensor_; }

private:
  // Callbacks
  void forceCb(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    last_wrench_ = *msg;
    has_wrench_ = true;
  }
  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
    last_jstate_ = *msg;
    has_joint_state_ = true;
  }

  // Récup position d’un lien pour sécurité spatiale (non utilisée par défaut)
  bool getLinkPosition(const std::string& link, Eigen::Vector3d& p_out) {
    try {
      geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(base_frame_, link, ros::Time(0), ros::Duration(0.1));
      p_out = Eigen::Vector3d(tf.transform.translation.x,
                              tf.transform.translation.y,
                              tf.transform.translation.z);
      return true;
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("TF lookup failed for " << link << ": " << e.what());
      return false;
    }
  }

  // Transforme la wrench capteur -> base_link, avec filtrage des quaternions "cassés"
  bool transformWrenchToBaseFrame(const geometry_msgs::WrenchStamped& in,
                                  geometry_msgs::WrenchStamped& out) {
    const double seuil_norme_q = 0.1;

    geometry_msgs::TransformStamped tfst;
    try {
      tfst = tf_buffer_.lookupTransform(base_frame_, tool_frame_, ros::Time(0), ros::Duration(0.05));
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("TF error base->tool: " << e.what());
      return false;
    }

    // Quaternion -> matrice
    tf2::Quaternion q;
    tf2::fromMsg(tfst.transform.rotation, q);
    tf2::Matrix3x3 tfm(q);

    Eigen::Matrix3d R_test;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R_test(r,c) = tfm[r][c];

    bool use_current = true;
    if (last_R_valid_) {
      double diff = (R_test - last_R_).norm(); // norme de Frobenius
      if (diff > seuil_norme_q) {
        use_current = false;
      }
    }

    Eigen::Matrix3d R_base_tool;
    if (use_current) {
      R_base_tool = R_test * correction_; // applique correction capteur
      last_R_ = R_test;
      last_R_valid_ = true;
    } else {
      R_base_tool = last_R_ * correction_;
    }

    // Force/torque dans le repère capteur
    Eigen::Vector3d F(in.wrench.force.x,  in.wrench.force.y,  in.wrench.force.z);
    Eigen::Vector3d T(in.wrench.torque.x, in.wrench.torque.y, in.wrench.torque.z);

    // (option) déport: d x F (conservé comme dans le script Python; non réutilisé ensuite)
    Eigen::Vector3d d(0.0, 0.05, 0.07);
    Eigen::Vector3d T_handle = T + d.cross(F);
    (void)T_handle; // évite un warning si non utilisé

    Eigen::Vector3d Fb = R_base_tool * F;
    Eigen::Vector3d Tb = R_base_tool * T;

    out.header = in.header;
    out.header.frame_id = tool_frame_; // même que Python
    out.wrench.force.x  = Fb.x();
    out.wrench.force.y  = Fb.y();
    out.wrench.force.z  = Fb.z();
    out.wrench.torque.x = Tb.x();
    out.wrench.torque.y = Tb.y();
    out.wrench.torque.z = Tb.z();

    return true;
  }

  // Rotation autour de Z
  static Eigen::Matrix3d rotZ(double ang) {
    double c = std::cos(ang), s = std::sin(ang);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_wrench_, sub_jstate_;
  ros::Publisher  pub_joint_vel_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string which_sensor_;
  std::string tool_frame_, base_frame_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver>        jac_solver_;

  // États
  bool has_wrench_ = false, has_joint_state_ = false;
  geometry_msgs::WrenchStamped last_wrench_;
  sensor_msgs::JointState      last_jstate_;

  // Admittance
  Eigen::VectorXd v_k_, x_k_, a_k_;

  // Correction & filtrage quaternion
  Eigen::Matrix3d correction_;
  Eigen::Matrix3d last_R_;
  bool last_R_valid_;

  // Joints
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "admittance");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    ROS_INFO("node initialisation");
    AdmittanceControl ac(nh, pnh);

    // (optionnel) gérer les publishers capteurs externes comme dans le script Python
    ac.manageSensorPublishers();

    // Switch contrôleurs (par précaution)
    std::vector<std::string> start{"joint_group_vel_controller"};
    std::vector<std::string> stop{"scaled_pos_joint_traj_controller"};
    ac.switchControllers(start, stop);

    // Lancer l’admittance
    ac.run();
    ROS_INFO("end of the admittance");
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
  }
  return 0;
}

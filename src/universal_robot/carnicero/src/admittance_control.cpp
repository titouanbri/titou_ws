// Admittance control (6D) — C++/ROS1 port of the provided Python script
// Author: ChatGPT (portage depuis le code Python fourni)
//
// Package deps (catkin): roscpp, geometry_msgs, std_msgs, sensor_msgs,
// controller_manager_msgs, tf2_ros, tf2_geometry_msgs, urdf, kdl_parser,
// orocos-kdl, eigen
//
// Build: add this file to a catkin package and link against the libs above.
// Example CMakeLists additions:
// -------------------------------------------------------------------------
// find_package(catkin REQUIRED COMPONENTS
//   roscpp geometry_msgs std_msgs sensor_msgs controller_manager_msgs
//   tf2_ros tf2_geometry_msgs urdf kdl_parser
// )
// find_package(orocos_kdl REQUIRED)
// find_package(Eigen3 REQUIRED)
// include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
// add_executable(admittance_control_node src/admittance_control.cpp)
// target_link_libraries(admittance_control_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
// -------------------------------------------------------------------------

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager_msgs/SwitchController.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <vector>
#include <numeric>
#include <cstdlib>
#include <cmath>

class AdmittanceControl {
public:
  AdmittanceControl() :
    nh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    have_force_(false),
    have_js_(false),
    have_last_rot_(false)
  {
    // Frames
    tool_frame_ = "tool0";
    base_frame_ = "base_link";

    // Subscribers
    sub_force_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/force_sensor_eth", 1,
                   &AdmittanceControl::forceCb, this);
    sub_js_    = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                   &AdmittanceControl::jointStateCb, this);

    // Publisher
    pub_vel_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    // Load URDF & build KDL chain
    std::string robot_description;
    if (!ros::param::get("/robot_description", robot_description)) {
      ROS_FATAL("Param /robot_description introuvable.");
      throw std::runtime_error("robot_description not found");
    }

    urdf::Model model;
    if (!model.initString(robot_description)) {
      ROS_FATAL("Impossible de parser l'URDF.");
      throw std::runtime_error("URDF parse error");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      ROS_FATAL("kdl_parser: échec de la conversion URDF -> Tree.");
      throw std::runtime_error("KDL tree error");
    }

    if (!tree.getChain(base_frame_, tool_frame_, kdl_chain_)) {
      ROS_FATAL_STREAM("KDL: échec getChain(" << base_frame_ << ", " << tool_frame_ << ")");
      throw std::runtime_error("KDL chain error");
    }

    jnt_to_jac_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    // Constant correction (sensor -> tool) : rotation Z de -pi/2
    Eigen::AngleAxisd corr(-M_PI/2.0, Eigen::Vector3d::UnitZ());
    correction_ = corr.toRotationMatrix();

    joint_names_ = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    v_k_.setZero(6);
    x_k_.setZero(6);
    a_k_.setZero(6);

    ROS_INFO("AdmittanceControl initialisé.");
  }

  // Switch controllers: start_list/stop_list
  bool switchControllers(const std::vector<std::string>& start_list,
                         const std::vector<std::string>& stop_list) {
    ros::ServiceClient client = nh_.serviceClient<controller_manager_msgs::SwitchController>(
        "/controller_manager/switch_controller");
    if (!client.waitForExistence(ros::Duration(5.0))) {
      ROS_ERROR("Service switch_controller indisponible.");
      return false;
    }
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = start_list;
    srv.request.stop_controllers  = stop_list;
    srv.request.strictness = 1; // BEST_EFFORT
    if (!client.call(srv)) {
      ROS_ERROR("Appel service switch_controller a échoué.");
      return false;
    }
    return srv.response.ok;
  }

  void spin() {
    ROS_INFO("Démarrage contrôle d'admittance full-body (6D)...");

    // Params d'admittance
    const double c = 12.0;
    const double M = 6.0, B = c*M, K = 0.0;            // translation
    const double M_rot = 0.06, B_rot = c*M_rot, K_rot = 0.0; // rotation
    const double freq = 1000.0; // Hz
    const double dt = 1.0 / freq;

    const double force_dead_cart = 0.08;
    const double force_dead_rot  = 0.005;

    const double F_alpha = 0.015; // Filtre exp. sur forces
    const double V_alpha = 0.15; // Filtre exp. sur vitesses

    Eigen::VectorXd filtered_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd smoothed_rel   = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd B_vec(6), K_vec(6), M_vec(6);
    B_vec << B, B, B, B_rot, B_rot, B_rot;
    K_vec << K, K, K, K_rot, K_rot, K_rot;
    M_vec << M, M, M, M_rot, M_rot, M_rot;

    const double lambda_dls = 0.015; // Damping near singularities

    std_msgs::Float64MultiArray vel_msg;
    vel_msg.data.resize(6, 0.0);

    ros::Rate rate(freq);

    // Attendre données initiales
    while (ros::ok() && (!have_force_ || !have_js_)) {
      ros::spinOnce();
      rate.sleep();
    }

    Eigen::VectorXd joint_velocity_smoothed; // non initialisé tant que 1ère valeur pas connue

    while (ros::ok()) {
      ros::spinOnce();

      geometry_msgs::WrenchStamped wrench_in;
      sensor_msgs::JointState js;
      {
        // copies locales atomiques (callbacks peuvent mettre à jour)
        std::lock_guard<std::mutex> lk(mutex_);
        wrench_in = last_force_;
        js = last_js_;
      }

      geometry_msgs::WrenchStamped wrench_global;
      if (!transformWrenchToBaseFrame(wrench_in, wrench_global)) {
        rate.sleep();
        continue;
      }

      // Force/torque from transformed wrench
      Eigen::VectorXd current_force(6);
      current_force <<
        wrench_global.wrench.force.x,
        wrench_global.wrench.force.y,
        wrench_global.wrench.force.z,
        wrench_global.wrench.torque.x,
        wrench_global.wrench.torque.y,
        wrench_global.wrench.torque.z;

      // Réordonner les joints selon joint_names_
      std::vector<double> joint_values_v;
      joint_values_v.reserve(joint_names_.size());
      for (const auto& name : joint_names_) {
        auto it = std::find(js.name.begin(), js.name.end(), name);
        if (it == js.name.end()) {
          ROS_WARN_STREAM_THROTTLE(1.0, "Joint manquant dans JointState: " << name);
          break;
        }
        size_t idx = std::distance(js.name.begin(), it);
        joint_values_v.push_back(js.position[idx]);
      }
      if (joint_values_v.size() != joint_names_.size()) {
        rate.sleep();
        continue;
      }

      Eigen::VectorXd joint_values = Eigen::Map<Eigen::VectorXd>(joint_values_v.data(), joint_values_v.size());

      // Filtrage exponentiel des forces
      smoothed_rel = (1.0 - F_alpha) * smoothed_rel + F_alpha * current_force;
      filtered_force = smoothed_rel;

      // Zones mortes
      if (filtered_force.head<3>().norm() < force_dead_cart) {
        filtered_force(0) = 0.0; filtered_force(1) = 0.0; filtered_force(2) = 0.0;
      }
      if (filtered_force.tail<3>().norm() < force_dead_rot) {
        filtered_force(3) = 0.0; filtered_force(4) = 0.0; filtered_force(5) = 0.0;
      }

      // Jacobien KDL
      KDL::JntArray jnt_array(kdl_chain_.getNrOfJoints());
      for (unsigned i = 0; i < jnt_array.rows(); ++i) jnt_array(i) = joint_values_v[i];

      KDL::Frame fk_frame;
      fk_solver_->JntToCart(jnt_array, fk_frame); // non utilisé, mais dispo comme en Python

      KDL::Jacobian J_kdl(kdl_chain_.getNrOfJoints());
      jnt_to_jac_->JntToJac(jnt_array, J_kdl);

      Eigen::MatrixXd J(6, J_kdl.columns());
      for (unsigned r = 0; r < 6; ++r) {
        for (unsigned c = 0; c < J_kdl.columns(); ++c) {
          J(r, c) = J_kdl(r, c);
        }
      }

      // Admittance (acc, vel, pos virtuelles)
      a_k_ = (filtered_force - B_vec.cwiseProduct(v_k_) - K_vec.cwiseProduct(x_k_)).cwiseQuotient(M_vec);
      v_k_ = v_k_ + a_k_ * dt;
      x_k_ = v_k_ * dt; // K = 0 -> pure intégration vitesse

      Eigen::VectorXd ee_vel = v_k_;

      // DLS pseudo-inverse: J^+ = V * diag(S/(S^2 + lambda^2)) * U^T
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::VectorXd S = svd.singularValues();
      Eigen::MatrixXd U = svd.matrixU();
      Eigen::MatrixXd V = svd.matrixV();

      Eigen::VectorXd Sd = S.array() / (S.array().square() + lambda_dls*lambda_dls);
      Eigen::MatrixXd J_pinv = V * Sd.asDiagonal() * U.transpose(); // (n x 6)

      Eigen::VectorXd joint_vel = J_pinv * ee_vel; // (n)

      // Filtre exp. sur vitesses articulaires
      if (joint_velocity_smoothed.size() == 0) {
        joint_velocity_smoothed = joint_vel;
      } else {
        joint_velocity_smoothed = V_alpha * joint_vel + (1.0 - V_alpha) * joint_velocity_smoothed;
      }

      // Publication
      for (int i = 0; i < 6; ++i) vel_msg.data[i] = joint_velocity_smoothed(i);
      pub_vel_.publish(vel_msg);

      rate.sleep();
    }

    // Arrêt propre: rampe à zéro
    {
      std_msgs::Float64MultiArray ramp = vel_msg;
      for (int k = 0; k < 100; ++k) {
        for (double &v : ramp.data) v /= 1.03;
        pub_vel_.publish(ramp);
        ros::Duration(0.0025).sleep();
      }
      std_msgs::Float64MultiArray zero;
      zero.data.assign(6, 0.0);
      pub_vel_.publish(zero);
    }
  }

  // Equivalent du transform_wrench_to_base_frame() en Python
  bool transformWrenchToBaseFrame(const geometry_msgs::WrenchStamped& in,
                                  geometry_msgs::WrenchStamped& out) {
    const double seuil_norme_q = 0.1;

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, tool_frame_, ros::Time(0), ros::Duration(0.01));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "TF transform error: %s", ex.what());
      return false;
    }

    const geometry_msgs::Quaternion& q = transform.transform.rotation;
    Eigen::Quaterniond q_eig(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R_test = q_eig.normalized().toRotationMatrix();

    Eigen::Matrix3d R_use;
    if (have_last_rot_) {
      double diff = (R_test - last_R_).norm();
      if (diff <= seuil_norme_q) {
        R_use = R_test;
        last_R_ = R_test;
      } else {
        R_use = last_R_;
      }
    } else {
      R_use = R_test;
      last_R_ = R_test;
      have_last_rot_ = true;
    }

    // Appliquer correction fixe (sensor -> tool)
    R_use = R_use * correction_;

    // Vecteurs force/torque capteur
    Eigen::Vector3d f(in.wrench.force.x, in.wrench.force.y, in.wrench.force.z);
    Eigen::Vector3d tau(in.wrench.torque.x, in.wrench.torque.y, in.wrench.torque.z);

    // Déport couple: capteur -> manche
    Eigen::Vector3d d(0.0, 0.05, 0.07);
    Eigen::Vector3d tau_manche = tau + d.cross(f);

    // Rotation vers base
    Eigen::Vector3d f_trans = R_use * f;
    Eigen::Vector3d t_trans = R_use * tau_manche;

    out.header = in.header;
    out.header.frame_id = tool_frame_; // fidèle au script Python (même si les valeurs sont en base)
    out.wrench.force.x = f_trans.x();
    out.wrench.force.y = f_trans.y();
    out.wrench.force.z = f_trans.z();
    out.wrench.torque.x = t_trans.x();
    out.wrench.torque.y = t_trans.y();
    out.wrench.torque.z = t_trans.z();

    return true;
  }

  // Optionnel: switch helper unique
  bool switchToVelocityController() {
    return switchControllers({"joint_group_vel_controller"}, {"scaled_pos_joint_traj_controller"});
  }

  // Kill/launch des scripts de capteur (optionnel pour mimer le Python)
  void restartForcePublisherProcesses() {
    // ATTENTION: ces appels system() supposent la présence de pkill/rosrun et du pkg "carnicero".
    std::system("pkill -f force_sensor_eth_publisher.py");
    std::system("rosrun carnicero force_sensor_eth_publisher.py &");
  }

private:
  void forceCb(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_force_ = *msg;
    have_force_ = true;
  }

  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_js_ = *msg;
    have_js_ = true;
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_force_;
  ros::Subscriber sub_js_;
  ros::Publisher  pub_vel_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<std::string> joint_names_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // State
  std::mutex mutex_;
  geometry_msgs::WrenchStamped last_force_;
  sensor_msgs::JointState last_js_;
  bool have_force_;
  bool have_js_;

  // Filters / admittance states
  Eigen::VectorXd v_k_; // 6
  Eigen::VectorXd x_k_; // 6
  Eigen::VectorXd a_k_; // 6

  // TF rotation smoothing
  Eigen::Matrix3d last_R_;
  Eigen::Matrix3d correction_;
  bool have_last_rot_;

  // Frames
  std::string tool_frame_;
  std::string base_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "admittance");

  try {
    ROS_INFO("Node initialisation");

    AdmittanceControl ac;

    // Optionnel: relancer le publisher du capteur comme dans le script Python
    ac.restartForcePublisherProcesses();

    // Basculer le contrôleur (précaution)
    ac.switchToVelocityController();

    // Boucle
    ac.spin();

    ROS_INFO("Fin de l'admittance");
    return 0;

  } catch (const std::exception& e) {
    ROS_ERROR("Exception: %s", e.what());
    return 1;
  }
}

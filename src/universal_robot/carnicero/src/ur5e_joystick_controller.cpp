#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <eigen3/Eigen/Dense>

#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

// ====== Globals (comme dans le script Python) ======
static KDL::Chain g_chain;
static std::unique_ptr<KDL::ChainJntToJacSolver> g_jac_solver;

static sensor_msgs::JointState::ConstPtr g_last_joint_state; // dernier JointState
static std::vector<double> g_target_positions;               // cible intégrée (gelée si joystick neutre)
static Eigen::Matrix<double,6,1> g_v_cartesian = Eigen::Matrix<double,6,1>::Zero(); // [vx,vy,vz,wx,wy,wz]

static double g_gain_linear  = 0.25;
static double g_gain_angular = 0.25;

static std::vector<std::string> JOINT_NAMES = {
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint"
};

// ====== Utils ======
static void pseudoInverse(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_pinv, double eps = 1e-6)
{
  // pinv(M) = V * S^+ * U^T
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();
  Eigen::VectorXd S_inv = S;

  // Tolérance relative inspirée de numpy.linalg.pinv
  const double tol = std::max(M.rows(), M.cols()) * S(0) * std::numeric_limits<double>::epsilon();
  for (int i = 0; i < S.size(); ++i)
    S_inv(i) = (S(i) > std::max(eps, tol)) ? (1.0 / S(i)) : 0.0;

  M_pinv = svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
}

static Eigen::MatrixXd kdlJacobianToEigen(const KDL::Jacobian& J)
{
  Eigen::MatrixXd eJ(J.rows(), J.columns());
  for (unsigned int r = 0; r < J.rows(); ++r)
    for (unsigned int c = 0; c < J.columns(); ++c)
      eJ(r, c) = J(r, c);
  return eJ;
}

// ====== Callbacks ======
static void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Adaptation axes si nécessaire (ici: axes[0..5])
  if (msg->axes.size() >= 6)
  {
    g_v_cartesian(0) = msg->axes[0] * g_gain_linear;
    g_v_cartesian(1) = msg->axes[1] * g_gain_linear;
    g_v_cartesian(2) = msg->axes[2] * (g_gain_linear * 0.5);
    g_v_cartesian(3) = msg->axes[3] * g_gain_angular;
    g_v_cartesian(4) = msg->axes[4] * g_gain_angular;
    g_v_cartesian(5) = msg->axes[5] * g_gain_angular;
  }
  else
  {
    ROS_WARN_THROTTLE(5.0, "[joy] Moins de 6 axes fournis (%zu). Vitesse cartésienne ignorée.", msg->axes.size());
  }
}

static void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  g_last_joint_state = msg;
  if (!msg->position.empty())
  {
    ROS_INFO_THROTTLE(2.0, "JointState reçu (%zu positions).", msg->position.size());
  }
}

// ====== Main ======
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5e_joystick_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Params (permet de tout régler comme tu veux)
  std::string base_link = "base_link";
  std::string tip_link  = "tool0";
  std::string joy_topic = "/IK_joy_publisher";
  std::string js_topic  = "/joint_states";
  std::string traj_topic = "/scaled_pos_joint_traj_controller/command"; // pour UR5e réel
  // std::string traj_topic = "/eff_joint_traj_controller/command";     // pour Gazebo

  int rate_hz = 100;
  double vel_threshold = 1e-6; // seuil sous lequel on "freeze" target
  double resync_threshold = 0.5; // rad

  pnh.param("gain_linear",   g_gain_linear,   g_gain_linear);
  pnh.param("gain_angular",  g_gain_angular,  g_gain_angular);
  pnh.param("base_link",     base_link,       base_link);
  pnh.param("tip_link",      tip_link,        tip_link);
  pnh.param("joy_topic",     joy_topic,       joy_topic);
  pnh.param("joint_states_topic", js_topic,   js_topic);
  pnh.param("traj_topic",    traj_topic,      traj_topic);
  pnh.param("rate_hz",       rate_hz,         rate_hz);
  pnh.param("vel_threshold", vel_threshold,   vel_threshold);
  pnh.param("resync_threshold", resync_threshold, resync_threshold);

  // Récupération du robot_description -> KDL Tree -> KDL Chain
  KDL::Tree tree;
  if (!kdl_parser::treeFromParam("robot_description", tree))
  {
    ROS_ERROR("Paramètre 'robot_description' introuvable ou invalide.");
    return 1;
  }
  if (!tree.getChain(base_link, tip_link, g_chain))
  {
    ROS_ERROR("Impossible d'extraire la chaîne KDL de '%s' à '%s'.", base_link.c_str(), tip_link.c_str());
    return 1;
  }
  if (g_chain.getNrOfJoints() == 0)
  {
    ROS_ERROR("La chaîne KDL contient 0 articulations.");
    return 1;
  }

  g_jac_solver.reset(new KDL::ChainJntToJacSolver(g_chain));
  ROS_INFO("KDL Chain OK: %u joints.", g_chain.getNrOfJoints());

  // ROS I/O
  auto sub_joy = nh.subscribe(joy_topic, 1, joyCallback);
  auto sub_js  = nh.subscribe(js_topic,  10, jointStateCallback);
  auto pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>(traj_topic, 1);

  ros::Rate rate(rate_hz);
  const double delta_t = 1.0 / static_cast<double>(rate_hz);

  while (ros::ok())
  {
    ros::spinOnce();

    if (!g_last_joint_state)
    {
      rate.sleep();
      continue;
    }

    try
    {
      const unsigned int n_joints = g_chain.getNrOfJoints();
      // positions_ordered dans l'ordre JOINT_NAMES
      std::vector<double> positions_ordered(n_joints, 0.0);
      std::unordered_map<std::string, size_t> name_to_idx;
      for (size_t i = 0; i < g_last_joint_state->name.size(); ++i)
        name_to_idx[g_last_joint_state->name[i]] = i;

      for (size_t i = 0; i < JOINT_NAMES.size() && i < n_joints; ++i)
      {
        auto it = name_to_idx.find(JOINT_NAMES[i]);
        if (it != name_to_idx.end() && it->second < g_last_joint_state->position.size())
        {
          positions_ordered[i] = g_last_joint_state->position[it->second];
        }
        else
        {
          ROS_WARN_THROTTLE(5.0, "Joint '%s' absent de /joint_states ; utilisation de 0.",
                            JOINT_NAMES[i].c_str());
        }
      }

      // init target_positions la 1ère fois
      if (g_target_positions.size() != JOINT_NAMES.size())
        g_target_positions = positions_ordered;

      // Jacobienne au point mesuré
      KDL::JntArray q_kdl(n_joints);
      for (unsigned int i = 0; i < n_joints; ++i) q_kdl(i) = positions_ordered[i];

      KDL::Jacobian J_kdl(n_joints);
      g_jac_solver->JntToJac(q_kdl, J_kdl);
      auto J = kdlJacobianToEigen(J_kdl); // (6 x n)

      if (J.size() == 0)
      {
        ROS_WARN_THROTTLE(2.0, "Jacobian vide -> pas de calcul q_dot.");
        rate.sleep();
        continue;
      }

      // q_dot = pinv(J) * v_cart
      Eigen::MatrixXd J_pinv;
      pseudoInverse(J, J_pinv);               // (n x 6)
      Eigen::VectorXd q_dot_full = J_pinv * g_v_cartesian; // (n)

      // s'assurer de la taille finale (comme le Python)
      std::vector<double> q_dot_ordered(JOINT_NAMES.size(), 0.0);
      const size_t L = std::min<size_t>(q_dot_full.size(), q_dot_ordered.size());
      for (size_t i = 0; i < L; ++i) q_dot_ordered[i] = q_dot_full(i);

      // Intégration si vitesse significative, sinon on "freeze" target
      double max_abs_qdot = 0.0;
      for (double v : q_dot_ordered) max_abs_qdot = std::max(max_abs_qdot, std::abs(v));

      if (max_abs_qdot > vel_threshold)
      {
        for (size_t i = 0; i < g_target_positions.size(); ++i)
          g_target_positions[i] += q_dot_ordered[i] * delta_t;
      }

      // Sécurité : resync si divergence trop grande
      double max_abs_err = 0.0;
      for (size_t i = 0; i < g_target_positions.size(); ++i)
        max_abs_err = std::max(max_abs_err, std::abs(g_target_positions[i] - positions_ordered[i]));
      if (max_abs_err > resync_threshold)
      {
        ROS_WARN_THROTTLE(1.0, "Target diverge de la mesure (>%0.2f rad). Resynchronisation.", resync_threshold);
        g_target_positions = positions_ordered;
      }

      // Publication JointTrajectory (positions + vitesses + accels)
      trajectory_msgs::JointTrajectory jt;
      jt.joint_names = JOINT_NAMES;

      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions = g_target_positions;
      pt.velocities = q_dot_ordered;
      pt.accelerations.assign(JOINT_NAMES.size(), 0.0);
      pt.time_from_start = ros::Duration(delta_t);

      jt.points.push_back(pt);
      pub_traj.publish(jt);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Erreur dans la boucle de contrôle : %s", e.what());
    }

    rate.sleep();
  }

  return 0;
}


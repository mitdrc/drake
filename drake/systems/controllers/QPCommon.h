#ifndef _QPCOMMON_H_
#define _QPCOMMON_H_

#include "drake/systems/controllers/controlUtil.h"
#include "drake/util/drakeUtil.h"
#include <vector>
#include <Eigen/Core>
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/robotInterfaces/Side.h"

struct QPControllerState {
  double t_prev;
  bool foot_contact_prev[2];
  Eigen::VectorXd vref_integrator_state;
  Eigen::VectorXd q_integrator_state;
  std::set<int> active;
  // first is val bin_lb_ub - Ain_lb_ub * x; should >= 0
  std::set<std::pair<double, std::string>> active_ineq;
  int num_active_contact_pts;

  // center of mass observer
  Eigen::Vector4d center_of_mass_observer_state;
  Eigen::Vector3d last_com_ddot;

  // gurobi active set params
  int* vbasis;
  int* cbasis;
  int vbasis_len;
  int cbasis_len;
};

struct PositionIndices {
  std::map<Side, std::vector<int>> legs;
  std::map<Side, int> knees;
  std::map<Side, std::vector<int>> ankles;
  std::map<Side, std::vector<int>> arms;
  std::vector<int> neck;
  int back_bkz;
  int back_bky;
};

struct RobotPropertyCache {
  PositionIndices position_indices;
  std::map<Side, int> foot_ids;
  std::map<Side, int> hand_ids;
  int pelvis_id;
  int torso_id;
};

struct VRefIntegratorParams {
  VRefIntegratorParams()
      : zero_ankles_on_contact(false), eta(0.0), delta_max(0.0) {}

  bool zero_ankles_on_contact;
  double eta;
  double delta_max;

  friend bool operator==(const VRefIntegratorParams& lhs,
                         const VRefIntegratorParams& rhs) {
    return lhs.zero_ankles_on_contact == rhs.zero_ankles_on_contact &&
           lhs.eta == rhs.eta && lhs.delta_max == rhs.delta_max;
  }

  friend std::ostream& operator<< (std::ostream& stream, const VRefIntegratorParams& param) {
    stream << "VRefIntegratorParams:\n"
      << "zero_ankles_on_contact: " << param.zero_ankles_on_contact << std::endl
      << "eta: " << param.eta << std::endl
      << "delta_max: " << param.delta_max << std::endl;
    return stream;
  }
};

struct IntegratorParams {
  IntegratorParams(const RigidBodyTree& robot)
      : gains(Eigen::VectorXd::Zero(robot.num_positions)),
        clamps(Eigen::VectorXd::Zero(robot.num_positions)),
        eta(0.0) {}

  Eigen::VectorXd gains;
  Eigen::VectorXd clamps;
  double eta;

  friend bool operator==(const IntegratorParams& lhs,
                         const IntegratorParams& rhs) {
    return lhs.gains.isApprox(rhs.gains) && lhs.clamps.isApprox(rhs.clamps) &&
           lhs.eta == rhs.eta;
  }

  friend std::ostream& operator<< (std::ostream& stream, IntegratorParams& param) {
    stream << "IntegratorParams:\n"
      << "gains: " << param.gains << std::endl
      << "clamps: " << param.clamps << std::endl
      << "eta: " << param.eta << std::endl;
    return stream;
  }
};

struct Bounds {
  Bounds(const Eigen::Ref<const Eigen::VectorXd>& min_,
         const Eigen::Ref<const Eigen::VectorXd>& max_)
      : min(min_), max(max_) {}

  Eigen::VectorXd min;
  Eigen::VectorXd max;

  friend bool operator==(const Bounds& lhs, const Bounds& rhs) {
    return lhs.min.isApprox(rhs.min) && lhs.max.isApprox(rhs.max);
  }

  friend std::ostream& operator<< (std::ostream& stream, Bounds& bounds) {
    stream << "Bounds:\n"
      << "min: " << bounds.min.transpose() << std::endl
      << "max: " << bounds.max.transpose() << std::endl;
    return stream;
  }
};

struct JointSoftLimitParams {
  JointSoftLimitParams(const RigidBodyTree& robot)
      : enabled(
            Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(robot.num_positions)),
        disable_when_body_in_support(
            Eigen::VectorXi::Zero(robot.num_positions)),
        lb(Eigen::VectorXd::Zero(robot.num_positions)),
        ub(Eigen::VectorXd::Zero(robot.num_positions)),
        kp(Eigen::VectorXd::Zero(robot.num_positions)),
        kd(Eigen::VectorXd::Zero(robot.num_positions)),
        weight(Eigen::VectorXd::Zero(robot.num_positions)),
        k_logistic(Eigen::VectorXd::Zero(robot.num_positions)) {}

  Eigen::Matrix<bool, Eigen::Dynamic, 1> enabled;
  Eigen::VectorXi disable_when_body_in_support;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd weight;
  Eigen::VectorXd k_logistic;

  friend bool operator==(const JointSoftLimitParams& lhs,
                         const JointSoftLimitParams& rhs) {
    bool is_equal =
        lhs.enabled == rhs.enabled &&
        lhs.disable_when_body_in_support == rhs.disable_when_body_in_support &&
        lhs.lb == rhs.lb && lhs.ub == rhs.ub && lhs.kp.isApprox(rhs.kp) &&
        lhs.kd.isApprox(rhs.kd) && lhs.weight.isApprox(rhs.weight) &&
        lhs.k_logistic.isApprox(rhs.k_logistic);
    return is_equal;
  }

  friend std::ostream& operator<< (std::ostream& stream, JointSoftLimitParams& param) {
    stream << "JointSoftLimitParams: NEED TO IMPLEMENT THIS\n";
    return stream;
  }
};

struct WholeBodyParams {
  WholeBodyParams(const RigidBodyTree& robot)
      : Kp(Eigen::VectorXd::Zero(robot.num_positions)),
        Kd(Eigen::VectorXd::Zero(robot.num_positions)),
        w_qdd(Eigen::VectorXd::Zero(robot.num_velocities)),
        integrator(robot),
        qdd_bounds(Eigen::VectorXd::Zero(robot.num_velocities),
                   Eigen::VectorXd::Zero(robot.num_velocities)) {}

  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  Eigen::VectorXd w_qdd;

  // double damping_ratio;
  IntegratorParams integrator;
  Bounds qdd_bounds;

  friend bool operator==(const WholeBodyParams& lhs,
                         const WholeBodyParams& rhs) {
    return lhs.Kp.isApprox(rhs.Kp) && lhs.Kd.isApprox(rhs.Kd) &&
           lhs.w_qdd.isApprox(rhs.w_qdd) && lhs.integrator == rhs.integrator &&
           lhs.qdd_bounds == rhs.qdd_bounds;
  }

  friend std::ostream& operator<< (std::ostream& stream, WholeBodyParams& params) {
    stream << "WholeBodyParams: NEED TO IMPLEMENT THIS\n";
    return stream;
  }
};

struct BodyMotionParams {
  BodyMotionParams()
      : Kp(Eigen::Vector6d::Zero()),
        Kd(Eigen::Vector6d::Zero()),
        accel_bounds(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)),
        weight(0.0) {}

  Eigen::Vector6d Kp;
  Eigen::Vector6d Kd;
  Bounds accel_bounds;
  double weight;

  friend bool operator==(const BodyMotionParams& lhs,
                         const BodyMotionParams& rhs) {
    return lhs.Kp.isApprox(rhs.Kp) && lhs.Kd.isApprox(rhs.Kd) &&
           lhs.accel_bounds == rhs.accel_bounds && lhs.weight == rhs.weight;
  }

  friend std::ostream& operator<< (std::ostream& stream, BodyMotionParams& param) {
    stream << "BodyMotionParams:\n"
      << "Kp: " << param.Kp.transpose() << std::endl
      << "Kd: " << param.Kd.transpose() << std::endl
      << param.accel_bounds
      << "weight: " << param.weight << std::endl;
    return stream;
  }
};

struct HardwareGains {
  HardwareGains(const RigidBodyTree& robot)
      : k_f_p(Eigen::VectorXd::Zero(robot.actuators.size())),
        k_q_p(Eigen::VectorXd::Zero(robot.actuators.size())),
        k_q_i(Eigen::VectorXd::Zero(robot.actuators.size())),
        k_qd_p(Eigen::VectorXd::Zero(robot.actuators.size())),
        ff_qd(Eigen::VectorXd::Zero(robot.actuators.size())),
        ff_f_d(Eigen::VectorXd::Zero(robot.actuators.size())),
        ff_const(Eigen::VectorXd::Zero(robot.actuators.size())),
        ff_qd_d(Eigen::VectorXd::Zero(robot.actuators.size())) {}

  Eigen::VectorXd k_f_p;
  Eigen::VectorXd k_q_p;
  Eigen::VectorXd k_q_i;
  Eigen::VectorXd k_qd_p;
  Eigen::VectorXd ff_qd;
  Eigen::VectorXd ff_f_d;
  Eigen::VectorXd ff_const;
  Eigen::VectorXd ff_qd_d;

  friend bool operator==(const HardwareGains& lhs, const HardwareGains& rhs) {
    bool is_equal =
        lhs.k_f_p.isApprox(rhs.k_f_p) && lhs.k_q_p.isApprox(rhs.k_q_p) &&
        lhs.k_q_i.isApprox(rhs.k_q_i) && lhs.k_qd_p.isApprox(rhs.k_qd_p) &&
        lhs.ff_qd.isApprox(rhs.ff_qd) && lhs.ff_f_d.isApprox(rhs.ff_f_d) &&
        lhs.ff_const.isApprox(rhs.ff_const) &&
        lhs.ff_qd_d.isApprox(rhs.ff_qd_d);
    return is_equal;
  }

  friend std::ostream& operator<< (std::ostream& stream, HardwareGains& gains) {
    stream << "HardwareGains: NEED TO IMPLEMENT THIS\n";
    return stream;
  }
};

struct HardwareParams {
  HardwareParams(const RigidBodyTree& robot)
      : gains(robot),
        joint_is_force_controlled(Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(
            robot.actuators.size())),
        joint_is_position_controlled(
            Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(
                robot.actuators.size())),
        maxDeltaPerSecond(Eigen::VectorXd::Zero(robot.actuators.size())) {}

  HardwareGains gains;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_force_controlled;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_position_controlled;
  Eigen::VectorXd maxDeltaPerSecond;

  friend bool operator==(const HardwareParams& lhs, const HardwareParams& rhs) {
    return lhs.gains == rhs.gains &&
           lhs.joint_is_force_controlled == rhs.joint_is_force_controlled &&
           lhs.joint_is_position_controlled == rhs.joint_is_position_controlled;
  }

  friend std::ostream& operator<< (std::ostream& stream, HardwareParams& gains) {
    stream << "HardwareParams: NEED TO IMPLEMENT THIS\n";
    return stream;
  }
};

struct QPControllerParams {
  QPControllerParams(const RigidBodyTree& robot)
      : whole_body(robot),
        body_motion(robot.bodies.size()),
        vref_integrator(),
        joint_soft_limits(robot),
        hardware(robot),
        W_kdot(Eigen::Matrix3d::Zero()),
        Kp_ang(0.0),
        w_slack(0.0),
        slack_limit(0.0),
        w_grf(0.0),
        w_qdd_delta(0.0),
        Kp_accel(0.0),
        contact_threshold(0.0),
        mu(0.7),
        Jpdotv_multiplier(1.0),
        w_zmp(1.0),
        w_comdd_delta(0),
        w_z_trq(0.0),
        min_knee_angle(0.0),
        ankle_torque_alpha(0.0),
        use_center_of_mass_observer(false),
        center_of_mass_observer_gain(Eigen::Matrix4d::Zero()) {}

  WholeBodyParams whole_body;
  std::vector<BodyMotionParams> body_motion;
  VRefIntegratorParams vref_integrator;
  JointSoftLimitParams joint_soft_limits;
  HardwareParams hardware;
  Eigen::Matrix3d W_kdot;
  double Kp_ang;
  double w_slack;
  double slack_limit;
  double w_grf;
  double Jpdotv_multiplier;
  double Kp_accel;
  double contact_threshold;
  bool useTorqueAlphaFilter;
  double w_zmp;
  double w_comdd_delta;
  double w_z_trq;
  double w_qdd_delta;
  double mu;
  double min_knee_angle;
  double ankle_torque_alpha;
  bool use_center_of_mass_observer;
  Eigen::Matrix4d center_of_mass_observer_gain;

  friend bool operator==(const QPControllerParams& lhs,
                         const QPControllerParams& rhs) {
    bool is_equal =
        lhs.whole_body == rhs.whole_body
        // && lhs.body_motion == rhs.body_motion
        &&
        lhs.vref_integrator == rhs.vref_integrator &&
        lhs.joint_soft_limits == rhs.joint_soft_limits &&
        lhs.hardware == rhs.hardware && lhs.W_kdot.isApprox(rhs.W_kdot) &&
        lhs.Kp_ang == rhs.Kp_ang && lhs.w_slack == rhs.w_slack &&
        lhs.slack_limit == rhs.slack_limit && lhs.w_grf == rhs.w_grf &&
        lhs.Kp_accel == rhs.Kp_accel &&
        lhs.contact_threshold == rhs.contact_threshold &&
        lhs.min_knee_angle == rhs.min_knee_angle &&
        lhs.ankle_torque_alpha == rhs.ankle_torque_alpha &&
        lhs.use_center_of_mass_observer == rhs.use_center_of_mass_observer &&
        lhs.center_of_mass_observer_gain.isApprox(
            rhs.center_of_mass_observer_gain);
    return is_equal;
  }

  friend std::ostream& operator<< (std::ostream& stream, QPControllerParams& params) {
    stream << "QPControllerParams:\n"
      << params.whole_body;
    for (size_t i = 0; i < params.body_motion.size(); i++)
      stream << params.body_motion[i];
    stream << params.vref_integrator
      << params.joint_soft_limits
      << params.hardware
      << "W_kdot: " << params.W_kdot << std::endl
      << "Kp_ang: " << params.Kp_ang << std::endl
      << "w_slack: " << params.w_slack << std::endl
      << "slack_limit: " << params.slack_limit << std::endl
      << "w_grf: " << params.w_grf << std::endl
      << "Jpdotv_multiplier: " << params.Jpdotv_multiplier << std::endl
      << "Kp_accel: " << params.Kp_accel << std::endl
      << "contact_threshold: " << params.contact_threshold << std::endl
      << "useTorqueAlphaFilter: " << params.useTorqueAlphaFilter << std::endl
      << "w_zmp: " << params.w_zmp << std::endl
      << "w_comdd_delta: " << params.w_comdd_delta << std::endl
      << "w_z_trq: " << params.w_z_trq << std::endl
      << "w_qdd_delta: " << params.w_qdd_delta << std::endl
      << "mu: " << params.mu << std::endl
      << "min_knee_angle: " << params.min_knee_angle << std::endl
      << "ankle_torque_alpha: " << params.ankle_torque_alpha << std::endl
      << "use_center_of_mass_observer: " << params.use_center_of_mass_observer << std::endl
      << "center_of_mass_observer_gain: " << params.center_of_mass_observer_gain << std::endl;
    return stream;
  }
};

std::unordered_map<std::string, int> computeBodyOrFrameNameToIdMap(
    const RigidBodyTree& robot);

struct DesiredBodyAcceleration {
  DesiredBodyAcceleration()
      : accel_bounds(Eigen::VectorXd(6), Eigen::VectorXd(6)) {}

  int body_or_frame_id0;
  Eigen::Vector6d body_vdot; // this is the TWIST
  Eigen::Vector6d body_vdot_in_task_frame; // acceleration (not TWIST) in task frame
  double weight;
  Bounds accel_bounds;
  bool control_pose_when_in_contact;
  bool use_spatial_velocity;
  KinematicPath body_path;
  Eigen::Isometry3d T_task_to_world;
  Eigen::Vector6d weight_multiplier;
};

struct QPDesiredBodyMotion {
  std::string name;
  // these are all in task frame, first 3 are position, second 3 are angular.
  // interpolated from the nominal trajectory.
  Eigen::Matrix<double, 6, 1> body_q_d;
  Eigen::Matrix<double, 6, 1> body_v_d;
  Eigen::Matrix<double, 6, 1> body_vdot_d;

  // these are in task frame, computed with PD + ff acc with the above setpoints.
  Eigen::Matrix<double, 6, 1> body_vdot_with_pd;
};

// everything expressed in the world frame
struct QPControllerContactOutput {
  std::string body_name;
  Eigen::Matrix<double, 6, 1> wrench;
  Eigen::Matrix<double, 3, 1> ref_point;
  std::vector<Eigen::Vector3d> contact_points;
  std::vector<Eigen::Vector3d> contact_forces;
};

struct QPControllerOutput {
  // input
  std::vector<QPDesiredBodyMotion> desired_body_motions; // specified in world frame
  Eigen::VectorXd comdd_d; // computed with the unconstrained cost function

  Eigen::VectorXd q_ref;
  Eigen::VectorXd qd_ref;
  Eigen::VectorXd qdd;
  Eigen::VectorXd u;

  Eigen::VectorXd q_des; // just what we evaluate fromt the spline
  Eigen::VectorXd qdot_des; //desired velocity coming from spline
  Eigen::VectorXd qdd_des_w_pd; // coming from the pid, actual input to the QP

  // output
  Eigen::VectorXd comdd;
  Eigen::VectorXd footdd[2];
  Eigen::VectorXd pelvdd;
  Eigen::VectorXd slack;
  std::vector<QPControllerContactOutput> contact_output;

  bool fastQPFailed;
  int qpInfo;
};

struct QPControllerDebugData {
  std::vector<SupportStateElement,
              Eigen::aligned_allocator<SupportStateElement>> active_supports;
  int nc;
  Eigen::MatrixXd normals;
  Eigen::MatrixXd B;
  Eigen::VectorXd alpha;
  Eigen::VectorXd f;
  Eigen::MatrixXd Aeq;
  Eigen::VectorXd beq;
  Eigen::MatrixXd Ain_lb_ub;
  Eigen::VectorXd bin_lb_ub;
  Eigen::MatrixXd Qnfdiag;
  Eigen::MatrixXd Qneps;
  Eigen::VectorXd x_bar;
  Eigen::MatrixXd S;
  Eigen::VectorXd s1;
  Eigen::VectorXd s1dot;
  double s2dot;
  Eigen::MatrixXd A_ls;
  Eigen::MatrixXd B_ls;
  Eigen::MatrixXd Jcom;
  Eigen::VectorXd Jcomdotv;
  Eigen::VectorXd beta;
};

struct PIDOutput {
  Eigen::VectorXd q_ref;
  Eigen::VectorXd qddot_des;
};

// enum PlanShiftMode {NONE, XYZ, Z_ONLY, Z_AND_ZMP};

class Attachment {
 public:
  std::string attach_to_frame;
  std::string urdf_filename;
  DrakeJoint::FloatingBaseType joint_type;

  Attachment(
      const std::string& attach_to_frame_, const std::string& urdf_filename_,
      const DrakeJoint::FloatingBaseType& joint_type_ = DrakeJoint::FIXED)
      : attach_to_frame(attach_to_frame_),
        urdf_filename(urdf_filename_),
        joint_type(joint_type_) {
    // empty
  }

  Attachment() {
    // empty no-argument constructor for YAML::decode
  }
};

class KinematicModifications {
 public:
  std::set<std::string> collision_groups_to_keep;
  std::vector<Attachment> attachments;

  KinematicModifications(
      const std::set<std::string>& collision_groups_to_keep_ = {"heel", "toe"},
      const std::vector<Attachment>& attachments_ = std::vector<Attachment>())
      : collision_groups_to_keep(collision_groups_to_keep_),
        attachments(attachments_) {
    // empty
  }
};



#endif

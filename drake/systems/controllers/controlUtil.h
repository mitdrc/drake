#ifndef _CONTROL_UTIL_H_
#define _CONTROL_UTIL_H_

#include <math.h>
#include <set>
#include <vector>
#include <array>
#include <list>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/drakeControlUtil_export.h"
#include "drake/systems/robotInterfaces/Side.h"

const int m_surface_tangents =
    2;  // number of faces in the friction cone approx

#define EPSILON 10e-8

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;
}

typedef struct _support_state_element {
  int body_idx;
  double total_normal_force_upper_bound;
  double total_normal_force_lower_bound;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      contact_pts;
  bool support_logic_map[4];
  Eigen::Vector4d support_surface;  // 4-vector describing a support surface:
                                    // [v; b] such that v' * [x;y;z] + b == 0
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} SupportStateElement;

class ContactState {
 public:
  enum ContactBody {
    PELVIS = 0,
    L_FOOT,
    R_FOOT,
    L_HAND,
    R_HAND, // add more here
    SIZE_OF_CONTACT_BODY
  };

  ContactState() {
    for (size_t i = 0; i < body_in_contact_.size(); i++)
      body_in_contact_.at(i) = false;
  }

  explicit ContactState(const std::list<ContactBody> &contacts) {
    for (size_t i = 0; i < body_in_contact_.size(); i++)
      body_in_contact_.at(i) = false;
    for (auto it = contacts.begin(); it != contacts.end(); it++)
      body_in_contact_.at((size_t)(*it)) = true;
  }

  inline static const ContactState &DS() {
    static ContactState ret({L_FOOT, R_FOOT});
    return ret;
  }

  inline static const ContactState &SSL() {
    static ContactState ret({L_FOOT});
    return ret;
  }

  inline static const ContactState &SSR() {
    static ContactState ret({R_FOOT});
    return ret;
  }

  inline bool is_in_contact(const ContactBody body) const {
    return body_in_contact_.at(body);
  }

  inline void set_contact(const ContactBody body) {
    body_in_contact_.at(body) = true;
  }

  inline void remove_contact(const ContactBody body) {
    body_in_contact_.at(body) = false;
  }

  inline bool is_double_support() const {
    return (is_in_contact(L_FOOT) && is_in_contact(R_FOOT));
  }

  inline bool is_single_support_left() const {
    return (is_in_contact(L_FOOT) && !is_in_contact(R_FOOT));
  }

  inline bool is_single_support_right() const {
    return (!is_in_contact(L_FOOT) && is_in_contact(R_FOOT));
  }

  inline bool is_hand_in_contact(Side side) const {
    if (side.underlying() == Side::LEFT)
      return is_in_contact(L_HAND);
    else
      return is_in_contact(R_HAND);
  }

  inline bool is_foot_in_contact(Side side) const {
    if (side.underlying() == Side::LEFT)
      return is_in_contact(L_FOOT);
    else
      return is_in_contact(R_FOOT);
  }

  inline size_t num_bodies_in_contact() const {
    size_t ctr = 0;
    for (size_t i = 0; i < body_in_contact_.size(); i++)
      if (body_in_contact_.at(i))
        ctr++;
    return ctr;
  }

  const std::list<ContactBody> bodies_in_contact() const {
    std::list<ContactBody> contacts;
    for (size_t i = 0; i < body_in_contact_.size(); i++)
      if (body_in_contact_.at(i))
        contacts.push_back((ContactBody)i);
    return contacts;
  }

 private:
  std::array<bool, SIZE_OF_CONTACT_BODY> body_in_contact_;
};

struct ResidualState {
  Eigen::VectorXd residual;
};

struct DrakeRobotState {
  // drake-ordered position and velocity vectors, with timestamp (in s)
  double t;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;

  ContactState contact_state;
};

struct DrakeRobotStateWithTorque{
  double t;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd torque;
};

DRAKECONTROLUTIL_EXPORT Eigen::Vector6d GetTaskSpaceVel(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    const RigidBody& body,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
DRAKECONTROLUTIL_EXPORT Eigen::Vector6d GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    const RigidBody& body,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
DRAKECONTROLUTIL_EXPORT Eigen::MatrixXd GetTaskSpaceJacobian(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    const RigidBody& body,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());

DRAKECONTROLUTIL_EXPORT bool isSupportElementActive(
    SupportStateElement *se, bool contact_force_detected,
    bool kinematic_contact_detected);

DRAKECONTROLUTIL_EXPORT Eigen::Matrix<bool, Eigen::Dynamic, 1>
getActiveSupportMask(
    RigidBodyTree *r, Eigen::VectorXd q, Eigen::VectorXd qd,
    std::vector<SupportStateElement,
                Eigen::aligned_allocator<SupportStateElement>> &
        available_supports,
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>> &
        contact_force_detected,
    double contact_threshold);

DRAKECONTROLUTIL_EXPORT std::vector<
    SupportStateElement, Eigen::aligned_allocator<SupportStateElement>>
getActiveSupports(
    const RigidBodyTree &r, const Eigen::VectorXd &q, const Eigen::VectorXd &qd,
    std::vector<SupportStateElement,
                Eigen::aligned_allocator<SupportStateElement>> &
        available_supports,
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>> &
        contact_force_detected,
    double contact_threshold);

template <typename DerivedA, typename DerivedB>
DRAKECONTROLUTIL_EXPORT void getRows(std::set<int> &rows,
                                     Eigen::MatrixBase<DerivedA> const &M,
                                     Eigen::MatrixBase<DerivedB> &Msub);

template <typename DerivedA, typename DerivedB>
DRAKECONTROLUTIL_EXPORT void getCols(std::set<int> &cols,
                                     Eigen::MatrixBase<DerivedA> const &M,
                                     Eigen::MatrixBase<DerivedB> &Msub);

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
DRAKECONTROLUTIL_EXPORT void angleDiff(
    Eigen::MatrixBase<DerivedPhi1> const &phi1,
    Eigen::MatrixBase<DerivedPhi2> const &phi2, Eigen::MatrixBase<DerivedD> &d);

DRAKECONTROLUTIL_EXPORT bool inSupport(
    const std::vector<SupportStateElement,
                      Eigen::aligned_allocator<SupportStateElement>> &supports,
    int body_idx);
DRAKECONTROLUTIL_EXPORT void surfaceTangents(
    const Eigen::Vector3d &normal,
    Eigen::Matrix<double, 3, m_surface_tangents> &d);
DRAKECONTROLUTIL_EXPORT int contactPhi(const RigidBodyTree &r,
                                       const KinematicsCache<double> &cache,
                                       SupportStateElement &supp,
                                       Eigen::VectorXd &phi);
DRAKECONTROLUTIL_EXPORT int contactConstraintsBV(
    const RigidBodyTree &r, const KinematicsCache<double> &cache, int nc,
    std::vector<double> support_mus,
    std::vector<SupportStateElement,
                Eigen::aligned_allocator<SupportStateElement>> &supp,
    Eigen::MatrixXd &B, Eigen::MatrixXd &JB, Eigen::MatrixXd &Jp,
    Eigen::VectorXd &Jpdotv, Eigen::MatrixXd &normals);
DRAKECONTROLUTIL_EXPORT Eigen::MatrixXd individualSupportCOPs(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    const std::vector<SupportStateElement,
                      Eigen::aligned_allocator<SupportStateElement>> &
        active_supports,
    const Eigen::MatrixXd &normals, const Eigen::MatrixXd &B,
    const Eigen::VectorXd &beta);
DRAKECONTROLUTIL_EXPORT Eigen::Vector6d bodySpatialMotionPD(
    const RigidBodyTree &r, const DrakeRobotState &robot_state,
    const int body_index, const Eigen::Isometry3d &body_pose_des,
    const Eigen::Ref<const Eigen::Vector6d> &body_v_des,
    const Eigen::Ref<const Eigen::Vector6d> &body_vdot_des,
    const Eigen::Ref<const Eigen::Vector6d> &Kp, const Eigen::Ref<const Eigen::Vector6d> &Kd,
    const Eigen::Isometry3d &T_task_to_world,
    Eigen::Vector6d &body_vdot_with_pd);

DRAKECONTROLUTIL_EXPORT void evaluateXYZExpmapCubicSpline(
    double t, const PiecewisePolynomial<double> &spline,
    Eigen::Isometry3d &body_pose_des, Eigen::Vector6d &xyzdot_angular_vel,
    Eigen::Vector6d &xyzddot_angular_accel);

struct RobotJointIndexMap {
  Eigen::VectorXi drake_to_robot;
  Eigen::VectorXi robot_to_drake;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct JointNames {
  std::vector<std::string> robot;
  std::vector<std::string> drake;
};

DRAKECONTROLUTIL_EXPORT void getRobotJointIndexMap(
    const JointNames *joint_names, RobotJointIndexMap *joint_map);

#endif

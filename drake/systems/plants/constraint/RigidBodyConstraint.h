//-----------------------------------------
/**
 * @class RigidBodyConstraint       The abstract base class. All the constraints
 * used in the inverse kinematics problem are inherited from
 * RigidBodyConstraint. There are 6 main categories of the RigidBodyConstraint,
 * each category has its own interface
 */
#ifndef __RIGIDBODYCONSTRAINT_H__
#define __RIGIDBODYCONSTRAINT_H__
#include <iostream>
#include <Eigen/StdVector>
#include <set>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <sstream>
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/drakeRigidBodyConstraint_export.h"
#include "drake/util/drakeUtil.h"

template <typename Scalar>
class KinematicsCache;
class RigidBodyTree;

namespace DrakeRigidBodyConstraint {
extern DRAKERIGIDBODYCONSTRAINT_EXPORT Eigen::Vector3d com_pts;
extern DRAKERIGIDBODYCONSTRAINT_EXPORT const int WorldCoMDefaultRobotNum[1];
extern DRAKERIGIDBODYCONSTRAINT_EXPORT Eigen::Vector2d default_tspan;
}

DRAKERIGIDBODYCONSTRAINT_EXPORT void drakePrintMatrix(
    const Eigen::MatrixXd &mat);

class DRAKERIGIDBODYCONSTRAINT_EXPORT RigidBodyConstraint {
 protected:
  int category;
  int type;
  RigidBodyTree *robot;
  double tspan[2];

 public:
  RigidBodyConstraint(
      int category, RigidBodyTree *robot,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  RigidBodyConstraint(const RigidBodyConstraint &rhs);
  int getType() const { return this->type; };
  int getCategory() const { return this->category; };
  RigidBodyTree *getRobotPointer() const { return this->robot; };
  virtual ~RigidBodyConstraint(void) = 0;
  /* In each category, constraint class share the same function interface, this
   * value needs to be in consistent with that in MATLAB*/
  static const int SingleTimeKinematicConstraintCategory = -1;
  static const int MultipleTimeKinematicConstraintCategory = -2;
  static const int QuasiStaticConstraintCategory = -3;
  static const int PostureConstraintCategory = -4;
  static const int MultipleTimeLinearPostureConstraintCategory = -5;
  static const int SingleTimeLinearPostureConstraintCategory = -6;
  /* Each non-abstrac RigidBodyConstraint class has a unique type. Make sure
   * this value stays in consistent with the value in MATLAB*/
  static const int QuasiStaticConstraintType = 1;
  static const int PostureConstraintType = 2;
  static const int SingleTimeLinearPostureConstraintType = 3;
  static const int AllBodiesClosestDistanceConstraintType = 4;
  static const int WorldEulerConstraintType = 5;
  static const int WorldGazeDirConstraintType = 6;
  static const int WorldGazeOrientConstraintType = 7;
  static const int WorldGazeTargetConstraintType = 8;
  static const int RelativeGazeTargetConstraintType = 9;
  static const int WorldCoMConstraintType = 10;
  static const int WorldPositionConstraintType = 11;
  static const int WorldPositionInFrameConstraintType = 12;
  static const int WorldQuatConstraintType = 13;
  static const int Point2PointDistanceConstraintType = 14;
  static const int Point2LineSegDistConstraintType = 15;
  static const int WorldFixedPositionConstraintType = 16;
  static const int WorldFixedOrientConstraintType = 17;
  static const int WorldFixedBodyPoseConstraintType = 18;
  static const int PostureChangeConstraintType = 19;
  static const int RelativePositionConstraintType = 20;
  static const int RelativeQuatConstraintType = 24;
  static const int RelativeGazeDirConstraintType = 25;
  static const int MinDistanceConstraintType = 26;
  static const int GravityCompensationTorqueConstraintType = 27;

 protected:
  std::string getTimeString(const double *t) const;
};

/**
 * @class QuasiStaticConstraint       -- Constrain the Center of Mass (CoM) is
 *within the support polygon. The support polygon is a shrunk area of the
 *contact polygon
 * @param robot
 * @param tspan           -- The time span of this constraint being active
 * @param robotnumset     -- The set of the robots in the RigidBodyTree for
 *which the CoM is computed
 * @param shrinkFactor    -- The factor to shrink the contact polygon. The
 *shrunk area is the support polygon.
 * @param active          -- Whether the constraint is on/off. If active =
 *false, even the time t is within tspan, the constraint is still inactive
 * @param num_bodies      -- The total number of ground contact bodies/frames
 * @param num_pts         -- The total number of ground contact points
 * @param bodies          -- The index of ground contact bodies/frames
 * @param num_body_pts    -- The number of contact points on each contact
 *body/frame
 * @param body_pts        -- The contact points on each contact body/frame
 *
 * Function:
 *  @function eval     --evaluate the constraint
 *    @param t       --the time to evaluate the constraint
 *    @param weights --the weight associate with each ground contact point
 *    @param c       -- c = CoM-weights'*support_vertex
 *    @param dc      -- dc = [dcdq dcdweiths]
 *  @function addContact  -- add contact body and points
 *    @param num_new_bodies      -- number of new contact bodies
 *    @param body                -- the index of new contact bodies/frames
 *    @param body_pts            -- body_pts[i] are the contact points on
 *body[i]
 */

class DRAKERIGIDBODYCONSTRAINT_EXPORT QuasiStaticConstraint
    : public RigidBodyConstraint {
 protected:
  std::set<int> m_robotnumset;
  double shrinkFactor;
  bool active;
  int num_bodies;
  int num_pts;
  std::vector<int> bodies;
  std::vector<int> num_body_pts;
  std::vector<Eigen::Matrix3Xd> body_pts;
  static const std::set<int> defaultRobotNumSet;

 public:
  QuasiStaticConstraint(
      RigidBodyTree *robot,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan,
      const std::set<int> &robotnumset =
          QuasiStaticConstraint::defaultRobotNumSet);
  QuasiStaticConstraint(const QuasiStaticConstraint &rhs);
  bool isTimeValid(const double *t) const;
  int getNumConstraint(const double *t) const;
  void eval(const double *t, KinematicsCache<double> &cache,
            const double *weights, Eigen::VectorXd &c,
            Eigen::MatrixXd &dc) const;
  void bounds(const double *t, Eigen::VectorXd &lb, Eigen::VectorXd &ub) const;
  void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~QuasiStaticConstraint(void);
  bool isActive() const { return this->active; };
  int getNumWeights() const { return this->num_pts; };
  void addContact(int num_new_bodies, const int *body,
                  const Eigen::Matrix3Xd *body_pts);

  void addContact(std::vector<int> body,
                  const Eigen::Matrix3Xd& body_pts)
    {
      addContact(body.size(), body.data(), &body_pts);
    }


  void setShrinkFactor(double factor);
  void setActive(bool flag) { this->active = flag; };
  void updateRobot(RigidBodyTree *robot);
  void updateRobotnum(std::set<int> &robotnumset);
};

/*
 * @class PostureConstraint   constrain the joint limits
 * @param tspan          -- The time span of the constraint being valid
 * @param lb             -- The lower bound of the joints
 * @param ub             -- The upper bound of the joints
 *
 * @function setJointLimits   set the limit of some joints
 *   @param num_idx    The number of joints whose limits are going to be set
 *   @param joint_idx  joint_idx[i] is the index of the i'th joint whose limits
 *are going to be set
 *   @param lb         lb[i] is the lower bound of the joint joint_idx[i]
 *   @param ub         ub[i] is the upper bound of the joint joint_idx[i]
 */
class DRAKERIGIDBODYCONSTRAINT_EXPORT PostureConstraint
    : public RigidBodyConstraint {
 protected:
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::VectorXd joint_limit_min0;
  Eigen::VectorXd joint_limit_max0;

 public:
  PostureConstraint(
      RigidBodyTree *model,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  PostureConstraint(const PostureConstraint &rhs);
  bool isTimeValid(const double *t) const;
  void setJointLimits(int num_idx, const int *joint_idx,
                      const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);
  void setJointLimits(const Eigen::VectorXi &joint_idx,
                      const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);
  void bounds(const double *t, Eigen::VectorXd &joint_min,
              Eigen::VectorXd &joint_max) const;
  virtual ~PostureConstraint(void){};
};

/*
 * @class MultipleTimeLinearPostureConstraint constrain the posture such that
 *lb(t(1),t(2),...,t(n))<=A_mat(t(1),t(2),t(n))*[q(t(1));q(t(2));...;q(t(n))]<=ub(t(1),t(2),...,t(n))
 *where A_mat is a sparse matrix that only depends on t(1),t(2),...,t(n)
 *
 * @function eval return the value and gradient of the constraint
 *   @param t      array of time
 *   @param n_breaks   the length of array t
 *   @param q     q.col(i) is the posture at t[i]
 *   @param c    the value of the constraint
 *   @param dc   the gradient of the constraint w.r.t. q
 *
 * @function feval returns the value of the constraint
 *
 * @function geval returns the gradient of the constraint, written in the sprase
 *matrix form
 *   @return iAfun    The row index of the non-zero entries in the gradient
 *matrix
 *   @return jAvar    The column index of the non-zero entries in the gradient
 *matrix
 *   @return A        The value of the non-zero entries in the gradient matrix
 */
class DRAKERIGIDBODYCONSTRAINT_EXPORT MultipleTimeLinearPostureConstraint
    : public RigidBodyConstraint {
 protected:
  int numValidTime(const std::vector<bool> &valid_flag) const;
  void validTimeInd(const std::vector<bool> &valid_flag,
                    Eigen::VectorXi &valid_t_ind) const;

 public:
  MultipleTimeLinearPostureConstraint(
      RigidBodyTree *model,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  MultipleTimeLinearPostureConstraint(
      const MultipleTimeLinearPostureConstraint &rhs);
  std::vector<bool> isTimeValid(const double *t, int n_breaks) const;
  void eval(const double *t, int n_breaks, const Eigen::MatrixXd &q,
            Eigen::VectorXd &c, Eigen::SparseMatrix<double> &dc) const;
  virtual int getNumConstraint(const double *t, int n_breaks) const = 0;
  virtual void feval(const double *t, int n_breaks, const Eigen::MatrixXd &q,
                     Eigen::VectorXd &c) const = 0;
  virtual void geval(const double *t, int n_breaks, Eigen::VectorXi &iAfun,
                     Eigen::VectorXi &jAvar, Eigen::VectorXd &A) const = 0;
  virtual void name(const double *t, int n_breaks,
                    std::vector<std::string> &name_str) const = 0;
  virtual void bounds(const double *t, int n_breaks, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const = 0;
  virtual ~MultipleTimeLinearPostureConstraint(){};
};

/*
 * @class SingleTimeLinearPostureConstraint constrain the posture satisfies lb<=
 *A_mat*q <=ub at any time, where A_mat is a sparse matrix
 *
 * @function SingleTimeLinearPostureConstraint
 *   @param robot
 *   @param iAfun    The row indices of non zero entries
 *   @param jAvar    The column indices of non zero entries
 *   @param A        The values of non zero entries
 *   @param lb       The lower bound of the constraint, a column vector.
 *   @param ub       The upper bound of the constraint, a column vector.
 *   @param tspan    The time span [tspan[0] tspan[1]] is the time span of the
 *constraint being active
 * @function eval return the value and gradient of the constraint
 *   @param t      array of time
 *   @param n_breaks   the length of array t
 *   @param q     q.col(i) is the posture at t[i]
 *   @param c    the value of the constraint
 *   @param dc   the gradient of the constraint w.r.t. q
 *
 * @function feval returns the value of the constraint
 *
 * @function geval returns the gradient of the constraint, written in the sprase
 *matrix form
 *   @return iAfun    The row index of the non-zero entries in the gradient
 *matrix
 *   @return jAvar    The column index of the non-zero entries in the gradient
 *matrix
 *   @return A        The value of the non-zero entries in the gradient matrix
 */
class DRAKERIGIDBODYCONSTRAINT_EXPORT SingleTimeLinearPostureConstraint
    : public RigidBodyConstraint {
 protected:
  Eigen::VectorXi iAfun;
  Eigen::VectorXi jAvar;
  Eigen::VectorXd A;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  int num_constraint;
  Eigen::SparseMatrix<double> A_mat;

 public:
  SingleTimeLinearPostureConstraint(
      RigidBodyTree *robot, const Eigen::VectorXi &iAfun,
      const Eigen::VectorXi &jAvar, const Eigen::VectorXd &A,
      const Eigen::VectorXd &lb, const Eigen::VectorXd &ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  SingleTimeLinearPostureConstraint(
      const SingleTimeLinearPostureConstraint &rhs);
  bool isTimeValid(const double *t) const;
  int getNumConstraint(const double *t) const;
  void bounds(const double *t, Eigen::VectorXd &lb, Eigen::VectorXd &ub) const;
  void feval(const double *t, const Eigen::VectorXd &q,
             Eigen::VectorXd &c) const;
  void geval(const double *t, Eigen::VectorXi &iAfun, Eigen::VectorXi &jAvar,
             Eigen::VectorXd &A) const;
  void eval(const double *t, const Eigen::VectorXd &q, Eigen::VectorXd &c,
            Eigen::SparseMatrix<double> &dc) const;
  void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~SingleTimeLinearPostureConstraint(void){};
};

/*
 * class SingleTimeKinematicConstraint   An abstract class that constrain the
 * kinematics of the robot at individual time. Need to call doKinematics first
 * for the robot and then evaulate this constraint.
 */
class DRAKERIGIDBODYCONSTRAINT_EXPORT SingleTimeKinematicConstraint
    : public RigidBodyConstraint {
 protected:
  int num_constraint;

 public:
  SingleTimeKinematicConstraint(
      RigidBodyTree *model,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  SingleTimeKinematicConstraint(const SingleTimeKinematicConstraint &rhs);
  bool isTimeValid(const double *t) const;
  int getNumConstraint(const double *t) const;
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const = 0;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const = 0;
  virtual void name(const double *t,
                    std::vector<std::string> &name_str) const = 0;
  virtual void updateRobot(RigidBodyTree *robot);
  virtual ~SingleTimeKinematicConstraint(){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT MultipleTimeKinematicConstraint
    : public RigidBodyConstraint {
 protected:
  int numValidTime(const double *t, int n_breaks) const;

 public:
  MultipleTimeKinematicConstraint(
      RigidBodyTree *model,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  MultipleTimeKinematicConstraint(const MultipleTimeKinematicConstraint &rhs);
  std::vector<bool> isTimeValid(const double *t, int n_breaks) const;
  virtual int getNumConstraint(const double *t, int n_breaks) const = 0;
  void eval(const double *t, int n_breaks, const Eigen::MatrixXd &q,
            Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void eval_valid(const double *valid_t, int num_valid_t,
                          const Eigen::MatrixXd &valid_q, Eigen::VectorXd &c,
                          Eigen::MatrixXd &dc_valid) const = 0;
  virtual void bounds(const double *t, int n_breaks, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const = 0;
  virtual void name(const double *t, int n_breaks,
                    std::vector<std::string> &name_str) const = 0;
  virtual void updateRobot(RigidBodyTree *robot);
  virtual ~MultipleTimeKinematicConstraint(){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT PositionConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  std::vector<bool> null_constraint_rows;
  Eigen::Matrix3Xd pts;
  int n_pts;
  virtual void evalPositions(KinematicsCache<double> &cache,
                             Eigen::Matrix3Xd &pos,
                             Eigen::MatrixXd &J) const = 0;
  virtual void evalNames(const double *t,
                         std::vector<std::string> &cnst_names) const = 0;

 public:
  PositionConstraint(
      RigidBodyTree *model, const Eigen::Matrix3Xd &pts, Eigen::MatrixXd lb,
      Eigen::MatrixXd ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  PositionConstraint(const PositionConstraint &rhs);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~PositionConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldPositionConstraint
    : public PositionConstraint {
 protected:
  int body;
  std::string body_name;
  virtual void evalPositions(KinematicsCache<double> &cache,
                             Eigen::Matrix3Xd &pos, Eigen::MatrixXd &J) const;
  virtual void evalNames(const double *t,
                         std::vector<std::string> &cnst_names) const;

 public:
  WorldPositionConstraint(
      RigidBodyTree *model, int body, const Eigen::Matrix3Xd &pts,
      Eigen::MatrixXd lb, Eigen::MatrixXd ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldPositionConstraint();
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldCoMConstraint
    : public PositionConstraint {
 protected:
  std::set<int> m_robotnum;
  int body;
  std::string body_name;
  virtual void evalPositions(KinematicsCache<double> &cache,
                             Eigen::Matrix3Xd &pos, Eigen::MatrixXd &J) const;
  virtual void evalNames(const double *t,
                         std::vector<std::string> &cnst_names) const;
  static const std::set<int> defaultRobotNumSet;

 public:
  WorldCoMConstraint(
      RigidBodyTree *model, Eigen::Vector3d lb, Eigen::Vector3d ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan,
      const std::set<int> &robotnum = WorldCoMConstraint::defaultRobotNumSet);
  void updateRobotnum(const std::set<int> &robotnum);
  virtual ~WorldCoMConstraint();
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT RelativePositionConstraint
    : public PositionConstraint {
 protected:
  int bodyA_idx;
  int bodyB_idx;
  std::string bodyA_name;
  std::string bodyB_name;
  Eigen::Isometry3d bpTb;
  virtual void evalPositions(KinematicsCache<double> &cache,
                             Eigen::Matrix3Xd &pos, Eigen::MatrixXd &J) const;
  virtual void evalNames(const double *t,
                         std::vector<std::string> &cnst_names) const;

 public:
  RelativePositionConstraint(RigidBodyTree *model, const Eigen::Matrix3Xd &pts,
                             const Eigen::MatrixXd &lb,
                             const Eigen::MatrixXd &ub, int bodyA_idx,
                             int bodyB_idx,
                             const Eigen::Matrix<double, 7, 1> &bTbp,
                             const Eigen::Vector2d &tspan);
  virtual ~RelativePositionConstraint();
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT QuatConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  double tol;
  virtual void evalOrientationProduct(const KinematicsCache<double> &cache,
                                      double &prod,
                                      Eigen::MatrixXd &dprod) const = 0;

 public:
  QuatConstraint(
      RigidBodyTree *model, double tol,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~QuatConstraint();
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldQuatConstraint
    : public QuatConstraint {
 protected:
  int body;
  std::string body_name;
  Eigen::Vector4d quat_des;
  virtual void evalOrientationProduct(const KinematicsCache<double> &cache,
                                      double &prod,
                                      Eigen::MatrixXd &dprod) const;

 public:
  WorldQuatConstraint(
      RigidBodyTree *model, int body, const Eigen::Vector4d &quat_des,
      double tol,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~WorldQuatConstraint();

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT RelativeQuatConstraint
    : public QuatConstraint {
 protected:
  int bodyA_idx;
  int bodyB_idx;
  std::string bodyA_name;
  std::string bodyB_name;
  Eigen::Vector4d quat_des;
  virtual void evalOrientationProduct(const KinematicsCache<double> &cache,
                                      double &prod,
                                      Eigen::MatrixXd &dprod) const;

 public:
  RelativeQuatConstraint(
      RigidBodyTree *model, int bodyA_idx, int bodyB_idx,
      const Eigen::Vector4d &quat_des, double tol,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~RelativeQuatConstraint();

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT EulerConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  Eigen::VectorXd ub;
  Eigen::VectorXd lb;
  bool null_constraint_rows[3];
  Eigen::VectorXd avg_rpy;
  virtual void evalrpy(const KinematicsCache<double> &cache,
                       Eigen::Vector3d &rpy, Eigen::MatrixXd &J) const = 0;

 public:
  EulerConstraint(
      RigidBodyTree *model, const Eigen::Vector3d &lb,
      const Eigen::Vector3d &ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  EulerConstraint(const EulerConstraint &rhs);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~EulerConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldEulerConstraint
    : public EulerConstraint {
 protected:
  int body;
  std::string body_name;
  virtual void evalrpy(const KinematicsCache<double> &cache,
                       Eigen::Vector3d &rpy, Eigen::MatrixXd &J) const;

 public:
  WorldEulerConstraint(
      RigidBodyTree *model, int body, const Eigen::Vector3d &lb,
      const Eigen::Vector3d &ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~WorldEulerConstraint();
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT GazeConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  Eigen::Vector3d axis;
  double conethreshold;

 public:
  GazeConstraint(
      RigidBodyTree *model, const Eigen::Vector3d &axis,
      double conethreshold = 0.0,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~GazeConstraint(void){};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT GazeOrientConstraint
    : public GazeConstraint {
 protected:
  double threshold;
  Eigen::Vector4d quat_des;
  virtual void evalOrientation(const KinematicsCache<double> &cache,
                               Eigen::Vector4d &quat,
                               Eigen::MatrixXd &dquat_dq) const = 0;

 public:
  GazeOrientConstraint(
      RigidBodyTree *model, const Eigen::Vector3d &axis,
      const Eigen::Vector4d &quat_des, double conethreshold, double threshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~GazeOrientConstraint(void){};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldGazeOrientConstraint
    : public GazeOrientConstraint {
 protected:
  int body;
  std::string body_name;
  virtual void evalOrientation(const KinematicsCache<double> &cache,
                               Eigen::Vector4d &quat,
                               Eigen::MatrixXd &dquat_dq) const;

 public:
  WorldGazeOrientConstraint(
      RigidBodyTree *model, int body, const Eigen::Vector3d &axis,
      const Eigen::Vector4d &quat_des, double conethreshold, double threshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~WorldGazeOrientConstraint(){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT GazeDirConstraint
    : public GazeConstraint {
 protected:
  Eigen::Vector3d dir;

 public:
  GazeDirConstraint(
      RigidBodyTree *model, const Eigen::Vector3d &axis,
      const Eigen::Vector3d &dir, double conethreshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~GazeDirConstraint(void){};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldGazeDirConstraint
    : public GazeDirConstraint {
 protected:
  int body;
  std::string body_name;

 public:
  WorldGazeDirConstraint(
      RigidBodyTree *model, int body, const Eigen::Vector3d &axis,
      const Eigen::Vector3d &dir, double conethreshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~WorldGazeDirConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT GazeTargetConstraint
    : public GazeConstraint {
 protected:
  Eigen::Vector3d target;
  Eigen::Vector3d gaze_origin;

 public:
  GazeTargetConstraint(
      RigidBodyTree *model, const Eigen::Vector3d &axis,
      const Eigen::Vector3d &target, const Eigen::Vector3d &gaze_origin,
      double conethreshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~GazeTargetConstraint(void){};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldGazeTargetConstraint
    : public GazeTargetConstraint {
 protected:
  int body;
  std::string body_name;

 public:
  WorldGazeTargetConstraint(
      RigidBodyTree *model, int body, const Eigen::Vector3d &axis,
      const Eigen::Vector3d &target, const Eigen::Vector3d &gaze_origin,
      double conethreshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~WorldGazeTargetConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT RelativeGazeTargetConstraint
    : public GazeTargetConstraint {
 protected:
  int bodyA_idx;
  int bodyB_idx;
  std::string bodyA_name;
  std::string bodyB_name;

 public:
  RelativeGazeTargetConstraint(
      RigidBodyTree *model, int bodyA_idx, int bodyB_idx,
      const Eigen::Vector3d &axis, const Eigen::Vector3d &target,
      const Eigen::Vector3d &gaze_origin, double conethreshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~RelativeGazeTargetConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT RelativeGazeDirConstraint
    : public GazeDirConstraint {
 protected:
  int bodyA_idx;
  int bodyB_idx;
  std::string bodyA_name;
  std::string bodyB_name;

 public:
  RelativeGazeDirConstraint(
      RigidBodyTree *model, int bodyA_idx, int bodyB_idx,
      const Eigen::Vector3d &axis, const Eigen::Vector3d &dir,
      double conethreshold,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual ~RelativeGazeDirConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT Point2PointDistanceConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  int bodyA;
  int bodyB;
  Eigen::Matrix3Xd ptA;
  Eigen::Matrix3Xd ptB;
  Eigen::VectorXd dist_lb;
  Eigen::VectorXd dist_ub;

 public:
  Point2PointDistanceConstraint(
      RigidBodyTree *model, int bodyA, int bodyB, const Eigen::Matrix3Xd &ptA,
      const Eigen::Matrix3Xd &ptB, const Eigen::VectorXd &lb,
      const Eigen::VectorXd &ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~Point2PointDistanceConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT Point2LineSegDistConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  int pt_body;
  int line_body;
  Eigen::Vector3d pt;
  Eigen::Matrix3Xd line_ends;
  double dist_lb;
  double dist_ub;

 public:
  Point2LineSegDistConstraint(
      RigidBodyTree *model, int pt_body, const Eigen::Vector3d &pt,
      int line_body, const Eigen::Matrix<double, 3, 2> &line_ends,
      double dist_lb, double dist_ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name_str) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~Point2LineSegDistConstraint(void){};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldFixedPositionConstraint
    : public MultipleTimeKinematicConstraint {
 protected:
  int body;
  std::string body_name;
  Eigen::Matrix3Xd pts;

 public:
  WorldFixedPositionConstraint(
      RigidBodyTree *model, int body, const Eigen::Matrix3Xd &pts,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual int getNumConstraint(const double *t, int n_breaks) const;
  virtual void eval_valid(const double *valid_t, int num_valid_t,
                          const Eigen::MatrixXd &valid_q, Eigen::VectorXd &c,
                          Eigen::MatrixXd &dc_valid) const;
  virtual void bounds(const double *t, int n_breaks, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual void name(const double *t, int n_breaks,
                    std::vector<std::string> &name_str) const;
  virtual ~WorldFixedPositionConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldFixedOrientConstraint
    : public MultipleTimeKinematicConstraint {
 protected:
  int body;
  std::string body_name;

 public:
  WorldFixedOrientConstraint(
      RigidBodyTree *model, int body,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual int getNumConstraint(const double *t, int n_breaks) const;
  virtual void eval_valid(const double *valid_t, int num_valid_t,
                          const Eigen::MatrixXd &valid_q, Eigen::VectorXd &c,
                          Eigen::MatrixXd &dc_valid) const;
  virtual void bounds(const double *t, int n_breaks, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual void name(const double *t, int n_breaks,
                    std::vector<std::string> &name_str) const;
  virtual ~WorldFixedOrientConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldFixedBodyPoseConstraint
    : public MultipleTimeKinematicConstraint {
 protected:
  int body;
  std::string body_name;

 public:
  WorldFixedBodyPoseConstraint(
      RigidBodyTree *model, int body,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual int getNumConstraint(const double *t, int n_breaks) const;
  virtual void eval_valid(const double *valid_t, int num_valid_t,
                          const Eigen::MatrixXd &valid_q, Eigen::VectorXd &c,
                          Eigen::MatrixXd &dc_valid) const;
  virtual void bounds(const double *t, int n_breaks, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual void name(const double *t, int n_breaks,
                    std::vector<std::string> &name_str) const;
  virtual ~WorldFixedBodyPoseConstraint(void){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT AllBodiesClosestDistanceConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  double ub;
  double lb;
  std::vector<int> active_bodies_idx;
  std::set<std::string> active_group_names;

 public:
  AllBodiesClosestDistanceConstraint(
      RigidBodyTree *model, double lb, double ub,
      const std::vector<int> &active_bodies_idx,
      const std::set<std::string> &active_group_names,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  // AllBodiesClosestDistanceConstraint(const
  // AllBodiesClosestDistanceConstraint& rhs);
  virtual void updateRobot(RigidBodyTree *robot);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~AllBodiesClosestDistanceConstraint(){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT MinDistanceConstraint
    : public SingleTimeKinematicConstraint {
 protected:
  double min_distance;
  std::vector<int> active_bodies_idx;
  std::set<std::string> active_group_names;

 public:
  MinDistanceConstraint(
      RigidBodyTree *model, double min_distance,
      const std::vector<int> &active_bodies_idx,
      const std::set<std::string> &active_group_names,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  // MinDistanceConstraint(const MinDistanceConstraint& rhs);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name) const;
  void scaleDistance(const Eigen::VectorXd &dist, Eigen::VectorXd &scaled_dist,
                     Eigen::MatrixXd &dscaled_dist_ddist) const;
  void penalty(const Eigen::VectorXd &dist, Eigen::VectorXd &cost,
               Eigen::MatrixXd &dcost_ddist) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~MinDistanceConstraint(){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT WorldPositionInFrameConstraint
    : public WorldPositionConstraint {
 protected:
  Eigen::Matrix4d T_world_to_frame;
  Eigen::Matrix4d T_frame_to_world;
  virtual void evalPositions(KinematicsCache<double> &cache,
                             Eigen::Matrix3Xd &pos, Eigen::MatrixXd &J) const;
  virtual void evalNames(const double *t,
                         std::vector<std::string> &cnst_names) const;

 public:
  WorldPositionInFrameConstraint(
      RigidBodyTree *model, int body, const Eigen::Matrix3Xd &pts,
      const Eigen::Matrix4d &T_world_to_frame, const Eigen::MatrixXd &lb,
      const Eigen::MatrixXd &ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldPositionInFrameConstraint();

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT PostureChangeConstraint
    : public MultipleTimeLinearPostureConstraint {
 protected:
  Eigen::VectorXi joint_ind;
  Eigen::VectorXd lb_change;
  Eigen::VectorXd ub_change;
  virtual void setJointChangeBounds(const Eigen::VectorXi &joint_ind,
                                    const Eigen::VectorXd &lb_change,
                                    const Eigen::VectorXd &ub_change);

 public:
  PostureChangeConstraint(
      RigidBodyTree *model, const Eigen::VectorXi &joint_ind,
      const Eigen::VectorXd &lb_change, const Eigen::VectorXd &ub_change,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual int getNumConstraint(const double *t, int n_breaks) const;
  virtual void feval(const double *t, int n_breaks, const Eigen::MatrixXd &q,
                     Eigen::VectorXd &c) const;
  virtual void geval(const double *t, int n_breaks, Eigen::VectorXi &iAfun,
                     Eigen::VectorXi &jAvar, Eigen::VectorXd &A) const;
  virtual void name(const double *t, int n_breaks,
                    std::vector<std::string> &name_str) const;
  virtual void bounds(const double *t, int n_breaks, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~PostureChangeConstraint(){};
};

class DRAKERIGIDBODYCONSTRAINT_EXPORT GravityCompensationTorqueConstraint
    : public SingleTimeKinematicConstraint {
 public:
  GravityCompensationTorqueConstraint(
      RigidBodyTree *model, const Eigen::VectorXi &joint_indices,
      const Eigen::VectorXd &lb, const Eigen::VectorXd &ub,
      const Eigen::Vector2d &tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void eval(const double *t, KinematicsCache<double> &cache,
                    Eigen::VectorXd &c, Eigen::MatrixXd &dc) const;
  virtual void name(const double *t, std::vector<std::string> &name) const;
  virtual void bounds(const double *t, Eigen::VectorXd &lb,
                      Eigen::VectorXd &ub) const;
  virtual ~GravityCompensationTorqueConstraint(){};

 protected:
  Eigen::VectorXi joint_indices;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
};
#endif

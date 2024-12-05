// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <limits>
#include <vector>

#include "altro/constraints/constraint.hpp"
#include "altro/eigentypes.hpp"
#include "altro/utils/utils.hpp"
#include "./SDPsolver/SDPsolver.hpp"
#include "Altro_control/rotation_tool.hpp"
namespace altro {
namespace examples {

class GoalConstraint : public constraints::Constraint<constraints::Equality> {
 public:
  explicit GoalConstraint(const VectorXd& xf) : xf_(xf) {}

  static constraints::ConstraintPtr<constraints::Equality> Create(const VectorXd& xf) {
    return std::make_shared<GoalConstraint>(xf);
  }

  std::string GetLabel() const override { return "Goal Constraint"; }
  int StateDimension() const override { return xf_.size(); }
  int OutputDimension() const override { return xf_.size(); }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> c) override {
    ALTRO_UNUSED(u);
    c = x - xf_;
  }
  
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    ALTRO_UNUSED(x);
    ALTRO_UNUSED(u);
    jac.setIdentity();
  }

 private:
  VectorXd xf_;
};

class ControlBound : public constraints::Constraint<constraints::NegativeOrthant> {
 public:
  explicit ControlBound(const int m)
      : m_(m),
        lower_bound_(m, -std::numeric_limits<double>::infinity()),
        upper_bound_(m, +std::numeric_limits<double>::infinity()) {}

  ControlBound(const std::vector<double>& lb, const std::vector<double>& ub)
      : m_(lb.size()), lower_bound_(lb), upper_bound_(ub) {
    ALTRO_ASSERT(lb.size() == ub.size(), "Upper and lower bounds must have the same length.");
    ALTRO_ASSERT(lb.size() > 0, "Cannot pass in empty bounds.");
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  void SetUpperBound(const std::vector<double>& ub) {
    ALTRO_ASSERT(ub.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting upper bound.");
    upper_bound_ = ub;
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    ValidateBounds();
  }

  void SetUpperBound(std::vector<double>&& ub) {
    ALTRO_ASSERT(ub.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting upper bound.");
    upper_bound_ = std::move(ub);
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    ValidateBounds();
  }

  void SetLowerBound(const std::vector<double>& lb) {
    ALTRO_ASSERT(lb.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting lower bound.");
    lower_bound_ = lb;
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  void SetLowerBound(std::vector<double>&& lb) {
    ALTRO_ASSERT(lb.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting lower bound.");
    lower_bound_ = std::move(lb);
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  std::string GetLabel() const override { return "Control Bound";}

  int ControlDimension() const override { return m_; }

  int OutputDimension() const override {
    return index_lower_bound_.size() + index_upper_bound_.size();
  }

  void Evaluate(const VectorXdRef& /*x*/, const VectorXdRef& u,
                Eigen::Ref<VectorXd> c) override {
    ALTRO_ASSERT(u.size() == m_, "Inconsistent control dimension when evaluating control bound.");

    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      size_t j = index_lower_bound_[i];
      c(i) = lower_bound_.at(j) - u(j);
    }
    int offset = index_lower_bound_.size();
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      size_t j = index_upper_bound_[i];
      c(i + offset) = u(j) - upper_bound_.at(j);
    }
  }

  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    (void) u; // surpress erroneous unused variable error
    ALTRO_ASSERT(u.size() == m_, "Inconsistent control dimension when evaluating control bound.");
    jac.setZero();

    int n = x.size();  // state dimension
    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      size_t j = index_lower_bound_[i];
      jac(i, n + j) = -1;
    }
    int offset = index_lower_bound_.size();
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      size_t j = index_upper_bound_[i];
      jac(i + offset, n + j) = 1;
    }
  }

 private:
  void ValidateBounds() {
    for (int i = 0; i < m_; ++i) {
      ALTRO_ASSERT(lower_bound_[i] <= upper_bound_[i],
                   "Lower bound isn't less than the upper bound.");
    }
  }
  static void GetFiniteIndices(const std::vector<double>& bound, std::vector<size_t>* index) {
    index->clear();
    for (size_t i = 0; i < bound.size(); ++i) {
      if (std::abs(bound[i]) < std::numeric_limits<double>::max()) {
        index->emplace_back(i);
      }
    }
  }
  int m_;
  std::vector<double> lower_bound_;
  std::vector<double> upper_bound_;
  std::vector<size_t> index_lower_bound_;
  std::vector<size_t> index_upper_bound_;
};


class SDPConstraint : public constraints::Constraint<constraints::NegativeOrthant> {
 public:
explicit SDPConstraint(const string &envname, const Eigen::MatrixXd &A, const Eigen::VectorXd &b, const Eigen::MatrixXd &F, const Eigen::VectorXd &g, const Eigen::VectorXd &c):
  envname_(envname), A_(A), b_(b), F_(F), g_(g), c_(c)
  {
    std::cout << "SDPConstraint created start" << std::endl;
    std::cout << "SDPConstraint created" << std::endl;
  }

   //name is "SDP constraint" + envname
   std::string GetLabel() const override { return "SDP Constraint";}

  int ControlDimension() const override { return 0; }

  int OutputDimension() const override { return 1; }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> c) override {
    //create a static x_init with all zeros
    static Eigen::VectorXd x_init = Eigen::VectorXd::Zero(6);
    static bool need_update = false;

    //every time the function is called, compare the x with x_init
    //if x is not equal to x_init, update x_init with x and set the need_update to true
    if (x != x_init)
    {
      x_init = x;
      need_update = true;
    }
    // ALTRO_UNUSED(x);
    ALTRO_UNUSED(u);
    // ALTRO_UNUSED(c);
    Eigen::VectorXd q;
    q = x;
    if (need_update)
    {
      sdp.solveSDP(envname_, A_, b_, F_, g_, q, c_);
      // std::cout << "SDPConstraint:  need to update!!!!!!!!!!!!!!!!!" << std::endl;
      // use fmt to print with green color
      // fmt::print(fmt::color::red, "SDPConstraint:  need to update\n");
      need_update = false;
    }
    else
    {
      // std::cout << "SDPConstraint: No need to update" << std::endl;
    }
    Eigen::VectorXd alpha_temp;
    alpha_temp.resize(1);
    //sdp.getAlpha() is double, put it in alpha_temp
    constraint_value_ = alpha_temp(0) = sdp.getAlpha()-0.98;
    c = alpha_temp;
    // if (envname_ == "sdp5")
    // {
    //   std::cout << "SDP EVA TEST " << c << std::endl;
    // }
  }
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    ALTRO_UNUSED(x);
    ALTRO_UNUSED(u);

    // if (sdp.getAlpha() > 1)
    // {
      Eigen::VectorXd jac_x= sdp.getGradient();
      if (envname_ == "sdp0" || envname_ == "sdp1" || envname_ == "sdp2")
      {
        // jac_x(2) = 0;
        jac_x(3) = 0;
        jac_x(4) = 0;
      }
      //print jac_x
      // std::cout << "jac_x: " << jac_x << std::endl;
      Eigen::VectorXd jac_u;
      jac_u.resize(1);
      jac << jac_x.transpose(),jac_u;
      
    // }
    // else
    // {
    //   jac.setZero();
    // }
  }

  bool CheckSafetyConstraints() const override {

        if (constraint_value_ < 0)
        {
            return true;
        }
        else
        {
            std::cout << "constraint_value_: " << envname_ <<":"<<constraint_value_ << std::endl;
            return false;
        }
    }
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::MatrixXd F_;
  Eigen::VectorXd g_;
  Eigen::VectorXd c_;
 private:
  Eigen::VectorXd xf_;
  SDPsolver sdp;
  string envname_;
  double constraint_value_;
  bool safety_;

  // Eigen::VectorXd q_;

};

class StateBound : public constraints::Constraint<constraints::NegativeOrthant> {
 public:
  explicit StateBound(const int m)
      : m_(m),
        lower_bound_(m, -std::numeric_limits<double>::infinity()),
        upper_bound_(m, +std::numeric_limits<double>::infinity()) {}

  StateBound(const std::vector<double>& lb, const std::vector<double>& ub, const std::vector<double>& s, int ind)
      : m_(lb.size()), lower_bound_(lb), upper_bound_(ub), arm_state_(s), ind_(ind) {
    ALTRO_ASSERT(lb.size() == ub.size(), "Upper and lower bounds must have the same length.");
    ALTRO_ASSERT(lb.size() > 0, "Cannot pass in empty bounds.");
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
    dt_ = 0.5;
  }

  void SetUpperBound(const std::vector<double>& ub) {
    ALTRO_ASSERT(ub.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting upper bound.");
    upper_bound_ = ub;
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    ValidateBounds();
  }

  void SetUpperBound(std::vector<double>&& ub) {
    ALTRO_ASSERT(ub.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting upper bound.");
    upper_bound_ = std::move(ub);
    GetFiniteIndices(upper_bound_, &index_upper_bound_);
    ValidateBounds();
  }

  void SetLowerBound(const std::vector<double>& lb) {
    ALTRO_ASSERT(lb.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting lower bound.");
    lower_bound_ = lb;
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  void SetLowerBound(std::vector<double>&& lb) {
    ALTRO_ASSERT(lb.size() == static_cast<size_t>(m_),
                 "Inconsistent control dimension when setting lower bound.");
    lower_bound_ = std::move(lb);
    GetFiniteIndices(lower_bound_, &index_lower_bound_);
    ValidateBounds();
  }

  std::string GetLabel() const override { return "State Bound";}

  int ControlDimension() const override { return m_; }

  int OutputDimension() const override {
    return index_lower_bound_.size() + index_upper_bound_.size();
  }

  void Evaluate(const VectorXdRef& , const VectorXdRef& u,
                Eigen::Ref<VectorXd> c) override {
  
    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      size_t j = index_lower_bound_[i];
      c(i) = lower_bound_.at(j) - (arm_state_[ind_-3] + u(j)* dt_ );
    }
    int offset = index_lower_bound_.size();
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      size_t j = index_upper_bound_[i];
      c(i + offset) = arm_state_[ind_-3] + u(j) * dt_ - upper_bound_.at(j);
    }
  }

  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
    (void) u; // surpress erroneous unused variable error
    ALTRO_ASSERT(u.size() == m_, "Inconsistent control dimension when evaluating control bound.");
    jac.setZero();

    int n = x.size();  // state dimension
    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      size_t j = index_lower_bound_[i];
      jac(i, n + j) = -dt_;
    }
    int offset = index_lower_bound_.size();
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      size_t j = index_upper_bound_[i];
      jac(i + offset, n + j) = dt_;
    }
  }

 private:
  void ValidateBounds() {
    for (int i = 0; i < m_; ++i) {
      ALTRO_ASSERT(lower_bound_[i] <= upper_bound_[i],
                   "Lower bound isn't less than the upper bound.");
    }
  }
  static void GetFiniteIndices(const std::vector<double>& bound, std::vector<size_t>* index) {
    index->clear();
    for (size_t i = 0; i < bound.size(); ++i) {
      if (std::abs(bound[i]) < std::numeric_limits<double>::max()) {
        index->emplace_back(i);
      }
    }
  }
  int m_;
  std::vector<double> lower_bound_;
  std::vector<double> upper_bound_;
  std::vector<size_t> index_lower_bound_;
  std::vector<size_t> index_upper_bound_;
  std::vector<double> arm_state_;
  int ind_;
  double dt_;
};


class SDPCBFonstraint : public constraints::Constraint<constraints::NegativeOrthant> {
 public:
 explicit SDPCBFonstraint(const string &envname, const Eigen::MatrixXd &A, const Eigen::VectorXd &b, const Eigen::MatrixXd &F, const Eigen::VectorXd &g, const Eigen::VectorXd &c):
  envname_(envname), A_(A), b_(b), F_(F), g_(g), c_(c)
  {
    std::cout << "SDP_BF created start" << std::endl;
    std::cout << "SDP_BF created" << std::endl;
    mu_ = 5e-5;
    delta_ = 10e-4;
  }

   //name is "SDP constraint" + envname
  std::string GetLabel() const override { return "SDP_BF";}

  int ControlDimension() const override { return 0; }

  int OutputDimension() const override { return 1; }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> c) override {
        ALTRO_UNUSED(u);
        // ALTRO_UNUSED(c);
        Eigen::VectorXd q;
        q = x;
        sdp.solveSDP(envname_, A_, b_, F_, g_, q, c_);
        Eigen::VectorXd alpha_temp;
        alpha_temp.resize(1);
       
        double h = -1*(sdp.getAlpha()-0.98);
        constraint_value_ = h;  
        if (h > delta_) {
            alpha_temp(0) = -mu_ * log(h);
        } else {
            alpha_temp(0) = mu_ * (-log(delta_) + double(0.5) * pow((h - 2.0 * delta_) / delta_, 2.0) - double(0.5));
        }
        c = alpha_temp;
        //print c(0) and h
        // std::cout << "c: " << c(0) << " h: " << h << std::endl;
  }
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
        ALTRO_UNUSED(x);
        ALTRO_UNUSED(u);
        Eigen::VectorXd jac_x= sdp.getGradient();
        double h = -1*(sdp.getAlpha() - 0.98);
        if (h > delta_)
        {
            jac_x = -1*(-mu_ / h * jac_x);
        }
        else
        { 
            jac_x = -1*(mu_ * ((h - 2.0 * delta_) / (delta_ * delta_)) * jac_x);
        }
        if (envname_ == "sdp0" || envname_ == "sdp1" || envname_ == "sdp2")
        {
            jac_x(3) = 0;
            jac_x(4) = 0;
        }
        Eigen::VectorXd jac_u;
        jac_u.resize(1);
        jac << jac_x.transpose(),jac_u;
        //print jac
         std::cout << "jac: " << jac << std::endl;
  }
    bool CheckSafetyConstraints() const override {

        if (constraint_value_ < 0)
        {
            return true;
        }
        else
        {
            std::cout << "constraint_value_: " << envname_ <<":"<<constraint_value_ << std::endl;
            return false;
        }
    }

  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::MatrixXd F_;
  Eigen::VectorXd g_;
  Eigen::VectorXd c_;
 private:
             double  mu_;
            double  delta_;
  Eigen::VectorXd xf_;
  SDPsolver sdp;
  string envname_;
  double constraint_value_;
  bool safety_;

  // Eigen::VectorXd q_;

};


class PointConstraint : public constraints::Constraint<constraints::NegativeOrthant> {
  public:
  explicit PointConstraint(string envname, const Eigen::VectorXd& A, const double b, const Eigen::Vector3d& point)
        : envname_(envname), A_(A), b_(b), point_(point) {}
    std::string GetLabel() const override { return "Point Constraint";}

  int ControlDimension() const override { return 0; }

  int OutputDimension() const override { return 1; }

   void Evaluate(const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<VectorXd> c) override {
    //create a static x_init with all zeros
    auto rpy = x.segment<3>(3);
    Eigen::Matrix3d R =  RPYtoRotation_Matrix(rpy[0], rpy[1], rpy[2]);
    Eigen::Vector3d p = x.segment<3>(0);
    Eigen::Vector3d p_point = R * point_ + p;
    c(0) = A_.dot(p_point) - b_;
    if (c(0) > 0)
    {
       std::cout << "PointConstraint: "<< envname_ << c(0) << std::endl;
    }
    // std::cout << "PointConstraint: "<< envname_ << c(0) << std::endl;
    // std::cout << "Point:  " << p_point.transpose() << std::endl;
  }

    void Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                Eigen::Ref<MatrixXd> jac) override {
        ALTRO_UNUSED(x);
        ALTRO_UNUSED(u);
        Eigen::VectorXd jac_x;
        jac_x.resize(6);
        auto rpy = x.segment<3>(3);
        auto [dR_dRoll, dR_dPitch, dR_dYaw] = RPYtoRotation_MatrixJacobian(rpy[0], rpy[1], rpy[2]);

        jac_x << A_, A_.dot(dR_dRoll * point_), A_.dot( dR_dPitch * point_), A_.dot(dR_dYaw * point_);

        Eigen::VectorXd jac_u;
        jac_u.resize(1);
        jac << jac_x.transpose(),jac_u;
    }

  void setCone(const Eigen::VectorXd& A, const double b) {
    A_ = A;
    b_ = b;
  }
  private:
    string envname_;
    Eigen::VectorXd A_;
    double b_;
    Eigen::Vector3d point_;
};

class PointConstraints {
  public:
    PointConstraints(string envname ,const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::MatrixXd& F, const Eigen::VectorXd& g, const Eigen::VectorXd& c)
        : envname_(envname), A_(A), b_(b), F_(F), g_(g), c_(c) {
          buildPointConstraints();
        }
    
    void buildPointConstraints() {
      ALTRO_ASSERT(b_.size() == 6, "b_ must have size 6.");
      // double x1 = b_(0);
      // double x2 = -b_(1);
      // double y1 = b_(2);
      // double y2 = -b_(3);
      // double z1 = b_(2);
      // double z2 = -b_(5);
      std::vector<Eigen::Vector3d> points;
      points.push_back(Eigen::Vector3d(b_(0), b_(2), b_(4)));
      points.push_back(Eigen::Vector3d(-b_(1), b_(2), b_(4)));
      points.push_back(Eigen::Vector3d(b_(0), -b_(3), b_(4)));
      points.push_back(Eigen::Vector3d(-b_(1), -b_(3), b_(4)));
      points.push_back(Eigen::Vector3d(b_(0), b_(2), -b_(5)));
      points.push_back(Eigen::Vector3d(-b_(1), b_(2), -b_(5)));
      points.push_back(Eigen::Vector3d(b_(0), -b_(3), -b_(5)));
      points.push_back(Eigen::Vector3d(-b_(1), -b_(3), -b_(5)));

      point_constraints_.clear();
      for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < g_.size(); ++j) {
          point_constraints_.push_back(std::make_shared<PointConstraint>(envname_+" " + std::to_string(( g_.size())*(i) + j)+ ": ", F_.row(j), g_(j), points[i]));
        }
       
      }
    }

    std::vector<std::shared_ptr<PointConstraint>> getPointConstraints() {
      return point_constraints_;
    }
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;
    Eigen::MatrixXd F_;
    Eigen::VectorXd g_;
    Eigen::VectorXd c_;
    string envname_;
  private:

    std::vector<std::shared_ptr<PointConstraint>> point_constraints_;
};

}  // namespace examples
}  // namespace altro
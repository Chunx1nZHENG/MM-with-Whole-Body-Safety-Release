
#pragma once


#include <limits>
#include <vector>

#include "altro/constraints/constraint.hpp"
#include "altro/eigentypes.hpp"
#include "altro/utils/utils.hpp"
#include "./SDPsolver/SDPsolver.hpp"
#include "altro/problem/costfunction.hpp"

namespace altro {
namespace costs {
    class SDPRelaxedBarrierPenalty : public problem::CostFunction {
        public:
         SDPRelaxedBarrierPenalty(const string &envname, const Eigen::MatrixXd &A, const Eigen::VectorXd &b, const Eigen::MatrixXd &F, const Eigen::VectorXd &g, const Eigen::VectorXd &c):
          envname_(envname), A_(A), b_(b), F_(F), g_(g), c_(c)
            {
                std::cout << "SDP_BF created start" << std::endl;
                std::cout << "SDP_BF created" << std::endl;
                mu_ = 5e-5;
                delta_ = 10e-4;
            }
        std::string GetLabel() const override { return "SDP cost";}

        int ControlDimension() const override { return 1; }

        int StateDimension() const override { return 6; }
        double Evaluate(const VectorXdRef& x,
                  const VectorXdRef& u) override;
        void Gradient(const VectorXdRef& x, const VectorXdRef& u,
                        Eigen::Ref<VectorXd> dx, Eigen::Ref<VectorXd> du) override;
        void Hessian(const VectorXdRef& x, const VectorXdRef& u,
               Eigen::Ref<MatrixXd> dxdx, Eigen::Ref<MatrixXd> dxdu,
               Eigen::Ref<MatrixXd> dudu) override;

        // const VectorXd& Getq() const { return q_; }
        // const VectorXd& Getr() const { return r_; }
        // double GetConstant() const { return c_; }

        bool CheckSafetyConstraints() const {
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
            bool sdp_activate_;
    };

    class sumCost : public problem::CostFunction {
        public: sumCost() {}
        std::string GetLabel() const override{ return "sumCost";}
        int ControlDimension() const override { return 1; }
        int StateDimension() const override { return 6; }
        double Evaluate(const VectorXdRef& x,
                  const VectorXdRef& u) override
                  {
                    double cost = 0;
                    for(int i = 0; i < costs_ptr_.size(); ++i)
                    {
                        cost += costs_ptr_[i]->Evaluate(x,u);
                    }
                    return cost;
                  }
        void Gradient(const VectorXdRef& x, const VectorXdRef& u,
                    Eigen::Ref<VectorXd> dx, Eigen::Ref<VectorXd> du) override
        {
            dx.setZero();
            du.setZero();
            
            // 临时变量用于存储每次的增量
            Eigen::VectorXd dx_temp(dx.size());
            Eigen::VectorXd du_temp(du.size());
            
            for(int i = 0; i < costs_ptr_.size(); ++i)
            {
                // 每次循环前清零临时变量
                dx_temp.setZero();
                du_temp.setZero();
                
                // 计算增量
                costs_ptr_[i]->Gradient(x, u, dx_temp, du_temp);
                
                // 将增量累加到总的 dx 和 du 中
                dx += dx_temp;
                du += du_temp;
            }
        }
        void Hessian(const VectorXdRef& x, const VectorXdRef& u,
                        Eigen::Ref<MatrixXd> dxdx, Eigen::Ref<MatrixXd> dxdu,
                        Eigen::Ref<MatrixXd> dudu) override
                        {
                            dxdx.setZero();
                            dxdu.setZero();
                            dudu.setZero();
                            Eigen::MatrixXd dxdx_temp = dxdx;
                            Eigen::MatrixXd dxdu_temp = dxdu;
                            Eigen::MatrixXd dudu_temp = dudu;
                            for(int i = 0; i < costs_ptr_.size(); ++i)
                            {
                                costs_ptr_[i]->Hessian(x,u,dxdx_temp,dxdu_temp,dudu_temp);
                                dxdx += dxdx_temp;
                                dxdu += dxdu_temp;
                                dudu += dudu_temp;
                            }
                        }
        void AddCost(std::shared_ptr<problem::CostFunction> cost)
        {
            costs_ptr_.push_back(cost);
        }

        std::vector<std::shared_ptr<problem::CostFunction>> GetCosts() const
        {
            return costs_ptr_;
        }
        private:
         //   costs_ptr_
            std::vector<std::shared_ptr<problem::CostFunction>> costs_ptr_;

    };
} // namespace examples
}  // namespace altro
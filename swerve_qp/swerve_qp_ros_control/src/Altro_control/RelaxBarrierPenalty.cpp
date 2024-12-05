#include "Altro_control/RelaxBarrierPenalty.hpp"
namespace altro {
namespace costs {
double SDPRelaxedBarrierPenalty::Evaluate(const VectorXdRef& x,
                                const VectorXdRef& u) {
        ALTRO_UNUSED(u);

        // ALTRO_UNUSED(c);
        Eigen::VectorXd q;
        q = x;
        sdp.solveSDP(envname_, A_, b_, F_, g_, q, c_);
        Eigen::VectorXd alpha_temp;
        alpha_temp.resize(1);
        //sdp.getAlpha() is double, put it in alpha_temp
        constraint_value_ = alpha_temp(0) = -(sdp.getAlpha()-0.98);
        double h = alpha_temp(0);
        if (h > delta_) {
            return -mu_ * log(h);
        } else {
            return mu_ * (-log(delta_) + 0.5 * pow((h - 2.0 * delta_) / delta_, 2.0) - 0.5);
        }
    }

void SDPRelaxedBarrierPenalty::Gradient(const VectorXdRef& x, const VectorXdRef& u,
                                Eigen::Ref<VectorXd> dx, Eigen::Ref<VectorXd> du) {
        ALTRO_UNUSED(x);
        ALTRO_UNUSED(u);
        Eigen::VectorXd jac_x= sdp.getGradient();
        double h = -(sdp.getAlpha() - 0.98);
        if (h > delta_)
        {
            jac_x = mu_ / h * jac_x;
        }
        else
        {
            jac_x = -mu_ * ((h - 2.0 * delta_) / (delta_ * delta_)) * jac_x;
        }
        if (envname_ == "sdp0" || envname_ == "sdp1" || envname_ == "sdp2")
        {
            jac_x(3) = 0;
            jac_x(4) = 0;
        }
        Eigen::VectorXd jac_u;
        jac_u.resize(1);
        dx = jac_x;
        du.setZero();
    }

void SDPRelaxedBarrierPenalty::Hessian(const VectorXdRef& x, const VectorXdRef& u,
                                Eigen::Ref<MatrixXd> dxdx, Eigen::Ref<MatrixXd> dxdu,
                                Eigen::Ref<MatrixXd> dudu) {
        ALTRO_UNUSED(x);
        ALTRO_UNUSED(u);
        dxdx.setZero();
        dxdu.setZero();
        dudu.setZero();
    }
}  // namespace costs
}  // namespace altro

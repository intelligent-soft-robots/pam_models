#include "pam_models/hill/serial_elastic_element.hpp"

namespace pam_models
{
namespace hill
{
SerialElasticElement::SerialElasticElement(double l_0,
                                           double deltaU_nll,
                                           double deltaU_l,
                                           double deltaF_0)
    : l_0_(l_0),
      deltaU_nll_(deltaU_nll),
      deltaU_l_(deltaU_l),
      deltaF_0_(deltaF_0),
      l_nll_((1 + deltaU_nll) * l_0),
      v_(deltaU_nll / deltaU_l),
      k_nl_(deltaF_0 / pow(deltaU_nll * l_0, v_)),
      k_l_(deltaF_0 / (deltaU_l * l_0))
{
}

double SerialElasticElement::init_serial_elastic_element_force(double l_MTC,
                                                               double l_CE)
{
    double F_SEE_init;
    double l_SEE = fabs(l_MTC - l_CE);

    // non-linear part
    if ((l_SEE > l_0_) && (l_SEE < l_nll_))
    {
        F_SEE_init = k_nl_ * pow(l_SEE - l_0_, v_);
    }
    // linear part
    else if (l_SEE >= l_nll_)
    {
        F_SEE_init = deltaF_0_ + k_l_ * (l_SEE - l_nll_);
    }
    // slack length
    else
    {
        F_SEE_init = 0.0;
    }
    return F_SEE_init;
}

double SerialElasticElement::get_force(double l_MTC, double l_CE)
{
    double l_SEE = fabs(l_MTC - l_CE);
    double F_SEE = 0.0;

    if (l_SEE >= l_nll_)
    {
        F_SEE = deltaF_0_ + k_l_ * (l_SEE - l_nll_);
    }

    if ((l_SEE > l_0_) && (l_SEE < l_nll_))
    {
        F_SEE += k_nl_ * pow(l_SEE - l_0_, v_);
    }

    if (l_SEE < l_0_)
    {
        F_SEE = 0.0;
    }

    return F_SEE;
}

double SerialElasticElement::get_l_nll() const
{
    return l_nll_;
}

double SerialElasticElement::get_v() const
{
    return v_;
}

double SerialElasticElement::get_k_l() const
{
    return k_l_;
}

double SerialElasticElement::get_k_nl() const
{
    return k_nl_;
}

}  // namespace hill

}  // namespace pam_models

#include "pam_models/hill/serial_damping_element.hpp"

namespace pam_models
{
namespace hill
{
SerialDampingElement::SerialDampingElement(
    const ContractileElement& contractile_element,
    double MP_SDE_D_SE,
    double MP_SDE_R_SE)
    : MP_SDE_D_SE_(MP_SDE_D_SE),
      MP_SDE_R_SE_(MP_SDE_R_SE),
      MP_SDE_d_SEmax_(MP_SDE_D_SE *
                      (contractile_element.MP_CE_F_max_ *
                       contractile_element.MP_CE_A_rel0_) /
                      (contractile_element.MP_CE_l_CEopt_ *
                       contractile_element.MP_CE_B_rel0_))
{
}

// line 205
double SerialDampingElement::get_force(double F_CE,
                                       double F_PEE,
                                       double MP_CE_F_max,
                                       double dot_l_MTC,
                                       double dot_l_CE)
{
    double t1 = (1 - MP_SDE_R_SE_) * ((F_CE + F_PEE) / MP_CE_F_max);
    double t2 = dot_l_MTC - dot_l_CE;
    return MP_SDE_d_SEmax_ * (t1 + MP_SDE_R_SE_) * (t2);
}

double SerialDampingElement::get_maximum_damping_coefficent() const
{
    return MP_SDE_d_SEmax_;
}

}  // namespace hill

}  // namespace pam_models
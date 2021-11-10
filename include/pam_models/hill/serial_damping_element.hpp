#pragma once

#include "pam_models/hill/contractile_element.hpp"

namespace pam_models
{

namespace hill
{

class SerialDampingElement
{
public:
    SerialDampingElement(const ContractileElement& contractile_element,
                         double MP_SDE_D_SE,
                         double MP_SDE_R_SE);
    double get_force(double F_CE,
                     double F_PEE,
                     double MP_CE_F_max,
                     double dot_l_MTC,
                     double dot_l_CE);

private:
    friend double init_muscle_force_equilibrium(double,
                                                const ParallelElasticElement&,
                                                const ContractileElement&,
                                                const SerialElasticElement&,
                                                double,
                                                double);
    friend class Muscle;

    /*! xxx dimensionless factor to scale d_SEmax (Moerl et al., 2012) */
    double MP_SDE_D_SE_;
    /*! minimum value of d_SE normalised to d_SEmax (Moerl et al., 2012) */
    double MP_SDE_R_SE_;
    /*! maximum value in d_SE in [Ns/m] (Moerl et al., 2012) */
    double MP_SDE_d_SEmax_;
};

}  // namespace hill

}  // namespace pam_models

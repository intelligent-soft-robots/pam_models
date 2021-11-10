#pragma once

#include <math.h>
#include <stdlib.h>
#include "pam_models/hill/contractile_element.hpp"

namespace pam_models
{

namespace hill
{

class SerialElasticElement
{
public:
    SerialElasticElement(double l_0,
                         double deltaU_nll,
                         double deltaU_l,
                         double deltaF_0);
    double init_serial_elastic_element_force(double l_mtc, double l_ce);
    double get_force(double l_MTC, double l_CE);

private:
    friend double init_muscle_force_equilibrium(double,
                                                const ParallelElasticElement&,
                                                const ContractileElement&,
                                                const SerialElasticElement&,
                                                double,
                                                double);
    friend class Muscle;

    /** rest length of SEE in [m] (Kistemaker et al., 2006) */
    double l_0_;
    /** relative stretch at non-linear linear transition (Moerl et al., 2012) */
    double deltaU_nll_;
    /** relative additional stretch in the linear part
     * providing a force increase of deltaF_0 (Moerl, 2012)
     */
    double deltaU_l_;
    /** both force at the transition and force increase
     *  in the linear part in [N] (~ 40// of the maximal isometric muscle force)
     */
    double deltaF_0_;

    double l_nll_;
    double v_;
    double k_nl_;
    double k_l_;
};

}  // namespace hill

}  // namespace pam_models

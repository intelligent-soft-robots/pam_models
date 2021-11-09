#pragma once

#include <math.h>

namespace pam_models
{

namespace hill
{

// forward declaration for
// using friend keyword
class ParallelElasticElement;
class SerialDampingElement;
class Muscle;
class ContractileElement;
class SerialElasticElement;
double init_muscle_force_equilibrium(double,
                                     const ParallelElasticElement&,
                                     const ContractileElement&,
                                     const SerialElasticElement&,
                                     double,
                                     double);

class ContractileElement
{
public:
    ContractileElement(double f_max,
                       double MP_CE_l_CEopt,
                       double MP_CE_DeltaW_limb_des,
                       double MP_CE_DeltaW_limb_asc,
                       double MP_CE_v_CElimb_des,
                       double MP_CE_v_CElimb_asc,
                       double MP_CE_A_rel0,
                       double MP_CE_B_rel0,
                       double MP_CE_S_eccentric,
                       double MP_CE_F_eccentric);

    double get_isometric_force(double l_CE);
    double get_a_relative(double l_CE, double F_isom, double a);
    double get_b_relative(double a);

private:
    friend double init_muscle_force_equilibrium(double,
                                                const ParallelElasticElement&,
                                                const ContractileElement&,
                                                const SerialElasticElement&,
                                                double,
                                                double);
    friend class ParallelElasticElement;
    friend class SerialDampingElement;
    friend class Muscle;

    /*! F_max in [N] for Extensor (Kistemaker et al., 2006) */
    double MP_CE_F_max_;
    /*! optimal length of CE in [m] for Extensor (Kistemaker et al., 2006) */
    double MP_CE_l_CEopt_;
    /*! width of normalized bell curve in descending branch (Moerl et al., 2012)
     */
    double MP_CE_DeltaW_limb_des_;
    /*! width of normalized bell curve in ascending branch (Moerl et al., 2012)
     */
    double MP_CE_DeltaW_limb_asc_;
    /*! exponent for descending branch (Moerl et al., 2012) */
    double MP_CE_v_CElimb_des_;
    /*! exponent for ascending branch (Moerl et al., 2012) */
    double MP_CE_v_CElimb_asc_;
    /*! parameter for contraction dynamics: maximum value of A_rel (Guenther,
     * 1997, S. 82) */
    double MP_CE_A_rel0_;
    /*! parameter for contraction dynmacis: maximum value of B_rel (Guenther,
     * 1997, S. 82)*/
    double MP_CE_B_rel0_;
    /*! eccentric force-velocity relation:
     * relation between F(v) slopes at v_CE=0 (van Soest & Bobbert, 1993)*/
    double MP_CE_S_eccentric_;
    /*! eccentric force-velocity relation:
     *  factor by which the force can exceed F_isom for large eccentric
     * velocities (van Soest & Bobbert, 1993) */
    double MP_CE_F_eccentric_;
};

}  // namespace hill

}  // namespace pam_models
